// v1.5.2 — 2026-04-12 21:05
/*******************************************************************************
 * WATER HEATER MONITOR — ESP32 T-Display S3
 *
 * Monitors a Resideo (Honeywell) gas water heater by reading the gas valve's
 * status LED flash codes (TSL2591 light sensor) and measuring tank surface
 * temperature (DS18B20 on hot water outlet pipe). Displays on built-in LCD
 * and reports to Home Assistant via WiFi.
 *
 * Hardware: Lilygo T-Display S3, TSL2591 (I2C/STEMMA QT), DS18B20 (1-Wire)
 ******************************************************************************/

#include <WiFi.h>
#include <Wire.h>
#include <Preferences.h>
#include <time.h>

#include <TFT_eSPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2591.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ArduinoJson.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <ElegantOTA.h>
#include <HTTPClient.h>
#include <ArduinoOTA.h>

/*******************************************************************************
 * PIN DEFINITIONS
 ******************************************************************************/
#define PIN_DS18B20       21    // 1-Wire data bus for DS18B20
#define PIN_I2C_SDA       43    // I2C SDA (STEMMA QT default)
#define PIN_I2C_SCL       44    // I2C SCL (STEMMA QT default)
#define PIN_BUTTON_1      0     // Left button (Boot)
#define PIN_BUTTON_2      14    // Right button
#define PIN_LCD_BL        38    // LCD backlight PWM
#define PIN_LCD_POWER     15    // LCD power enable

/*******************************************************************************
 * CONFIGURATION CONSTANTS
 ******************************************************************************/
#define TSL2591_ADDR             0x29
#define TEMP_READ_INTERVAL_MS    10000   // 10 seconds
#define LIGHT_SAMPLE_INTERVAL_MS 50      // 50ms for flash detection
#define HA_PUSH_INTERVAL_MS      30000   // 30 seconds
#define SCREEN_COUNT             4
#define MAX_CYCLES               100
#define MAX_LOG_ENTRIES           50
#define TEMP_HISTORY_SIZE         360    // ~12 hours at 2-min intervals
#define HOURLY_BUCKETS            48     // 48 hours of hourly data

// Flash detection timing
#define FLASH_GAP_TIMEOUT_MS     2000   // >2s gap = end of pattern
#define PILOT_OUT_TIMEOUT_MS     10000  // >10s no flash = pilot out
#define FLASH_DEBOUNCE_MS        200    // Minimum time between flashes

// Temperature cycle detection
#define CYCLE_TEMP_RISE_THRESHOLD 2.0   // 2 degrees F rise
#define CYCLE_DETECT_WINDOW_MS   300000 // 5 minutes

// WiFi AP defaults
#define AP_SSID     "WaterHeaterMonitor"
#define AP_PASSWORD "configure"
#define AP_IP       IPAddress(192, 168, 4, 1)

// NTP
#define NTP_SERVER  "pool.ntp.org"
#define GMT_OFFSET  -18000  // US Central (-5h) — configurable later
#define DST_OFFSET  3600

/*******************************************************************************
 * DATA STRUCTURES
 ******************************************************************************/

// Flash code status definitions
struct FlashCodeInfo {
  int code;
  const char* text;
  const char* severity;
  uint16_t color;  // TFT color
};

const FlashCodeInfo FLASH_CODES[] = {
  {0, "Control Off / Pilot Out",     "ERROR",    TFT_RED},
  {1, "Normal Operation",            "OK",       TFT_GREEN},
  {2, "Thermopile Voltage Low",      "WARNING",  TFT_YELLOW},
  {4, "High Temperature Shutdown",   "CRITICAL", TFT_RED},
  {5, "Temperature Sensor Failure",  "CRITICAL", TFT_RED},
  {7, "Gas Control / Valve Failure", "CRITICAL", TFT_RED},
  {8, "Flame Sense Error",           "ERROR",    TFT_RED},
};
const int FLASH_CODE_COUNT = sizeof(FLASH_CODES) / sizeof(FLASH_CODES[0]);

// Flash detection state machine
enum FlashState {
  FLASH_WAITING,
  FLASH_COUNTING,
  FLASH_PAUSE_DETECTED,
  FLASH_CONFIRMED
};

// Heating cycle record
struct HeatingCycle {
  time_t startTime;
  time_t endTime;
  uint32_t durationSec;
  float peakTempF;
  bool active;
};

// System configuration (stored in Preferences)
struct Config {
  char wifiSSID[64];
  char wifiPassword[64];
  uint16_t lightThreshold;
  uint8_t lightGain;         // 0=auto, 1=1x, 2=25x, 3=428x, 4=9876x
  uint16_t lightIntegration; // ms: 100,200,300,400,500,600
  uint16_t tempReadInterval; // seconds
  bool useFahrenheit;
  float highTempWarning;     // degrees F
  float criticalTemp;        // degrees F
  bool haEnabled;
  char haURL[128];
  char haToken[256];
  char haEntityPrefix[64];
  uint8_t screenRotationSec;
  bool screensEnabled[SCREEN_COUNT];
  uint8_t brightness;
  uint8_t dimBrightness;
  uint8_t dimStartHour;
  uint8_t dimEndHour;
  uint16_t lightBaseline;
};

// Log entry
struct LogEntry {
  time_t timestamp;
  char message[128];
};

/*******************************************************************************
 * GLOBAL OBJECTS
 ******************************************************************************/
TFT_eSPI g_tft = TFT_eSPI();
TFT_eSprite g_sprite = TFT_eSprite(&g_tft);

Adafruit_TSL2591 g_tsl = Adafruit_TSL2591(2591);
OneWire g_oneWire(PIN_DS18B20);
DallasTemperature g_ds18b20(&g_oneWire);

AsyncWebServer g_server(80);
Preferences g_prefs;

/*******************************************************************************
 * GLOBAL STATE
 ******************************************************************************/
Config g_cfg;

// Sensor availability
bool g_tslFound = false;
bool g_ds18b20Found = false;
DeviceAddress g_ds18b20Addr;

// Flash detection state
FlashState g_flashState = FLASH_WAITING;
int g_flashCount = 0;
int g_pendingFlashCount = -1;
int g_confirmedFlashCode = -1;  // -1 = unknown
unsigned long g_lastFlashTime = 0;
unsigned long g_lastLightSampleTime = 0;
bool g_lastLightHigh = false;
uint16_t g_lightBaseline = 0;
uint16_t g_lightPeak = 0;

// Temperature state
float g_currentTempF = NAN;
float g_currentTempC = NAN;
unsigned long g_lastTempRead = 0;
bool g_tempConversionPending = false;
unsigned long g_tempConversionStart = 0;

// Heating cycle detection
bool g_burnerActive = false;
float g_cycleStartTemp = NAN;
unsigned long g_cycleStartTime = 0;
float g_cyclePeakTemp = NAN;
float g_tempHistory2Min[60];  // Last 60 readings for slope detection (10s intervals = 10 min)
int g_tempHistIdx = 0;
int g_tempHistCount = 0;

// Cycle history
HeatingCycle g_cycles[MAX_CYCLES];
int g_cycleCount = 0;
int g_cycleHead = 0;  // Circular buffer head

// Temperature graph history (2-min intervals)
float g_tempGraphData[TEMP_HISTORY_SIZE];
int g_tempGraphCount = 0;
int g_tempGraphHead = 0;
unsigned long g_lastTempGraphSample = 0;

// Hourly heating minutes (circular, last 48h)
float g_hourlyHeatMin[HOURLY_BUCKETS];
int g_hourlyHead = 0;
uint8_t g_lastRecordedHour = 255;

// Display state
int g_currentScreen = 3;  // Default to Hot Water ON/OFF screen
unsigned long g_lastScreenChange = 0;
unsigned long g_lastButtonPress = 0;
bool g_manualScreenNav = false;

// Log buffer
LogEntry g_log[MAX_LOG_ENTRIES];
int g_logHead = 0;
int g_logCount = 0;

// WiFi state
bool g_wifiConnected = false;
bool g_apMode = false;
unsigned long g_bootTime = 0;

// Home Assistant
unsigned long g_lastHAPush = 0;
bool g_haStatusChanged = false;

// Button state
bool g_btn1Pressed = false;
bool g_btn2Pressed = false;

/*******************************************************************************
 * LOGGING
 ******************************************************************************/
void addLog(const char* msg) {
  struct tm timeinfo;
  LogEntry& entry = g_log[g_logHead];
  if (getLocalTime(&timeinfo, 0)) {
    entry.timestamp = mktime(&timeinfo);
  } else {
    entry.timestamp = millis() / 1000;
  }
  strncpy(entry.message, msg, sizeof(entry.message) - 1);
  entry.message[sizeof(entry.message) - 1] = '\0';
  g_logHead = (g_logHead + 1) % MAX_LOG_ENTRIES;
  if (g_logCount < MAX_LOG_ENTRIES) g_logCount++;
  Serial.printf("[LOG] %s\n", msg);
}

/*******************************************************************************
 * CONFIGURATION — PREFERENCES
 ******************************************************************************/
void loadConfig() {
  g_prefs.begin("whm", true);  // read-only

  strlcpy(g_cfg.wifiSSID,       g_prefs.getString("wifiSSID", "").c_str(),       sizeof(g_cfg.wifiSSID));
  strlcpy(g_cfg.wifiPassword,   g_prefs.getString("wifiPass", "").c_str(),       sizeof(g_cfg.wifiPassword));
  g_cfg.lightThreshold     = g_prefs.getUShort("lightThresh", 100);
  g_cfg.lightGain          = g_prefs.getUChar("lightGain", 3);       // 428x default
  g_cfg.lightIntegration   = g_prefs.getUShort("lightInt", 100);     // 100ms
  g_cfg.tempReadInterval   = g_prefs.getUShort("tempInt", 10);       // 10 seconds
  g_cfg.useFahrenheit      = g_prefs.getBool("useF", true);
  g_cfg.highTempWarning    = g_prefs.getFloat("highTempWarn", 150.0);
  g_cfg.criticalTemp       = g_prefs.getFloat("critTemp", 160.0);
  g_cfg.haEnabled          = g_prefs.getBool("haEnabled", false);
  strlcpy(g_cfg.haURL,          g_prefs.getString("haURL", "").c_str(),          sizeof(g_cfg.haURL));
  strlcpy(g_cfg.haToken,        g_prefs.getString("haToken", "").c_str(),        sizeof(g_cfg.haToken));
  strlcpy(g_cfg.haEntityPrefix, g_prefs.getString("haPrefix", "water_heater_monitor").c_str(), sizeof(g_cfg.haEntityPrefix));
  g_cfg.screenRotationSec  = g_prefs.getUChar("scrRotSec", 10);
  g_cfg.screensEnabled[0]  = g_prefs.getBool("scrEn0", true);
  g_cfg.screensEnabled[1]  = g_prefs.getBool("scrEn1", true);
  g_cfg.screensEnabled[2]  = g_prefs.getBool("scrEn2", true);
  g_cfg.screensEnabled[3]  = g_prefs.getBool("scrEn3", true);
  g_cfg.brightness         = g_prefs.getUChar("bright", 200);
  g_cfg.dimBrightness      = g_prefs.getUChar("dimBright", 30);
  g_cfg.dimStartHour       = g_prefs.getUChar("dimStart", 22);
  g_cfg.dimEndHour         = g_prefs.getUChar("dimEnd", 6);
  g_cfg.lightBaseline      = g_prefs.getUShort("lightBase", 0);

  g_prefs.end();
}

void saveConfig() {
  g_prefs.begin("whm", false);

  g_prefs.putString("wifiSSID",  g_cfg.wifiSSID);
  g_prefs.putString("wifiPass",  g_cfg.wifiPassword);
  g_prefs.putUShort("lightThresh", g_cfg.lightThreshold);
  g_prefs.putUChar("lightGain",  g_cfg.lightGain);
  g_prefs.putUShort("lightInt",  g_cfg.lightIntegration);
  g_prefs.putUShort("tempInt",   g_cfg.tempReadInterval);
  g_prefs.putBool("useF",       g_cfg.useFahrenheit);
  g_prefs.putFloat("highTempWarn", g_cfg.highTempWarning);
  g_prefs.putFloat("critTemp",  g_cfg.criticalTemp);
  g_prefs.putBool("haEnabled",  g_cfg.haEnabled);
  g_prefs.putString("haURL",    g_cfg.haURL);
  g_prefs.putString("haToken",  g_cfg.haToken);
  g_prefs.putString("haPrefix", g_cfg.haEntityPrefix);
  g_prefs.putUChar("scrRotSec", g_cfg.screenRotationSec);
  g_prefs.putBool("scrEn0",     g_cfg.screensEnabled[0]);
  g_prefs.putBool("scrEn1",     g_cfg.screensEnabled[1]);
  g_prefs.putBool("scrEn2",     g_cfg.screensEnabled[2]);
  g_prefs.putBool("scrEn3",     g_cfg.screensEnabled[3]);
  g_prefs.putUChar("bright",    g_cfg.brightness);
  g_prefs.putUChar("dimBright", g_cfg.dimBrightness);
  g_prefs.putUChar("dimStart",  g_cfg.dimStartHour);
  g_prefs.putUChar("dimEnd",    g_cfg.dimEndHour);
  g_prefs.putUShort("lightBase", g_cfg.lightBaseline);

  g_prefs.end();
}

/*******************************************************************************
 * SENSOR INITIALIZATION
 ******************************************************************************/
bool initTSL2591() {
  if (!g_tsl.begin()) {
    Serial.println("TSL2591 not found at 0x29");
    addLog("TSL2591 not found on I2C bus");
    return false;
  }

  // Configure gain
  switch (g_cfg.lightGain) {
    case 1: g_tsl.setGain(TSL2591_GAIN_LOW);  break;   // 1x
    case 2: g_tsl.setGain(TSL2591_GAIN_MED);  break;   // 25x
    case 3: g_tsl.setGain(TSL2591_GAIN_HIGH); break;   // 428x
    case 4: g_tsl.setGain(TSL2591_GAIN_MAX);  break;   // 9876x
    default: g_tsl.setGain(TSL2591_GAIN_MED); break;
  }

  // Configure integration time (100ms for flash detection)
  g_tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS);

  addLog("TSL2591 initialized OK");
  Serial.println("TSL2591 initialized");
  return true;
}

bool initDS18B20() {
  g_ds18b20.begin();
  int deviceCount = g_ds18b20.getDeviceCount();

  if (deviceCount == 0) {
    Serial.println("No DS18B20 found on 1-Wire bus");
    addLog("DS18B20 not found on GPIO21");
    return false;
  }

  if (!g_ds18b20.getAddress(g_ds18b20Addr, 0)) {
    addLog("DS18B20 address read failed");
    return false;
  }

  g_ds18b20.setResolution(g_ds18b20Addr, 12);  // 12-bit = 0.0625C
  g_ds18b20.setWaitForConversion(false);        // Non-blocking

  char addrStr[24];
  snprintf(addrStr, sizeof(addrStr), "DS18B20 addr: %02X%02X%02X%02X%02X%02X%02X%02X",
           g_ds18b20Addr[0], g_ds18b20Addr[1], g_ds18b20Addr[2], g_ds18b20Addr[3],
           g_ds18b20Addr[4], g_ds18b20Addr[5], g_ds18b20Addr[6], g_ds18b20Addr[7]);
  addLog(addrStr);
  Serial.println(addrStr);
  return true;
}

/*******************************************************************************
 * WIFI MANAGEMENT
 ******************************************************************************/
void startAP() {
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(AP_IP, AP_IP, IPAddress(255, 255, 255, 0));
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  g_apMode = true;
  addLog("AP mode started: WaterHeaterMonitor");
  Serial.printf("AP started: %s @ %s\n", AP_SSID, WiFi.softAPIP().toString().c_str());
}

void connectWiFi() {
  if (strlen(g_cfg.wifiSSID) == 0) {
    startAP();
    return;
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(g_cfg.wifiSSID, g_cfg.wifiPassword);

  Serial.printf("Connecting to %s", g_cfg.wifiSSID);
  unsigned long startAttempt = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 15000) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    g_wifiConnected = true;
    g_apMode = false;

    // Sync NTP
    configTime(GMT_OFFSET, DST_OFFSET, NTP_SERVER);

    // ArduinoOTA for command-line firmware updates
    ArduinoOTA.setHostname("waterheater");
    ArduinoOTA.begin();

    char msg[80];
    snprintf(msg, sizeof(msg), "WiFi connected: %s IP: %s", g_cfg.wifiSSID, WiFi.localIP().toString().c_str());
    addLog(msg);
    Serial.println(msg);
  } else {
    addLog("WiFi connection failed, starting AP");
    startAP();
  }
}

/*******************************************************************************
 * FLASH DETECTION STATE MACHINE
 ******************************************************************************/
const FlashCodeInfo* getFlashCodeInfo(int code) {
  for (int i = 0; i < FLASH_CODE_COUNT; i++) {
    if (FLASH_CODES[i].code == code) return &FLASH_CODES[i];
  }
  return nullptr;
}

void processFlashDetection() {
  if (!g_tslFound) return;

  unsigned long now = millis();
  if (now - g_lastLightSampleTime < LIGHT_SAMPLE_INTERVAL_MS) return;
  g_lastLightSampleTime = now;

  // Read raw sensor data (fast path — no lux calculation)
  uint32_t lum = g_tsl.getFullLuminosity();
  uint16_t ir = lum >> 16;
  uint16_t full = lum & 0xFFFF;

  // Use full spectrum for flash detection — try IR channel if it gives better contrast
  uint16_t reading = full;

  // Debug: print raw values every 500ms
  static unsigned long lastDebugPrint = 0;
  if (now - lastDebugPrint >= 500) {
    uint16_t threshold = g_cfg.lightThreshold > 0 ? g_cfg.lightThreshold : (g_lightBaseline * 2 + 50);
    Serial.printf("LIGHT: full=%u ir=%u baseline=%u threshold=%u %s\n",
                  full, ir, g_lightBaseline, threshold, reading > threshold ? "<<FLASH>>" : "");
    lastDebugPrint = now;
  }

  // Update baseline with exponential moving average (slow decay)
  if (g_lightBaseline == 0) {
    g_lightBaseline = reading;
  } else if (reading < g_lightBaseline + 10) {
    // Only update baseline when light is not flashing
    g_lightBaseline = (g_lightBaseline * 15 + reading) / 16;
  }

  // Determine if light is "high" (flash detected)
  uint16_t threshold = g_cfg.lightThreshold > 0 ? g_cfg.lightThreshold : (g_lightBaseline * 2 + 50);
  bool lightHigh = reading > threshold;

  // Track peak for debug
  if (reading > g_lightPeak) g_lightPeak = reading;

  // Rising edge detection
  bool risingEdge = lightHigh && !g_lastLightHigh;
  g_lastLightHigh = lightHigh;

  switch (g_flashState) {
    case FLASH_WAITING:
      if (risingEdge) {
        g_flashCount = 1;
        g_lastFlashTime = now;
        g_flashState = FLASH_COUNTING;
      } else if (now - g_lastFlashTime > PILOT_OUT_TIMEOUT_MS && g_lastFlashTime > 0) {
        // No flash for >10 seconds
        g_flashCount = 0;
        g_flashState = FLASH_PAUSE_DETECTED;
      } else if (g_lastFlashTime == 0 && now - g_bootTime > PILOT_OUT_TIMEOUT_MS) {
        // Never seen a flash since boot
        g_flashCount = 0;
        g_flashState = FLASH_PAUSE_DETECTED;
      }
      break;

    case FLASH_COUNTING:
      if (risingEdge && (now - g_lastFlashTime > FLASH_DEBOUNCE_MS)) {
        g_flashCount++;
        g_lastFlashTime = now;
      } else if (now - g_lastFlashTime > FLASH_GAP_TIMEOUT_MS) {
        // Gap detected — pattern complete
        g_flashState = FLASH_PAUSE_DETECTED;
      }
      break;

    case FLASH_PAUSE_DETECTED: {
      // Use flash count from this cycle (g_flashCount is set by COUNTING state)
      // For pilot-out detection, g_flashCount stays at 0 since we never entered COUNTING
      int count = g_flashCount;
      Serial.printf("PAUSE: count=%d pending=%d confirmed=%d\n", count, g_pendingFlashCount, g_confirmedFlashCode);

      if (g_pendingFlashCount == count) {
        // Same count two cycles in a row — confirmed
        int prevCode = g_confirmedFlashCode;
        g_confirmedFlashCode = count;
        g_flashState = FLASH_WAITING;
        g_lastFlashTime = millis();

        if (prevCode != g_confirmedFlashCode) {
          const FlashCodeInfo* info = getFlashCodeInfo(g_confirmedFlashCode);
          char msg[80];
          snprintf(msg, sizeof(msg), "Flash code changed: %d (%s)", g_confirmedFlashCode,
                   info ? info->text : "Unknown");
          addLog(msg);
          g_haStatusChanged = true;
        }
      } else {
        g_pendingFlashCount = count;
        g_flashState = FLASH_WAITING;
        g_lastFlashTime = millis();
      }
      break;
    }

    case FLASH_CONFIRMED:
      g_flashState = FLASH_WAITING;
      g_lastFlashTime = millis();
      break;
  }
}

/*******************************************************************************
 * TEMPERATURE READING & CYCLE DETECTION
 ******************************************************************************/
void processTemperature() {
  if (!g_ds18b20Found) return;

  unsigned long now = millis();
  uint32_t interval = (uint32_t)g_cfg.tempReadInterval * 1000;

  // Start conversion
  if (!g_tempConversionPending && (now - g_lastTempRead >= interval)) {
    g_ds18b20.requestTemperatures();
    g_tempConversionPending = true;
    g_tempConversionStart = now;
    return;
  }

  // Wait for conversion (~750ms for 12-bit)
  if (g_tempConversionPending && (now - g_tempConversionStart >= 800)) {
    g_tempConversionPending = false;
    g_lastTempRead = now;

    float tempC = g_ds18b20.getTempC(g_ds18b20Addr);
    if (tempC == DEVICE_DISCONNECTED_C) {
      addLog("DS18B20 read error — disconnected?");
      return;
    }

    g_currentTempC = tempC;
    g_currentTempF = tempC * 9.0 / 5.0 + 32.0;

    // Store in slope-detection buffer
    g_tempHistory2Min[g_tempHistIdx] = g_currentTempF;
    g_tempHistIdx = (g_tempHistIdx + 1) % 60;
    if (g_tempHistCount < 60) g_tempHistCount++;

    // Store in graph history (every 2 minutes)
    if (now - g_lastTempGraphSample >= 120000 || g_lastTempGraphSample == 0) {
      g_tempGraphData[g_tempGraphHead] = g_currentTempF;
      g_tempGraphHead = (g_tempGraphHead + 1) % TEMP_HISTORY_SIZE;
      if (g_tempGraphCount < TEMP_HISTORY_SIZE) g_tempGraphCount++;
      g_lastTempGraphSample = now;
    }

    // Cycle detection
    detectHeatingCycle();
  }
}

void detectHeatingCycle() {
  if (g_tempHistCount < 30) return;  // Need at least 5 minutes of data

  // Calculate temperature rise over last 5 minutes (30 readings at 10s)
  int oldest = (g_tempHistIdx - 30 + 60) % 60;
  float tempRise = g_currentTempF - g_tempHistory2Min[oldest];

  if (!g_burnerActive) {
    // Detect burner ON: temperature rising >2F in 5 minutes
    if (tempRise > CYCLE_TEMP_RISE_THRESHOLD) {
      g_burnerActive = true;
      g_cycleStartTime = millis();
      g_cycleStartTemp = g_currentTempF;
      g_cyclePeakTemp = g_currentTempF;
      addLog("Heating cycle started");
      g_haStatusChanged = true;
    }
  } else {
    // Track peak
    if (g_currentTempF > g_cyclePeakTemp) {
      g_cyclePeakTemp = g_currentTempF;
    }

    // Detect burner OFF: temperature stops rising or falls
    if (tempRise <= 0.0) {
      g_burnerActive = false;

      // Record cycle
      HeatingCycle cycle;
      struct tm timeinfo;
      if (getLocalTime(&timeinfo, 0)) {
        cycle.startTime = mktime(&timeinfo) - (millis() - g_cycleStartTime) / 1000;
      } else {
        cycle.startTime = (millis() - g_cycleStartTime) / 1000;  // Relative
      }
      cycle.endTime = cycle.startTime + (millis() - g_cycleStartTime) / 1000;
      cycle.durationSec = (millis() - g_cycleStartTime) / 1000;
      cycle.peakTempF = g_cyclePeakTemp;
      cycle.active = false;

      // Store in circular buffer
      g_cycles[g_cycleHead] = cycle;
      g_cycleHead = (g_cycleHead + 1) % MAX_CYCLES;
      if (g_cycleCount < MAX_CYCLES) g_cycleCount++;

      // Update hourly bucket
      updateHourlyBucket(cycle.durationSec);

      char msg[64];
      snprintf(msg, sizeof(msg), "Heating cycle ended: %lus, peak %.1fF", cycle.durationSec, cycle.peakTempF);
      addLog(msg);
      g_haStatusChanged = true;
    }
  }
}

void updateHourlyBucket(uint32_t durationSec) {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo, 0)) return;

  uint8_t hour = timeinfo.tm_hour;
  if (hour != g_lastRecordedHour) {
    // New hour — advance bucket
    g_hourlyHead = (g_hourlyHead + 1) % HOURLY_BUCKETS;
    g_hourlyHeatMin[g_hourlyHead] = 0;
    g_lastRecordedHour = hour;
  }
  g_hourlyHeatMin[g_hourlyHead] += durationSec / 60.0;
}

// Helper: count cycles today and total heat time
void getCyclesToday(int& count, uint32_t& totalSec) {
  count = 0;
  totalSec = 0;

  struct tm now;
  if (!getLocalTime(&now, 0)) return;

  // Start of today
  struct tm todayStart = now;
  todayStart.tm_hour = 0;
  todayStart.tm_min = 0;
  todayStart.tm_sec = 0;
  time_t startOfDay = mktime(&todayStart);

  for (int i = 0; i < g_cycleCount; i++) {
    int idx = (g_cycleHead - 1 - i + MAX_CYCLES) % MAX_CYCLES;
    if (g_cycles[idx].startTime >= startOfDay) {
      count++;
      totalSec += g_cycles[idx].durationSec;
    }
  }
}

/*******************************************************************************
 * DISPLAY — DOUBLE BUFFERED WITH TFT_eSprite
 ******************************************************************************/

// Color helpers
uint16_t tempToColor(float tempF) {
  if (tempF >= 160.0) return TFT_RED;
  if (tempF >= 150.0) return TFT_YELLOW;
  if (tempF >= 100.0) return TFT_GREEN;
  return TFT_CYAN;
}

uint16_t statusColor() {
  const FlashCodeInfo* info = getFlashCodeInfo(g_confirmedFlashCode);
  return info ? info->color : TFT_DARKGREY;
}

void drawScreenStatus() {
  g_sprite.fillSprite(TFT_BLACK);

  // Title
  g_sprite.setTextColor(TFT_CYAN, TFT_BLACK);
  g_sprite.setTextDatum(TC_DATUM);
  g_sprite.setTextSize(1);
  g_sprite.drawString("WATER HEATER MONITOR", 160, 4, 2);

  g_sprite.drawFastHLine(10, 24, 300, TFT_DARKGREY);

  // Status
  const FlashCodeInfo* info = getFlashCodeInfo(g_confirmedFlashCode);
  const char* statusText = info ? info->text : "Detecting...";
  uint16_t sColor = info ? info->color : TFT_DARKGREY;

  g_sprite.setTextDatum(TL_DATUM);
  g_sprite.setTextColor(TFT_WHITE, TFT_BLACK);
  g_sprite.drawString("Status:", 10, 32, 2);
  g_sprite.setTextColor(sColor, TFT_BLACK);
  g_sprite.drawString(statusText, 80, 32, 2);

  // Flash code
  g_sprite.setTextColor(TFT_WHITE, TFT_BLACK);
  g_sprite.drawString("Flash Code:", 10, 52, 2);
  char codeStr[8];
  snprintf(codeStr, sizeof(codeStr), "%d", g_confirmedFlashCode >= 0 ? g_confirmedFlashCode : -1);
  g_sprite.setTextColor(sColor, TFT_BLACK);
  g_sprite.drawString(g_confirmedFlashCode >= 0 ? codeStr : "?", 110, 52, 2);

  // Animated flash dots
  if (g_confirmedFlashCode > 0) {
    int dotCount = g_confirmedFlashCode;
    int startX = 10;
    int dotY = 74;
    unsigned long cycle = (millis() / 300) % (dotCount + 3);  // Animate
    for (int i = 0; i < dotCount; i++) {
      uint16_t dotColor = (i < (int)cycle) ? sColor : TFT_DARKGREY;
      g_sprite.fillCircle(startX + i * 18, dotY, 5, dotColor);
    }
  }

  g_sprite.drawFastHLine(10, 88, 300, TFT_DARKGREY);

  // Temperature
  g_sprite.setTextColor(TFT_WHITE, TFT_BLACK);
  g_sprite.drawString("Tank Temp:", 10, 96, 2);
  if (!isnan(g_currentTempF)) {
    char tempStr[16];
    if (g_cfg.useFahrenheit) {
      snprintf(tempStr, sizeof(tempStr), "%.1f F", g_currentTempF);
    } else {
      snprintf(tempStr, sizeof(tempStr), "%.1f C", g_currentTempC);
    }
    g_sprite.setTextColor(tempToColor(g_currentTempF), TFT_BLACK);
    g_sprite.drawString(tempStr, 110, 96, 2);
  } else {
    g_sprite.setTextColor(TFT_DARKGREY, TFT_BLACK);
    g_sprite.drawString("N/A", 110, 96, 2);
  }

  // Burner status
  g_sprite.setTextColor(TFT_WHITE, TFT_BLACK);
  g_sprite.drawString("Burner:", 10, 116, 2);
  g_sprite.setTextColor(g_burnerActive ? TFT_ORANGE : TFT_GREEN, TFT_BLACK);
  g_sprite.drawString(g_burnerActive ? "ON" : "OFF", 80, 116, 2);

  // Last cycle
  g_sprite.setTextColor(TFT_WHITE, TFT_BLACK);
  g_sprite.drawString("Last Cycle:", 10, 136, 2);
  if (g_cycleCount > 0) {
    int lastIdx = (g_cycleHead - 1 + MAX_CYCLES) % MAX_CYCLES;
    unsigned long ago = (millis() - g_bootTime) / 1000;  // Simplified
    uint32_t cycleDur = g_cycles[lastIdx].durationSec;
    char agoStr[32];
    if (ago < 3600) {
      snprintf(agoStr, sizeof(agoStr), "%lum ago (%lus)", ago / 60, cycleDur);
    } else {
      snprintf(agoStr, sizeof(agoStr), "%luh %lum ago", ago / 3600, (ago % 3600) / 60);
    }
    g_sprite.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
    g_sprite.drawString(agoStr, 110, 136, 2);
  } else {
    g_sprite.setTextColor(TFT_DARKGREY, TFT_BLACK);
    g_sprite.drawString("None", 110, 136, 2);
  }

  // WiFi indicator
  g_sprite.setTextColor(g_wifiConnected ? TFT_GREEN : TFT_RED, TFT_BLACK);
  g_sprite.setTextDatum(TR_DATUM);
  g_sprite.drawString(g_wifiConnected ? "WiFi OK" : (g_apMode ? "AP Mode" : "No WiFi"), 310, 156, 1);

  g_sprite.pushSprite(0, 0);
}

void drawScreenCycles() {
  g_sprite.fillSprite(TFT_BLACK);

  // Title
  g_sprite.setTextColor(TFT_CYAN, TFT_BLACK);
  g_sprite.setTextDatum(TC_DATUM);
  g_sprite.drawString("HEATING CYCLES - TODAY", 160, 4, 2);
  g_sprite.drawFastHLine(10, 24, 300, TFT_DARKGREY);

  int cyclesToday = 0;
  uint32_t totalHeatSec = 0;
  getCyclesToday(cyclesToday, totalHeatSec);

  g_sprite.setTextDatum(TL_DATUM);
  g_sprite.setTextColor(TFT_WHITE, TFT_BLACK);

  char buf[48];
  snprintf(buf, sizeof(buf), "Cycles: %d", cyclesToday);
  g_sprite.drawString(buf, 10, 32, 2);

  uint32_t totalMin = totalHeatSec / 60;
  snprintf(buf, sizeof(buf), "Total Heat Time: %luh %lum", totalMin / 60, totalMin % 60);
  g_sprite.drawString(buf, 10, 52, 2);

  if (cyclesToday > 0) {
    float avgMin = (float)totalHeatSec / cyclesToday / 60.0;
    snprintf(buf, sizeof(buf), "Avg Cycle: %.1f min", avgMin);
    g_sprite.drawString(buf, 10, 72, 2);
  }

  // 24-hour bar chart
  g_sprite.drawFastHLine(10, 100, 300, TFT_DARKGREY);
  g_sprite.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  g_sprite.setTextDatum(TC_DATUM);

  int barWidth = 12;
  int barGap = 1;
  int chartX = 10;
  int chartBottom = 148;
  int chartHeight = 40;

  // Find max for scaling
  float maxMin = 1.0;
  for (int i = 0; i < 24; i++) {
    int idx = (g_hourlyHead - i + HOURLY_BUCKETS) % HOURLY_BUCKETS;
    if (g_hourlyHeatMin[idx] > maxMin) maxMin = g_hourlyHeatMin[idx];
  }

  // Draw bars (most recent 24 hours, right to left)
  for (int i = 0; i < 24; i++) {
    int idx = (g_hourlyHead - (23 - i) + HOURLY_BUCKETS) % HOURLY_BUCKETS;
    float val = g_hourlyHeatMin[idx];
    int barH = (int)(val / maxMin * chartHeight);
    if (barH < 1 && val > 0) barH = 1;

    int x = chartX + i * (barWidth + barGap);

    // Color by intensity
    uint16_t barColor;
    if (val > maxMin * 0.7) barColor = TFT_RED;
    else if (val > maxMin * 0.4) barColor = TFT_ORANGE;
    else if (val > 0) barColor = TFT_GREEN;
    else barColor = TFT_DARKGREY;

    g_sprite.fillRect(x, chartBottom - barH, barWidth, barH, barColor);
  }

  // Time labels
  g_sprite.setTextSize(1);
  g_sprite.drawString("24h ago", chartX, chartBottom + 4, 1);
  g_sprite.drawString("Now", chartX + 23 * (barWidth + barGap), chartBottom + 4, 1);

  g_sprite.pushSprite(0, 0);
}

void drawScreenTempGraph() {
  g_sprite.fillSprite(TFT_BLACK);

  // Title
  g_sprite.setTextColor(TFT_CYAN, TFT_BLACK);
  g_sprite.setTextDatum(TC_DATUM);
  g_sprite.drawString("TEMPERATURE - LAST 12 HRS", 160, 4, 2);
  g_sprite.drawFastHLine(10, 24, 300, TFT_DARKGREY);

  if (g_tempGraphCount < 2) {
    g_sprite.setTextColor(TFT_DARKGREY, TFT_BLACK);
    g_sprite.drawString("Collecting data...", 160, 80, 2);
    g_sprite.pushSprite(0, 0);
    return;
  }

  // Graph area
  int gx = 40, gy = 30, gw = 270, gh = 110;

  // Find min/max temp
  float minT = 999, maxT = -999;
  int points = min(g_tempGraphCount, TEMP_HISTORY_SIZE);
  for (int i = 0; i < points; i++) {
    int idx = (g_tempGraphHead - points + i + TEMP_HISTORY_SIZE) % TEMP_HISTORY_SIZE;
    float t = g_tempGraphData[idx];
    if (t < minT) minT = t;
    if (t > maxT) maxT = t;
  }

  // Add some padding
  float range = maxT - minT;
  if (range < 10) { range = 10; minT = (maxT + minT) / 2 - 5; maxT = minT + 10; }
  minT -= 2; maxT += 2; range = maxT - minT;

  // Y-axis labels
  g_sprite.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  g_sprite.setTextDatum(MR_DATUM);
  char lbl[8];
  snprintf(lbl, sizeof(lbl), "%.0f", maxT);
  g_sprite.drawString(lbl, gx - 4, gy, 1);
  snprintf(lbl, sizeof(lbl), "%.0f", (maxT + minT) / 2);
  g_sprite.drawString(lbl, gx - 4, gy + gh / 2, 1);
  snprintf(lbl, sizeof(lbl), "%.0f", minT);
  g_sprite.drawString(lbl, gx - 4, gy + gh, 1);

  // Draw graph border
  g_sprite.drawRect(gx, gy, gw, gh, TFT_DARKGREY);

  // Plot line
  int prevX = -1, prevY = -1;
  for (int i = 0; i < points; i++) {
    int idx = (g_tempGraphHead - points + i + TEMP_HISTORY_SIZE) % TEMP_HISTORY_SIZE;
    float t = g_tempGraphData[idx];

    int px = gx + (i * gw) / points;
    int py = gy + gh - (int)((t - minT) / range * gh);
    py = constrain(py, gy, gy + gh);

    // Color gradient
    uint16_t lineColor = tempToColor(t);

    if (prevX >= 0) {
      g_sprite.drawLine(prevX, prevY, px, py, lineColor);
    }
    prevX = px;
    prevY = py;
  }

  // Unit label
  g_sprite.setTextDatum(TR_DATUM);
  g_sprite.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  g_sprite.drawString(g_cfg.useFahrenheit ? "deg F" : "deg C", 310, gy + gh + 6, 1);

  g_sprite.pushSprite(0, 0);
}

void drawScreenHotWater() {
  g_sprite.fillSprite(TFT_BLACK);
  g_sprite.setTextDatum(MC_DATUM);

  // "Hot Water" on top half — white, large
  g_sprite.setTextColor(TFT_WHITE, TFT_BLACK);
  g_sprite.drawString("Hot Water", 160, 50, 4);

  // ON or OFF — fills bottom half, bold
  bool on = (g_confirmedFlashCode == 1);
  uint16_t stateColor = on ? TFT_GREEN : TFT_RED;
  const char* stateText = on ? "ON" : "OFF";

  g_sprite.setTextColor(stateColor, TFT_BLACK);
  g_sprite.setTextSize(3);
  g_sprite.drawString(stateText, 160, 115, 4);
  g_sprite.setTextSize(1);

  g_sprite.pushSprite(0, 0);
}

void drawErrorScreen() {
  g_sprite.fillSprite(TFT_BLACK);
  g_sprite.setTextDatum(TC_DATUM);
  g_sprite.setTextColor(TFT_RED, TFT_BLACK);
  g_sprite.drawString("SENSOR ERROR", 160, 20, 4);

  int y = 60;
  g_sprite.setTextColor(TFT_WHITE, TFT_BLACK);
  if (!g_tslFound) {
    g_sprite.drawString("TSL2591 NOT FOUND", 160, y, 2);
    g_sprite.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
    g_sprite.drawString("Check I2C @ 0x29", 160, y + 20, 2);
    y += 50;
  }
  if (!g_ds18b20Found) {
    g_sprite.setTextColor(TFT_WHITE, TFT_BLACK);
    g_sprite.drawString("DS18B20 NOT FOUND", 160, y, 2);
    g_sprite.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
    g_sprite.drawString("Check 1-Wire @ GPIO21", 160, y + 20, 2);
  }

  g_sprite.pushSprite(0, 0);
}

void updateDisplay() {
  unsigned long now = millis();

  // Default screen is Hot Water ON/OFF (screen 3)
  // Auto-rotate only when manually navigating; return to default after timeout
  if (g_manualScreenNav) {
    // Cycle through other screens on timer while in manual mode
    if (g_cfg.screenRotationSec > 0 &&
        now - g_lastScreenChange >= (uint32_t)g_cfg.screenRotationSec * 1000) {
      for (int i = 1; i <= SCREEN_COUNT; i++) {
        int next = (g_currentScreen + i) % SCREEN_COUNT;
        if (g_cfg.screensEnabled[next]) {
          g_currentScreen = next;
          break;
        }
      }
      g_lastScreenChange = now;
    }
    // Return to default screen after 30 seconds of no button press
    if (now - g_lastButtonPress > 30000) {
      g_manualScreenNav = false;
      g_currentScreen = 3;  // Hot Water ON/OFF
      g_lastScreenChange = now;
    }
  }

  // Show error screen only if the light sensor is missing (primary sensor)
  // DS18B20 absence is handled gracefully — screens show N/A for temp data
  if (!g_tslFound) {
    drawErrorScreen();
    return;
  }

  switch (g_currentScreen) {
    case 0: drawScreenStatus();    break;
    case 1: if (g_ds18b20Found) drawScreenCycles(); else drawScreenStatus(); break;
    case 2: if (g_ds18b20Found) drawScreenTempGraph(); else drawScreenStatus(); break;
    case 3: drawScreenHotWater();  break;
    default: drawScreenStatus();   break;
  }
}

/*******************************************************************************
 * BACKLIGHT CONTROL
 ******************************************************************************/
void updateBacklight() {
  uint8_t targetBright = g_cfg.brightness;

  struct tm timeinfo;
  if (getLocalTime(&timeinfo, 0)) {
    uint8_t hour = timeinfo.tm_hour;
    bool dimTime;
    if (g_cfg.dimStartHour > g_cfg.dimEndHour) {
      dimTime = (hour >= g_cfg.dimStartHour || hour < g_cfg.dimEndHour);
    } else {
      dimTime = (hour >= g_cfg.dimStartHour && hour < g_cfg.dimEndHour);
    }
    if (dimTime) {
      targetBright = g_cfg.dimBrightness;
    }
  }

  analogWrite(PIN_LCD_BL, targetBright);
}

/*******************************************************************************
 * BUTTON HANDLING
 ******************************************************************************/
void processButtons() {
  bool btn1 = (digitalRead(PIN_BUTTON_1) == LOW);
  bool btn2 = (digitalRead(PIN_BUTTON_2) == LOW);

  // Button 1: previous screen
  if (btn1 && !g_btn1Pressed) {
    g_manualScreenNav = true;
    g_lastButtonPress = millis();
    g_currentScreen = (g_currentScreen - 1 + SCREEN_COUNT) % SCREEN_COUNT;
    g_lastScreenChange = millis();
  }

  // Button 2: next screen
  if (btn2 && !g_btn2Pressed) {
    g_manualScreenNav = true;
    g_lastButtonPress = millis();
    g_currentScreen = (g_currentScreen + 1) % SCREEN_COUNT;
    g_lastScreenChange = millis();
  }

  g_btn1Pressed = btn1;
  g_btn2Pressed = btn2;
}

/*******************************************************************************
 * HOME ASSISTANT INTEGRATION
 ******************************************************************************/
void pushToHomeAssistant() {
  if (!g_cfg.haEnabled || !g_wifiConnected) return;
  if (strlen(g_cfg.haURL) == 0 || strlen(g_cfg.haToken) == 0) return;

  unsigned long now = millis();
  bool shouldPush = (now - g_lastHAPush >= HA_PUSH_INTERVAL_MS) || g_haStatusChanged;
  if (!shouldPush) return;

  g_lastHAPush = now;
  g_haStatusChanged = false;

  // Use HTTPClient for HA REST API
  // Each entity gets POSTed to /api/states/<entity_id>
  // Small delays between calls to avoid overwhelming ESP32 WiFi stack
  pushHAEntity("sensor", "status", getFlashCodeInfo(g_confirmedFlashCode) ? getFlashCodeInfo(g_confirmedFlashCode)->text : "Unknown", "");
  delay(100);

  char valBuf[16];
  snprintf(valBuf, sizeof(valBuf), "%d", g_confirmedFlashCode);
  pushHAEntity("sensor", "flash_code", valBuf, "");
  delay(100);

  // Push pilot_out binary sensor for HomeKit integration
  pushHAEntity("binary_sensor", "pilot_out",
    g_confirmedFlashCode != 1 ? "on" : "off", "problem");

  // Only push temperature-related entities if DS18B20 is connected
  if (g_ds18b20Found) {
    if (!isnan(g_currentTempF)) {
      snprintf(valBuf, sizeof(valBuf), "%.1f", g_cfg.useFahrenheit ? g_currentTempF : g_currentTempC);
      pushHAEntity("sensor", "temperature", valBuf, g_cfg.useFahrenheit ? "°F" : "°C");
    }

    pushHAEntity("binary_sensor", "burner", g_burnerActive ? "on" : "off", "");

    int ct; uint32_t ts;
    getCyclesToday(ct, ts);
    snprintf(valBuf, sizeof(valBuf), "%d", ct);
    pushHAEntity("sensor", "cycles_today", valBuf, "");

    snprintf(valBuf, sizeof(valBuf), "%lu", ts / 60);
    pushHAEntity("sensor", "daily_heat_time", valBuf, "min");
  }
}

void pushHAEntity(const char* domain, const char* name, const char* state, const char* unit) {
  // Build entity_id
  char entityId[128];
  snprintf(entityId, sizeof(entityId), "%s.%s_%s", domain, g_cfg.haEntityPrefix, name);

  // Build URL
  char url[256];
  snprintf(url, sizeof(url), "%s/api/states/%s", g_cfg.haURL, entityId);

  // Build JSON payload
  JsonDocument doc;
  doc["state"] = state;
  // For binary_sensor, unit is used as device_class
  if (strcmp(domain, "binary_sensor") == 0 && strlen(unit) > 0) {
    doc["attributes"]["device_class"] = unit;
  } else if (strlen(unit) > 0) {
    doc["attributes"]["unit_of_measurement"] = unit;
  }
  doc["attributes"]["friendly_name"] = name;

  String payload;
  serializeJson(doc, payload);

  // HTTP POST using WiFiClient (non-blocking approach would be better, but
  // this works for the MVP — each call is fast on a local network)
  WiFiClient client;
  HTTPClient http;
  http.begin(client, url);
  http.addHeader("Content-Type", "application/json");

  char authHeader[300];
  snprintf(authHeader, sizeof(authHeader), "Bearer %s", g_cfg.haToken);
  http.addHeader("Authorization", authHeader);

  int httpCode = http.POST(payload);
  if (httpCode < 200 || httpCode >= 300) {
    char msg[80];
    snprintf(msg, sizeof(msg), "HA push failed: %s HTTP %d", entityId, httpCode);
    addLog(msg);
  }
  http.end();
}

/*******************************************************************************
 * WEB SERVER — CONFIG PORTAL & API
 ******************************************************************************/

const char CONFIG_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Water Heater Monitor</title>
<style>
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body { font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif;
         background: #1a1a2e; color: #e0e0e0; padding: 20px; max-width: 600px; margin: 0 auto; }
  h1 { color: #00d4ff; margin-bottom: 8px; font-size: 1.4em; }
  h2 { color: #00d4ff; margin: 20px 0 10px; font-size: 1.1em; border-bottom: 1px solid #333; padding-bottom: 6px; }
  .status-bar { background: #16213e; padding: 12px; border-radius: 8px; margin-bottom: 16px; }
  .status-bar .ok { color: #4caf50; } .status-bar .warn { color: #ff9800; } .status-bar .err { color: #f44336; }
  label { display: block; margin: 8px 0 4px; font-size: 0.9em; color: #aaa; }
  input[type="text"], input[type="password"], input[type="number"], select {
    width: 100%%; padding: 10px; background: #16213e; border: 1px solid #333;
    border-radius: 6px; color: #e0e0e0; font-size: 1em; }
  input:focus, select:focus { border-color: #00d4ff; outline: none; }
  .row { display: flex; gap: 12px; } .row > div { flex: 1; }
  button, .btn { display: inline-block; padding: 10px 20px; border: none; border-radius: 6px;
    cursor: pointer; font-size: 1em; margin: 4px 2px; }
  .btn-primary { background: #00d4ff; color: #000; font-weight: 600; }
  .btn-secondary { background: #333; color: #e0e0e0; }
  .btn-danger { background: #f44336; color: #fff; }
  .btn:hover { opacity: 0.85; }
  .checkbox-row { display: flex; align-items: center; gap: 8px; margin: 6px 0; }
  .checkbox-row input[type="checkbox"] { width: 18px; height: 18px; }
  .info { font-size: 0.85em; color: #888; margin-top: 2px; }
  .card { background: #16213e; padding: 14px; border-radius: 8px; margin: 8px 0; }
  footer { text-align: center; margin-top: 30px; color: #555; font-size: 0.8em; }
  a { color: #00d4ff; }
</style>
</head>
<body>
<h1>Water Heater Monitor</h1>

<div class="status-bar" id="statusBar">Loading status...</div>

<form id="configForm" method="POST" action="/config">

<h2>WiFi Settings</h2>
<label>SSID</label>
<input type="text" name="wifiSSID" id="wifiSSID" value="%WIFI_SSID%">
<label>Password</label>
<input type="password" name="wifiPass" id="wifiPass" value="%WIFI_PASS%">

<h2>Light Sensor</h2>
<div class="row">
  <div>
    <label>Threshold</label>
    <input type="number" name="lightThresh" id="lightThresh" value="%LIGHT_THRESH%">
    <div class="info">0 = auto (2x baseline)</div>
  </div>
  <div>
    <label>Gain</label>
    <select name="lightGain" id="lightGain">
      <option value="1" %LG1%>1x (Low)</option>
      <option value="2" %LG2%>25x (Med)</option>
      <option value="3" %LG3%>428x (High)</option>
      <option value="4" %LG4%>9876x (Max)</option>
    </select>
  </div>
</div>
<div class="row">
  <div>
    <label>Baseline</label>
    <input type="number" name="lightBase" id="lightBase" value="%LIGHT_BASE%" readonly>
  </div>
  <div style="display:flex;align-items:flex-end;">
    <button type="button" class="btn btn-secondary" onclick="calibrate()">Auto-Calibrate</button>
  </div>
</div>

<h2>Temperature Sensor</h2>
<div class="row">
  <div>
    <label>Read Interval (sec)</label>
    <input type="number" name="tempInt" value="%TEMP_INT%" min="5" max="60">
  </div>
  <div>
    <label>Unit</label>
    <select name="useF">
      <option value="1" %UF1%>Fahrenheit</option>
      <option value="0" %UF0%>Celsius</option>
    </select>
  </div>
</div>

<h2>Alert Thresholds</h2>
<div class="row">
  <div>
    <label>High Temp Warning (F)</label>
    <input type="number" name="highTempWarn" value="%HIGH_TEMP%" step="1">
  </div>
  <div>
    <label>Critical Temp (F)</label>
    <input type="number" name="critTemp" value="%CRIT_TEMP%" step="1">
  </div>
</div>

<h2>Home Assistant</h2>
<div class="checkbox-row">
  <input type="checkbox" name="haEnabled" id="haEnabled" value="1" %HA_EN%>
  <label for="haEnabled" style="display:inline;margin:0;">Enable HA Integration</label>
</div>
<label>HA URL</label>
<input type="text" name="haURL" value="%HA_URL%" placeholder="http://homeassistant.local:8123">
<label>Long-Lived Access Token</label>
<input type="password" name="haToken" value="%HA_TOKEN%">
<label>Entity ID Prefix</label>
<input type="text" name="haPrefix" value="%HA_PREFIX%">

<h2>Display Settings</h2>
<div class="row">
  <div>
    <label>Rotation (sec)</label>
    <input type="number" name="scrRotSec" value="%SCR_ROT%" min="3" max="60">
  </div>
  <div>
    <label>Brightness</label>
    <input type="number" name="bright" value="%BRIGHT%" min="1" max="255">
  </div>
</div>
<div class="checkbox-row">
  <input type="checkbox" name="scrEn0" value="1" %SE0%><label style="display:inline;margin:0;">Status Screen</label>
</div>
<div class="checkbox-row">
  <input type="checkbox" name="scrEn1" value="1" %SE1%><label style="display:inline;margin:0;">Cycles Screen</label>
</div>
<div class="checkbox-row">
  <input type="checkbox" name="scrEn2" value="1" %SE2%><label style="display:inline;margin:0;">Temp Graph Screen</label>
</div>
<div class="checkbox-row">
  <input type="checkbox" name="scrEn3" value="1" %SE3%><label style="display:inline;margin:0;">Hot Water ON/OFF Screen</label>
</div>

<h2>Auto-Dim</h2>
<div class="row">
  <div>
    <label>Dim Brightness</label>
    <input type="number" name="dimBright" value="%DIM_BRIGHT%" min="0" max="255">
  </div>
  <div>
    <label>Start Hour</label>
    <input type="number" name="dimStart" value="%DIM_START%" min="0" max="23">
  </div>
  <div>
    <label>End Hour</label>
    <input type="number" name="dimEnd" value="%DIM_END%" min="0" max="23">
  </div>
</div>

<br>
<button type="submit" class="btn btn-primary" style="width:100%%;">Save & Reboot</button>
</form>

<div style="margin-top:16px; text-align:center;">
  <a href="/update" class="btn btn-secondary">OTA Update</a>
  <a href="/api/status" class="btn btn-secondary">API Status</a>
  <a href="/api/debug" class="btn btn-secondary">Debug</a>
</div>

<footer>Water Heater Monitor v1.0 | <span id="uptimeSpan"></span></footer>

<script>
var calibrate=function(){
  fetch('/api/calibrate',{method:'POST'})
    .then(function(r){return r.json();})
    .then(function(d){
      document.getElementById('lightBase').value=d.baseline;
      document.getElementById('lightThresh').value=d.threshold;
      alert('Calibrated! Baseline: '+d.baseline+', Threshold: '+d.threshold);
    })
    .catch(function(){alert('Calibration failed');});
};
var loadStatus=function(){
  fetch('/api/status')
    .then(function(r){return r.json();})
    .then(function(d){
      var sev=d.severity||'UNKNOWN';
      var cls=sev==='OK'?'ok':(sev==='WARNING'?'warn':'err');
      var bar=document.getElementById('statusBar');
      bar.innerHTML='<strong class="'+cls+'">'+(d.status_text||'Unknown')+'</strong>'+
        ' | Temp: '+(d.temperature_f!==null?d.temperature_f.toFixed(1)+'&deg;F':'N/A')+
        ' | Burner: '+(d.burner_active?'ON':'OFF')+
        ' | RSSI: '+(d.wifi_rssi||'N/A')+' dBm';
      var rem=d.uptime_sec-Math.floor(d.uptime_sec/3600)*3600;
      document.getElementById('uptimeSpan').textContent='Uptime: '+Math.floor(d.uptime_sec/3600)+'h '+Math.floor(rem/60)+'m';
    })
    .catch(function(){
      document.getElementById('statusBar').innerHTML='<span class="err">Cannot reach device</span>';
    });
};
loadStatus();
setInterval(loadStatus,10000);
</script>
</body>
</html>
)rawliteral";

String templateProcessor(const String& var) {
  if (var == "WIFI_SSID")    return g_cfg.wifiSSID;
  if (var == "WIFI_PASS")    return g_cfg.wifiPassword;
  if (var == "LIGHT_THRESH") return String(g_cfg.lightThreshold);
  if (var == "LIGHT_BASE")   return String(g_cfg.lightBaseline);
  if (var == "LG1") return g_cfg.lightGain == 1 ? "selected" : "";
  if (var == "LG2") return g_cfg.lightGain == 2 ? "selected" : "";
  if (var == "LG3") return g_cfg.lightGain == 3 ? "selected" : "";
  if (var == "LG4") return g_cfg.lightGain == 4 ? "selected" : "";
  if (var == "TEMP_INT")     return String(g_cfg.tempReadInterval);
  if (var == "UF1") return g_cfg.useFahrenheit ? "selected" : "";
  if (var == "UF0") return !g_cfg.useFahrenheit ? "selected" : "";
  if (var == "HIGH_TEMP")    return String(g_cfg.highTempWarning, 0);
  if (var == "CRIT_TEMP")    return String(g_cfg.criticalTemp, 0);
  if (var == "HA_EN")        return g_cfg.haEnabled ? "checked" : "";
  if (var == "HA_URL")       return g_cfg.haURL;
  if (var == "HA_TOKEN")     return g_cfg.haToken;
  if (var == "HA_PREFIX")    return g_cfg.haEntityPrefix;
  if (var == "SCR_ROT")      return String(g_cfg.screenRotationSec);
  if (var == "BRIGHT")       return String(g_cfg.brightness);
  if (var == "SE0") return g_cfg.screensEnabled[0] ? "checked" : "";
  if (var == "SE1") return g_cfg.screensEnabled[1] ? "checked" : "";
  if (var == "SE2") return g_cfg.screensEnabled[2] ? "checked" : "";
  if (var == "SE3") return g_cfg.screensEnabled[3] ? "checked" : "";
  if (var == "DIM_BRIGHT")   return String(g_cfg.dimBrightness);
  if (var == "DIM_START")    return String(g_cfg.dimStartHour);
  if (var == "DIM_END")      return String(g_cfg.dimEndHour);
  return String();
}

void setupWebServer() {
  // Config page
  g_server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send_P(200, "text/html", CONFIG_HTML, templateProcessor);
  });

  // Config form submission
  g_server.on("/config", HTTP_POST, [](AsyncWebServerRequest* request) {
    if (request->hasParam("wifiSSID", true))
      strlcpy(g_cfg.wifiSSID, request->getParam("wifiSSID", true)->value().c_str(), sizeof(g_cfg.wifiSSID));
    if (request->hasParam("wifiPass", true))
      strlcpy(g_cfg.wifiPassword, request->getParam("wifiPass", true)->value().c_str(), sizeof(g_cfg.wifiPassword));
    if (request->hasParam("lightThresh", true))
      g_cfg.lightThreshold = request->getParam("lightThresh", true)->value().toInt();
    if (request->hasParam("lightGain", true))
      g_cfg.lightGain = request->getParam("lightGain", true)->value().toInt();
    if (request->hasParam("tempInt", true))
      g_cfg.tempReadInterval = request->getParam("tempInt", true)->value().toInt();
    if (request->hasParam("useF", true))
      g_cfg.useFahrenheit = request->getParam("useF", true)->value().toInt() == 1;
    if (request->hasParam("highTempWarn", true))
      g_cfg.highTempWarning = request->getParam("highTempWarn", true)->value().toFloat();
    if (request->hasParam("critTemp", true))
      g_cfg.criticalTemp = request->getParam("critTemp", true)->value().toFloat();
    g_cfg.haEnabled = request->hasParam("haEnabled", true);
    if (request->hasParam("haURL", true))
      strlcpy(g_cfg.haURL, request->getParam("haURL", true)->value().c_str(), sizeof(g_cfg.haURL));
    if (request->hasParam("haToken", true))
      strlcpy(g_cfg.haToken, request->getParam("haToken", true)->value().c_str(), sizeof(g_cfg.haToken));
    if (request->hasParam("haPrefix", true))
      strlcpy(g_cfg.haEntityPrefix, request->getParam("haPrefix", true)->value().c_str(), sizeof(g_cfg.haEntityPrefix));
    if (request->hasParam("scrRotSec", true))
      g_cfg.screenRotationSec = request->getParam("scrRotSec", true)->value().toInt();
    g_cfg.screensEnabled[0] = request->hasParam("scrEn0", true);
    g_cfg.screensEnabled[1] = request->hasParam("scrEn1", true);
    g_cfg.screensEnabled[2] = request->hasParam("scrEn2", true);
    g_cfg.screensEnabled[3] = request->hasParam("scrEn3", true);
    if (request->hasParam("bright", true))
      g_cfg.brightness = request->getParam("bright", true)->value().toInt();
    if (request->hasParam("dimBright", true))
      g_cfg.dimBrightness = request->getParam("dimBright", true)->value().toInt();
    if (request->hasParam("dimStart", true))
      g_cfg.dimStartHour = request->getParam("dimStart", true)->value().toInt();
    if (request->hasParam("dimEnd", true))
      g_cfg.dimEndHour = request->getParam("dimEnd", true)->value().toInt();

    saveConfig();

    request->send(200, "text/html", "<html><body style='background:#1a1a2e;color:#fff;text-align:center;padding:40px;font-family:sans-serif;'>"
                  "<h2>Settings Saved!</h2><p>Rebooting in 3 seconds...</p></body></html>");

    delay(3000);
    ESP.restart();
  });

  // API: Status
  g_server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest* request) {
    JsonDocument doc;

    doc["status_code"] = g_confirmedFlashCode;
    const FlashCodeInfo* info = getFlashCodeInfo(g_confirmedFlashCode);
    doc["status_text"] = info ? info->text : "Unknown";
    doc["severity"] = info ? info->severity : "UNKNOWN";
    if (isnan(g_currentTempF)) {
      doc["temperature_f"] = nullptr;
      doc["temperature_c"] = nullptr;
    } else {
      doc["temperature_f"] = roundf(g_currentTempF * 10) / 10;
      doc["temperature_c"] = roundf(g_currentTempC * 10) / 10;
    }
    doc["burner_active"] = g_burnerActive;

    if (g_cycleCount > 0) {
      int lastIdx = (g_cycleHead - 1 + MAX_CYCLES) % MAX_CYCLES;
      JsonObject lastCycle = doc["last_cycle"].to<JsonObject>();
      lastCycle["duration_sec"] = g_cycles[lastIdx].durationSec;
      lastCycle["peak_temp_f"] = g_cycles[lastIdx].peakTempF;
    } else {
      doc["last_cycle"] = nullptr;
    }

    int ct; uint32_t ts;
    getCyclesToday(ct, ts);
    doc["cycles_today"] = ct;
    doc["total_heat_time_today_sec"] = ts;
    doc["uptime_sec"] = (millis() - g_bootTime) / 1000;
    doc["wifi_rssi"] = g_wifiConnected ? WiFi.RSSI() : 0;
    doc["free_heap"] = ESP.getFreeHeap();
    doc["light_sensor_als_raw"] = g_lastLightHigh ? g_lightPeak : 0;
    doc["light_sensor_baseline"] = g_lightBaseline;

    String json;
    serializeJson(doc, json);
    request->send(200, "application/json", json);
  });

  // API: History
  g_server.on("/api/history", HTTP_GET, [](AsyncWebServerRequest* request) {
    JsonDocument doc;

    JsonArray cycles = doc["cycles"].to<JsonArray>();
    for (int i = 0; i < g_cycleCount; i++) {
      int idx = (g_cycleHead - g_cycleCount + i + MAX_CYCLES) % MAX_CYCLES;
      JsonObject c = cycles.add<JsonObject>();
      c["duration_sec"] = g_cycles[idx].durationSec;
      c["peak_temp_f"] = g_cycles[idx].peakTempF;
      c["start"] = g_cycles[idx].startTime;
      c["end"] = g_cycles[idx].endTime;
    }

    JsonArray hourly = doc["hourly_heat_minutes"].to<JsonArray>();
    for (int i = 0; i < HOURLY_BUCKETS; i++) {
      int idx = (g_hourlyHead - HOURLY_BUCKETS + 1 + i + HOURLY_BUCKETS) % HOURLY_BUCKETS;
      hourly.add(roundf(g_hourlyHeatMin[idx] * 10) / 10);
    }

    JsonObject tempLog = doc["temperature_log"].to<JsonObject>();
    tempLog["interval_sec"] = 120;
    JsonArray readings = tempLog["readings"].to<JsonArray>();
    for (int i = 0; i < g_tempGraphCount; i++) {
      int idx = (g_tempGraphHead - g_tempGraphCount + i + TEMP_HISTORY_SIZE) % TEMP_HISTORY_SIZE;
      readings.add(roundf(g_tempGraphData[idx] * 10) / 10);
    }

    String json;
    serializeJson(doc, json);
    request->send(200, "application/json", json);
  });

  // API: Debug
  g_server.on("/api/debug", HTTP_GET, [](AsyncWebServerRequest* request) {
    JsonDocument doc;

    doc["free_heap"] = ESP.getFreeHeap();
    doc["free_psram"] = ESP.getFreePsram();
    doc["chip_model"] = ESP.getChipModel();
    doc["chip_revision"] = ESP.getChipRevision();
    doc["cpu_freq_mhz"] = ESP.getCpuFreqMHz();
    doc["flash_size"] = ESP.getFlashChipSize();
    doc["uptime_sec"] = (millis() - g_bootTime) / 1000;

    doc["wifi_ssid"] = g_wifiConnected ? WiFi.SSID() : "N/A";
    doc["wifi_rssi"] = g_wifiConnected ? WiFi.RSSI() : 0;
    doc["wifi_ip"] = g_wifiConnected ? WiFi.localIP().toString() : (g_apMode ? WiFi.softAPIP().toString() : "N/A");
    doc["wifi_mac"] = WiFi.macAddress();
    doc["ap_mode"] = g_apMode;

    doc["tsl2591_found"] = g_tslFound;
    doc["ds18b20_found"] = g_ds18b20Found;
    doc["light_baseline"] = g_lightBaseline;
    doc["light_peak"] = g_lightPeak;
    doc["light_threshold"] = g_cfg.lightThreshold;
    doc["flash_state"] = (int)g_flashState;
    doc["flash_count"] = g_flashCount;
    doc["confirmed_flash_code"] = g_confirmedFlashCode;
    doc["pending_flash_count"] = g_pendingFlashCount;

    doc["current_temp_f"] = g_currentTempF;
    doc["burner_active"] = g_burnerActive;
    doc["cycle_count"] = g_cycleCount;
    doc["temp_graph_count"] = g_tempGraphCount;

    // I2C scan
    JsonArray i2c = doc["i2c_devices"].to<JsonArray>();
    for (uint8_t addr = 1; addr < 127; addr++) {
      Wire.beginTransmission(addr);
      if (Wire.endTransmission() == 0) {
        char addrStr[8];
        snprintf(addrStr, sizeof(addrStr), "0x%02X", addr);
        i2c.add(addrStr);
      }
    }

    String json;
    serializeJson(doc, json);
    request->send(200, "application/json", json);
  });

  // API: Log
  g_server.on("/api/log", HTTP_GET, [](AsyncWebServerRequest* request) {
    JsonDocument doc;
    JsonArray entries = doc["entries"].to<JsonArray>();

    for (int i = 0; i < g_logCount; i++) {
      int idx = (g_logHead - g_logCount + i + MAX_LOG_ENTRIES) % MAX_LOG_ENTRIES;
      JsonObject entry = entries.add<JsonObject>();
      entry["timestamp"] = g_log[idx].timestamp;
      entry["message"] = g_log[idx].message;
    }

    String json;
    serializeJson(doc, json);
    request->send(200, "application/json", json);
  });

  // API: Calibrate light sensor
  g_server.on("/api/calibrate", HTTP_POST, [](AsyncWebServerRequest* request) {
    if (!g_tslFound) {
      request->send(400, "application/json", "{\"error\":\"TSL2591 not found\"}");
      return;
    }

    // Read current ambient level (average 10 samples)
    uint32_t total = 0;
    for (int i = 0; i < 10; i++) {
      uint32_t lum = g_tsl.getFullLuminosity();
      uint16_t full = lum & 0xFFFF;
      total += full;
      delay(50);
    }
    uint16_t avg = total / 10;

    g_cfg.lightBaseline = avg;
    g_cfg.lightThreshold = avg * 2 + 50;
    g_lightBaseline = avg;
    saveConfig();

    JsonDocument doc;
    doc["baseline"] = g_cfg.lightBaseline;
    doc["threshold"] = g_cfg.lightThreshold;

    String json;
    serializeJson(doc, json);

    char msg[64];
    snprintf(msg, sizeof(msg), "Calibrated: baseline=%u threshold=%u", g_cfg.lightBaseline, g_cfg.lightThreshold);
    addLog(msg);

    request->send(200, "application/json", json);
  });

  // ElegantOTA
  ElegantOTA.begin(&g_server);

  g_server.begin();
  addLog("Web server started");
}

/*******************************************************************************
 * SETUP
 ******************************************************************************/
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== Water Heater Monitor v1.0 ===");

  g_bootTime = millis();

  // Initialize buttons
  pinMode(PIN_BUTTON_1, INPUT_PULLUP);
  pinMode(PIN_BUTTON_2, INPUT_PULLUP);

  // Initialize LCD
  pinMode(PIN_LCD_POWER, OUTPUT);
  digitalWrite(PIN_LCD_POWER, HIGH);

  g_tft.init();
  g_tft.setRotation(1);  // Landscape
  g_tft.fillScreen(TFT_BLACK);

  // Create sprite for double buffering (320x170 for landscape)
  g_sprite.createSprite(320, 170);
  g_sprite.setTextWrap(false);

  // Backlight
  pinMode(PIN_LCD_BL, OUTPUT);
  analogWrite(PIN_LCD_BL, 200);

  // Show boot screen
  g_sprite.fillSprite(TFT_BLACK);
  g_sprite.setTextDatum(MC_DATUM);
  g_sprite.setTextColor(TFT_CYAN, TFT_BLACK);
  g_sprite.drawString("Water Heater Monitor", 160, 60, 4);
  g_sprite.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  g_sprite.drawString("Initializing...", 160, 100, 2);
  g_sprite.pushSprite(0, 0);

  // Load config from flash
  loadConfig();
  addLog("Config loaded from flash");

  // Initialize I2C for TSL2591
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);

  // Scan I2C bus
  Serial.println("I2C scan:");
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  Found device at 0x%02X\n", addr);
    }
  }

  // Initialize sensors
  g_tslFound = initTSL2591();
  g_ds18b20Found = initDS18B20();

  // Initialize arrays
  memset(g_tempHistory2Min, 0, sizeof(g_tempHistory2Min));
  memset(g_tempGraphData, 0, sizeof(g_tempGraphData));
  memset(g_hourlyHeatMin, 0, sizeof(g_hourlyHeatMin));
  memset(g_cycles, 0, sizeof(g_cycles));
  memset(g_log, 0, sizeof(g_log));

  // Connect WiFi
  connectWiFi();

  // Start web server
  setupWebServer();

  // Update backlight
  updateBacklight();

  g_lastScreenChange = millis();

  char bootMsg[64];
  snprintf(bootMsg, sizeof(bootMsg), "Boot complete. TSL:%s DS18B20:%s WiFi:%s",
           g_tslFound ? "OK" : "FAIL",
           g_ds18b20Found ? "OK" : "FAIL",
           g_wifiConnected ? "OK" : (g_apMode ? "AP" : "FAIL"));
  addLog(bootMsg);
  Serial.println(bootMsg);
}

/*******************************************************************************
 * MAIN LOOP
 ******************************************************************************/
void loop() {
  // Flash detection — high frequency sampling
  processFlashDetection();

  // Temperature reading
  processTemperature();

  // Button handling
  processButtons();

  // Display update (~30 fps max, throttled by sprite push)
  static unsigned long lastDisplayUpdate = 0;
  if (millis() - lastDisplayUpdate >= 200) {  // 5 fps is plenty
    updateDisplay();
    lastDisplayUpdate = millis();
  }

  // Backlight adjustment (check every 60 seconds)
  static unsigned long lastBLCheck = 0;
  if (millis() - lastBLCheck >= 60000) {
    updateBacklight();
    lastBLCheck = millis();
  }

  // Home Assistant push
  pushToHomeAssistant();

  // ElegantOTA
  ElegantOTA.loop();
  ArduinoOTA.handle();

  // WiFi reconnect check
  static unsigned long lastWiFiCheck = 0;
  if (!g_apMode && millis() - lastWiFiCheck >= 30000) {
    if (WiFi.status() != WL_CONNECTED) {
      g_wifiConnected = false;
      WiFi.reconnect();
    } else {
      g_wifiConnected = true;
    }
    lastWiFiCheck = millis();
  }
}
