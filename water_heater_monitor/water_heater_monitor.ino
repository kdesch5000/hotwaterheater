// v1.7.0 — 2026-08-20
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
#include <FFat.h>
#include <WiFiUdp.h>
#include <esp_system.h>
#include <stdarg.h>

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
#define AP_RETRY_INTERVAL_MS 120000  // Re-attempt the configured network every 2 min
#define AP_RETRY_TIMEOUT_MS  30000   // Give each attempt 30s before falling back

// NTP / timezone
// POSIX TZ string handles DST transitions automatically — no fixed offsets.
// Default is US Central: CST (UTC-6), CDT (UTC-5), DST 2nd Sun Mar → 1st Sun Nov.
#define NTP_SERVER_1    "pool.ntp.org"
#define NTP_SERVER_2    "time.nist.gov"
#define DEFAULT_TZ      "CST6CDT,M3.2.0/2,M11.1.0/2"
#define NTP_SYNC_EPOCH  1700000000  // Any time_t below this means NTP hasn't landed yet

// Persistent event log (FFat)
// The board's partition table has a 9.875MB "ffat" partition and no SPIFFS/LittleFS
// partition. Repartitioning requires a USB flash, so FFat is the only OTA-safe choice.
#define EVENT_LOG_PATH      "/events.jsonl"
#define EVENT_LOG_PREV_PATH "/events.1.jsonl"
#define EVENT_LOG_MAX_BYTES 524288  // 512KB per file, 2 files kept (~12k events)
#define EVENT_PENDING_MAX   16      // Events buffered in RAM until NTP provides a wall clock
#define EVENT_READ_BYTES    192     // Assumed bytes/event when seeking back for a tail read

// Syslog (RFC 3164 over UDP)
#define SYSLOG_FACILITY   1    // user-level messages
#define SYSLOG_APPNAME    "waterheater"
#define DEFAULT_SYSLOG_HOST "netwatch.mf"
#define DEFAULT_SYSLOG_PORT 514

// Heartbeat: a periodic "still alive" syslog line carrying current state.
// Without it, a dead device is indistinguishable from a quiet one — every
// other syslog emission here is edge-triggered, so a healthy week is silent.
// Alert on the ABSENCE of these on the collector side.
#define DEFAULT_HEARTBEAT_MIN 15

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

// Sentinel emitted in the config page in place of secret values. When POSTed back
// unchanged, the existing stored value is preserved — keeps secrets out of the rendered HTML.
#define SECRET_SENTINEL "__unchanged__"

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
  char tzString[48];         // POSIX TZ, e.g. "CST6CDT,M3.2.0/2,M11.1.0/2"
  char ntpServer[64];
  bool syslogEnabled;
  char syslogHost[64];
  uint16_t syslogPort;
  uint16_t heartbeatMin;     // Syslog heartbeat interval in minutes; 0 = off
  bool eventLogEnabled;      // Persist events to FFat
};

// Log entry (volatile RAM ring — diagnostics, served by /api/log)
struct LogEntry {
  time_t timestamp;
  char message[128];
};

/*******************************************************************************
 * EVENT LOG
 *
 * Notable, infrequent occurrences (flash code changes, temperature threshold
 * crossings, sensor failures, boots, WiFi transitions). Written to FFat as
 * JSON Lines so the record survives reboots, and mirrored to syslog so it
 * survives the ESP32 itself. Heating cycles are deliberately NOT events —
 * at 8-16/day they would bury the things you actually want to see.
 ******************************************************************************/
enum EventSeverity {
  EV_INFO,
  EV_WARNING,
  EV_ERROR,
  EV_CRITICAL
};

// Maps to syslog severity: info=6, warning=4, error=3, critical=2
const uint8_t EVENT_SYSLOG_SEVERITY[] = {6, 4, 3, 2};
const char* const EVENT_SEVERITY_NAME[] = {"INFO", "WARNING", "ERROR", "CRITICAL"};

// An event held in RAM because NTP had not yet supplied a wall clock when it
// occurred. Backfilled with a real timestamp once sync lands.
struct PendingEvent {
  unsigned long uptimeMs;
  EventSeverity severity;
  char category[16];
  char message[128];
  float tempF;
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
bool g_tempGraphBurner[TEMP_HISTORY_SIZE];
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

// Event log state
bool g_ffatMounted = false;
bool g_timeSynced = false;
uint32_t g_eventCount = 0;        // Events written to FFat this boot
PendingEvent g_pendingEvents[EVENT_PENDING_MAX];
int g_pendingEventCount = 0;
WiFiUDP g_syslogUdp;

// Threshold-crossing edge state, so we log the crossing rather than every reading
bool g_tempWarnActive = false;
bool g_tempCritActive = false;
bool g_tempReadFailed = false;
bool g_wifiWasConnected = true;

// Syslog heartbeat
unsigned long g_lastHeartbeat = 0;

// AP-mode station retry state
unsigned long g_lastApRetry = 0;
unsigned long g_apRetryStart = 0;
bool g_apRetryInProgress = false;

// Set by POST /api/events/format, acted on in loop(). The format must not run
// inside the AsyncTCP handler — it blocks for seconds with the WDT disabled.
volatile bool g_formatRequested = false;

// Queued HA event. logEvent() only sets the flag; the HTTP POST happens in
// pushToHomeAssistant(), which already runs in a context where blocking is
// expected. Keeps network latency out of the flash-detection state machine.
bool g_haEventPending = false;
EventSeverity g_haEventSeverity = EV_INFO;
char g_haEventCategory[16] = "";
char g_haEventMessage[128] = "";

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
 * EVENT LOG — PERSISTENT (FFat) + SYSLOG MIRROR
 ******************************************************************************/

// True once NTP has given us a plausible wall clock. Until then, timestamps
// would be uptime seconds, which are worthless in a permanent record.
bool timeIsSynced() {
  time_t now = time(nullptr);
  return now > NTP_SYNC_EPOCH;
}

// Mount only — NEVER format here.
//
// A format of the 9.875MB ffat partition blocks long enough to starve the idle
// task and trip the task watchdog, which panics and reboots. Doing that at boot
// produced an unrecoverable loop: reboot, format, panic, repeat, never reaching
// WiFi and therefore never reachable by OTA. Formatting is now an explicit,
// user-triggered action performed from loop() with the watchdog disabled, and
// this function is called AFTER the network is up so a filesystem problem can
// never cost us the ability to push new firmware.
void initEventLog() {
  g_ffatMounted = FFat.begin(false);
  if (!g_ffatMounted) {
    addLog("FFat not mounted — event log needs formatting (see /events)");
    Serial.println("FFat mount failed — partition likely unformatted");
    return;
  }
  Serial.printf("FFat mounted: %lu bytes total, %lu free\n",
                (unsigned long)FFat.totalBytes(), (unsigned long)FFat.freeBytes());
  // Events captured before the filesystem was available can now be written
  flushPendingEvents();
}

// Quick format (no full wipe) of the ffat partition. Called only from loop(),
// never from setup() or an async web handler, with the watchdogs disabled for
// the duration. Still takes a few seconds and blocks the display.
void performEventLogFormat() {
  addLog("Formatting FFat partition...");
  Serial.println("Formatting FFat...");

  disableLoopWDT();
  disableCore0WDT();
  bool ok = FFat.format(false);   // false = quick format, not a full wipe
  enableCore0WDT();
  enableLoopWDT();

  if (!ok) {
    addLog("FFat format FAILED");
    g_ffatMounted = false;
    return;
  }

  g_ffatMounted = FFat.begin(false);
  addLog(g_ffatMounted ? "FFat formatted and mounted" : "FFat formatted but mount failed");
  if (g_ffatMounted) flushPendingEvents();
}

// RFC 3164 syslog over UDP. Fire-and-forget: no connection, no retry, no
// blocking. Costs nothing when the collector is down, which is the point —
// unlike the HA push, this must never stall loop().
void sendSyslog(EventSeverity sev, const char* category, const char* message) {
  if (!g_cfg.syslogEnabled || !g_wifiConnected) return;
  if (strlen(g_cfg.syslogHost) == 0) return;

  uint8_t pri = SYSLOG_FACILITY * 8 + EVENT_SYSLOG_SEVERITY[sev];

  char timestamp[20] = "";
  struct tm timeinfo;
  if (getLocalTime(&timeinfo, 0)) {
    // "Mmm dd hh:mm:ss" with a space-padded day, per RFC 3164
    strftime(timestamp, sizeof(timestamp), "%b %e %H:%M:%S", &timeinfo);
  }

  char packet[448];  // Headroom for the heartbeat, which is longer than an event
  snprintf(packet, sizeof(packet), "<%u>%s %s %s: [%s] %s",
           pri, timestamp, SYSLOG_APPNAME, SYSLOG_APPNAME, category, message);

  if (g_syslogUdp.beginPacket(g_cfg.syslogHost, g_cfg.syslogPort)) {
    g_syslogUdp.write((const uint8_t*)packet, strlen(packet));
    g_syslogUdp.endPacket();
  }
}

// Periodic "still alive" line to syslog, carrying a full state snapshot.
//
// Syslog ONLY — deliberately not written to the FFat event log, which exists
// for rare notable occurrences; 96 heartbeats a day would bury them (the same
// reason heating cycles are excluded).
//
// Alerting is the collector's job: page when these STOP arriving. A device that
// has died, wedged, or is stuck in AP mode goes silent, and silence is
// otherwise indistinguishable from a healthy, uneventful week.
void sendHeartbeat() {
  if (g_cfg.heartbeatMin == 0) return;
  if (!g_cfg.syslogEnabled || !g_wifiConnected) return;

  unsigned long now = millis();
  uint32_t interval = (uint32_t)g_cfg.heartbeatMin * 60000UL;
  if (g_lastHeartbeat != 0 && now - g_lastHeartbeat < interval) return;
  g_lastHeartbeat = now;

  const FlashCodeInfo* info = getFlashCodeInfo(g_confirmedFlashCode);

  char tempStr[16];
  if (isnan(g_currentTempF)) strlcpy(tempStr, "n/a", sizeof(tempStr));
  else snprintf(tempStr, sizeof(tempStr), "%.1f", g_currentTempF);

  int cycles; uint32_t heatSec;
  getCyclesToday(cycles, heatSec);

  // key=value so it greps and parses cleanly on the collector
  char msg[288];
  snprintf(msg, sizeof(msg),
           "alive flash_code=%d status=\"%s\" temp_f=%s burner=%s cycles_today=%d "
           "heat_min_today=%lu rssi=%d uptime_s=%lu heap=%lu fs=%s",
           g_confirmedFlashCode,
           info ? info->text : "Unknown",
           tempStr,
           g_burnerActive ? "on" : "off",
           cycles,
           (unsigned long)(heatSec / 60),
           WiFi.RSSI(),
           (unsigned long)((millis() - g_bootTime) / 1000),
           (unsigned long)ESP.getFreeHeap(),
           g_ffatMounted ? "ok" : "unmounted");

  // Raise severity when the heater itself is unhappy, so a collector-side rule
  // can act on a heartbeat's contents as well as its absence.
  EventSeverity sev = (g_confirmedFlashCode == 1) ? EV_INFO
                                                  : severityForFlashCode(g_confirmedFlashCode);
  sendSyslog(sev, "heartbeat", msg);
}

// Keep the log bounded: when the active file hits the cap, it becomes the
// previous file and a fresh one starts. Two generations are retained.
void rotateEventLogIfNeeded() {
  if (!g_ffatMounted) return;

  fs::File f = FFat.open(EVENT_LOG_PATH, FILE_READ);
  if (!f) return;
  size_t sz = f.size();
  f.close();

  if (sz < EVENT_LOG_MAX_BYTES) return;

  if (FFat.exists(EVENT_LOG_PREV_PATH)) FFat.remove(EVENT_LOG_PREV_PATH);
  FFat.rename(EVENT_LOG_PATH, EVENT_LOG_PREV_PATH);
  addLog("Event log rotated");
}

// One JSON object per line: t=epoch, s=severity, c=category, m=message,
// tf=tank temp if known, u=1 when the timestamp is uptime rather than wall clock.
void writeEventLine(time_t ts, bool unsynced, EventSeverity sev,
                    const char* category, const char* message, float tempF) {
  if (!g_ffatMounted || !g_cfg.eventLogEnabled) return;

  rotateEventLogIfNeeded();

  fs::File f = FFat.open(EVENT_LOG_PATH, FILE_APPEND);
  if (!f) {
    Serial.println("Event log append failed");
    return;
  }

  JsonDocument doc;
  doc["t"] = (uint32_t)ts;
  doc["s"] = EVENT_SEVERITY_NAME[sev];
  doc["c"] = category;
  doc["m"] = message;
  if (!isnan(tempF)) doc["tf"] = roundf(tempF * 10) / 10.0;
  if (unsynced)      doc["u"] = 1;

  serializeJson(doc, f);
  f.print('\n');
  f.close();
  g_eventCount++;
}

// Once NTP lands, replay events captured beforehand with reconstructed
// timestamps derived from how long ago (in uptime) each one happened.
void flushPendingEvents() {
  // Needs both a wall clock to backfill from and somewhere to write to
  if (g_pendingEventCount == 0 || !g_ffatMounted || !g_timeSynced) return;

  time_t nowEpoch = time(nullptr);
  unsigned long nowMs = millis();

  for (int i = 0; i < g_pendingEventCount; i++) {
    PendingEvent& pe = g_pendingEvents[i];
    time_t ts = nowEpoch - (time_t)((nowMs - pe.uptimeMs) / 1000);
    writeEventLine(ts, false, pe.severity, pe.category, pe.message, pe.tempF);
  }

  char msg[64];
  snprintf(msg, sizeof(msg), "Backfilled %d pre-NTP events", g_pendingEventCount);
  addLog(msg);
  g_pendingEventCount = 0;
}

// Primary entry point. Records the event to the RAM ring (for /api/log), the
// persistent FFat log, and syslog. Safe to call before NTP sync or before FFat
// mounts — the event is buffered and backfilled.
void logEvent(EventSeverity sev, const char* category, const char* fmt, ...) {
  char message[128];
  va_list args;
  va_start(args, fmt);
  vsnprintf(message, sizeof(message), fmt, args);
  va_end(args);

  addLog(message);
  sendSyslog(sev, category, message);

  // Anything above INFO is worth waking an automation for. Queue only — the
  // POST is done from the HA push path.
  if (sev >= EV_WARNING) {
    g_haEventPending = true;
    g_haEventSeverity = sev;
    strlcpy(g_haEventCategory, category, sizeof(g_haEventCategory));
    strlcpy(g_haEventMessage, message, sizeof(g_haEventMessage));
    g_haStatusChanged = true;  // Push promptly rather than waiting for the 30s tick
  }

  float tempF = g_ds18b20Found ? g_currentTempF : NAN;

  if (timeIsSynced()) {
    if (!g_timeSynced) {
      g_timeSynced = true;
      flushPendingEvents();
    }
    if (g_ffatMounted) {
      writeEventLine(time(nullptr), false, sev, category, message, tempF);
      return;
    }
    // Synced but no filesystem yet (early boot, or awaiting format) — fall
    // through and buffer so the event survives until storage is available.
  }

  // No wall clock or no storage yet — buffer for backfill. If the buffer fills
  // first (WiFi down, say), write with uptime seconds and flag it as unsynced
  // so the record is honest rather than silently wrong.
  if (g_pendingEventCount < EVENT_PENDING_MAX) {
    PendingEvent& pe = g_pendingEvents[g_pendingEventCount++];
    pe.uptimeMs = millis();
    pe.severity = sev;
    strlcpy(pe.category, category, sizeof(pe.category));
    strlcpy(pe.message, message, sizeof(pe.message));
    pe.tempF = tempF;
  } else {
    writeEventLine((time_t)(millis() / 1000), true, sev, category, message, tempF);
  }
}

// Stream the last `budget` bytes of a JSONL event file into a JSON array,
// starting at a line boundary. Seeking rather than counting lines keeps memory
// use flat regardless of how large the log has grown.
void streamEventTail(AsyncResponseStream* res, const char* path, size_t budget, bool& first) {
  if (!FFat.exists(path)) return;

  fs::File f = FFat.open(path, FILE_READ);
  if (!f) return;

  size_t size = f.size();
  if (size > budget) {
    f.seek(size - budget);
    f.readStringUntil('\n');  // Discard the partial line we landed mid-way into
  }

  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.length() == 0 || line[0] != '{') continue;
    if (!first) res->print(',');
    res->print(line);
    first = false;
  }
  f.close();
}

// Human-readable ESP32 reset reason for the boot event.
const char* resetReasonText() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:  return "power-on";
    case ESP_RST_EXT:      return "external";
    case ESP_RST_SW:       return "software";      // Normal OTA / config reboot
    case ESP_RST_PANIC:    return "PANIC";
    case ESP_RST_INT_WDT:  return "int-watchdog";
    case ESP_RST_TASK_WDT: return "task-watchdog";
    case ESP_RST_WDT:      return "watchdog";
    case ESP_RST_DEEPSLEEP: return "deepsleep";
    case ESP_RST_BROWNOUT: return "BROWNOUT";
    case ESP_RST_SDIO:     return "sdio";
    default:               return "unknown";
  }
}

// Called from loop() so a late NTP sync still triggers the backfill even if no
// new event happens to arrive after it.
void checkTimeSync() {
  if (g_timeSynced || !timeIsSynced()) return;
  g_timeSynced = true;
  addLog("NTP time synchronized");
  flushPendingEvents();
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
  strlcpy(g_cfg.tzString,       g_prefs.getString("tz", DEFAULT_TZ).c_str(),          sizeof(g_cfg.tzString));
  strlcpy(g_cfg.ntpServer,      g_prefs.getString("ntp", NTP_SERVER_1).c_str(),       sizeof(g_cfg.ntpServer));
  g_cfg.syslogEnabled      = g_prefs.getBool("sysEn", true);
  strlcpy(g_cfg.syslogHost,     g_prefs.getString("sysHost", DEFAULT_SYSLOG_HOST).c_str(), sizeof(g_cfg.syslogHost));
  g_cfg.syslogPort         = g_prefs.getUShort("sysPort", DEFAULT_SYSLOG_PORT);
  g_cfg.heartbeatMin       = g_prefs.getUShort("hbMin", DEFAULT_HEARTBEAT_MIN);
  g_cfg.eventLogEnabled    = g_prefs.getBool("evtEn", true);

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
  g_prefs.putString("tz",       g_cfg.tzString);
  g_prefs.putString("ntp",      g_cfg.ntpServer);
  g_prefs.putBool("sysEn",      g_cfg.syslogEnabled);
  g_prefs.putString("sysHost",  g_cfg.syslogHost);
  g_prefs.putUShort("sysPort",  g_cfg.syslogPort);
  g_prefs.putUShort("hbMin",    g_cfg.heartbeatMin);
  g_prefs.putBool("evtEn",      g_cfg.eventLogEnabled);

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
  // Modem sleep is a well-known cause of repeated drop/reconnect churn on a
  // marginal signal, which is what this device sits on (~-74 dBm at the tank).
  // Costs a little power; we are USB-powered, so it is a clear win.
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.begin(g_cfg.wifiSSID, g_cfg.wifiPassword);

  Serial.printf("Connecting to %s", g_cfg.wifiSSID);
  unsigned long startAttempt = millis();
  // 30s, not 15s: signal at the water heater is around -70 dBm, and association
  // there can take longer than a bench test suggests.
  while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 30000) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    g_wifiConnected = true;
    g_apMode = false;

    // Sync NTP using a POSIX TZ string so DST transitions are handled by the
    // C library rather than a hardcoded offset that is wrong half the year.
    configTzTime(g_cfg.tzString, g_cfg.ntpServer, NTP_SERVER_2);

    // ArduinoOTA for command-line firmware updates
    ArduinoOTA.setHostname("waterheater");
    ArduinoOTA.begin();

    // Give NTP a moment to land so boot events get real timestamps rather than
    // going through the pending-backfill path.
    unsigned long ntpWait = millis();
    while (!timeIsSynced() && millis() - ntpWait < 5000) delay(100);
    if (timeIsSynced()) g_timeSynced = true;

    logEvent(EV_INFO, "wifi", "WiFi connected: %s IP: %s",
             g_cfg.wifiSSID, WiFi.localIP().toString().c_str());
  } else {
    logEvent(EV_WARNING, "wifi", "WiFi connection failed, starting AP");
    startAP();
  }
}

// While stranded in AP mode, periodically re-attempt the configured network.
// Runs AP+STA concurrently so the config portal stays reachable throughout, and
// is fully non-blocking — a 30s blocking retry inside loop() would stall flash
// detection and the display.
void retryStationFromAP() {
  if (!g_apMode || strlen(g_cfg.wifiSSID) == 0) return;

  unsigned long now = millis();

  if (!g_apRetryInProgress) {
    if (now - g_lastApRetry < AP_RETRY_INTERVAL_MS) return;
    g_lastApRetry = now;
    g_apRetryInProgress = true;
    g_apRetryStart = now;
    WiFi.mode(WIFI_AP_STA);
    WiFi.setSleep(false);
    WiFi.begin(g_cfg.wifiSSID, g_cfg.wifiPassword);
    Serial.printf("AP mode: retrying %s\n", g_cfg.wifiSSID);
    return;
  }

  if (WiFi.status() == WL_CONNECTED) {
    g_apRetryInProgress = false;
    g_apMode = false;
    g_wifiConnected = true;
    g_wifiWasConnected = true;
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_STA);

    configTzTime(g_cfg.tzString, g_cfg.ntpServer, NTP_SERVER_2);
    ArduinoOTA.setHostname("waterheater");
    ArduinoOTA.begin();

    logEvent(EV_INFO, "wifi", "Recovered from AP mode: %s IP: %s",
             g_cfg.wifiSSID, WiFi.localIP().toString().c_str());
    return;
  }

  if (now - g_apRetryStart > AP_RETRY_TIMEOUT_MS) {
    g_apRetryInProgress = false;
    WiFi.disconnect();
    WiFi.mode(WIFI_AP);
    Serial.println("AP mode: retry failed, staying in AP");
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

// Translate a flash code's severity string into an event severity, so a pilot
// outage lands in the log (and syslog) at the priority it deserves.
EventSeverity severityForFlashCode(int code) {
  const FlashCodeInfo* info = getFlashCodeInfo(code);
  if (!info) return EV_WARNING;                              // Unrecognized pattern
  if (strcmp(info->severity, "OK") == 0)       return EV_INFO;
  if (strcmp(info->severity, "WARNING") == 0)  return EV_WARNING;
  if (strcmp(info->severity, "CRITICAL") == 0) return EV_CRITICAL;
  return EV_ERROR;
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
          logEvent(severityForFlashCode(g_confirmedFlashCode), "status",
                   "Flash code %d: %s", g_confirmedFlashCode,
                   info ? info->text : "Unknown code");
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
      // Edge-triggered: a permanently unplugged probe must not write an event
      // every 10 seconds for as long as it stays unplugged.
      if (!g_tempReadFailed) {
        g_tempReadFailed = true;
        logEvent(EV_ERROR, "sensor", "DS18B20 read failed — probe disconnected?");
      }
      return;
    }
    if (g_tempReadFailed) {
      g_tempReadFailed = false;
      logEvent(EV_INFO, "sensor", "DS18B20 reading recovered");
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
      g_tempGraphBurner[g_tempGraphHead] = g_burnerActive;
      g_tempGraphHead = (g_tempGraphHead + 1) % TEMP_HISTORY_SIZE;
      if (g_tempGraphCount < TEMP_HISTORY_SIZE) g_tempGraphCount++;
      g_lastTempGraphSample = now;
    }

    // Temperature threshold crossings — logged on the edge, not per reading
    checkTempThresholds();

    // Cycle detection
    detectHeatingCycle();
  }
}

// Log only transitions across the warning/critical thresholds. Without edge
// tracking a hot tank would write an event every 10 seconds and drown the log.
void checkTempThresholds() {
  if (isnan(g_currentTempF)) return;

  // Small hysteresis band so a reading hovering on the line doesn't oscillate
  const float HYST = 2.0;

  if (!g_tempCritActive && g_currentTempF >= g_cfg.criticalTemp) {
    g_tempCritActive = true;
    logEvent(EV_CRITICAL, "temp", "Temperature CRITICAL: %.1fF (threshold %.0fF)",
             g_currentTempF, g_cfg.criticalTemp);
  } else if (g_tempCritActive && g_currentTempF < g_cfg.criticalTemp - HYST) {
    g_tempCritActive = false;
    logEvent(EV_INFO, "temp", "Temperature back below critical: %.1fF", g_currentTempF);
  }

  if (!g_tempWarnActive && g_currentTempF >= g_cfg.highTempWarning) {
    g_tempWarnActive = true;
    logEvent(EV_WARNING, "temp", "Temperature high: %.1fF (threshold %.0fF)",
             g_currentTempF, g_cfg.highTempWarning);
  } else if (g_tempWarnActive && g_currentTempF < g_cfg.highTempWarning - HYST) {
    g_tempWarnActive = false;
    logEvent(EV_INFO, "temp", "Temperature back to normal: %.1fF", g_currentTempF);
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

    // Detect burner OFF: 5-min slope has gone flat or negative
    // (temp stopped climbing = burner is no longer heating)
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
  if (g_burnerActive) {
    // Flame icon: outer orange flame + inner yellow core
    int fx = 108, fy = 114;
    g_sprite.fillTriangle(fx, fy+12, fx-5, fy+2, fx+5, fy+2, TFT_ORANGE);
    g_sprite.fillTriangle(fx, fy-2, fx-4, fy+6, fx+4, fy+6, TFT_ORANGE);
    g_sprite.fillTriangle(fx, fy+1, fx-2, fy+7, fx+2, fy+7, TFT_YELLOW);
  }

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

  // Flame icon next to "ON" when burner is firing
  if (on && g_burnerActive) {
    int fx = 245, fy = 115;
    g_sprite.fillTriangle(fx, fy+32, fx-18, fy-2,  fx+18, fy-2,  TFT_ORANGE);
    g_sprite.fillTriangle(fx, fy-32, fx-14, fy+14, fx+14, fy+14, TFT_ORANGE);
    g_sprite.fillTriangle(fx, fy-12, fx-7,  fy+18, fx+7,  fy+18, TFT_YELLOW);
    g_sprite.fillTriangle(fx, fy+22, fx-4,  fy+4,  fx+4,  fy+4,  TFT_YELLOW);
  }

  // WiFi signal indicator — lower left
  {
    const int barX = 8;
    const int barBase = 160;
    const int barWidth = 4;
    const int barGap = 2;
    const int heights[4] = {4, 8, 12, 16};
    int rssi = g_wifiConnected ? WiFi.RSSI() : 0;
    int level = 0;
    if (g_wifiConnected) {
      if      (rssi >= -55) level = 4;
      else if (rssi >= -65) level = 3;
      else if (rssi >= -75) level = 2;
      else if (rssi >= -85) level = 1;
    }
    uint16_t color = TFT_GREEN;
    if      (!g_wifiConnected) color = TFT_DARKGREY;
    else if (level <= 1)       color = TFT_RED;
    else if (level == 2)       color = TFT_YELLOW;
    for (int i = 0; i < 4; i++) {
      int x = barX + i * (barWidth + barGap);
      int y = barBase - heights[i];
      if (i < level) g_sprite.fillRect(x, y, barWidth, heights[i], color);
      else           g_sprite.drawRect(x, y, barWidth, heights[i], TFT_DARKGREY);
    }
    char rssiStr[16];
    if (g_wifiConnected) snprintf(rssiStr, sizeof(rssiStr), "%d dBm", rssi);
    else                 snprintf(rssiStr, sizeof(rssiStr), "no wifi");
    g_sprite.setTextDatum(BL_DATUM);
    g_sprite.setTextColor(color, TFT_BLACK);
    g_sprite.drawString(rssiStr, barX + 4 * (barWidth + barGap) + 4, barBase, 2);
  }

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

  if (g_haEventPending) {
    g_haEventPending = false;
    pushHAEvent();
  }
}

// Fire a Home Assistant event so automations can trigger on the occurrence
// itself rather than polling entity state. Fires for WARNING and above only.
//
// HA automation trigger:
//   trigger:
//     - platform: event
//       event_type: water_heater_alert
void pushHAEvent() {
  char url[256];
  snprintf(url, sizeof(url), "%s/api/events/water_heater_alert", g_cfg.haURL);

  JsonDocument doc;
  doc["severity"] = EVENT_SEVERITY_NAME[g_haEventSeverity];
  doc["category"] = g_haEventCategory;
  doc["message"]  = g_haEventMessage;
  doc["flash_code"] = g_confirmedFlashCode;
  if (!isnan(g_currentTempF)) doc["temperature_f"] = roundf(g_currentTempF * 10) / 10.0;

  String payload;
  serializeJson(doc, payload);

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
    snprintf(msg, sizeof(msg), "HA event push failed: HTTP %d", httpCode);
    addLog(msg);  // addLog, not logEvent — a failed push must not loop back
  }
  http.end();
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
<link rel="icon" href='data:image/svg+xml,<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100"><text y=".9em" font-size="90">🔥</text></svg>'>
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
    width: auto; padding: 10px; background: #16213e; border: 1px solid #333;
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

<div id="chartsContainer"></div>

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

<h2>Time &amp; Timezone</h2>
<label>POSIX Timezone String</label>
<input type="text" name="tz" value="%TZ%" placeholder="CST6CDT,M3.2.0/2,M11.1.0/2">
<div style="color:#8a8aa0;font-size:0.8em;margin:-8px 0 10px">Handles DST automatically. US Central: CST6CDT,M3.2.0/2,M11.1.0/2 &middot; Eastern: EST5EDT,M3.2.0/2,M11.1.0/2 &middot; Mountain: MST7MDT,M3.2.0/2,M11.1.0/2 &middot; Pacific: PST8PDT,M3.2.0/2,M11.1.0/2</div>
<label>NTP Server</label>
<input type="text" name="ntp" value="%NTP%" placeholder="pool.ntp.org">

<h2>Event Log</h2>
<div class="checkbox-row">
  <input type="checkbox" name="evtEn" id="evtEn" value="1" %EVT_EN%>
  <label for="evtEn" style="display:inline;margin:0;">Persist events to flash (FFat)</label>
</div>
<div class="checkbox-row">
  <input type="checkbox" name="sysEn" id="sysEn" value="1" %SYS_EN%>
  <label for="sysEn" style="display:inline;margin:0;">Mirror events to syslog</label>
</div>
<div class="row">
  <div>
    <label>Syslog Host</label>
    <input type="text" name="sysHost" value="%SYS_HOST%" placeholder="netwatch.mf">
  </div>
  <div>
    <label>Syslog Port (UDP)</label>
    <input type="number" name="sysPort" value="%SYS_PORT%" min="1" max="65535">
  </div>
</div>
<label>Heartbeat Interval (minutes, 0 = off)</label>
<input type="number" name="hbMin" value="%HB_MIN%" min="0" max="1440">
<div style="color:#8a8aa0;font-size:0.8em;margin:-8px 0 10px">Periodic &quot;still alive&quot; syslog line with current state. Alert on these <em>stopping</em> &mdash; every other syslog message here is edge-triggered, so a healthy device is otherwise silent. Not written to the event log.</div>
<div style="margin-bottom:10px">
  <button type="button" onclick="testSyslog()">Send Test Message</button>
  <a href="/events" style="color:#00d4ff;margin-left:10px">View Event Log &rarr;</a>
</div>
<div id="syslogResult" style="color:#8a8aa0;font-size:0.85em;margin-bottom:10px"></div>

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
<button type="submit" class="btn btn-primary" style="display:block;width:auto;">Save & Reboot</button>
</form>

<div style="margin-top:16px; text-align:center;">
  <a href="/events" class="btn btn-secondary">Event Log</a>
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
var testSyslog=function(){
  var out=document.getElementById('syslogResult');
  out.textContent='Sending...';
  fetch('/api/syslog/test',{method:'POST'})
    .then(function(r){return r.json().then(function(d){return {ok:r.ok,d:d};});})
    .then(function(res){
      if(res.ok){
        out.innerHTML='<span style="color:#6ad06a">Sent to '+res.d.host+':'+res.d.port+
          ' &mdash; check your collector. UDP is fire-and-forget, so delivery is not confirmed here.</span>';
      }else{
        out.innerHTML='<span style="color:#ff8a4d">'+(res.d.error||'Failed')+
          ' &mdash; save settings first if you just changed them.</span>';
      }
    })
    .catch(function(){out.innerHTML='<span style="color:#ff8a4d">Request failed</span>';});
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
<script src="/charts.js"></script>
</body>
</html>
)rawliteral";

const char CHARTS_JS[] PROGMEM = R"rawliteral(
(function(){
  var ct=document.getElementById('chartsContainer');
  ct.innerHTML=
    '<div class="card"><h2 style="margin-top:0">Temperature History</h2>'+
    '<canvas id="tempChart" width="560" height="220" style="width:100%;height:220px"></canvas>'+
    '<div style="color:#aaa;font-size:0.85em;margin-top:6px">'+
    '<span style="display:inline-block;width:14px;height:3px;background:#00d4ff;vertical-align:middle;margin-right:4px"></span>Burner off'+
    '<span style="display:inline-block;width:14px;height:3px;background:#ff6b00;vertical-align:middle;margin-left:14px;margin-right:4px"></span>Burner on'+
    '</div></div>';

  function fmtHour(h){
    h=((h%24)+24)%24;
    if(h===0)return '12a';
    if(h===12)return '12p';
    return h<12?h+'a':(h-12)+'p';
  }

  function drawTempChart(readings,burner,interval){
    var c=document.getElementById('tempChart');
    var ctx=c.getContext('2d');
    var W=c.width,H=c.height;
    var pad={t:10,b:30,l:45,r:10};
    var gw=W-pad.l-pad.r,gh=H-pad.t-pad.b;
    ctx.clearRect(0,0,W,H);
    if(!readings||readings.length<2){
      ctx.fillStyle='#555';ctx.font='14px sans-serif';ctx.textAlign='center';
      ctx.fillText('Collecting data...',W/2,H/2);return;
    }
    var mn=999,mx=-999;
    for(var i=0;i<readings.length;i++){if(readings[i]<mn)mn=readings[i];if(readings[i]>mx)mx=readings[i];}
    var rng=mx-mn;if(rng<5){mn-=2.5;mx+=2.5;rng=5;}mn-=1;mx+=1;rng=mx-mn;
    // Y gridlines + labels
    ctx.strokeStyle='#333';ctx.lineWidth=1;
    for(var g=0;g<=4;g++){
      var y=pad.t+gh-g/4*gh;
      ctx.beginPath();ctx.moveTo(pad.l,y);ctx.lineTo(pad.l+gw,y);ctx.stroke();
      ctx.fillStyle='#888';ctx.font='11px sans-serif';ctx.textAlign='right';
      ctx.fillText((mn+rng*g/4).toFixed(1),pad.l-4,y+4);
    }
    // X axis: time-of-day labels — tick interval scales with data span
    var nowMs=Date.now();
    var oldestMs=nowMs-(readings.length-1)*interval*1000;
    var spanMin=(nowMs-oldestMs)/60000;
    var tickMin=spanMin<10?2:spanMin<30?5:spanMin<90?15:spanMin<240?30:spanMin<480?60:120;
    var tickMs=tickMin*60000;
    // First tick at or after oldestMs, aligned to local-time tick boundary
    var first=new Date(oldestMs);
    first.setSeconds(0,0);
    var bucket=Math.floor(first.getMinutes()/tickMin)*tickMin;
    first.setMinutes(bucket,0,0);
    while(first.getTime()<oldestMs)first=new Date(first.getTime()+tickMs);
    function fmtTick(d){
      if(tickMin>=60&&d.getMinutes()===0)return fmtHour(d.getHours());
      var h=d.getHours(),m=d.getMinutes();
      var ap=h<12?'a':'p';var hh=h%12;if(hh===0)hh=12;
      return hh+':'+(m<10?'0':'')+m+ap;
    }
    ctx.fillStyle='#888';ctx.font='11px sans-serif';ctx.textAlign='center';
    var span=Math.max(nowMs-oldestMs,1);
    for(var tk=first.getTime();tk<=nowMs;tk+=tickMs){
      var d=new Date(tk);
      var frac=(tk-oldestMs)/span;
      var x=pad.l+frac*gw;
      ctx.strokeStyle='#222';ctx.beginPath();ctx.moveTo(x,pad.t);ctx.lineTo(x,pad.t+gh);ctx.stroke();
      ctx.fillStyle='#888';ctx.fillText(fmtTick(d),x,pad.t+gh+14);
    }
    // "Now" marker on the right
    ctx.fillStyle='#aaa';ctx.textAlign='right';
    ctx.fillText('now',pad.l+gw,pad.t+gh+14);
    // Plot line — color each segment by burner state
    ctx.lineWidth=2;
    var hasBurner=burner&&burner.length===readings.length;
    for(var i=1;i<readings.length;i++){
      var x1=pad.l+((i-1)/(readings.length-1))*gw;
      var y1=pad.t+gh-(readings[i-1]-mn)/rng*gh;
      var x2=pad.l+(i/(readings.length-1))*gw;
      var y2=pad.t+gh-(readings[i]-mn)/rng*gh;
      var on=hasBurner&&(burner[i]||burner[i-1]);
      ctx.strokeStyle=on?'#ff6b00':'#00d4ff';
      ctx.beginPath();ctx.moveTo(x1,y1);ctx.lineTo(x2,y2);ctx.stroke();
    }
  }

  function loadHistory(){
    fetch('/api/history')
      .then(function(r){return r.json();})
      .then(function(d){
        var t=d.temperature_log||{};
        drawTempChart(t.readings,t.burner,t.interval_sec);
      })
      .catch(function(){});
  }
  loadHistory();
  setInterval(loadHistory,60000);
})();
)rawliteral";

/*******************************************************************************
 * EVENT LOG PAGE
 *
 * Served WITHOUT templateProcessor — its JS uses % for modulo and CSS widths,
 * which the ESPAsyncWebServer template processor would strip. All data comes
 * from /api/events at runtime.
 ******************************************************************************/
const char EVENTS_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<link rel="icon" href='data:image/svg+xml,<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100"><text y=".9em" font-size="90">🔥</text></svg>'>
<title>Event Log — Water Heater Monitor</title>
<style>
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body { font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif;
         background: #1a1a2e; color: #e0e0e0; padding: 20px; max-width: 900px; margin: 0 auto; }
  h1 { color: #00d4ff; font-size: 1.4em; margin-bottom: 4px; }
  .sub { color: #8a8aa0; font-size: 0.85em; margin-bottom: 16px; }
  .bar { display: flex; flex-wrap: wrap; gap: 8px; align-items: center; margin-bottom: 14px; }
  select, button, a.btn {
    background: #24243e; color: #e0e0e0; border: 1px solid #3a3a5a;
    border-radius: 6px; padding: 7px 12px; font-size: 0.9em; cursor: pointer;
    text-decoration: none; display: inline-block; }
  button:hover, a.btn:hover { background: #2e2e50; }
  button.danger { border-color: #6a2a2a; color: #ff9090; }
  .spacer { flex: 1; }
  table { width: 100%; border-collapse: collapse; font-size: 0.88em; }
  th { text-align: left; color: #8a8aa0; font-weight: 600; padding: 8px 6px;
       border-bottom: 1px solid #3a3a5a; font-size: 0.82em; text-transform: uppercase; }
  td { padding: 8px 6px; border-bottom: 1px solid #24243e; vertical-align: top; }
  tr:hover td { background: #20203a; }
  .time { white-space: nowrap; color: #b0b0c8; font-variant-numeric: tabular-nums; }
  .sev { font-weight: 700; font-size: 0.78em; letter-spacing: 0.5px; white-space: nowrap; }
  .sev-INFO { color: #6ad06a; }
  .sev-WARNING { color: #ffd24d; }
  .sev-ERROR { color: #ff8a4d; }
  .sev-CRITICAL { color: #ff5a5a; }
  .cat { color: #7aa8d0; font-size: 0.85em; white-space: nowrap; }
  .temp { color: #8a8aa0; white-space: nowrap; font-variant-numeric: tabular-nums; }
  .unsynced { color: #ffd24d; font-size: 0.75em; }
  .empty { color: #8a8aa0; padding: 30px 0; text-align: center; }
  footer { margin-top: 20px; color: #8a8aa0; font-size: 0.8em; }
  footer a { color: #00d4ff; }
</style>
</head>
<body>
<h1>Event Log</h1>
<div class="sub" id="meta">Loading...</div>

<div id="fmtBox" style="display:none;background:#3a2a1a;border:1px solid #6a4a2a;border-radius:8px;padding:12px;margin-bottom:14px">
  <strong style="color:#ffd24d">Storage not initialized</strong>
  <div style="color:#c8c8d8;font-size:0.87em;margin:6px 0 10px">
    The flash partition is unformatted, so events are held in RAM only and lost on reboot.
    Syslog delivery is unaffected. Formatting takes a few seconds, during which the
    display freezes and the device stops responding &mdash; this is expected.
  </div>
  <button onclick="formatFs()">Format Storage</button>
</div>

<div class="bar">
  <select id="sevFilter">
    <option value="">All severities</option>
    <option value="WARNING">Warning and above</option>
    <option value="ERROR">Error and above</option>
    <option value="CRITICAL">Critical only</option>
  </select>
  <select id="catFilter"><option value="">All categories</option></select>
  <select id="limitSel">
    <option value="50">Last 50</option>
    <option value="100">Last 100</option>
    <option value="200">Last 200</option>
  </select>
  <button onclick="load()">Refresh</button>
  <span class="spacer"></span>
  <a class="btn" href="/api/events/download">Download</a>
  <button class="danger" onclick="clearLog()">Clear</button>
  <a class="btn" href="/">Settings</a>
</div>

<table>
  <thead><tr><th>Time</th><th>Severity</th><th>Category</th><th>Message</th><th>Temp</th></tr></thead>
  <tbody id="rows"></tbody>
</table>
<div class="empty" id="empty" style="display:none">No events recorded yet.</div>

<footer>Persistent event log on FFat, mirrored to syslog. <a href="/">Back to settings</a></footer>

<script>
var RANK = { INFO: 0, WARNING: 1, ERROR: 2, CRITICAL: 3 };
var cats = {};

function pad(n) { return n < 10 ? '0' + n : '' + n; }

function fmtTime(ev) {
  if (ev.u) return 'uptime +' + ev.t + 's';
  var d = new Date(ev.t * 1000);
  return d.getFullYear() + '-' + pad(d.getMonth() + 1) + '-' + pad(d.getDate()) +
         ' ' + pad(d.getHours()) + ':' + pad(d.getMinutes()) + ':' + pad(d.getSeconds());
}

function esc(s) {
  return String(s).replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;');
}

function render(events) {
  var minRank = RANK[document.getElementById('sevFilter').value];
  if (minRank === undefined) minRank = 0;
  var cat = document.getElementById('catFilter').value;

  var shown = events.filter(function (ev) {
    if ((RANK[ev.s] || 0) < minRank) return false;
    if (cat && ev.c !== cat) return false;
    return true;
  });

  var html = shown.map(function (ev) {
    var temp = (ev.tf === undefined) ? '' : ev.tf.toFixed(1) + '&deg;F';
    var flag = ev.u ? ' <span class="unsynced">(no NTP)</span>' : '';
    return '<tr>' +
      '<td class="time">' + fmtTime(ev) + flag + '</td>' +
      '<td class="sev sev-' + esc(ev.s) + '">' + esc(ev.s) + '</td>' +
      '<td class="cat">' + esc(ev.c) + '</td>' +
      '<td>' + esc(ev.m) + '</td>' +
      '<td class="temp">' + temp + '</td>' +
      '</tr>';
  }).join('');

  document.getElementById('rows').innerHTML = html;
  document.getElementById('empty').style.display = shown.length ? 'none' : 'block';
}

var lastEvents = [];

function load() {
  var limit = document.getElementById('limitSel').value;
  fetch('/api/events?limit=' + limit)
    .then(function (r) { return r.json(); })
    .then(function (d) {
      // Newest first for reading; the file is written oldest-first
      lastEvents = (d.events || []).slice().reverse();

      lastEvents.forEach(function (ev) {
        if (ev.c && !cats[ev.c]) {
          cats[ev.c] = true;
          var o = document.createElement('option');
          o.value = ev.c; o.textContent = ev.c;
          document.getElementById('catFilter').appendChild(o);
        }
      });

      var parts = [lastEvents.length + ' events shown'];
      if (!d.synced) parts.push('clock not yet synced');
      if (d.pending) parts.push(d.pending + ' buffered in RAM');
      document.getElementById('meta').textContent = parts.join(' · ');

      // Storage unavailable is actionable, not just informational
      document.getElementById('fmtBox').style.display = d.mounted ? 'none' : 'block';

      render(lastEvents);
    })
    .catch(function () {
      document.getElementById('meta').textContent = 'Failed to load events';
    });
}

function clearLog() {
  if (!confirm('Permanently delete the entire event log?')) return;
  fetch('/api/events/clear', { method: 'POST' }).then(load);
}

function formatFs() {
  if (!confirm('Format the flash storage partition? Takes a few seconds.')) return;
  document.getElementById('meta').textContent = 'Formatting — the device will be unresponsive briefly...';
  fetch('/api/events/format', { method: 'POST' })
    .then(function () { setTimeout(load, 8000); })
    .catch(function () { setTimeout(load, 8000); });
}

document.getElementById('sevFilter').onchange = function () { render(lastEvents); };
document.getElementById('catFilter').onchange = function () { render(lastEvents); };
document.getElementById('limitSel').onchange = load;

load();
setInterval(load, 30000);
</script>
</body>
</html>
)rawliteral";

String templateProcessor(const String& var) {
  if (var == "WIFI_SSID")    return g_cfg.wifiSSID;
  if (var == "WIFI_PASS")    return strlen(g_cfg.wifiPassword) ? SECRET_SENTINEL : "";
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
  if (var == "HA_TOKEN")     return strlen(g_cfg.haToken) ? SECRET_SENTINEL : "";
  if (var == "HA_PREFIX")    return g_cfg.haEntityPrefix;
  if (var == "TZ")           return g_cfg.tzString;
  if (var == "NTP")          return g_cfg.ntpServer;
  if (var == "EVT_EN")       return g_cfg.eventLogEnabled ? "checked" : "";
  if (var == "SYS_EN")       return g_cfg.syslogEnabled ? "checked" : "";
  if (var == "SYS_HOST")     return g_cfg.syslogHost;
  if (var == "SYS_PORT")     return String(g_cfg.syslogPort);
  if (var == "HB_MIN")       return String(g_cfg.heartbeatMin);
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

  // Charts JS — served WITHOUT template processor to avoid % mangling
  g_server.on("/charts.js", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send_P(200, "application/javascript", CHARTS_JS);
  });

  // Config form submission
  g_server.on("/config", HTTP_POST, [](AsyncWebServerRequest* request) {
    if (request->hasParam("wifiSSID", true))
      strlcpy(g_cfg.wifiSSID, request->getParam("wifiSSID", true)->value().c_str(), sizeof(g_cfg.wifiSSID));
    if (request->hasParam("wifiPass", true)) {
      String v = request->getParam("wifiPass", true)->value();
      if (v != SECRET_SENTINEL)
        strlcpy(g_cfg.wifiPassword, v.c_str(), sizeof(g_cfg.wifiPassword));
    }
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
    if (request->hasParam("haToken", true)) {
      String v = request->getParam("haToken", true)->value();
      if (v != SECRET_SENTINEL)
        strlcpy(g_cfg.haToken, v.c_str(), sizeof(g_cfg.haToken));
    }
    if (request->hasParam("haPrefix", true))
      strlcpy(g_cfg.haEntityPrefix, request->getParam("haPrefix", true)->value().c_str(), sizeof(g_cfg.haEntityPrefix));
    if (request->hasParam("tz", true))
      strlcpy(g_cfg.tzString, request->getParam("tz", true)->value().c_str(), sizeof(g_cfg.tzString));
    if (request->hasParam("ntp", true))
      strlcpy(g_cfg.ntpServer, request->getParam("ntp", true)->value().c_str(), sizeof(g_cfg.ntpServer));
    g_cfg.eventLogEnabled = request->hasParam("evtEn", true);
    g_cfg.syslogEnabled   = request->hasParam("sysEn", true);
    if (request->hasParam("sysHost", true))
      strlcpy(g_cfg.syslogHost, request->getParam("sysHost", true)->value().c_str(), sizeof(g_cfg.syslogHost));
    if (request->hasParam("sysPort", true))
      g_cfg.syslogPort = request->getParam("sysPort", true)->value().toInt();
    if (request->hasParam("hbMin", true))
      g_cfg.heartbeatMin = request->getParam("hbMin", true)->value().toInt();
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
    JsonArray burner = tempLog["burner"].to<JsonArray>();
    for (int i = 0; i < g_tempGraphCount; i++) {
      int idx = (g_tempGraphHead - g_tempGraphCount + i + TEMP_HISTORY_SIZE) % TEMP_HISTORY_SIZE;
      readings.add(roundf(g_tempGraphData[idx] * 10) / 10);
      burner.add(g_tempGraphBurner[idx] ? 1 : 0);
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

    // Event log / time diagnostics
    JsonObject ev = doc["event_log"].to<JsonObject>();
    ev["ffat_mounted"] = g_ffatMounted;
    ev["enabled"] = g_cfg.eventLogEnabled;
    ev["written_this_boot"] = g_eventCount;
    ev["pending_ntp_backfill"] = g_pendingEventCount;
    if (g_ffatMounted) {
      ev["ffat_total_bytes"] = FFat.totalBytes();
      ev["ffat_free_bytes"] = FFat.freeBytes();
      fs::File f = FFat.open(EVENT_LOG_PATH, FILE_READ);
      ev["log_bytes"] = f ? f.size() : 0;
      if (f) f.close();
      ev["rotated_file_exists"] = FFat.exists(EVENT_LOG_PREV_PATH);
    }

    JsonObject sl = doc["syslog"].to<JsonObject>();
    sl["enabled"] = g_cfg.syslogEnabled;
    sl["host"] = g_cfg.syslogHost;
    sl["port"] = g_cfg.syslogPort;
    sl["heartbeat_min"] = g_cfg.heartbeatMin;
    sl["heartbeat_age_sec"] = g_lastHeartbeat ? (millis() - g_lastHeartbeat) / 1000 : -1;

    JsonObject tm = doc["time"].to<JsonObject>();
    tm["tz"] = g_cfg.tzString;
    tm["ntp_server"] = g_cfg.ntpServer;
    tm["synced"] = g_timeSynced;
    tm["epoch"] = (uint32_t)time(nullptr);
    {
      struct tm ti;
      char buf[32] = "unsynced";
      if (getLocalTime(&ti, 0)) strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S %Z", &ti);
      tm["local"] = buf;
    }
    tm["reset_reason"] = resetReasonText();
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

  // Events page (separate PROGMEM, served WITHOUT templateProcessor — its JS
  // uses % for modulo and CSS widths, which the template processor would eat)
  g_server.on("/events", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send_P(200, "text/html", EVENTS_HTML);
  });

  // NOTE: /api/events/* GET routes MUST be registered before /api/events.
  // ESPAsyncWebServer matches a handler when the request URL merely starts with
  // the registered path, so /api/events would otherwise swallow its own
  // subpaths. (The POST subroutes are safe by method, but keep them together.)

  // API: Raw event log download, for archiving off the device
  g_server.on("/api/events/download", HTTP_GET, [](AsyncWebServerRequest* request) {
    if (!g_ffatMounted || !FFat.exists(EVENT_LOG_PATH)) {
      request->send(404, "application/json", "{\"error\":\"no event log\"}");
      return;
    }
    AsyncWebServerResponse* res = request->beginResponse(FFat, EVENT_LOG_PATH, "application/x-ndjson");
    res->addHeader("Content-Disposition", "attachment; filename=\"events.jsonl\"");
    request->send(res);
  });

  // API: Persistent event log (newest last). `limit` is approximate — we seek
  // back a byte window rather than counting lines, to keep RAM bounded.
  g_server.on("/api/events", HTTP_GET, [](AsyncWebServerRequest* request) {
    int limit = 50;
    if (request->hasParam("limit")) limit = request->getParam("limit")->value().toInt();
    if (limit < 1)   limit = 1;
    if (limit > 200) limit = 200;

    AsyncResponseStream* res = request->beginResponseStream("application/json");
    res->printf("{\"mounted\":%s,\"synced\":%s,\"written\":%lu,\"pending\":%d,\"events\":[",
                g_ffatMounted ? "true" : "false",
                g_timeSynced ? "true" : "false",
                (unsigned long)g_eventCount, g_pendingEventCount);

    size_t budget = (size_t)limit * EVENT_READ_BYTES;
    bool first = true;

    // Pull from the rotated file first only if the active one is short on data
    size_t activeSize = 0;
    if (g_ffatMounted) {
      fs::File a = FFat.open(EVENT_LOG_PATH, FILE_READ);
      if (a) { activeSize = a.size(); a.close(); }
      if (activeSize < budget && FFat.exists(EVENT_LOG_PREV_PATH)) {
        streamEventTail(res, EVENT_LOG_PREV_PATH, budget - activeSize, first);
      }
      streamEventTail(res, EVENT_LOG_PATH, budget, first);
    }

    res->print("]}");
    request->send(res);
  });

  // API: Clear the event log (both generations)
  g_server.on("/api/events/clear", HTTP_POST, [](AsyncWebServerRequest* request) {
    if (!g_ffatMounted) {
      request->send(500, "application/json", "{\"error\":\"FFat not mounted\"}");
      return;
    }
    if (FFat.exists(EVENT_LOG_PATH))      FFat.remove(EVENT_LOG_PATH);
    if (FFat.exists(EVENT_LOG_PREV_PATH)) FFat.remove(EVENT_LOG_PREV_PATH);
    g_eventCount = 0;
    request->send(200, "application/json", "{\"ok\":true}");
    // addLog + syslog rather than logEvent: this runs in the AsyncTCP task, and
    // writing to FFat from there while loop() may be appending is worth avoiding.
    addLog("Event log cleared via web UI");
    sendSyslog(EV_INFO, "system", "Event log cleared via web UI");
  });

  // API: Format the FFat partition. Explicit and user-triggered — the format
  // itself happens in loop(), not here.
  g_server.on("/api/events/format", HTTP_POST, [](AsyncWebServerRequest* request) {
    if (g_ffatMounted) {
      request->send(400, "application/json",
                    "{\"error\":\"already mounted — clear the log instead of reformatting\"}");
      return;
    }
    g_formatRequested = true;
    request->send(200, "application/json",
                  "{\"ok\":true,\"note\":\"formatting; the display will freeze for a few seconds\"}");
  });

  // API: Send a test syslog message, to verify the collector is receiving
  g_server.on("/api/syslog/test", HTTP_POST, [](AsyncWebServerRequest* request) {
    if (!g_cfg.syslogEnabled || strlen(g_cfg.syslogHost) == 0) {
      request->send(400, "application/json", "{\"error\":\"syslog not configured\"}");
      return;
    }
    sendSyslog(EV_INFO, "test", "Syslog test message from water heater monitor");
    // Also emit a real heartbeat, so the exact line the collector will see on
    // an ongoing basis can be verified now rather than after waiting an interval
    g_lastHeartbeat = 0;
    sendHeartbeat();
    JsonDocument doc;
    doc["sent"] = true;
    doc["host"] = g_cfg.syslogHost;
    doc["port"] = g_cfg.syslogPort;
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

  // Apply the timezone immediately so any pre-WiFi timestamp formatting is
  // correct; configTzTime() in connectWiFi() adds the NTP servers.
  // NOTE: the event log is mounted later, after WiFi and the web server are up.
  setenv("TZ", g_cfg.tzString, 1);
  tzset();

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
  memset(g_tempGraphBurner, 0, sizeof(g_tempGraphBurner));
  memset(g_hourlyHeatMin, 0, sizeof(g_hourlyHeatMin));
  memset(g_cycles, 0, sizeof(g_cycles));
  memset(g_log, 0, sizeof(g_log));

  // Connect WiFi
  connectWiFi();

  // Start web server
  setupWebServer();

  // Mount the event log only now that WiFi and OTA are live. If the filesystem
  // is in any way unhappy, the device stays reachable and reflashable.
  initEventLog();

  // Update backlight
  updateBacklight();

  g_lastScreenChange = millis();

  // Boot event carries the reset reason — an unexpected reboot (panic, brownout,
  // watchdog) is exactly the kind of thing a persistent log exists to catch.
  logEvent(EV_INFO, "boot", "Boot: reset=%s TSL:%s DS18B20:%s WiFi:%s",
           resetReasonText(),
           g_tslFound ? "OK" : "FAIL",
           g_ds18b20Found ? "OK" : "FAIL",
           g_wifiConnected ? "OK" : (g_apMode ? "AP" : "FAIL"));

  // Sensor failures are logged separately so they are filterable as their own
  // category rather than buried inside the boot line.
  if (!g_tslFound)     logEvent(EV_ERROR, "sensor", "TSL2591 light sensor not detected at 0x29");
  if (!g_ds18b20Found) logEvent(EV_ERROR, "sensor", "DS18B20 temperature probe not detected on GPIO21");
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

  // Periodic syslog heartbeat (dead-man's switch)
  sendHeartbeat();

  // ElegantOTA
  ElegantOTA.loop();
  ArduinoOTA.handle();

  // Backfill pre-NTP events once the clock lands (may be well after boot if
  // WiFi took a while to come up)
  checkTimeSync();

  // Handle a requested filesystem format, safely outside the async handler
  if (g_formatRequested) {
    g_formatRequested = false;
    performEventLogFormat();
  }

  // AP-mode fallback must not be a one-way door. Previously, once a boot-time
  // association failed the device served its own AP forever and could only be
  // recovered physically — fatal for an OTA-only device on a marginal signal.
  retryStationFromAP();

  // WiFi reconnect check
  static unsigned long lastWiFiCheck = 0;
  static uint8_t wifiFailStreak = 0;
  if (!g_apMode && millis() - lastWiFiCheck >= 30000) {
    lastWiFiCheck = millis();

    if (WiFi.status() == WL_CONNECTED) {
      wifiFailStreak = 0;
      g_wifiConnected = true;
    } else {
      // Require two consecutive failures before declaring the link down.
      // A single bad sample was previously enough to log a drop and force a
      // reconnect, which produced the observed connect/drop/connect churn and
      // could restart an association that was still in progress.
      if (wifiFailStreak < 255) wifiFailStreak++;
      if (wifiFailStreak >= 2) {
        g_wifiConnected = false;
        // Let the SDK's auto-reconnect work first; only force it if that has
        // clearly failed for a while.
        if (wifiFailStreak >= 4 && wifiFailStreak % 4 == 0) WiFi.reconnect();
      }
    }

    // Log WiFi transitions on the edge. A drop is logged to FFat only —
    // syslog cannot reach the collector with the network down — and the
    // restore that follows makes the outage window visible in the log.
    if (g_wifiWasConnected && !g_wifiConnected) {
      g_wifiWasConnected = false;
      logEvent(EV_WARNING, "wifi", "WiFi connection lost (RSSI was %d dBm)", WiFi.RSSI());
    } else if (!g_wifiWasConnected && g_wifiConnected) {
      g_wifiWasConnected = true;
      logEvent(EV_INFO, "wifi", "WiFi reconnected: IP %s (%d dBm)",
               WiFi.localIP().toString().c_str(), WiFi.RSSI());
    }
  }
}
