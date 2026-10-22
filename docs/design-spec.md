# Water Heater Monitor — CLAUDE.md

## Project Overview

**water-heater-monitor** is an ESP32-based monitoring system for a conventional gas water heater with a Resideo (Honeywell) gas control valve. It reads the valve’s status LED flash codes using a light sensor, measures tank surface temperature to detect heating cycles, and displays everything on a built-in color LCD. Data is reported to Home Assistant via WiFi.

The hardware platform is the **Lilygo T-Display S3** (same as the [info-display-hub](https://github.com/kdesch5000/info-display-hub) project), keeping the toolchain, display library, and architecture consistent across projects.

**Key design decision:** Hybrid wiring approach based on physical layout constraints. The gas valve (with status LED) is at the **bottom** of the tank; the hot water outlet pipe is at the **top** — roughly 4–5 feet apart. I2C/STEMMA QT is limited to ~50cm reliable range, so:

- **TSL2591 light sensor** → STEMMA QT plug-and-play to T-Display (both at bottom, near gas valve)
- **DS18B20 temp probe** → 1-Wire protocol via DFRobot adapter + 90cm cable run to pipe at top (1-Wire works reliably at 5+ meters)

The T-Display S3 and TSL2591 mount together at the bottom near the Resideo gas valve. The DS18B20 probe cable runs up to the hot water outlet pipe at the top of the tank.

## Architecture & Conventions

Follow the same patterns established in **info-display-hub**:

- **Single-file Arduino sketch** (`.ino`) — all logic in one file, no framework beyond Arduino
- **TFT_eSPI + TFT_eSprite double buffering** — draw to off-screen sprite, push in one call (no flicker)
- **ESPAsyncWebServer** — non-blocking web server for config portal + API endpoints
- **Preferences library** — persistent config stored in NVMe flash
- **ElegantOTA** — over-the-air firmware updates via browser
- **AP mode fallback** — first boot creates WiFi AP for initial configuration
- **Interval-based data collection** — sensor reads on fixed intervals, no blocking delays in loop()
- **Web config portal** — all settings changeable from browser, no code changes needed
- **JSON API endpoints** — `/api/status`, `/api/history`, `/api/debug`, `/api/log`

## Hardware Bill of Materials

### Required Components

|Component                                                |Purpose                                         |Approx Cost|Buy Link                                                                                      |Notes                                                                                                                                                                                                                                   |
|---------------------------------------------------------|------------------------------------------------|-----------|----------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|**Lilygo T-Display S3**                                  |Main MCU + 1.9” 170×320 IPS LCD                 |~$20       |[Amazon](https://www.amazon.com/dp/B09J112YR7)                                                |ESP32-S3, WiFi, USB-C, built-in display, 2 buttons, built-in STEMMA QT / Qwiic connector                                                                                                                                                |
|**PAMEENCOS TSL2591 Light Sensor** (with QWIIC connector)|Detect status LED flash patterns                |~$8        |[Amazon](https://www.amazon.com/PAMEENCOS-TSL2591-Breakout-Connection-Terminals/dp/B0D1JGDFL2)|600M:1 dynamic range; 188 µLux to 88,000 Lux; separate IR + full-spectrum channels; I2C with STEMMA QT / Qwiic connector for plug-and-play. I2C address: **0x29** (fixed). Also has header pins. Arduino library: **Adafruit_TSL2591**  |
|**DFRobot DS18B20 Waterproof Temp Sensor Kit**           |Measure tank surface temperature                |~$8        |[Amazon](https://www.amazon.com/DFRobot-Waterproof-DS18B20-Temperature-Sensor/dp/B07434MB77)  |Kit includes: waterproof DS18B20 probe (90cm cable), terminal sensor adapter (DFR0055) with built-in 4.7kΩ pull-up resistor, and Gravity sensor cable. 1-Wire protocol — works reliably over long cable runs (5m+). No soldering needed.|
|**STEMMA QT cable** (1×)                                 |Connect TSL2591 to T-Display                    |~$3        |[Amazon](https://www.amazon.com/dp/B09FPF5HJH)                                                |100mm JST SH 4-pin cable                                                                                                                                                                                                                |
|**Dupont jumper wires** (3× female-to-male)              |Connect DFRobot adapter to T-Display GPIO header|~$6        |[Amazon](https://www.amazon.com/dp/B01EV70C78) (120-pack)                                     |Only 3 wires needed: Signal → GPIO21, VCC → 3.3V, GND → GND. Skip if you already have Dupont wires.                                                                                                                                     |
|**USB-C cable + charger**                                |Power                                           |—          |—                                                                                             |Any 5V USB-C charger; the T-Display S3 powers everything                                                                                                                                                                                |

### Optional / Recommended

|Component                    |Purpose                     |Notes                                                                                               |
|-----------------------------|----------------------------|----------------------------------------------------------------------------------------------------|
|**3D printed enclosure**     |Protection                  |Mount near water heater; should have opening for cable runs                                         |
|**Kapton tape**              |Attach DS18B20 probe to pipe|Wrap probe against the hot water outlet pipe; cover with pipe insulation for better thermal coupling|
|**Adhesive putty or bracket**|Mount TSL2591 over viewport |Hold light sensor against the Resideo status light viewport window                                  |
|**Pipe insulation foam**     |Improve temp readings       |Cover the DS18B20 probe after mounting to isolate from ambient air                                  |

### Total estimated cost: ~$39 from Amazon (less if you already have Dupont wires)

## Wiring

Two separate connections from the T-Display S3 — one plug-and-play, one with 3 Dupont wires:

```
┌─────────────────────┐
│  LILYGO T-DISPLAY S3 │
│                       │
│  [STEMMA QT port]─────── STEMMA QT cable (100mm) ──► TSL2591 Light Sensor
│   (left side)         │                                (mount over gas valve viewport)
│                       │
│  [GPIO Header]        │
│   (right side)        │
│    3.3V  ◄──────────────── Red Dupont wire ──── DFRobot Adapter pin V (VCC)
│    GPIO21 ◄─────────────── Blue Dupont wire ─── DFRobot Adapter pin S (Signal)
│    GND   ◄──────────────── Black Dupont wire ── DFRobot Adapter pin G (GND)
│                       │                              │
└───────────────────────┘                              │
                                                       │ (90cm sensor cable)
                                          ┌────────────┘
                                          ▼
                                    DS18B20 Probe
                                    (taped to hot water outlet pipe
                                     at TOP of tank)
```

### Connection 1: TSL2591 Light Sensor (STEMMA QT — plug-and-play)

The PAMEENCOS TSL2591 board has a QWIIC / STEMMA QT connector. Plug a STEMMA QT cable from the T-Display S3’s built-in STEMMA QT port directly into the TSL2591 board. No soldering, no configuration. The I2C bus (GPIO43 SDA, GPIO44 SCL) and 3.3V power are all carried through the single 4-pin cable.

**Alternative:** The TSL2591 board also has header pins (VIN, GND, SDA, SCL, INT). If you prefer Dupont wires over the STEMMA QT cable, connect: VIN→3.3V, GND→GND, SDA→GPIO43, SCL→GPIO44.

### Connection 2: DFRobot DS18B20 Kit (3 Dupont wires to GPIO header)

The DFRobot kit (KIT0021) includes:

- **DS18B20 waterproof probe** — stainless steel, 90cm cable with 3 color-coded wires
- **Terminal sensor adapter** (DFR0055) — small PCB with screw terminals on one side (probe wires screw in) and a 3-pin Gravity connector on the other side (Signal/VCC/GND). Has a **built-in 4.7kΩ pull-up resistor** so no external resistor is needed.
- **Gravity sensor cable** — 3-pin cable with Dupont-compatible ends

Assembly:

1. Screw the DS18B20 probe wires into the terminal adapter: **Yellow → A (DATA)**, **Red → B (VCC)**, **Black → C (GND)**
1. Set the jumper on the adapter to **pull-up** position (required for DS18B20)
1. Connect the 3-pin Gravity cable from the adapter to the T-Display S3 GPIO header:

|DFRobot Adapter Pin|Wire Color|T-Display S3 Pin|Notes                      |
|-------------------|----------|----------------|---------------------------|
|**S** (Signal)     |Blue/Green|**GPIO21**      |1-Wire data bus            |
|**V** (VCC)        |Red       |**3.3V**        |Power (3.0–5.5V compatible)|
|**G** (GND)        |Black     |**GND**         |Ground                     |

### Important Note on T-Display S3 STEMMA QT Connector

The T-Display S3 default I2C is on GPIO43 (SDA) and GPIO44 (SCL), exposed on both the header pins and the STEMMA QT connector. However, some T-Display S3 revisions may have SDA/SCL swapped at the Qwiic connector vs the standard pinout. If the TSL2591 is not detected after connecting:

1. Run an I2C scanner sketch first to verify the device is found at 0x29
1. Try `Wire.begin(44, 43)` instead of `Wire.begin(43, 44)` if needed
1. As a last resort, use the TSL2591’s header pins with Dupont wires instead of the STEMMA QT connector

## Resideo Gas Valve Status Light Flash Codes

The gas control valve communicates status via a single LED that flashes in patterns. Each pattern consists of N flashes followed by a 3-second pause, then the pattern repeats.

|Flashes|Status                      |Severity|Description                                                  |
|-------|----------------------------|--------|-------------------------------------------------------------|
|0      |Control Off / Pilot Out     |ERROR   |No power to the control — pilot light is out                 |
|1      |Normal Operation            |OK      |Pilot is lit, system is functioning normally                 |
|2      |Thermopile Voltage Low      |WARNING |Thermopile generating insufficient voltage; pilot may be weak|
|4      |High Temperature Shutdown   |CRITICAL|Water exceeded safe temperature; manual reset required       |
|5      |Temperature Sensor Failure  |CRITICAL|Sensor is faulty; gas valve will not open                    |
|7      |Gas Control or Valve Failure|CRITICAL|Internal valve failure; replacement likely needed            |
|8      |Flame Sense Error           |ERROR   |Flame detection circuit malfunction                          |

### Flash Detection Algorithm

The status LED blinks at a consistent rate when active. The algorithm must:

1. **Sample the light sensor at high frequency** (every 50ms) to reliably detect individual flashes
1. **Detect rising edges** — transition from dark to light above a configurable threshold
1. **Count flashes within a window** — after detecting a flash, count subsequent flashes until a gap of >2 seconds (the 3-second pause between pattern repetitions)
1. **Require pattern confirmation** — the same flash count must be observed for at least 2 consecutive cycles before updating the displayed status (debounce)
1. **Handle ambient light** — the TSL2591 has adjustable gain and integration time; use the difference between baseline (no flash) and peak (flash) rather than absolute lux values
1. **Detect “0 flashes” (pilot out)** — if no flash is detected for >10 seconds, report status as 0 (Control Off / Pilot Out)

#### TSL2591 Configuration for Flash Detection

The TSL2591 has configurable gain and integration time. For detecting ~1 Hz LED flashes:

- **Integration time**: 100ms (`TSL2591_INTEGRATIONTIME_100MS`) — good starting point; shorter times (200ms, 300ms) available but 100ms gives good sensitivity while still catching individual flashes
- **Gain**: Start with `TSL2591_GAIN_MED` (25×); if readings are too low in a dark utility closet, switch to `TSL2591_GAIN_HIGH` (428×) or `TSL2591_GAIN_MAX` (9876×)
- Use `tsl.getFullLuminosity()` to get raw 32-bit value containing both IR and full-spectrum channels simultaneously — fastest read method, best for threshold comparison
- Extract channels: `uint16_t ir = luminosity >> 16; uint16_t full = luminosity & 0xFFFF;`
- Use `tsl.calculateLux(full, ir)` for calibrated lux if needed, but raw channel reads are better for flash detection
- The TSL2591 library: **Adafruit_TSL2591** — well-maintained, stable, Arduino Library Manager installable
- The TSL2591’s **separate IR channel** is particularly useful here — the Resideo status LED may emit IR that’s invisible to the eye but detectable by the sensor, improving flash detection reliability

### Suggested State Machine

```
States: WAITING_FOR_FLASH, COUNTING, PAUSE_DETECTED, CONFIRMED

WAITING_FOR_FLASH:
  - If light rises above threshold → record flash, transition to COUNTING
  - If no flash for >10 seconds → report status = 0 (pilot out)

COUNTING:
  - If light rises above threshold → increment flash count
  - If >2 seconds since last flash → transition to PAUSE_DETECTED

PAUSE_DETECTED:
  - Compare flash count to previous cycle
  - If match → transition to CONFIRMED, update reported status
  - If no match → store count as "pending", transition to WAITING_FOR_FLASH

CONFIRMED:
  - Update display and API with new status code
  - Transition to WAITING_FOR_FLASH for next cycle
```

## Temperature Monitoring & Heating Cycle Detection

The DS18B20 waterproof probe mounted on the hot water outlet pipe detects when the burner fires:

- **Read temperature every 10 seconds** via `DS18B20.requestTemperatures()` then `DS18B20.getTempFByIndex(0)`
- **Detect heating cycles** by monitoring temperature rise rate:
  - If temperature rises by >2°F (>1.1°C) within 2 minutes → burner is ON (heating cycle started)
  - If temperature stops rising or begins falling → burner is OFF (heating cycle ended)
- **Record each cycle**: start time, end time, duration, peak temperature
- **Store cycle history** in a circular buffer (last 48 hours / 100 cycles, whichever is less)
- **Calculate daily stats**: cycles today, total heating time today, average cycle duration

#### DS18B20 Configuration

- Libraries: **OneWire** by Jim Studt + **DallasTemperature** by Miles Burton
- Initialize with `OneWire oneWire(21);` and `DallasTemperature DS18B20(&oneWire);` (GPIO21)
- Set resolution to 12-bit (0.0625°C) with `DS18B20.setResolution(12)` — conversion time is ~750ms, fine for 10-second intervals
- The DFRobot terminal adapter (DFR0055) includes the required 4.7kΩ pull-up resistor on the data line
- Each DS18B20 has a unique 64-bit address; use `DS18B20.getAddress()` on first boot to identify the sensor

### Temperature Thresholds (configurable via web UI)

- **Normal range**: 110°F – 140°F (43°C – 60°C)
- **High warning**: >150°F (>65°C)
- **Critical alert**: >160°F (>71°C) — this should never happen with a working thermostat

### Note on Pipe-Mounted Temperature Readings

The DS18B20 stainless steel probe is mounted against the hot water outlet pipe surface, not immersed in water. Surface-mounted readings will be lower than actual water temperature and will lag behind changes. This is fine for cycle detection — we only need to detect the *change rate*, not the absolute water temperature. For best thermal coupling:

- Use Kapton tape to press the DS18B20 probe tight against bare copper pipe
- Wrap pipe insulation over the probe to isolate from ambient air
- Expect readings 10–20°F below actual water temperature
- The 90cm cable from the DFRobot kit is long enough to reach from the pipe at the top of a standard tank down to the adapter board near the gas valve at the bottom

## Display Screens

The T-Display S3 has a 170×320 pixel IPS LCD. Implement 3 main screens that rotate automatically (configurable interval, default 10 seconds) or are manually navigated with the two hardware buttons.

### Screen 1: Status Overview (Primary)

```
┌──────────────────────────────┐
│  🔥 WATER HEATER MONITOR    │
│                              │
│  Status: NORMAL              │
│  Flash Code: 1               │
│  ● ● ● (animated dots)      │
│                              │
│  Tank Temp: 127°F            │
│  Burner: OFF                 │
│  Last Cycle: 2h 15m ago      │
└──────────────────────────────┘
```

- Status text color-coded: GREEN for normal, YELLOW for warning, RED for error/critical
- Animated flash indicator that mimics the actual LED pattern in real-time
- Temperature color-coded based on thresholds

### Screen 2: Cycle History

```
┌──────────────────────────────┐
│  HEATING CYCLES — TODAY      │
│                              │
│  Cycles: 8                   │
│  Total Heat Time: 1h 42m     │
│  Avg Cycle: 12.7 min         │
│                              │
│  ▁▃▅▇▅▃▁▃▅▇▅▃▁▃  (24h bar) │
│  12a  6a  12p  6p  Now      │
└──────────────────────────────┘
```

- Bar chart showing heating activity over the last 24 hours
- Each bar represents a 1-hour bucket showing minutes of heating in that hour
- Bars color-coded by intensity (more heating = warmer color)

### Screen 3: Temperature Graph

```
┌──────────────────────────────┐
│  TEMPERATURE — LAST 12 HRS  │
│  140°F ┬                     │
│        │    ╱╲    ╱╲         │
│  120°F ┤───╱──╲──╱──╲────   │
│        │  ╱    ╲╱    ╲      │
│  100°F ┴───────────────────  │
│  6 PM   9 PM  12 AM  3 AM   │
└──────────────────────────────┘
```

- Line graph of temperature readings over the last 12 hours
- Sampled at 10-second intervals, downsampled to ~1 reading per 2 minutes for the graph (360 points)
- Color gradient on the line from blue (cool) to red (hot)

## Web Configuration Portal

Serve an HTML config page at `http://<device-ip>/` with these settings:

### WiFi Settings

- SSID
- Password

### Sensor Settings

- Light sensor threshold (auto-calibrate button that reads current ambient and sets threshold at 2× baseline)
- Light sensor gain (auto / 1× / 2×) and integration time (25ms / 50ms / 100ms)
- Temperature read interval (seconds, default 10)
- Temperature unit toggle (°F / °C)

### Alert Settings

- High temperature warning threshold (default 150°F)
- Critical temperature threshold (default 160°F)
- Enable/disable Home Assistant integration

### Home Assistant Integration

- HA URL (e.g., `http://homeassistant.local:8123`)
- Long-Lived Access Token
- Entity ID prefix (default: `water_heater_monitor`)

### Display Settings

- Screen rotation interval (seconds, default 10)
- Enable/disable individual screens
- Brightness (0-255, PWM backlight control)
- Auto-dim schedule (dim between configurable hours)

## API Endpoints

### `GET /api/status`

Returns current system state as JSON:

```json
{
  "status_code": 1,
  "status_text": "Normal Operation",
  "severity": "OK",
  "temperature_f": 127.4,
  "temperature_c": 53.0,
  "burner_active": false,
  "current_cycle": null,
  "last_cycle": {
    "start": "2026-04-06T14:22:00Z",
    "end": "2026-04-06T14:35:00Z",
    "duration_sec": 780,
    "peak_temp_f": 138.2
  },
  "cycles_today": 8,
  "total_heat_time_today_sec": 6120,
  "uptime_sec": 86400,
  "wifi_rssi": -42,
  "free_heap": 180000,
  "light_sensor_lux": 0.42,
  "light_sensor_als_raw": 342,
  "light_sensor_baseline": 28
}
```

### `GET /api/history`

Returns cycle history (last 48 hours):

```json
{
  "cycles": [
    {
      "start": "2026-04-06T14:22:00Z",
      "end": "2026-04-06T14:35:00Z",
      "duration_sec": 780,
      "peak_temp_f": 138.2
    }
  ],
  "hourly_heat_minutes": [0, 0, 12, 8, 0, 15, 10, ...],
  "temperature_log": {
    "interval_sec": 120,
    "readings": [127.4, 127.2, 128.1, ...]
  }
}
```

### `GET /api/debug`

System diagnostics (memory, sensor raw values, WiFi details, config dump, I2C scan results).

### `GET /api/log`

Last 50 log entries (timestamped events: status changes, cycle starts/ends, errors, reboots).

## Home Assistant Integration

Push sensor data to HA using the REST API. Create these entities:

|Entity ID                            |Type         |Value                                 |
|-------------------------------------|-------------|--------------------------------------|
|`sensor.water_heater_status`         |sensor       |Status text (e.g., “Normal Operation”)|
|`sensor.water_heater_flash_code`     |sensor       |Flash count (0, 1, 2, 4, 5, 7, 8)     |
|`sensor.water_heater_temperature`    |sensor       |Current temperature in configured unit|
|`binary_sensor.water_heater_burner`  |binary_sensor|ON when burner is firing              |
|`sensor.water_heater_cycles_today`   |sensor       |Count of cycles today                 |
|`sensor.water_heater_daily_heat_time`|sensor       |Total heating minutes today           |

Update interval: every 30 seconds or on status change (whichever is sooner).

Use HA REST API `POST /api/states/<entity_id>` with the Long-Lived Access Token.

## Build & Flash

### Arduino IDE Setup

1. **Board**: LilyGo T-Display-S3 (or ESP32S3 Dev Module)
1. **USB CDC On Boot**: Enabled
1. **Flash Size**: 16MB (128Mb)
1. **Partition Scheme**: Default 4MB with spiffs
1. **Port**: Select USB serial port

### Required Libraries

Install via Library Manager:

- **TFT_eSPI** by Bodmer
- **Adafruit TSL2591** by Adafruit (light sensor — I2C via STEMMA QT)
- **OneWire** by Jim Studt (DS18B20 1-Wire protocol)
- **DallasTemperature** by Miles Burton (DS18B20 temperature readings)
- **ArduinoJson** by Benoit Blanchon
- **ElegantOTA** by Ayush Sharma

Install from GitHub (ZIP):

- **AsyncTCP**: https://github.com/me-no-dev/AsyncTCP
- **ESPAsyncWebServer**: https://github.com/me-no-dev/ESPAsyncWebServer

### TFT_eSPI Configuration

Edit `Arduino/libraries/TFT_eSPI/User_Setup_Select.h`:

- Comment out: `#include <User_Setup.h>`
- Uncomment: `#include <User_Setups/Setup206_LilyGo_T_Display_S3.h>`

### First Flash

Upload via USB-C. All subsequent updates go over WiFi via ElegantOTA at `http://<device-ip>/update`.

## First Boot Sequence

1. Board creates AP: **SSID** = `WaterHeaterMonitor`, **Password** = `configure`
1. Connect from phone, browse to `http://192.168.4.1`
1. Enter WiFi credentials, set sensor thresholds
1. Click **Auto-Calibrate Light Sensor** (reads ambient and sets baseline)
1. Save & Reboot → device connects to WiFi and begins monitoring

### Sensor Detection on Boot

On startup, the sketch should:

1. Initialize I2C with `Wire.begin(43, 44)` (default T-Display S3 STEMMA QT pins)
1. Scan for TSL2591 at 0x29 — log success/failure
1. Initialize 1-Wire bus on GPIO21 and scan for DS18B20 devices — log address and success/failure
1. If either sensor is missing, display an error screen with the missing device
1. Continue operation with available sensors (degrade gracefully — e.g., temp monitoring still works even if light sensor fails)

## Physical Installation Notes

- **T-Display S3 placement**: Mount at the **bottom** of the water heater, near the gas valve, where the display is visible. Keep the USB-C port accessible for power. WiFi range from the water heater location should be verified.
- **TSL2591 placement**: Position the light sensor directly over or against the status light viewport window on the Resideo gas valve. The sensor needs clear line-of-sight to the LED. Use adhesive putty or a 3D-printed bracket. Block ambient light with a shroud if possible. Connected via short STEMMA QT cable (~100mm) to the T-Display right next to it.
- **DFRobot adapter placement**: Mount the small terminal adapter board near the T-Display at the bottom of the tank. The 3 Dupont wires run to the T-Display’s GPIO header. The DS18B20 probe cable (90cm) runs UP from the adapter to the pipe at the top.
- **DS18B20 probe placement**: Attach the stainless steel probe to the hot water outlet pipe at the **top** of the tank. Use Kapton tape to press the probe firmly against bare copper. Cover with pipe insulation for better thermal coupling and faster response.
- **Cable routing**: Run the DS18B20’s 90cm cable along the side of the tank from top to bottom. Secure with cable clips or tape. If 90cm isn’t enough, the 1-Wire protocol supports splicing in additional wire — just extend with standard 22 AWG wire.

## File Structure

```
water-heater-monitor/
├── water_heater_monitor/
│   └── water_heater_monitor.ino    # Main sketch (single file)
├── tests/
│   └── test_flash_detection.py     # Python simulation of flash detection algorithm
├── CLAUDE.md                        # This file
├── README.md                        # Project documentation
├── LICENSE                          # MIT License
└── enclosure/                       # Optional 3D print files
    └── README.md                    # Enclosure design notes
```

## Development Priorities

### Phase 1 — Core Monitoring (MVP)

1. I2C bus initialization and TSL2591 detection (scan for 0x29)
1. 1-Wire bus initialization on GPIO21 and DS18B20 detection
1. TSL2591 initialization with configurable gain/integration time
1. Flash detection state machine
1. Status code interpretation and display (Screen 1)
1. DS18B20 temperature reading and display
1. Web config portal (WiFi + sensor settings)
1. ElegantOTA setup
1. `/api/status` endpoint

### Phase 2 — Cycle Tracking

1. Heating cycle detection algorithm
1. Cycle history circular buffer
1. Screen 2 (Cycle History with bar chart)
1. Screen 3 (Temperature graph)
1. `/api/history` endpoint
1. Button navigation between screens

### Phase 3 — Home Assistant Integration

1. HA REST API push for all entities
1. Config UI for HA settings
1. Event-driven updates (push on status change)

### Phase 4 — Polish

1. Auto-calibrate light sensor button in web UI
1. Auto-dim display on schedule
1. `/api/debug` and `/api/log` endpoints
1. mDNS (`http://waterheater.local`)
1. 3D printable enclosure design

### Phase 5 — Apple HomeKit Notifications via Home Assistant

1. Enable HA HomeKit Bridge integration
1. Expose water heater entities to Apple Home
1. Create HA notification automations for alerts
1. Test notifications on iPhone

## Apple HomeKit Notifications (via Home Assistant)

The water heater monitor pushes sensor data to Home Assistant (Phase 3). HA then bridges to Apple HomeKit and sends iPhone notifications. No additional hardware or ESP32 code changes needed — this is all HA-side configuration.

### Step 1: Enable HomeKit Bridge in HA

In Home Assistant:

1. Go to **Settings → Devices & Services → Add Integration**
1. Search for **HomeKit Bridge** (not “HomeKit Controller” — that’s for importing HomeKit devices into HA, not the other way around)
1. Select the entities to expose to Apple Home:
- `sensor.water_heater_temperature` → appears as a temperature sensor
- `binary_sensor.water_heater_burner` → appears as an occupancy or contact sensor
- `sensor.water_heater_status` → can be exposed as a sensor
1. HA will generate a pairing QR code — scan it with the Apple Home app on your iPhone
1. The water heater sensors now appear in Apple Home

### Step 2: Create HA Notification Automations

Add these to your HA `automations.yaml` or create them via the UI:

#### Critical Alert: Pilot Out or Error Status

```yaml
automation:
  - alias: "Water Heater - Critical Alert"
    description: "Notify when water heater status is anything other than Normal"
    trigger:
      - platform: state
        entity_id: sensor.water_heater_status
    condition:
      - condition: template
        value_template: >
          {{ trigger.to_state.state not in ['Normal Operation', 'unavailable', 'unknown'] }}
    action:
      - service: notify.mobile_app_kristians_iphone
        data:
          title: "🔥 Water Heater Alert"
          message: >
            Status changed to: {{ trigger.to_state.state }}
            (Flash code: {{ states('sensor.water_heater_flash_code') }})
          data:
            push:
              sound:
                name: default
                critical: 1
                volume: 1.0
```

#### Warning: High Temperature

```yaml
  - alias: "Water Heater - High Temp Warning"
    description: "Notify when pipe temperature exceeds warning threshold"
    trigger:
      - platform: numeric_state
        entity_id: sensor.water_heater_temperature
        above: 150
    action:
      - service: notify.mobile_app_kristians_iphone
        data:
          title: "⚠️ Water Heater Temp High"
          message: >
            Temperature is {{ states('sensor.water_heater_temperature') }}°F
            — above 150°F warning threshold
```

#### Info: Pilot Out for Extended Period

```yaml
  - alias: "Water Heater - Pilot Out Extended"
    description: "Notify if pilot has been out for more than 30 minutes"
    trigger:
      - platform: state
        entity_id: sensor.water_heater_flash_code
        to: "0"
        for:
          minutes: 30
    action:
      - service: notify.mobile_app_kristians_iphone
        data:
          title: "🔥 Pilot Light Out"
          message: "Water heater pilot has been out for 30+ minutes. You may need to relight it."
```

#### Daily Summary

```yaml
  - alias: "Water Heater - Daily Summary"
    description: "Send daily usage summary at 9 PM"
    trigger:
      - platform: time
        at: "21:00:00"
    action:
      - service: notify.mobile_app_kristians_iphone
        data:
          title: "📊 Water Heater Daily Summary"
          message: >
            Cycles today: {{ states('sensor.water_heater_cycles_today') }}
            Total heat time: {{ states('sensor.water_heater_daily_heat_time') }} min
            Current temp: {{ states('sensor.water_heater_temperature') }}°F
            Status: {{ states('sensor.water_heater_status') }}
```

### Step 3: Optional — Apple Home Automations

Once the entities are exposed via HomeKit Bridge, you can also create automations directly in the Apple Home app. For example:

- **Home app → Automation → “When sensor detects something”** → select the water heater contact sensor → send notification
- These automations run on your Apple Home Hub (HomePod, Apple TV, or iPad)
- Simpler but less flexible than HA automations — no templating, no conditional logic

### Notification Architecture

```
ESP32 T-Display S3
    │
    │ HTTP REST API (every 30s or on status change)
    ▼
Home Assistant
    │
    ├──► HA Automations ──► iPhone push notifications (via HA Companion App)
    │                        - Critical alerts (sound override)
    │                        - Warnings
    │                        - Daily summaries
    │
    └──► HomeKit Bridge ──► Apple Home app
                            - Sensor values visible in Home app
                            - Basic automations via Home app
                            - Siri queries: "Hey Siri, what's the water heater status?"
```

### Prerequisites

- **HA Companion App** installed on iPhone (for `notify.mobile_app_*` service)
- **HomeKit Bridge** integration enabled in HA (for Apple Home visibility)
- **Apple Home Hub** (HomePod, Apple TV, or always-home iPad) for HomeKit automations to run when you’re away

## Known Challenges & Tips

- **TSL2591 sensitivity and gain tuning**: The TSL2591 has 4 gain levels (1×, 25×, 428×, 9876×) and 6 integration times (100ms–600ms). Start with medium gain (25×) and 100ms integration. The sensor’s separate IR channel is an advantage — the Resideo LED likely emits some IR, giving you a stronger signal. Use `getFullLuminosity()` for the fastest raw reads when doing flash detection in the main loop. The `calculateLux()` function is slower and unnecessary for threshold-based flash counting.
- **Ambient light interference**: The water heater is typically in a basement or utility closet. Ambient light can still interfere. A physical shroud around the sensor is the best solution. In software, use relative thresholds (current reading vs. rolling baseline average) rather than absolute lux values.
- **Flash timing**: The Resideo LED blinks at roughly 1 flash per second with a ~3-second pause between pattern repeats. The exact timing may vary by model. Make the flash duration and pause duration configurable.
- **DS18B20 on pipe surface**: Surface-mounted temperature readings will be lower than actual water temperature and will lag behind. This is fine for cycle detection — we only need to detect the *change rate*, not the absolute water temperature. Document this in the README.
- **DS18B20 cable length**: The included 90cm cable should reach from the top of a standard 40/50 gallon tank down to the adapter at the bottom. If it’s not long enough, splice in additional wire — 1-Wire works reliably at 5+ meters with the pull-up resistor the DFRobot adapter provides.
- **Two different buses**: The TSL2591 is on I2C (GPIO43/44 via STEMMA QT) and the DS18B20 is on 1-Wire (GPIO21). These are completely independent — no bus conflicts possible.
- **Memory management**: The T-Display S3 has PSRAM. Use it for the temperature history buffer and cycle log. Check `ESP.getFreePsram()` at startup.
- **NTP time**: Sync time via NTP on WiFi connect for accurate cycle timestamps. Use `configTime()` with a reasonable NTP server. Fall back to millis()-based relative time if NTP fails.

## Coding Style

- Use `#define` for pin assignments and configuration constants at top of file
- Use `struct` for config, status, and cycle data
- Prefix global variables: `g_` for globals, `cfg_` for config values
- Functions: `camelCase` for internal functions, `handleXxx` for web handlers, `drawScreenXxx` for display functions
- Comment each major section with a banner comment
- Keep all HTML in a `const char CONFIG_HTML[] PROGMEM = R"rawliteral(...)rawliteral";` block
- Log important events to a circular string buffer accessible via `/api/log`
