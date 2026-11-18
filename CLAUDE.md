# Water Heater Monitor — CLAUDE.md

ESP32 (Lilygo T-Display S3) monitor for a gas water heater with a Resideo (Honeywell) gas valve: reads the valve's status-LED flash codes with a TSL2591 light sensor, detects burner cycles from a DS18B20 on the hot-water outlet pipe, shows status on the built-in LCD, and pushes to Home Assistant. Single-file sketch: `water_heater_monitor/water_heater_monitor.ino`.

The original full design spec (BOM, wiring, screen mockups, API schemas, HA/HomeKit notification YAML, roadmap) lives in `docs/design-spec.md`. It predates the code; where they disagree, the code wins.

## Why the hardware is split across two buses

The gas valve (with the LED) is at the **bottom** of the tank; the outlet pipe is at the **top**, ~4–5 ft away. I2C/STEMMA QT is only reliable to ~50 cm, so:
- **TSL2591** → STEMMA QT (I2C, GPIO43 SDA / GPIO44 SCL), mounted with the T-Display at the bottom over the valve viewport.
- **DS18B20** → 1-Wire on GPIO21 via the DFRobot DFR0055 adapter (built-in 4.7kΩ pull-up; jumper must be in pull-up position), 90 cm cable up to the pipe. 1-Wire is fine at 5 m+; extend with 22 AWG if needed.

The buses are independent — no conflicts possible.

## Gotchas

- **STEMMA QT SDA/SCL may be swapped** on some T-Display S3 revisions. If the TSL2591 (fixed address 0x29) isn't found, run an I2C scan, then try `Wire.begin(44, 43)`; last resort is the header pins with Dupont wires.
- **Pipe-surface temperatures read 10–20°F below the actual water temp and lag.** Cycle detection relies on rate of change, not absolute value — don't "fix" thresholds to match tank temp.
- **Use relative light thresholds** (reading vs rolling baseline), not absolute lux; ambient light varies. Raw `getFullLuminosity()` reads are faster than `calculateLux()` and are what flash detection should use. The separate IR channel helps — the LED likely emits IR.
- **LED timing varies by valve model** (~1 flash/s, ~3 s pause between repeats) — keep flash/pause timings configurable.
- Degrade gracefully: if either sensor is missing at boot, show it on screen and keep running with the other.
- Use PSRAM for the temperature history and cycle log. Sync time via NTP; fall back to millis()-relative time if it fails.

## Resideo flash codes

Each pattern = N flashes, ~3 s pause, repeat. A status change requires the same count on 2 consecutive cycles; no flash for >10 s = code 0.

| Flashes | Status | Severity |
|---|---|---|
| 0 | Control Off / Pilot Out | ERROR |
| 1 | Normal Operation | OK |
| 2 | Thermopile Voltage Low | WARNING |
| 4 | High Temperature Shutdown (manual reset) | CRITICAL |
| 5 | Temperature Sensor Failure | CRITICAL |
| 7 | Gas Control / Valve Failure | CRITICAL |
| 8 | Flame Sense Error | ERROR |

## Build & flash

- Arduino IDE board: LilyGo T-Display-S3 (or ESP32S3 Dev Module); USB CDC On Boot **Enabled**; Flash 16MB; Partition "Default 4MB with spiffs".
- TFT_eSPI must be pointed at the board: in `Arduino/libraries/TFT_eSPI/User_Setup_Select.h` comment out `#include <User_Setup.h>` and uncomment `#include <User_Setups/Setup206_LilyGo_T_Display_S3.h>`.
- Production builds need `water_heater_monitor/local_config.h` (gitignored; copy `local_config.example.h`) so a factory-fresh device gets the real syslog host. GitHub (`git@github.com:kdesch5000/hotwaterheater.git`, public) is the system of record. Don't commit real hostnames or IPs; `test_no_site_specific_values_in_tracked_source` enforces it.
- AsyncTCP and ESPAsyncWebServer come from GitHub ZIPs (me-no-dev), not Library Manager.
- First flash over USB-C; after that, OTA via ElegantOTA at `http://<device-ip>/update`. First boot AP: SSID `WaterHeaterMonitor`, password `configure`, `http://192.168.4.1`.

## Conventions (match info-display-hub)

- One `.ino` file plus `detectors.h` (pure decision logic: flash codes, heating cycles, hourly buckets, alert hysteresis, reading validation). Any new *decision* logic goes in `detectors.h` with no Arduino includes and uint32_t ms times, so `tests/` can compile and property-test it on the host; hardware, network, logging and display stay in the `.ino`. TFT_eSprite double buffering; ESPAsyncWebServer; Preferences for config; no blocking delays in `loop()` — everything interval-based.
- `#define` for pins/constants at top; `struct`s for config/status/cycle data.
- Prefixes: `g_` globals, `cfg_` config values. Functions `camelCase`; web handlers `handleXxx`; display `drawScreenXxx`.
- Banner comment for each major section.
- All HTML in a `const char CONFIG_HTML[] PROGMEM = R"rawliteral(...)rawliteral";` block.
- Log important events to the circular log buffer (served at `/api/log`).

## Tests

`pytest tests/` (needs `requirements-dev.txt` and a host C++ compiler). Tests drive `detectors.h` through ctypes (`tests/harness/detectors_shim.cpp`, `tests/detectors_ffi.py`) — never re-implement firmware logic in Python; the old Python model drifted from the C++. When changing `detectors.h`, add or update the property in `tests/test_properties.py` that states the rule. The two `xfail(strict=True)` tests are open findings (CHANGELOG 2026-11-18), not flaky tests.
