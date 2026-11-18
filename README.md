# Water Heater Monitor

An ESP32-based monitor for a conventional gas water heater. It watches the
Resideo (Honeywell) gas control valve's status LED, measures pipe temperature
to detect heating cycles, shows everything on a built-in color LCD, and pushes
data to Home Assistant for iPhone alerts.

Built on the [Lilygo T-Display S3](https://www.amazon.com/dp/B09J112YR7)
(ESP32-S3 with a 170×320 IPS display).

## The Problem

Conventional tank-style gas water heaters are nearly silent failures. Until
you turn on a hot tap and find cold water, you have no idea that:

- **The pilot is out.** Modern Resideo / Honeywell gas valves use a
  thermopile-powered electronic control. If the pilot blows out, the only
  signal is a status LED that stops blinking — buried at the base of the tank
  in a utility closet or basement. You won't notice until the next shower.
- **Something is wrong.** That same LED communicates *seven different fault
  codes* by counting flashes (1 = normal, 2 = low thermopile, 4 = high temp
  shutdown, 5 = sensor fail, 7 = valve fail, 8 = flame sense error,
  0 = control off / pilot out). Reading them means crouching on the floor
  with a stopwatch.
- **Heating cycles are invisible.** There's no record of when the burner ran,
  for how long, or whether it's running more often than usual — which is one
  of the earliest signs of a failing dip tube, sediment buildup, or a leak.

There's no built-in connectivity, no app, no notifications. The status LED is
the entire user interface.

## What This Does

A small ESP32 board sits on top of the water heater with two sensors:

1. **A light sensor** (TSL2591) pressed against the gas valve's status LED
   viewport. It samples the LED 20× per second, counts flashes between the
   3-second pauses, debounces across cycles, and decodes the result into one
   of the seven status codes.
2. **A temperature probe** (DS18B20) taped to the hot water outlet pipe at
   the top of the tank. When the burner fires, the pipe heats noticeably
   within a few minutes; the firmware watches the rate of change to detect
   cycle starts and ends.

That data is then:

- **Shown on the built-in 1.9" LCD** — current status, temperature, cycle
  history bar chart, and a temperature-over-time graph colored by burner
  state. Three screens auto-rotate or step with the two hardware buttons.
- **Served as JSON** at `/api/status`, `/api/history`, `/api/debug`, `/api/log`.
- **Pushed to Home Assistant** as standard sensor entities, which can then
  fire notifications (pilot out, high temp, daily summary) to your phone via
  the HA Companion app, and surface in Apple Home via the HomeKit Bridge.

## Why Not Just One I²C Bus?

The natural design is to put both sensors on the same I²C / STEMMA QT bus.
That doesn't work here because of physical layout:

- The **gas valve and its status LED are at the bottom** of the tank.
- The **hot water outlet pipe is at the top** — typically 4–5 feet up.
- I²C / STEMMA QT is reliable to roughly 50 cm. Past that you start fighting
  capacitance and noise.

So this project uses a hybrid bus design:

```
Top of tank ──────── DS18B20 probe (1-Wire, 90 cm cable)
                              │
                              │ 1-Wire is reliable at 5+ meters
                              │
Bottom of tank ──── DFRobot terminal adapter ──── 3 Dupont wires ──┐
                                                                    │
                              ┌─────────────────────────────────────┘
                              ▼
                     T-Display S3 ESP32
                              ▲
                              │ STEMMA QT cable (~10 cm)
                              │
                     TSL2591 light sensor
                     (mounted over the LED viewport)
```

The light sensor uses the short, plug-and-play I²C path. The temperature
probe uses 1-Wire, which is purpose-built for long single-wire runs with a
4.7 kΩ pull-up — handled by the DFRobot adapter so there's no soldering.

## Hardware (~$39)

| Part | Approx | Why |
|------|--------|-----|
| Lilygo T-Display S3 | $20 | ESP32-S3 + WiFi + 170×320 LCD + STEMMA QT, all in one |
| PAMEENCOS TSL2591 (QWIIC) | $8 | 600M:1 dynamic range, separate IR channel — better LED detection than ambient light sensors |
| DFRobot DS18B20 kit (DFR0055) | $8 | Waterproof probe + adapter with built-in pull-up resistor + 90 cm cable |
| STEMMA QT cable, 100 mm | $3 | Connects light sensor to T-Display, no soldering |
| 3× Dupont jumpers | — | Adapter to GPIO header (signal/VCC/GND) |

Full BOM, links, and wiring detail are in `CLAUDE.md`.

## Software

Single-file Arduino sketch: `water_heater_monitor/water_heater_monitor.ino`.
No build system beyond the Arduino IDE / `arduino-cli`.

Architecture:

- **TFT_eSPI + TFT_eSprite** double-buffered drawing — no flicker
- **ESPAsyncWebServer** for the config portal and JSON APIs
- **ElegantOTA** for over-the-air firmware updates after the first USB flash
- **Preferences (NVS)** for persistent config — no credentials in source
- **AP-mode fallback**: first boot creates `WaterHeaterMonitor` SSID
  (password `configure`) so you can configure WiFi from a phone

### Flash detection state machine

```
WAITING_FOR_FLASH ──[edge above threshold]──► COUNTING
                                                 │
COUNTING ──[2 s gap with no flashes]──► PAUSE_DETECTED
                                                 │
PAUSE_DETECTED ──[count matches previous cycle]──► CONFIRMED
                                                 │
                                                 └──► update display & API
```

Two consecutive matching cycles must be observed before a status update is
published, which rejects spurious flashes from ambient light or noise.
A 10-second silence with no flashes is reported as status 0 (pilot out).

### Cycle detection

Temperature is sampled every 10 seconds. A sustained rise of >2 °F over a
5-minute window flags the burner as ON; once the rate of change goes flat or
negative, the cycle is closed and recorded (start, end, duration, peak temp).
History is kept in a circular buffer for ~48 hours / 100 cycles.

## Setup

1. **Flash** the sketch once over USB-C. All later updates use ElegantOTA at
   `http://<device-ip>/update`.
2. **Connect** to the `WaterHeaterMonitor` AP from your phone (password
   `configure`), browse to `http://192.168.4.1`, enter your WiFi credentials,
   set sensor thresholds, and click **Auto-Calibrate Light Sensor**.
3. **Mount** the boards: TSL2591 over the gas valve status LED viewport
   (cover its own green power LED with tape — it floods the sensor),
   T-Display next to it, DS18B20 probe taped to bare copper on the hot water
   outlet pipe with pipe insulation over the top.
4. **(Optional)** Add Home Assistant URL + long-lived access token in the
   web UI to enable HA push and iPhone notifications.

## Home Assistant Integration

The firmware POSTs to HA's REST API every 30 seconds (or on status change),
creating these entities:

| Entity | Type |
|--------|------|
| `sensor.water_heater_status` | Status text |
| `sensor.water_heater_flash_code` | 0, 1, 2, 4, 5, 7, 8 |
| `sensor.water_heater_temperature` | Pipe temp |
| `binary_sensor.water_heater_burner` | ON when burner is firing |
| `sensor.water_heater_cycles_today` | Daily count |
| `sensor.water_heater_daily_heat_time` | Daily total minutes |

Sample HA automation YAML for critical alerts, high-temp warnings, and
daily summaries is in `CLAUDE.md`. Expose the entities via the HomeKit
Bridge integration to also see them in Apple Home and ask Siri.

## Project Status

Currently running at v1.5.5 on the author's water heater with HA integration
live. Phase 1–3 (core monitoring, cycle tracking, HA push) are functional;
Phase 4–5 (mDNS, enclosure, polish) are in progress.

## License

MIT.

## Known issues

Motion screen-off (Wi-Fi CSI) is disabled by default since v1.9.2, so the display stays on. With it enabled, the board rebooted several times a day. A decoded crash dump from v1.9.1 shows a jump to address 0 in the Wi-Fi driver's interrupt handler (`wDev_ProcessFiq`) during `wDev_IndicateFrame`, with none of this project's code on the stack. Enabling it again from the config page brings that risk back.
