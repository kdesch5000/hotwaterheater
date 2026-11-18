/*******************************************************************************
 * DETECTORS — the monitor's decision logic, with no hardware attached
 *
 * Flash-code state machine, heating-cycle detector, hourly heat buckets,
 * temperature-alert hysteresis and reading validation. The .ino feeds these
 * sensor readings and millis(); everything that touches hardware, the network,
 * the log or the display stays in the .ino.
 *
 * tests/ compiles THIS file on the host and property-tests it, so the logic
 * that ships is the logic that's tested. Keep it free of Arduino and ESP-IDF
 * includes, and keep all times uint32_t milliseconds (millis() wraps at 2^32
 * on the ESP32; uint32_t subtraction gives the same wrap on the host).
 ******************************************************************************/
#pragma once

#include <stdint.h>
#include <math.h>

// Flash detection timing
#define FLASH_GAP_TIMEOUT_MS     2000   // >2s gap = end of pattern
#define PILOT_OUT_TIMEOUT_MS     10000  // >10s no flash = pilot out
#define FLASH_DEBOUNCE_MS        200    // Minimum time between flashes
#define FLASH_SAMPLE_GAP_MS      1500   // A longer gap between samples = not actually watching
#define LIGHT_BLIND_UNKNOWN_MS   60000  // Light steadily above threshold this long = status unknown

// Sample-gap diagnostics. The LED was measured on for ~150-250 ms (2026-11-18)
// and the TSL2591 integrates 100 ms per reading, so readings further apart
// than this can miss a whole flash. Diagnostic only: nothing decides on it yet.
#define GAP_BLIND_MS             250
#define GAP_BUCKETS              7

// Heating cycle detection
#define CYCLE_TEMP_RISE_THRESHOLD 2.0   // 2 degrees F rise
#define CYCLE_DETECT_WINDOW_SEC   300   // over 5 minutes
#define TEMP_SLOPE_BUF            60    // readings kept for the slope

#define HOURLY_BUCKETS            48    // 48 hours of hourly data

// DS18B20 rated range. Anything outside it (the library's -127 and -25x
// fault codes) or exactly 85.0 (power-on value of a glitched probe) is not a
// pipe temperature.
#define DS18B20_MIN_C            -55.0f
#define DS18B20_MAX_C            125.0f

/*******************************************************************************
 * FLASH CODES
 ******************************************************************************/
enum FlashState {
  FLASH_WAITING,
  FLASH_COUNTING,
  FLASH_PAUSE_DETECTED,
  FLASH_CONFIRMED
};

// Histogram of the time between consecutive light readings, since boot.
// Bucket i holds gaps <= GAP_BUCKET_MAX_MS[i]; the last bucket holds the rest.
static const uint32_t GAP_BUCKET_MAX_MS[GAP_BUCKETS - 1] = {150, 250, 500, 1000, 1500, 3000};

struct SampleGapStats {
  uint32_t count[GAP_BUCKETS] = {0};
  uint32_t maxMs = 0;
  uint32_t blindMs = 0;          // Total time inside gaps > GAP_BLIND_MS (saturates)
  uint32_t blindWaiting = 0;     // Such gaps while WAITING: the ones that matter for pilot out
};

inline void gapStatsAdd(SampleGapStats& g, uint32_t gapMs, bool waiting) {
  int b = 0;
  while (b < GAP_BUCKETS - 1 && gapMs > GAP_BUCKET_MAX_MS[b]) b++;
  if (g.count[b] != UINT32_MAX) g.count[b]++;
  if (gapMs > g.maxMs) g.maxMs = gapMs;
  if (gapMs > GAP_BLIND_MS) {
    g.blindMs = (UINT32_MAX - g.blindMs < gapMs) ? UINT32_MAX : g.blindMs + gapMs;
    if (waiting && g.blindWaiting != UINT32_MAX) g.blindWaiting++;
  }
}

// Gaps longer than GAP_BLIND_MS (buckets whose lower bound is >= it)
inline uint32_t gapBlindCount(const SampleGapStats& g) {
  uint32_t n = 0;
  for (int b = 1; b < GAP_BUCKETS; b++)
    if (GAP_BUCKET_MAX_MS[b - 1] >= GAP_BLIND_MS) n += g.count[b];
  return n;
}

struct FlashDetector {
  FlashState state = FLASH_WAITING;
  int count = 0;               // Flashes in the pattern being counted
  int pendingCount = -1;       // Last pattern's count, awaiting a repeat
  int confirmedCode = -1;      // -1 = unknown
  uint32_t lastFlashMs = 0;
  bool lastLightHigh = false;
  uint16_t baseline = 0;       // Slow EMA of the dark reading
  uint16_t peak = 0;           // Debug only
  uint32_t lastSampleMs = 0;   // 0 = no sample yet (or sensor just recovered)
  uint32_t lightHighSinceMs = 0;  // 0 = not steadily above threshold
  SampleGapStats gaps;
};

// What a sample did, for the caller to log/push
enum FlashEvent : uint8_t {
  FE_NONE         = 0,
  FE_CODE_CHANGED = 1 << 0,   // confirmedCode changed
  FE_BLINDED      = 1 << 1,   // steady light: status went unknown
  FE_PAUSE        = 1 << 2,   // a pattern ended (pauseCount holds its count)
};

struct FlashStepResult {
  uint8_t events = FE_NONE;
  int pauseCount = 0;
  bool lightHigh = false;
};

// Threshold for a sample. A configured threshold wins; 0 = auto from baseline.
// 32-bit on purpose: the old uint16_t `baseline * 2 + 50` wrapped above a
// baseline of 32742 and produced a tiny threshold.
inline uint32_t flashThreshold(uint16_t cfgThreshold, uint16_t baseline) {
  return cfgThreshold > 0 ? cfgThreshold : (uint32_t)baseline * 2 + 50;
}

// Threshold that /api/calibrate stores for a measured dark average. Returns
// false when the "dark" reading is so bright that no uint16 threshold above
// it exists — calibrating under a flashlight, say. The caller keeps the old
// values rather than saving a wrapped one.
inline bool calibrationThreshold(uint16_t avg, uint16_t* out) {
  uint32_t t = (uint32_t)avg * 2 + 50;
  if (t > 65535) return false;
  *out = (uint16_t)t;
  return true;
}

// We can't read the LED right now. That is not the same as the LED being
// dark, so the status becomes unknown rather than pilot out.
inline void flashSetUnknown(FlashDetector& f, uint32_t now) {
  f.confirmedCode = -1;
  f.pendingCount = -1;
  f.count = 0;
  f.state = FLASH_WAITING;
  f.lastFlashMs = now;
}

// One light-sensor sample. bootMs is millis() at boot.
inline FlashStepResult flashStep(FlashDetector& f, uint32_t now, uint16_t reading,
                                 uint16_t cfgThreshold, uint32_t bootMs) {
  FlashStepResult r;

  // Record the gap before anything acts on it (0 = first sample / just recovered)
  if (f.lastSampleMs != 0) gapStatsAdd(f.gaps, now - f.lastSampleMs, f.state == FLASH_WAITING);

  // "No flash for 10 s" only means something if we were looking the whole
  // time. A blocked loop() (HA pushes over a bad link, say) misses flashes,
  // so restart the window instead of counting it towards pilot out.
  if (f.lastSampleMs != 0 && now - f.lastSampleMs > FLASH_SAMPLE_GAP_MS && f.state == FLASH_WAITING) {
    f.lastFlashMs = now;
  }
  f.lastSampleMs = now;

  // Baseline: exponential moving average, updated only when not flashing
  if (f.baseline == 0) {
    f.baseline = reading;
  } else if (reading < f.baseline + 10) {
    f.baseline = (uint16_t)(((uint32_t)f.baseline * 15 + reading) / 16);
  }

  // A threshold no reading can exceed means the baseline alone is brighter
  // than the sensor's range: blinded, same as steady light.
  uint32_t threshold = flashThreshold(cfgThreshold, f.baseline);
  bool lightHigh = reading > threshold || threshold > 65535;
  r.lightHigh = lightHigh;

  if (reading > f.peak) f.peak = reading;

  // Steady light above the threshold (a room light, a flashlight) hides the
  // LED. No edges then doesn't mean no flashes.
  if (!lightHigh) {
    f.lightHighSinceMs = 0;
  } else if (f.lightHighSinceMs == 0) {
    f.lightHighSinceMs = now;
  } else if (now - f.lightHighSinceMs > LIGHT_BLIND_UNKNOWN_MS && f.confirmedCode != -1) {
    flashSetUnknown(f, now);
    r.events |= FE_BLINDED;
  }

  bool risingEdge = lightHigh && !f.lastLightHigh;
  f.lastLightHigh = lightHigh;

  switch (f.state) {
    case FLASH_WAITING:
      if (risingEdge) {
        f.count = 1;
        f.lastFlashMs = now;
        f.state = FLASH_COUNTING;
      } else if (lightHigh) {
        f.lastFlashMs = now;   // Blinded or mid-flash: no verdict yet
      } else if (now - f.lastFlashMs > PILOT_OUT_TIMEOUT_MS && f.lastFlashMs > 0) {
        f.count = 0;           // No flash for >10 seconds
        f.state = FLASH_PAUSE_DETECTED;
      } else if (f.lastFlashMs == 0 && now - bootMs > PILOT_OUT_TIMEOUT_MS) {
        f.count = 0;           // Never seen a flash since boot
        f.state = FLASH_PAUSE_DETECTED;
      }
      break;

    case FLASH_COUNTING:
      if (risingEdge && (now - f.lastFlashMs > FLASH_DEBOUNCE_MS)) {
        f.count++;
        f.lastFlashMs = now;
      } else if (now - f.lastFlashMs > FLASH_GAP_TIMEOUT_MS) {
        f.state = FLASH_PAUSE_DETECTED;   // Gap: pattern complete
      }
      break;

    case FLASH_PAUSE_DETECTED: {
      // Handled on the sample after the pause was seen, as it always was.
      // Pilot out arrives here with count 0 (never entered COUNTING).
      r.events |= FE_PAUSE;
      r.pauseCount = f.count;
      if (f.pendingCount == f.count) {
        // Same count two patterns in a row — confirmed
        int prev = f.confirmedCode;
        f.confirmedCode = f.count;
        if (prev != f.confirmedCode) r.events |= FE_CODE_CHANGED;
      } else {
        f.pendingCount = f.count;
      }
      f.state = FLASH_WAITING;
      f.lastFlashMs = now;
      break;
    }

    case FLASH_CONFIRMED:
      f.state = FLASH_WAITING;
      f.lastFlashMs = now;
      break;
  }
  return r;
}

/*******************************************************************************
 * TEMPERATURE
 ******************************************************************************/
inline bool tempReadingValid(float tempC) {
  if (isnan(tempC)) return false;
  if (tempC == 85.0f) return false;
  return tempC >= DS18B20_MIN_C && tempC <= DS18B20_MAX_C;
}

// Edge-triggered alert with hysteresis: rises at >= threshold, falls only
// below threshold - hyst, so a reading hovering on the line can't flap.
enum AlertEdge { EDGE_NONE, EDGE_RISE, EDGE_FALL };

inline AlertEdge alertEdge(bool& active, float value, float threshold, float hyst) {
  if (isnan(value)) return EDGE_NONE;
  if (!active && value >= threshold) { active = true;  return EDGE_RISE; }
  if (active && value < threshold - hyst) { active = false; return EDGE_FALL; }
  return EDGE_NONE;
}

/*******************************************************************************
 * HEATING CYCLES
 ******************************************************************************/
struct CycleDetector {
  float hist[TEMP_SLOPE_BUF] = {0};
  int idx = 0;                 // Next slot to write
  int count = 0;
  bool burnerActive = false;
  uint32_t startMs = 0;
  float startTempF = NAN;
  float peakTempF = NAN;
};

enum CycleEvent { CE_NONE, CE_STARTED, CE_ENDED };

inline void cycleAddReading(CycleDetector& c, float tempF) {
  c.hist[c.idx] = tempF;
  c.idx = (c.idx + 1) % TEMP_SLOPE_BUF;
  if (c.count < TEMP_SLOPE_BUF) c.count++;
}

// Readings in 5 minutes at the configured interval, kept inside the buffer
inline int cycleWindow(uint16_t readIntervalSec) {
  int iv = readIntervalSec < 1 ? 1 : readIntervalSec;
  int w = CYCLE_DETECT_WINDOW_SEC / iv;
  if (w < 1) w = 1;
  if (w > TEMP_SLOPE_BUF - 1) w = TEMP_SLOPE_BUF - 1;
  return w;
}

// Run after cycleAddReading(currentTempF). On CE_ENDED, *durationSec and
// c.peakTempF describe the finished cycle.
inline CycleEvent cycleDetect(CycleDetector& c, float currentTempF, uint32_t now,
                              uint16_t readIntervalSec, uint32_t* durationSec) {
  int window = cycleWindow(readIntervalSec);
  if (c.count <= window) return CE_NONE;   // Need 5 minutes of data

  // The newest reading is at idx - 1, so this is `window` readings before it
  int oldest = (c.idx - 1 - window + TEMP_SLOPE_BUF) % TEMP_SLOPE_BUF;
  float tempRise = currentTempF - c.hist[oldest];

  if (!c.burnerActive) {
    // Burner ON: temperature rising >2F in 5 minutes
    if (tempRise > CYCLE_TEMP_RISE_THRESHOLD) {
      c.burnerActive = true;
      c.startMs = now;
      c.startTempF = currentTempF;
      c.peakTempF = currentTempF;
      return CE_STARTED;
    }
    return CE_NONE;
  }

  if (currentTempF > c.peakTempF) c.peakTempF = currentTempF;

  // Burner OFF: 5-min slope has gone flat or negative
  if (tempRise <= 0.0f) {
    c.burnerActive = false;
    if (durationSec) *durationSec = (now - c.startMs) / 1000;
    return CE_ENDED;
  }
  return CE_NONE;
}

/*******************************************************************************
 * HOURLY HEAT BUCKETS
 ******************************************************************************/
struct HourlyBuckets {
  float heatMin[HOURLY_BUCKETS] = {0};
  int head = 0;
  uint32_t epochHour = 0;      // time()/3600 of the head bucket; 0 = not started
};

// Keep the head bucket on the current hour (epoch hours, so DST can't move it).
// A clock that steps backwards (an NTP correction across an hour boundary)
// leaves the buckets alone: unsigned `hour - epochHour` used to wrap to a huge
// step count and wipe all 48 hours.
inline void hourlyAdvance(HourlyBuckets& h, uint32_t hour) {
  if (h.epochHour == 0) h.epochHour = hour;
  if (hour <= h.epochHour) return;
  uint32_t steps = hour - h.epochHour;
  if (steps > HOURLY_BUCKETS) steps = HOURLY_BUCKETS;
  for (uint32_t i = 0; i < steps; i++) {
    h.head = (h.head + 1) % HOURLY_BUCKETS;
    h.heatMin[h.head] = 0;
  }
  h.epochHour = hour;
}

inline void hourlyAdd(HourlyBuckets& h, uint32_t hour, uint32_t durationSec) {
  hourlyAdvance(h, hour);
  h.heatMin[h.head] += durationSec / 60.0f;
}

// Minutes heated `hoursAgo` buckets behind the head (0 = current hour)
inline float hourlyAt(const HourlyBuckets& h, int hoursAgo) {
  return h.heatMin[(h.head - hoursAgo + HOURLY_BUCKETS * 2) % HOURLY_BUCKETS];
}
