// C ABI over water_heater_monitor/detectors.h so pytest (ctypes) can drive
// the exact logic the firmware ships. Built by tests/conftest.py.
#include "../../water_heater_monitor/detectors.h"

extern "C" {

// ---- Flash detector ---------------------------------------------------------
FlashDetector* fd_new() { return new FlashDetector(); }
void fd_free(FlashDetector* f) { delete f; }

// Returns the FlashEvent bitmask
uint8_t fd_step(FlashDetector* f, uint32_t now, uint16_t reading, uint16_t cfgThreshold, uint32_t bootMs) {
  return flashStep(*f, now, reading, cfgThreshold, bootMs).events;
}
void fd_set_unknown(FlashDetector* f, uint32_t now) { flashSetUnknown(*f, now); }
int fd_confirmed(const FlashDetector* f) { return f->confirmedCode; }
int fd_pending(const FlashDetector* f) { return f->pendingCount; }
int fd_state(const FlashDetector* f) { return (int)f->state; }
uint16_t fd_baseline(const FlashDetector* f) { return f->baseline; }
void fd_set_baseline(FlashDetector* f, uint16_t b) { f->baseline = b; }
uint32_t fd_threshold(uint16_t cfg, uint16_t baseline) { return flashThreshold(cfg, baseline); }
uint32_t fd_gap_count(const FlashDetector* f, int bucket) { return f->gaps.count[bucket]; }
uint32_t fd_gap_max(const FlashDetector* f) { return f->gaps.maxMs; }
uint32_t fd_gap_blind_ms(const FlashDetector* f) { return f->gaps.blindMs; }
uint32_t fd_gap_blind_waiting(const FlashDetector* f) { return f->gaps.blindWaiting; }
uint32_t fd_gap_blind_count(const FlashDetector* f) { return gapBlindCount(f->gaps); }
void fd_forget_last_sample(FlashDetector* f) { f->lastSampleMs = 0; }   // as tslRecover does
int gap_buckets() { return GAP_BUCKETS; }
uint32_t gap_bucket_max(int i) { return GAP_BUCKET_MAX_MS[i]; }
int fd_calibration(uint16_t avg, uint16_t* out) { return calibrationThreshold(avg, out) ? 1 : 0; }

// ---- Temperature ------------------------------------------------------------
int temp_valid(float c) { return tempReadingValid(c) ? 1 : 0; }

// active is in/out; returns AlertEdge
int alert_edge(int* active, float value, float threshold, float hyst) {
  bool a = *active != 0;
  int e = alertEdge(a, value, threshold, hyst);
  *active = a ? 1 : 0;
  return e;
}

// ---- Heating cycles ---------------------------------------------------------
CycleDetector* cd_new() { return new CycleDetector(); }
void cd_free(CycleDetector* c) { delete c; }

// One valid reading, in the firmware's order: add to the buffer, then detect.
// Returns CycleEvent; on CE_ENDED *durationSec is filled.
int cd_feed(CycleDetector* c, float tempF, uint32_t now, uint16_t intervalSec, uint32_t* durationSec) {
  cycleAddReading(*c, tempF);
  return cycleDetect(*c, tempF, now, intervalSec, durationSec);
}
int cd_active(const CycleDetector* c) { return c->burnerActive ? 1 : 0; }
float cd_peak(const CycleDetector* c) { return c->peakTempF; }
int cd_window(uint16_t intervalSec) { return cycleWindow(intervalSec); }

// ---- Hourly buckets ---------------------------------------------------------
HourlyBuckets* hb_new() { return new HourlyBuckets(); }
void hb_free(HourlyBuckets* h) { delete h; }
void hb_advance(HourlyBuckets* h, uint32_t hour) { hourlyAdvance(*h, hour); }
void hb_add(HourlyBuckets* h, uint32_t hour, uint32_t durationSec) { hourlyAdd(*h, hour, durationSec); }
float hb_at(const HourlyBuckets* h, int hoursAgo) { return hourlyAt(*h, hoursAgo); }
uint32_t hb_epoch(const HourlyBuckets* h) { return h->epochHour; }

}  // extern "C"
