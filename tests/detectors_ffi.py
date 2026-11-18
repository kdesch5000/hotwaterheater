"""ctypes bindings for water_heater_monitor/detectors.h (via harness/detectors_shim.cpp).

The tests drive the same C++ the ESP32 runs, compiled for the host. Nothing
here re-implements the firmware's logic.
"""

import ctypes
import os
import shutil
import subprocess

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(TESTS_DIR)
HEADER = os.path.join(PROJECT_DIR, "water_heater_monitor", "detectors.h")
SHIM = os.path.join(TESTS_DIR, "harness", "detectors_shim.cpp")

# FlashState / FlashEvent / CycleEvent / AlertEdge values from detectors.h
FLASH_WAITING, FLASH_COUNTING, FLASH_PAUSE_DETECTED, FLASH_CONFIRMED = range(4)
FE_CODE_CHANGED, FE_BLINDED, FE_PAUSE = 1, 2, 4
CE_NONE, CE_STARTED, CE_ENDED = range(3)
EDGE_NONE, EDGE_RISE, EDGE_FALL = range(3)

U32 = 1 << 32


def find_compiler():
    for cxx in (os.environ.get("CXX"), "c++", "g++", "clang++"):
        if cxx and shutil.which(cxx):
            return cxx
    return None


def build(out_dir):
    """Compile the shim + header into a shared library; return its path."""
    cxx = find_compiler()
    if cxx is None:
        raise RuntimeError("no C++ compiler found (set CXX)")
    lib = os.path.join(out_dir, "libdetectors.so")
    cmd = [cxx, "-std=c++17", "-O1", "-Wall", "-Wextra", "-Werror",
           "-shared", "-fPIC", SHIM, "-o", lib]
    subprocess.run(cmd, check=True, capture_output=True, text=True)
    return lib


def load(path):
    L = ctypes.CDLL(path)
    P = ctypes.c_void_p
    u8, u16, u32 = ctypes.c_uint8, ctypes.c_uint16, ctypes.c_uint32
    i, f = ctypes.c_int, ctypes.c_float

    def sig(name, res, *args):
        fn = getattr(L, name)
        fn.restype = res
        fn.argtypes = list(args)

    sig("fd_new", P)
    sig("fd_free", None, P)
    sig("fd_step", u8, P, u32, u16, u16, u32)
    sig("fd_set_unknown", None, P, u32)
    for n in ("fd_confirmed", "fd_pending", "fd_state"):
        sig(n, i, P)
    sig("fd_baseline", u16, P)
    sig("fd_set_baseline", None, P, u16)
    sig("fd_threshold", u32, u16, u16)
    sig("fd_calibration", i, u16, ctypes.POINTER(u16))
    sig("fd_gap_count", u32, P, i)
    for n in ("fd_gap_max", "fd_gap_blind_ms", "fd_gap_blind_waiting", "fd_gap_blind_count"):
        sig(n, u32, P)
    sig("fd_forget_last_sample", None, P)
    sig("gap_buckets", i)
    sig("gap_bucket_max", u32, i)
    sig("temp_valid", i, f)
    sig("alert_edge", i, ctypes.POINTER(i), f, f, f)
    sig("cd_new", P)
    sig("cd_free", None, P)
    sig("cd_feed", i, P, f, u32, u16, ctypes.POINTER(u32))
    sig("cd_active", i, P)
    sig("cd_peak", f, P)
    sig("cd_window", i, u16)
    sig("hb_new", P)
    sig("hb_free", None, P)
    sig("hb_advance", None, P, u32)
    sig("hb_add", None, P, u32, u32)
    sig("hb_at", f, P, i)
    sig("hb_epoch", u32, P)
    return L


class Flash:
    """One FlashDetector. Times are milliseconds and wrap like millis()."""

    def __init__(self, lib, threshold=1000, boot_ms=0):
        self.lib = lib
        self.p = lib.fd_new()
        self.threshold = threshold
        self.boot_ms = boot_ms % U32

    def __del__(self):
        if getattr(self, "p", None):
            self.lib.fd_free(self.p)

    def step(self, now_ms, reading):
        return self.lib.fd_step(self.p, int(now_ms) % U32, int(reading), self.threshold, self.boot_ms)

    @property
    def confirmed(self):
        return self.lib.fd_confirmed(self.p)

    @property
    def pending(self):
        return self.lib.fd_pending(self.p)

    @property
    def state(self):
        return self.lib.fd_state(self.p)

    @property
    def baseline(self):
        return self.lib.fd_baseline(self.p)

    def gap_stats(self):
        n = self.lib.gap_buckets()
        return {"hist": [self.lib.fd_gap_count(self.p, b) for b in range(n)],
                "max": self.lib.fd_gap_max(self.p),
                "blind_ms": self.lib.fd_gap_blind_ms(self.p),
                "blind_waiting": self.lib.fd_gap_blind_waiting(self.p),
                "blind_count": self.lib.fd_gap_blind_count(self.p)}

    def forget_last_sample(self):
        self.lib.fd_forget_last_sample(self.p)

    def set_unknown(self, now_ms):
        self.lib.fd_set_unknown(self.p, int(now_ms) % U32)


class Cycles:
    def __init__(self, lib, interval_s=10):
        self.lib = lib
        self.p = lib.cd_new()
        self.interval_s = interval_s

    def __del__(self):
        if getattr(self, "p", None):
            self.lib.cd_free(self.p)

    def feed(self, temp_f, now_ms):
        """Returns (event, duration_s)."""
        d = ctypes.c_uint32(0)
        ev = self.lib.cd_feed(self.p, float(temp_f), int(now_ms) % U32, self.interval_s, ctypes.byref(d))
        return ev, d.value

    @property
    def active(self):
        return bool(self.lib.cd_active(self.p))


class Hourly:
    def __init__(self, lib):
        self.lib = lib
        self.p = lib.hb_new()

    def __del__(self):
        if getattr(self, "p", None):
            self.lib.hb_free(self.p)

    def advance(self, hour):
        self.lib.hb_advance(self.p, hour)

    def add(self, hour, duration_s):
        self.lib.hb_add(self.p, hour, duration_s)

    def at(self, hours_ago):
        return self.lib.hb_at(self.p, hours_ago)

    def total(self):
        return sum(self.at(i) for i in range(48))


def alert_edge(lib, active, value, threshold, hyst):
    a = ctypes.c_int(1 if active else 0)
    e = lib.alert_edge(ctypes.byref(a), value, threshold, hyst)
    return e, bool(a.value)


def calibration(lib, avg):
    out = ctypes.c_uint16(0)
    ok = lib.fd_calibration(avg, ctypes.byref(out))
    return bool(ok), out.value
