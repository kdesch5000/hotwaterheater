"""Scenario tests for flash-code detection, plus the firmware build check.

Until 2026-11-18 these ran against a Python re-implementation of the state
machine that had drifted from the C++. They now drive
water_heater_monitor/detectors.h itself (see detectors_ffi.py); the
property-based tests in test_properties.py cover the same ground for all
inputs rather than these examples.
"""

import os
import random
import shutil
import subprocess

import pytest

from detectors_ffi import FLASH_WAITING, Flash
from sim import DARK, LED, THRESHOLD, pattern_schedule, run_light, sample_times

PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SKETCH_DIR = os.path.join(PROJECT_DIR, "water_heater_monitor")
FQBN = "esp32:esp32:lilygo_t_display_s3"
BOOT = 1_000


def fresh(lib):
    return Flash(lib, threshold=THRESHOLD, boot_ms=BOOT)


def play(det, code, start, repeats=4, rng=None, **kw):
    rng = rng or random.Random(1)
    iv, end = pattern_schedule(code, start, repeats, **kw)
    changes = run_light(det, iv, sample_times(start, end, rng), rng)
    return changes, end


def dark(det, start, seconds, rng=None):
    rng = rng or random.Random(2)
    end = start + seconds * 1000
    return run_light(det, [], sample_times(start, end, rng), rng), end


@pytest.mark.parametrize("code", [1, 2, 4, 5, 7, 8])
def test_each_table_code_is_detected(lib, code):
    det = fresh(lib)
    changes, _ = play(det, code, BOOT)
    assert [c for _, c in changes] == [code]
    assert det.state == FLASH_WAITING


def test_pilot_out_from_boot(lib):
    """Darkness from boot: code 0, which needs two 10 s windows."""
    det = fresh(lib)
    changes, _ = dark(det, BOOT, 25)
    assert [c for _, c in changes] == [0]


def test_code_transition(lib):
    det = fresh(lib)
    _, t = play(det, 1, BOOT)
    changes, _ = play(det, 4, t)
    assert [c for _, c in changes] == [4]


def test_boot_dark_then_flashes(lib):
    """The real boot sequence: dark -> pilot out, then flashes -> code 1."""
    det = fresh(lib)
    _, t = dark(det, BOOT, 25)
    assert det.confirmed == 0
    changes, _ = play(det, 1, t, repeats=5)
    assert [c for _, c in changes] == [1]


def test_pilot_out_then_recovery(lib):
    det = fresh(lib)
    _, t = play(det, 1, BOOT)
    changes, t = dark(det, t, 25)
    assert [c for _, c in changes] == [0]
    changes, _ = play(det, 1, t, repeats=5)
    assert [c for _, c in changes] == [1]


def test_noise_below_threshold_is_ignored(lib):
    det = fresh(lib)
    rng = random.Random(42)
    _, t = play(det, 1, BOOT)
    iv, end = pattern_schedule(1, t, 4)
    changes = run_light(det, iv, sample_times(t, end, rng), rng, noise=(-10, THRESHOLD - DARK - 1))
    assert changes == []


def test_steady_light_goes_unknown_after_a_minute(lib):
    det = fresh(lib)
    _, t = play(det, 1, BOOT)
    rng = random.Random(3)
    changes = run_light(det, [], sample_times(t, t + 70_000, rng), steady=(t, t + 70_000, LED))
    assert [c for _, c in changes] == [-1]


def test_set_unknown_clears_everything(lib):
    det = fresh(lib)
    _, t = play(det, 2, BOOT)
    det.set_unknown(t)
    assert (det.confirmed, det.pending, det.state) == (-1, -1, FLASH_WAITING)


def test_motion_sensing_defaults_off():
    """CSI crashes the board inside the Wi-Fi driver, so a fresh config must not enable it."""
    with open(os.path.join(SKETCH_DIR, "water_heater_monitor.ino")) as f:
        assert 'g_prefs.getUChar("moMode", 0)' in f.read()


def test_firmware_uses_the_tested_header():
    """The .ino must call into detectors.h, not carry its own copy of the logic."""
    with open(os.path.join(SKETCH_DIR, "water_heater_monitor.ino")) as f:
        src = f.read()
    assert '#include "detectors.h"' in src
    for call in ("flashStep(", "cycleDetect(", "cycleAddReading(", "hourlyAdd(",
                 "hourlyAdvance(", "alertEdge(", "tempReadingValid(", "calibrationThreshold("):
        assert call in src, call
    for stale in ("enum FlashState", "g_confirmedFlashCode", "g_tempHistory2Min"):
        assert stale not in src, stale


def test_no_site_specific_values_in_tracked_source():
    """The repo is public: real hostnames and LAN addresses belong in the
    gitignored local_config.h, not the source. (192.168.4.1 is the device's
    own setup AP, which is generic.)"""
    import re
    # Assembled from pieces so this line doesn't match itself
    bad = re.compile("|".join([r"\." + "mf" + r"\b", "mr" + "fish",
                               r"192\.168\.(?!4\.1\b)" + r"\d", "kdesch" + "@"]))
    out = subprocess.run(["git", "ls-files"], cwd=PROJECT_DIR, capture_output=True, text=True)
    if out.returncode != 0:
        pytest.skip("not a git checkout")
    hits = []
    for path in out.stdout.split():
        full = os.path.join(PROJECT_DIR, path)
        try:
            with open(full, encoding="utf-8") as f:
                for n, line in enumerate(f, 1):
                    if bad.search(line):
                        hits.append(f"{path}:{n}: {line.strip()[:80]}")
        except (UnicodeDecodeError, FileNotFoundError):
            continue
    assert hits == [], "\n".join(hits)


def _have_esp32_toolchain():
    if not shutil.which("arduino-cli"):
        return False
    r = subprocess.run(["arduino-cli", "core", "list"], capture_output=True, text=True)
    return "esp32:esp32" in r.stdout


@pytest.mark.skipif(not _have_esp32_toolchain(), reason="arduino-cli with the esp32 core not installed")
def test_compile():
    """End to end: the sketch builds for the board."""
    result = subprocess.run(["arduino-cli", "compile", "--fqbn", FQBN, SKETCH_DIR],
                            capture_output=True, text=True, timeout=600)
    assert result.returncode == 0, result.stderr[-4000:]
