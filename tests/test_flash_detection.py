"""
Simulation and tests for the water heater flash detection state machine.

This tests the algorithm logic in Python before deploying to the ESP32.
The state machine detects Resideo gas valve LED flash codes by counting
rising edges and confirming patterns over consecutive cycles.
"""

import os
import subprocess
import time
from enum import Enum, auto
from dataclasses import dataclass, field

# Project paths
PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SKETCH_DIR = os.path.join(PROJECT_DIR, "water_heater_monitor")
FQBN = "esp32:esp32:lilygo_t_display_s3"


class FlashState(Enum):
    WAITING = auto()
    COUNTING = auto()
    PAUSE_DETECTED = auto()
    CONFIRMED = auto()


FLASH_CODES = {
    0: ("Control Off / Pilot Out", "ERROR"),
    1: ("Normal Operation", "OK"),
    2: ("Thermopile Voltage Low", "WARNING"),
    4: ("High Temperature Shutdown", "CRITICAL"),
    5: ("Temperature Sensor Failure", "CRITICAL"),
    7: ("Gas Control / Valve Failure", "CRITICAL"),
    8: ("Flame Sense Error", "ERROR"),
}

# Timing constants (milliseconds)
SAMPLE_INTERVAL_MS = 50
FLASH_GAP_TIMEOUT_MS = 2000
PILOT_OUT_TIMEOUT_MS = 10000
FLASH_DEBOUNCE_MS = 200


@dataclass
class FlashDetector:
    threshold: int = 100
    baseline: int = 20

    state: FlashState = FlashState.WAITING
    flash_count: int = 0
    pending_flash_count: int = -1
    confirmed_code: int = -1
    last_flash_time: float = 0.0
    last_light_high: bool = False
    boot_time: float = field(default_factory=time.time)

    def process_sample(self, reading: int, now: float) -> int | None:
        """Process a single light sensor reading. Returns confirmed code on change, else None."""
        light_high = reading > self.threshold
        rising_edge = light_high and not self.last_light_high
        self.last_light_high = light_high

        result = None

        if self.state == FlashState.WAITING:
            if rising_edge:
                self.flash_count = 1
                self.last_flash_time = now
                self.state = FlashState.COUNTING
            elif self.last_flash_time > 0 and (now - self.last_flash_time) > PILOT_OUT_TIMEOUT_MS / 1000:
                self.flash_count = 0
                self.state = FlashState.PAUSE_DETECTED
            elif self.last_flash_time == 0 and (now - self.boot_time) > PILOT_OUT_TIMEOUT_MS / 1000:
                self.flash_count = 0
                self.state = FlashState.PAUSE_DETECTED

        elif self.state == FlashState.COUNTING:
            if rising_edge and (now - self.last_flash_time) > FLASH_DEBOUNCE_MS / 1000:
                self.flash_count += 1
                self.last_flash_time = now
            elif (now - self.last_flash_time) > FLASH_GAP_TIMEOUT_MS / 1000:
                self.state = FlashState.PAUSE_DETECTED

        if self.state == FlashState.PAUSE_DETECTED:
            count = self.flash_count

            if self.pending_flash_count == count:
                prev = self.confirmed_code
                self.confirmed_code = count
                self.state = FlashState.WAITING
                self.last_flash_time = now
                if prev != self.confirmed_code:
                    result = self.confirmed_code
            else:
                self.pending_flash_count = count
                self.state = FlashState.WAITING
                self.last_flash_time = now

        elif self.state == FlashState.CONFIRMED:
            self.state = FlashState.WAITING
            self.last_flash_time = now

        return result


def generate_flash_pattern(code: int, cycles: int = 3, flash_duration_s: float = 0.3,
                           flash_gap_s: float = 0.7, pause_s: float = 3.0,
                           sample_interval_s: float = 0.05) -> list[tuple[float, int]]:
    """Generate simulated light sensor readings for a flash pattern.

    Returns list of (timestamp, reading) tuples.
    """
    readings = []
    t = 0.0
    baseline = 20
    flash_level = 500

    for cycle in range(cycles):
        for flash in range(code):
            # Flash ON
            end = t + flash_duration_s
            while t < end:
                readings.append((t, flash_level))
                t += sample_interval_s
            # Flash OFF (gap between flashes)
            if flash < code - 1:
                end = t + flash_gap_s
                while t < end:
                    readings.append((t, baseline))
                    t += sample_interval_s

        # Pause between pattern repeats
        end = t + pause_s
        while t < end:
            readings.append((t, baseline))
            t += sample_interval_s

    return readings


def test_normal_operation():
    """Test detection of flash code 1 (Normal Operation)."""
    detector = FlashDetector(threshold=100, baseline=20)
    detector.boot_time = 0.0
    readings = generate_flash_pattern(1, cycles=4)

    results = []
    for ts, reading in readings:
        r = detector.process_sample(reading, ts)
        if r is not None:
            results.append(r)

    assert 1 in results, f"Expected flash code 1, got {results}"
    assert detector.confirmed_code == 1
    print("PASS: Normal operation (code 1)")


def test_thermopile_low():
    """Test detection of flash code 2 (Thermopile Voltage Low)."""
    detector = FlashDetector(threshold=100, baseline=20)
    detector.boot_time = 0.0
    readings = generate_flash_pattern(2, cycles=4)

    results = []
    for ts, reading in readings:
        r = detector.process_sample(reading, ts)
        if r is not None:
            results.append(r)

    assert 2 in results, f"Expected flash code 2, got {results}"
    assert detector.confirmed_code == 2
    print("PASS: Thermopile voltage low (code 2)")


def test_high_temp_shutdown():
    """Test detection of flash code 4 (High Temperature Shutdown)."""
    detector = FlashDetector(threshold=100, baseline=20)
    detector.boot_time = 0.0
    readings = generate_flash_pattern(4, cycles=4)

    results = []
    for ts, reading in readings:
        r = detector.process_sample(reading, ts)
        if r is not None:
            results.append(r)

    assert 4 in results, f"Expected flash code 4, got {results}"
    assert detector.confirmed_code == 4
    print("PASS: High temperature shutdown (code 4)")


def test_valve_failure():
    """Test detection of flash code 7 (Gas Control / Valve Failure)."""
    detector = FlashDetector(threshold=100, baseline=20)
    detector.boot_time = 0.0
    readings = generate_flash_pattern(7, cycles=4)

    results = []
    for ts, reading in readings:
        r = detector.process_sample(reading, ts)
        if r is not None:
            results.append(r)

    assert 7 in results, f"Expected flash code 7, got {results}"
    assert detector.confirmed_code == 7
    print("PASS: Valve failure (code 7)")


def test_pilot_out():
    """Test detection of flash code 0 (Pilot Out) — no flashes for >10 seconds."""
    detector = FlashDetector(threshold=100, baseline=20)
    detector.boot_time = 0.0

    # Feed 25 seconds of baseline (no flashes) — needs two timeout cycles to confirm
    t = 0.0
    results = []
    while t < 25.0:
        r = detector.process_sample(20, t)
        if r is not None:
            results.append(r)
        t += 0.05

    assert 0 in results, f"Expected flash code 0 (pilot out), got {results}"
    assert detector.confirmed_code == 0
    print("PASS: Pilot out (code 0)")


def test_code_transition():
    """Test transitioning from code 1 to code 4."""
    detector = FlashDetector(threshold=100, baseline=20)
    detector.boot_time = 0.0

    # First: code 1
    readings1 = generate_flash_pattern(1, cycles=4)
    for ts, reading in readings1:
        detector.process_sample(reading, ts)

    assert detector.confirmed_code == 1, f"Expected code 1, got {detector.confirmed_code}"

    # Transition to code 4
    base_time = readings1[-1][0] + 0.05
    readings4 = generate_flash_pattern(4, cycles=4)

    results = []
    for ts, reading in readings4:
        r = detector.process_sample(reading, ts + base_time)
        if r is not None:
            results.append(r)

    assert 4 in results, f"Expected transition to code 4, got {results}"
    assert detector.confirmed_code == 4
    print("PASS: Code transition (1 -> 4)")


def test_boot_then_flashes():
    """Test the real boot scenario: darkness triggers pilot-out, then flashes begin.

    This is the exact sequence that happens on the real device:
    1. Boot (t=0)
    2. 12 seconds of darkness -> pilot-out (code 0) detected
    3. Flashes start -> should transition to code 1
    """
    detector = FlashDetector(threshold=100, baseline=20)
    detector.boot_time = 0.0

    # Phase 1: 12 seconds of darkness (triggers pilot-out)
    t = 0.0
    results = []
    while t < 25.0:  # Need 25s for two timeout cycles to confirm code 0
        r = detector.process_sample(20, t)
        if r is not None:
            results.append(('pilot_out', r))
        t += 0.05

    assert detector.confirmed_code == 0, f"Expected pilot-out first, got {detector.confirmed_code}"

    # Phase 2: Flashes begin (code 1 = 1 flash per second, 3s pause)
    base_time = t
    readings = generate_flash_pattern(1, cycles=5)
    for ts, reading in readings:
        r = detector.process_sample(reading, ts + base_time)
        if r is not None:
            results.append(('flash', r))

    assert detector.confirmed_code == 1, \
        f"Expected code 1 after flashes, got {detector.confirmed_code}. Events: {results}"
    print("PASS: Boot -> pilot out -> flashes detected (code 0 -> 1)")


def test_pilot_out_then_recovery():
    """Test pilot goes out (code 0) then comes back (code 1)."""
    detector = FlashDetector(threshold=100, baseline=20)
    detector.boot_time = 0.0

    # Establish code 1 first
    readings = generate_flash_pattern(1, cycles=4)
    for ts, reading in readings:
        detector.process_sample(reading, ts)
    assert detector.confirmed_code == 1

    # Pilot goes out: 25 seconds of darkness
    base_time = readings[-1][0] + 0.05
    t = base_time
    while t < base_time + 25.0:
        detector.process_sample(20, t)
        t += 0.05
    assert detector.confirmed_code == 0, f"Expected pilot out, got {detector.confirmed_code}"

    # Pilot relit: flashes resume
    base_time2 = t
    readings2 = generate_flash_pattern(1, cycles=5)
    for ts, reading in readings2:
        detector.process_sample(reading, ts + base_time2)
    assert detector.confirmed_code == 1, f"Expected recovery to code 1, got {detector.confirmed_code}"
    print("PASS: Code 1 -> pilot out -> recovery to code 1")


def test_noisy_readings():
    """Test that noise below threshold doesn't trigger false detections."""
    detector = FlashDetector(threshold=100, baseline=20)
    detector.boot_time = 0.0

    # First establish code 1
    readings = generate_flash_pattern(1, cycles=4)
    for ts, reading in readings:
        detector.process_sample(reading, ts)

    assert detector.confirmed_code == 1

    # Now feed noisy readings (below threshold) mixed with code 1 pattern
    base_time = readings[-1][0] + 0.05
    noisy_readings = generate_flash_pattern(1, cycles=4)

    # Add noise to some baseline readings (but stay below threshold)
    import random
    random.seed(42)
    noisy = []
    for ts, reading in noisy_readings:
        noise = random.randint(-10, 50) if reading < 100 else 0
        noisy.append((ts, max(0, reading + noise)))

    for ts, reading in noisy:
        detector.process_sample(reading, ts + base_time)

    assert detector.confirmed_code == 1, f"Noise caused false detection: {detector.confirmed_code}"
    print("PASS: Noisy readings don't cause false detections")


def test_motion_sensing_defaults_off():
    """CSI crashes the board inside the Wi-Fi driver, so a fresh config must not enable it."""
    with open(os.path.join(SKETCH_DIR, "water_heater_monitor.ino")) as f:
        assert 'g_prefs.getUChar("moMode", 0)' in f.read()


def test_compile():
    """Test that the Arduino sketch compiles without errors."""
    result = subprocess.run(
        ["arduino-cli", "compile", "--fqbn", FQBN, SKETCH_DIR],
        capture_output=True, text=True, timeout=180
    )
    if result.returncode != 0:
        # Print stderr which contains the actual error messages
        print(f"FAIL: Sketch compilation failed!\n")
        print("--- Compiler Output ---")
        print(result.stderr)
        if result.stdout:
            print(result.stdout)
        print("--- End Output ---")
        assert False, "Sketch failed to compile"
    else:
        # Extract size info from stdout
        for line in result.stdout.splitlines():
            if "bytes" in line:
                print(f"  {line.strip()}")
    print("PASS: Sketch compiles successfully")


if __name__ == "__main__":
    print("=== Water Heater Monitor Tests ===\n")

    print("-- Compile Check --")
    test_compile()

    print("\n-- Flash Detection Algorithm --")
    test_normal_operation()
    test_thermopile_low()
    test_high_temp_shutdown()
    test_valve_failure()
    test_pilot_out()
    test_code_transition()
    test_boot_then_flashes()
    test_pilot_out_then_recovery()
    test_noisy_readings()

    print("\n=== All tests passed ===")
