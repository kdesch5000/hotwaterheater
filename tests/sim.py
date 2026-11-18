"""Light-sensor and pipe-temperature simulators for the detector tests.

The light model follows the hardware: the TSL2591 integrates for ~100 ms and
the firmware reads it ~120 ms after starting a conversion, so a reading is
the dark level plus the LED level scaled by how much of the integration
window the LED was on. A short flash can therefore land as a partial reading.
"""

INTEGRATION_MS = 100
CONVERT_MS = 120          # TSL_CONVERT_MS in the firmware

DARK = 300                # Typical baseline at 428x gain
LED = 15000               # Typical LED peak at 428x gain
THRESHOLD = 1000          # Typical calibrated threshold


def pattern_schedule(code, start_ms, repeats, on_ms=300, period_ms=1000, pause_ms=3000):
    """LED-on intervals for `repeats` patterns of `code` flashes.

    Returns (intervals, end_ms). pause_ms runs from the end of the last flash
    of one pattern to the start of the first flash of the next.
    """
    intervals = []
    t = start_ms
    for _ in range(repeats):
        for k in range(code):
            intervals.append((t, t + on_ms))
            t += on_ms if k == code - 1 else period_ms
        t += pause_ms
    return intervals, t


def led_overlap(intervals, a, b, cursor=0):
    """Milliseconds of [a, b) covered by intervals (sorted); returns (ms, cursor)."""
    while cursor < len(intervals) and intervals[cursor][1] <= a:
        cursor += 1
    on = 0
    j = cursor
    while j < len(intervals) and intervals[j][0] < b:
        s, e = intervals[j]
        on += max(0, min(e, b) - max(s, a))
        j += 1
    return on, cursor


def sample_times(start_ms, end_ms, rng, jitter_ms=(0, 40), stalls=()):
    """Sample timestamps: CONVERT_MS plus loop jitter apart, skipping stalls.

    stalls: iterable of (at_ms, length_ms) during which loop() is blocked.
    """
    stalls = sorted(stalls)
    t = start_ms
    out = []
    si = 0
    while t < end_ms:
        out.append(t)
        t += CONVERT_MS + rng.randint(*jitter_ms)
        while si < len(stalls) and stalls[si][0] <= t:
            at, length = stalls[si]
            t = max(t, at + length)
            si += 1
    return out


def run_light(det, intervals, times, rng=None, noise=(0, 0), glitches=None, steady=None):
    """Feed det one reading per timestamp. Returns the list of confirmed codes seen
    (one entry per change, with the time of the change).

    glitches: {sample_index: reading} overrides individual samples.
    steady:   (from_ms, to_ms, level) — a constant extra light source.
    """
    changes = []
    cursor = 0
    prev = det.confirmed
    for i, t in enumerate(times):
        on, cursor = led_overlap(intervals, t - CONVERT_MS, t - CONVERT_MS + INTEGRATION_MS, cursor)
        reading = DARK + LED * on // INTEGRATION_MS
        if rng is not None and noise != (0, 0):
            reading += rng.randint(*noise)
        if steady and steady[0] <= t < steady[1]:
            reading += steady[2]
        if glitches and i in glitches:
            reading = glitches[i]
        det.step(t, max(0, min(65535, reading)))
        if det.confirmed != prev:
            changes.append((t, det.confirmed))
            prev = det.confirmed
    return changes
