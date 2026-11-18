"""Property-based tests (Hypothesis) of the firmware's decision logic.

Every test drives detectors.h itself, compiled for the host. Each property is
a sentence about behaviour that must hold for ALL inputs in its domain, not
an example; Hypothesis searches the domain for a counterexample and shrinks
any it finds to the smallest one.
"""

import random

import pytest
from hypothesis import HealthCheck, assume, given, settings, strategies as st

from detectors_ffi import (CE_ENDED, CE_STARTED, EDGE_FALL, EDGE_NONE, EDGE_RISE, U32,
                           Cycles, Flash, Hourly, alert_edge, calibration)
from sim import DARK, LED, THRESHOLD, pattern_schedule, run_light, sample_times

SETTINGS = settings(max_examples=150, deadline=None,
                    suppress_health_check=[HealthCheck.too_slow, HealthCheck.function_scoped_fixture])

# Resideo pattern: N flashes ~1/s, ~3 s pause. Measured on this valve
# 2026-11-18 (code 1): flashes ~150-250 ms, one every 3.0-3.2 s. Timing varies
# by valve model, so the domain is wider than that.
flash_timing = st.fixed_dictionaries({
    "on_ms": st.integers(120, 600),
    "off_ms": st.integers(350, 900),
    "pause_ms": st.integers(2600, 6000),
})
codes = st.sampled_from([1, 2, 4, 5, 7, 8])
any_count = st.integers(1, 9)
seeds = st.integers(0, 2**32 - 1)
# Start anywhere in millis(), including just before the 49.7-day wrap
starts = st.one_of(st.integers(20_000, 10**8), st.integers(U32 - 120_000, U32 - 1))


def schedule(code, start, repeats, t):
    return pattern_schedule(code, start, repeats, on_ms=t["on_ms"],
                            period_ms=t["on_ms"] + t["off_ms"], pause_ms=t["pause_ms"])


def settled(lib, code, start, t, rng):
    """A detector that has already confirmed `code`; returns (det, time after)."""
    det = Flash(lib, threshold=THRESHOLD, boot_ms=start - 20_000)
    iv, end = schedule(code, start, 3, t)
    run_light(det, iv, sample_times(start, end, rng), rng)
    assert det.confirmed == code
    return det, end


# ---------------------------------------------------------------------------
# Flash codes
# ---------------------------------------------------------------------------

@SETTINGS
@given(code=any_count, t=flash_timing, seed=seeds, start=starts)
def test_any_pattern_is_read_correctly_from_boot(lib, code, t, seed, start):
    """From a fresh boot, N flashes per pattern confirms code N, with no
    other code reported on the way (counts outside the table included)."""
    rng = random.Random(seed)
    det = Flash(lib, threshold=THRESHOLD, boot_ms=start)
    iv, end = schedule(code, start, 4, t)
    changes = run_light(det, iv, sample_times(start, end, rng), rng, noise=(-50, 300))
    assert [c for _, c in changes] == [code]


@SETTINGS
@given(old=codes, new=codes, t=flash_timing, seed=seeds, start=starts)
def test_code_change_is_reported_once_and_correctly(lib, old, new, t, seed, start):
    """A valve going from code A to code B reports B exactly once, within
    three patterns, and never a code that wasn't shown."""
    assume(old != new)
    rng = random.Random(seed)
    det, t0 = settled(lib, old, start, t, rng)
    iv, end = schedule(new, t0, 3, t)
    changes = run_light(det, iv, sample_times(t0, end, rng), rng, noise=(-50, 300))
    assert [c for _, c in changes] == [new]


@SETTINGS
@given(code=codes, t=flash_timing, seed=seeds, start=starts,
       where=st.floats(0, 1), level=st.sampled_from([0, LED]))
def test_one_bad_sample_never_changes_the_code(lib, code, t, seed, start, where, level):
    """One glitched reading (a spike, or a dropout in the middle of a flash)
    can corrupt at most one pattern, and one pattern can't change the code."""
    rng = random.Random(seed)
    det, t0 = settled(lib, code, start, t, rng)
    iv, end = schedule(code, t0, 4, t)
    times = sample_times(t0, end, rng)
    changes = run_light(det, iv, times, rng, glitches={int(where * (len(times) - 1)): level})
    assert changes == []


@SETTINGS
@given(code=codes, t=flash_timing, seed=seeds, start=starts)
def test_pilot_out_is_reported_after_20s_of_darkness_and_not_before(lib, code, t, seed, start):
    """LED goes dark: code 0 is confirmed, no earlier than 20 s after the last
    flash (two 10 s windows) and no later than 24 s."""
    rng = random.Random(seed)
    det, t0 = settled(lib, code, start, t, rng)
    last_flash_start = t0 - t["pause_ms"] - t["on_ms"]
    changes = run_light(det, [], sample_times(t0, t0 + 40_000, rng), rng, noise=(-50, 300))
    assert [c for _, c in changes] == [0]
    when = changes[0][0] - last_flash_start
    assert 20_000 <= when <= 24_000, when


@SETTINGS
@given(code=codes, t=flash_timing, seed=seeds, start=starts,
       minutes=st.integers(1, 30), level=st.integers(THRESHOLD + 1, 30_000))
def test_steady_light_means_unknown_never_pilot_out(lib, code, t, seed, start, minutes, level):
    """A room light or flashlight over the sensor hides the LED. That must
    read as unknown (-1) after 60 s, never as pilot out (0)."""
    rng = random.Random(seed)
    det, t0 = settled(lib, code, start, t, rng)
    end = t0 + minutes * 60_000
    iv, _ = schedule(code, t0, 2 + minutes * 60 // 5, t)   # LED keeps flashing underneath
    changes = run_light(det, iv, sample_times(t0, end, rng), steady=(t0, end, level))
    assert 0 not in [c for _, c in changes]
    if minutes >= 2:
        assert det.confirmed == -1


stall_patterns = st.one_of(
    # one-off stalls anywhere
    st.lists(st.tuples(st.integers(0, 120_000), st.integers(1_600, 30_000)), min_size=1, max_size=8),
    # periodic: an HA push round every `every` ms, each blocking `length` ms
    # (7 entities x 2 s timeout = 14 s on a dead link)
    st.tuples(st.integers(10_000, 40_000), st.integers(1_600, 20_000)).map(
        lambda p: [(k * p[0], min(p[1], p[0] - 200)) for k in range(200_000 // p[0])]),
)


@SETTINGS
@given(t=flash_timing, seed=seeds, start=starts, stalls=stall_patterns)
def test_loop_stalls_never_fake_a_pilot_out(lib, t, seed, start, stalls):
    """loop() blocking for >1.5 s (HA pushes on a bad link) means we weren't
    watching. With the LED flashing normally, no stall pattern yields code 0."""
    rng = random.Random(seed)
    det, t0 = settled(lib, 1, start, t, rng)
    iv, end = schedule(1, t0, 60, t)
    times = sample_times(t0, end, rng, stalls=[(t0 + a, n) for a, n in stalls])
    changes = run_light(det, iv, times, rng)
    assert 0 not in [c for _, c in changes]


@pytest.mark.xfail(strict=True, reason=(
    "FINDING: samples arriving 0.6-1.5 s apart (loop() chronically slow but "
    "under FLASH_SAMPLE_GAP_MS) can miss every 300 ms flash and confirm a "
    "false pilot out. Fix needs the LED's real on-time; see CHANGELOG."))
@SETTINGS
@given(t=flash_timing, seed=seeds, start=starts, gap=st.integers(600, 1_450))
def test_slow_sampling_never_fakes_a_pilot_out(lib, t, seed, start, gap):
    rng = random.Random(seed)
    det, t0 = settled(lib, 1, start, t, rng)
    iv, end = schedule(1, t0, 15, t)
    times = list(range(t0, end, gap))
    changes = run_light(det, iv, times, rng)
    assert 0 not in [c for _, c in changes]


@given(cfg=st.integers(0, 65535), baseline=st.integers(0, 65535))
def test_threshold_never_wraps(lib, cfg, baseline):
    """The auto threshold sits above the baseline for every baseline (the old
    uint16 arithmetic wrapped above 32742); a configured one is used as is."""
    thr = lib.fd_threshold(cfg, baseline)
    assert thr == (cfg if cfg > 0 else baseline * 2 + 50)


@given(avg=st.integers(0, 65535))
def test_calibration_never_saves_a_wrapped_threshold(lib, avg):
    ok, thr = calibration(lib, avg)
    assert ok == (avg * 2 + 50 <= 65535)
    if ok:
        assert thr == avg * 2 + 50 and thr > avg


@SETTINGS
@given(t=flash_timing, seed=seeds, start=starts, bright=st.integers(32_743, 37_888),
       bright_s=st.integers(15, 120))
def test_booting_into_bright_light_is_never_pilot_out(lib, t, seed, start, bright, bright_s):
    """Auto threshold (0), booted with a light on: the first reading becomes
    the baseline and 2*baseline+50 no longer fits in 16 bits. That is a
    blinded sensor, never pilot out; once the light goes off the baseline
    decays and code 1 is read normally."""
    rng = random.Random(seed)
    det = Flash(lib, threshold=0, boot_ms=start)
    t0 = start + bright_s * 1000
    for now in sample_times(start, t0, rng):
        det.step(now, bright)
        assert det.confirmed != 0
    iv, end = schedule(1, t0 + 3000, 4, t)
    changes = run_light(det, iv, sample_times(t0, end, rng), rng)
    assert [c for _, c in changes] == [1]


# ---------------------------------------------------------------------------
# Temperature readings and alerts
# ---------------------------------------------------------------------------

@given(c=st.floats(allow_nan=True, allow_infinity=True, width=32))
def test_reading_validity(lib, c):
    """Valid exactly when inside the DS18B20's range and not its 85.0 power-on value."""
    expect = (c == c) and -55.0 <= c <= 125.0 and c != 85.0
    assert bool(lib.temp_valid(c)) == expect


@given(values=st.lists(st.floats(100, 200, width=32), max_size=300),
       threshold=st.floats(120, 180, width=32), hyst=st.floats(0.5, 5, width=32))
def test_alerts_alternate_and_respect_hysteresis(lib, values, threshold, hyst):
    """Alerts go RISE, FALL, RISE... A RISE needs value >= threshold; a FALL
    needs value < threshold - hyst. So a reading hovering on the line can't flap."""
    active = False
    last = EDGE_FALL
    for v in values:
        e, active = alert_edge(lib, active, v, threshold, hyst)
        if e == EDGE_RISE:
            assert last == EDGE_FALL and v >= threshold
            last = e
        elif e == EDGE_FALL:
            assert last == EDGE_RISE and v < threshold - hyst
            last = e
        assert active == (last == EDGE_RISE)


@given(threshold=st.floats(120, 180, width=32), hyst=st.floats(0.5, 5, width=32),
       seq=st.lists(st.floats(0, 0.999), min_size=2, max_size=200))
def test_hovering_on_the_line_alerts_once(lib, threshold, hyst, seq):
    """Readings that never drop hyst below the threshold produce at most one RISE."""
    active, edges = False, 0
    for x in seq:
        v = threshold - hyst * x
        e, active = alert_edge(lib, active, v, threshold, hyst)
        edges += e != EDGE_NONE
    assert edges <= 1


# ---------------------------------------------------------------------------
# Heating cycles
# ---------------------------------------------------------------------------

temps = st.floats(90, 140)
intervals = st.sampled_from([5, 10, 15, 30])


@SETTINGS
@given(base=temps, noise=st.floats(0, 1.0), seed=seeds, interval=intervals,
       hours=st.integers(1, 6))
def test_noise_alone_never_starts_a_cycle(lib, base, noise, seed, interval, hours):
    """A flat pipe with up to +/-1F of noise never rises >2F, so no cycle."""
    rng = random.Random(seed)
    cd = Cycles(lib, interval)
    for i in range(hours * 3600 // interval):
        ev, _ = cd.feed(base + rng.uniform(-noise, noise), i * interval * 1000)
        assert ev != CE_STARTED


@SETTINGS
@given(base=temps, rate=st.floats(0.5, 1.5), heat_min=st.integers(6, 20),
       cool=st.floats(0.05, 0.3), noise=st.floats(0, 0.15), seed=seeds, interval=intervals,
       start=starts)
def test_a_real_burn_is_exactly_one_cycle(lib, base, rate, heat_min, cool, noise, seed,
                                          interval, start):
    """A sustained rise (>=0.5F/min, i.e. >2.5F per 5 min) followed by
    cooling is reported as exactly one cycle whose length is the burn
    within the detector's lag (it sees the rise late and the stop late)."""
    rng = random.Random(seed)
    cd = Cycles(lib, interval)
    events = []
    total_s = (10 + heat_min + 60) * 60
    for k in range(total_s // interval):
        s = k * interval
        m = s / 60
        if m < 10:
            temp = base
        elif m < 10 + heat_min:
            temp = base + rate * (m - 10)
        else:
            temp = base + rate * heat_min - cool * (m - 10 - heat_min)
        ev, dur = cd.feed(temp + rng.uniform(-noise, noise), start + s * 1000)
        if ev:
            events.append((ev, dur))
    assert [e for e, _ in events] == [CE_STARTED, CE_ENDED]
    dur = events[1][1]
    assert heat_min * 60 - 300 <= dur <= heat_min * 60 + 300 + interval, dur


@SETTINGS
@given(seq=st.lists(st.floats(60, 180), min_size=40, max_size=600), interval=intervals)
def test_cycles_are_well_formed_for_any_readings(lib, seq, interval):
    """For any readings at all: starts and ends alternate, every duration is
    no longer than the time actually elapsed, and they never sum to more."""
    cd = Cycles(lib, interval)
    active, started_at, total = False, None, 0
    for i, v in enumerate(seq):
        now = i * interval * 1000
        ev, dur = cd.feed(v, now)
        if ev == CE_STARTED:
            assert not active
            active, started_at = True, now
        elif ev == CE_ENDED:
            assert active
            assert dur * 1000 <= now - started_at
            total += dur
            active = False
        assert cd.active == active
    assert total * 1000 <= len(seq) * interval * 1000


@pytest.mark.xfail(strict=True, reason=(
    "FINDING / spec question: one reading >2F above the last 5 minutes starts "
    "a cycle and the next normal reading ends it, logging a ~10 s phantom "
    "cycle. A pipe can't jump 2F in one reading, so starting a cycle could "
    "require the rise on 2 consecutive readings. Not changed without sign-off."))
@SETTINGS
@given(base=temps, spike=st.floats(2.1, 60), at=st.integers(31, 200), interval=intervals)
def test_one_outlier_reading_never_makes_a_cycle(lib, base, spike, at, interval):
    cd = Cycles(lib, interval)
    for i in range(260):
        ev, _ = cd.feed(base + (spike if i == at else 0), i * interval * 1000)
        assert ev != CE_STARTED


# ---------------------------------------------------------------------------
# Hourly heat buckets
# ---------------------------------------------------------------------------

@SETTINGS
@given(start=st.integers(400_000, 600_000),
       ops=st.lists(st.tuples(st.integers(0, 6), st.integers(0, 3600)), max_size=120))
def test_hourly_buckets_hold_exactly_the_last_48_hours(lib, start, ops):
    """Bucket i holds the minutes of cycles that ended i hours ago, for every
    sequence of hour gaps (0 = same hour) and cycle lengths."""
    hb = Hourly(lib)
    hour = start
    by_hour = {}
    for gap, dur in ops:
        hour += gap
        hb.add(hour, dur)
        by_hour[hour] = by_hour.get(hour, 0) + dur / 60
    for i in range(48):
        assert hb.at(i) == pytest.approx(by_hour.get(hour - i, 0), rel=1e-5, abs=1e-3)


@SETTINGS
@given(start=st.integers(400_000, 600_000), back=st.integers(1, 1000),
       durs=st.lists(st.integers(60, 3600), min_size=1, max_size=20))
def test_clock_stepping_back_never_erases_history(lib, start, back, durs):
    """An NTP correction that moves the clock back across an hour boundary
    must not clear the chart (it used to wipe all 48 buckets)."""
    hb = Hourly(lib)
    for k, d in enumerate(durs):
        hb.add(start + k, d)
    before = hb.total()
    hb.advance(start + len(durs) - 1 - back)
    assert hb.total() == pytest.approx(before)
    hb.add(start + len(durs) - 1 - back, 60)        # a cycle in the "past" hour
    assert hb.total() == pytest.approx(before + 1)


# ---------------------------------------------------------------------------
# Sample-gap diagnostics (/api/debug "sampling", heartbeat gap_* fields)
# ---------------------------------------------------------------------------

GAP_BLIND_MS = 250


@SETTINGS
@given(start=starts, gaps=st.lists(st.integers(1, 40_000), max_size=300),
       recover_at=st.sets(st.integers(0, 299), max_size=5),
       level=st.sampled_from([DARK, LED]))
def test_gap_stats_account_for_every_gap(lib, start, gaps, recover_at, level):
    """Every gap between consecutive readings lands in exactly one bucket (by
    its bounds), the max is the longest gap, and blind time is the sum of
    gaps over 250 ms. A sensor recovery starts afresh and records no gap."""
    det = Flash(lib, threshold=THRESHOLD, boot_ms=start)
    bounds = [lib.gap_bucket_max(i) for i in range(lib.gap_buckets() - 1)]
    expect = [0] * (len(bounds) + 1)
    now, recorded = start, []
    det.step(now, level)
    for i, g in enumerate(gaps):
        if i in recover_at:
            det.forget_last_sample()
        else:
            recorded.append(g)
            expect[sum(g > b for b in bounds)] += 1
        now += g
        det.step(now, level)
    s = det.gap_stats()
    assert s["hist"] == expect
    assert s["max"] == max(recorded, default=0)
    assert s["blind_ms"] == sum(g for g in recorded if g > GAP_BLIND_MS)
    assert s["blind_count"] == sum(g > GAP_BLIND_MS for g in recorded)
    assert s["blind_waiting"] <= sum(g > GAP_BLIND_MS for g in recorded)


@SETTINGS
@given(t=flash_timing, seed=seeds, start=starts)
def test_normal_sampling_is_never_blind(lib, t, seed, start):
    """At the firmware's normal pace (120 ms conversion + up to 40 ms of loop)
    no gap is long enough to miss a flash."""
    rng = random.Random(seed)
    det, _ = settled(lib, 1, start, t, rng)
    s = det.gap_stats()
    assert s["blind_ms"] == 0 and s["max"] <= 160
