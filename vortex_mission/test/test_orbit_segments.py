import math

from vortex_mission.orbit_segments import SweepTracker


def walk(tracker, start_deg, end_deg, step_deg=2.0, r=3.3, cx=12.0, cy=-4.0):
    """Feed poses on a circle from start_deg to end_deg; return the angles where a boundary fired."""
    fired = []
    n = int(abs(end_deg - start_deg) / step_deg)
    sign = 1.0 if end_deg >= start_deg else -1.0
    for k in range(n + 1):
        a = math.radians(start_deg + sign * k * step_deg)
        if tracker.update(cx + r * math.cos(a), cy + r * math.sin(a)):
            fired.append(start_deg + sign * k * step_deg)
    return fired


def test_clockwise_full_circle_gives_four_segments():
    # boundaries at 90/180/270 deg swept -> 4 segments; the 4th is closed by the global stop
    # at the end of the orbit, not by a boundary (that would open an empty 5th segment)
    t = SweepTracker(12.0, -4.0, 90.0)
    fired = walk(t, 180.0, 180.0 - 360.0)          # clockwise, crosses +-180 deg
    assert [round(f) for f in fired] == [90, 0, -90]
    assert abs(t.swept_deg - 360.0) < 1e-6


def test_counter_clockwise_and_wrap():
    t = SweepTracker(12.0, -4.0, 90.0)
    fired = walk(t, 170.0, 170.0 + 200.0)
    assert len(fired) == 2


def test_jump_is_ignored():
    t = SweepTracker(0.0, 0.0, 90.0)
    t.update(3.0, 0.0)
    assert not t.update(-3.0, 0.0)                 # 180 deg in one step: a glitch
    assert t.swept_deg == 0.0


def test_no_boundary_before_a_full_segment():
    t = SweepTracker(0.0, 0.0, 90.0)
    assert walk(t, 0.0, 88.0, cx=0.0, cy=0.0) == []
