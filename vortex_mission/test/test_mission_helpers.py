"""Offline tests for geo, rc_edge and approach (no ROS needed)."""

import math

import numpy as np
import pytest

from vortex_mission.approach import (face_yaw_delta, keepout_disc_points, plan_approach)
from vortex_mission.geo import latlon_to_meter_offset, meter_offset_to_latlon
from vortex_mission.orbit_geometry import OrbitParams, body_distance
from vortex_mission.rc_edge import RcEdgeDetector

from test_orbit_geometry import add_box, casualty_grid


def test_latlon_round_trip_under_1cm():
    lat0, lon0 = 28.7499, 77.1170          # DTU
    for east, north in [(0, 0), (123.4, -56.7), (-300.0, 250.0)]:
        lat, lon = meter_offset_to_latlon(lat0, lon0, east, north)
        e2, n2 = latlon_to_meter_offset(lat0, lon0, lat, lon)
        assert math.hypot(e2 - east, n2 - north) < 0.01


def test_rc_single_flick_fires_once():
    d = RcEdgeDetector()
    seq = [1000, 1000, 1900, 1900, 1900, 1000, 1900]
    fired = [d.update(p, t * 0.5) for t, p in enumerate(seq)]
    assert fired == [False, False, True, False, False, False, True]


def test_rc_switch_high_at_boot_does_not_fire():
    d = RcEdgeDetector()
    assert not d.update(1900, 0.0)
    assert not d.update(1900, 1.0)


def test_rc_noise_in_band_and_debounce():
    d = RcEdgeDetector(debounce_s=0.3)
    assert not d.update(1000, 0.0)
    assert d.update(1800, 0.1)
    assert not d.update(1500, 0.12)        # hysteresis band: no state change
    assert not d.update(1000, 0.15)
    assert not d.update(1800, 0.2)         # within debounce of the last flick
    assert not d.update(0, 0.3)            # lost channel


def test_face_yaw_delta():
    assert face_yaw_delta(3.3, 0.0, 0.0, 0.0, 0.0) == pytest.approx(math.pi)
    assert face_yaw_delta(0.0, 3.3, 0.0, 0.0, 0.0) == pytest.approx(-math.pi / 2)


def test_keepout_disc_is_filled():
    pts = keepout_disc_points([(5.0, 5.0)], 2.3)
    r = np.hypot(pts[:, 0] - 5.0, pts[:, 1] - 5.0)
    assert r.max() <= 2.3 + 1e-6
    assert (r < 0.1).any(), 'disc must be filled, not a ring'


def test_full_circle_candidates_nearest_first_and_safe():
    p = OrbitParams()
    plan = plan_approach(casualty_grid(), 0.0, 0.0, 10.0, 0.0, p, 3.3)
    assert plan.circle_free
    assert len(plan.candidates) == 16
    first = plan.candidates[0]
    assert abs(math.atan2(math.sin(first.theta), math.cos(first.theta))) < 0.2
    for c in plan.candidates:
        # parked facing the casualty: real front bumper ~2.9 m, never inside the keepout
        assert body_distance(0, 0, c.x, c.y, c.yaw, 0.8, 0.7) == pytest.approx(2.9, abs=1e-6)


def test_partial_circle_parks_at_ccw_end():
    p = OrbitParams()
    g = casualty_grid()
    add_box(g, -6.0, 2.6, 6.0, 7.0)          # wall north blocks ~46..134 deg
    plan = plan_approach(g, 0.0, 0.0, 0.0, -10.0, p, 3.3)
    assert not plan.circle_free and plan.arcs
    first = math.degrees(plan.candidates[0].theta) % 360.0
    assert 15.0 < first < 60.0, first
    # later candidates step clockwise along the arc
    second = math.degrees(plan.candidates[1].theta) % 360.0
    assert (first - second) % 360.0 == pytest.approx(15.0, abs=1e-6)
