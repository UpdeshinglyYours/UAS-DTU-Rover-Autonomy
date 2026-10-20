"""Offline tests for orbit_geometry: run with `python3 -m pytest test/` (no ROS needed)."""

import math

import numpy as np
import pytest

from vortex_mission.orbit_geometry import (
    LETHAL, Grid, OrbitParams, TrackerGains, blended_target, body_distance,
    clockwise_start_theta, footprint_collides, footprint_sample_points, free_arcs,
    profile_at, radius_profile, sample_angles, theta_to_index, tracker_command, wrap_angle)

CX, CY = 0.0, 0.0
REAL_L, REAL_W = 0.80, 0.70


def make_grid(res=0.1, half=10.0):
    n = int(2 * half / res)
    return Grid(res, -half, -half, np.zeros((n, n), dtype=np.uint8))


def add_box(g, x0, y0, x1, y1):
    i0 = int((x0 - g.origin_x) / g.resolution)
    i1 = int(math.ceil((x1 - g.origin_x) / g.resolution))
    j0 = int((y0 - g.origin_y) / g.resolution)
    j1 = int(math.ceil((y1 - g.origin_y) / g.resolution))
    g.data[j0:j1, i0:i1] = LETHAL


def add_keepout_disc(g, cx, cy, r=2.3):
    ys, xs = np.mgrid[0:g.size_y, 0:g.size_x]
    wx = g.origin_x + (xs + 0.5) * g.resolution
    wy = g.origin_y + (ys + 0.5) * g.resolution
    g.data[(wx - cx) ** 2 + (wy - cy) ** 2 <= r * r] = LETHAL


def casualty_grid():
    g = make_grid()
    add_keepout_disc(g, CX, CY)       # what casualty_manager puts in the costmap
    return g


# ---------------------------------------------------------------- geometry

def test_body_distance_side_and_corner():
    # Rover centred 2.9 m east, heading north (clockwise tangent): its left side faces west.
    assert body_distance(0, 0, 2.9, 0, math.pi / 2, REAL_L, REAL_W) == pytest.approx(2.55)
    # Facing the casualty: front bumper.
    assert body_distance(0, 0, 3.2, 0, math.pi, REAL_L, REAL_W) == pytest.approx(2.8)
    # Point inside the body.
    assert body_distance(0.1, 0.1, 0, 0, 0.3, REAL_L, REAL_W) == 0.0


def test_footprint_collides_detects_box():
    g = make_grid()
    pts = footprint_sample_points(1.0, 0.9, g.resolution / 2)
    add_box(g, 4.0, -0.2, 4.4, 0.2)
    assert footprint_collides(g, 4.0, 0.0, 0.0, pts)
    assert not footprint_collides(g, 6.0, 0.0, 0.0, pts)


def test_wrap_angle():
    assert wrap_angle(3 * math.pi) == pytest.approx(math.pi)
    assert wrap_angle(-0.5) == pytest.approx(-0.5)


# ---------------------------------------------------------------- profile

def test_open_area_full_circle_at_orbit_radius():
    p = OrbitParams()
    thetas, prof = radius_profile(casualty_grid(), CX, CY, p)
    assert not np.isnan(prof).any()
    assert np.allclose(prof, p.radius)
    arcs = free_arcs(prof)
    assert len(arcs) == 1 and arcs[0].full


def test_small_box_gives_smooth_outward_detour():
    p = OrbitParams()
    g = casualty_grid()
    add_box(g, -0.25, 2.7, 0.25, 3.1)          # 0.5 m box sitting on the circle at theta=90 deg
    thetas, prof = radius_profile(g, CX, CY, p)
    assert not np.isnan(prof).any(), "a small box must be detoured, not block the circle"
    assert np.nanmin(prof) >= p.radius - 1e-9, "detours never go inward"
    assert np.nanmax(prof) <= p.max_detour_radius + 1e-9
    i90 = theta_to_index(math.pi / 2, len(thetas))
    assert prof[i90] > p.radius + 0.4, "the profile bulges out at the box"
    steps = np.abs(np.diff(np.concatenate([prof, prof[:1]])))
    assert steps.max() <= p.max_radius_slope + 1e-9, "detour is smooth"
    i270 = theta_to_index(3 * math.pi / 2, len(thetas))
    assert prof[i270] == pytest.approx(p.radius), "far side untouched"


def test_wall_blocks_part_of_circle():
    p = OrbitParams()
    g = casualty_grid()
    add_box(g, -6.0, 2.6, 6.0, 7.0)            # a long wall across the north side
    thetas, prof = radius_profile(g, CX, CY, p)
    arcs = free_arcs(prof)
    assert len(arcs) == 1 and not arcs[0].full
    assert 120.0 < arcs[0].span_deg() < 300.0
    i90 = theta_to_index(math.pi / 2, len(thetas))
    assert np.isnan(prof[i90])
    i270 = theta_to_index(3 * math.pi / 2, len(thetas))
    assert not np.isnan(prof[i270])


def test_two_walls_leave_one_clean_arc():
    """East and north walls: the south-west side is one clean arc. The narrow pocket
    between the walls only fits the rover at a detour radius and never touches the orbit
    circle, so it is dropped as unreachable (it used to show up as a second arc, and the
    smoothing leak used to split the main arc too)."""
    p = OrbitParams()
    g = casualty_grid()
    add_box(g, 2.6, -1.0, 7.0, 1.0)            # east
    add_box(g, -1.5, 2.6, 1.5, 7.0)            # north
    thetas, prof = radius_profile(g, CX, CY, p)
    arcs = free_arcs(prof)
    assert len(arcs) == 1
    assert 180.0 < arcs[0].span_deg() < 260.0
    assert arcs[0].indices()[0] == theta_to_index(math.radians(123), len(thetas))


def test_fully_enclosed_no_arc():
    p = OrbitParams()
    g = casualty_grid()
    add_keepout_disc(g, CX, CY, r=5.0)          # obstacles everywhere out to 5 m
    _, prof = radius_profile(g, CX, CY, p)
    assert free_arcs(prof) == []


def test_clockwise_start_is_ccw_end_of_arc():
    p = OrbitParams()
    g = casualty_grid()
    add_box(g, -6.0, 2.6, 6.0, 7.0)            # wall north
    thetas, prof = radius_profile(g, CX, CY, p)
    arc = free_arcs(prof)[0]
    start = clockwise_start_theta(arc, thetas, p.edge_margin_deg)
    # The wall blocks roughly 46..134 deg. The free arc runs CCW from ~134 deg round to
    # ~43 deg, so a clockwise sweep must start at its CCW end, just below the wall's east side.
    s = math.degrees(start) % 360.0
    assert 15.0 < s < 60.0
    # Sweeping clockwise from there stays inside the arc for its whole length.
    idx = theta_to_index(start, len(thetas))
    assert idx in arc.indices()


# ---------------------------------------------------------------- tracker

def simulate(prof, thetas, start_theta, standby=3.2, sweep_deg=360.0, v=0.3, spiral_deg=25.0,
             dt=0.05, clockwise=True, noise=0.0, seed=0, grid=None):
    """Kinematic unicycle along one leg. Returns (min real body distance, max |e| after spiral, swept).

    If `grid` is given, also asserts the real footprint never covers a lethal cell
    other than the casualty's own keepout disc.
    """
    rng = np.random.default_rng(seed)
    real_pts = footprint_sample_points(REAL_L, REAL_W, 0.05)
    x = CX + standby * math.cos(start_theta)
    y = CY + standby * math.sin(start_theta)
    yaw = start_theta - math.pi / 2 if clockwise else start_theta + math.pi / 2
    g = TrackerGains()
    swept, prev_th = 0.0, start_theta
    total = math.radians(sweep_deg)
    spiral = math.radians(spiral_deg)
    min_body, max_err = float('inf'), 0.0
    for _ in range(int(4 * total * 3.0 / v / dt)):
        th = math.atan2(y - CY, x - CX)
        swept += abs(wrap_angle(th - prev_th))
        prev_th = th
        if swept >= total:
            break
        r_p, dr_p = profile_at(thetas, prof, th)
        r_t, dr_t = blended_target(r_p, dr_p, standby, swept, total - swept, spiral, clockwise)
        vv, w, e = tracker_command(x, y, yaw, CX, CY, r_t, dr_t, clockwise, v, g)
        if spiral < swept < total - spiral:
            max_err = max(max_err, abs(e))
        x += vv * math.cos(yaw) * dt + rng.normal(0, noise)
        y += vv * math.sin(yaw) * dt + rng.normal(0, noise)
        yaw += w * dt
        min_body = min(min_body, body_distance(CX, CY, x, y, yaw, REAL_L, REAL_W))
        if grid is not None:
            assert not footprint_collides(grid, x, y, yaw, real_pts), \
                f"real footprint hit an obstacle at ({x:.2f}, {y:.2f})"
    return min_body, max_err, math.degrees(swept)


def test_tracker_full_circle_keeps_keepout():
    p = OrbitParams()
    thetas, prof = radius_profile(casualty_grid(), CX, CY, p)
    min_body, max_err, swept = simulate(prof, thetas, start_theta=1.0, noise=0.002)
    assert swept >= 359.0
    assert min_body >= 2.4, f"real body came to {min_body:.3f} m"
    assert max_err < 0.1, f"radial error {max_err:.3f} m"


def test_tracker_detour_around_small_box():
    p = OrbitParams()
    g = casualty_grid()
    add_box(g, -0.25, 2.7, 0.25, 3.1)
    thetas, prof = radius_profile(g, CX, CY, p)
    # simulate() fails the test if the real footprint ever touches the box (or the disc).
    min_body, _, swept = simulate(prof, thetas, start_theta=math.radians(200.0), grid=g)
    assert swept >= 359.0
    assert min_body >= 2.4


def test_tracker_counter_clockwise_and_reverse_signs():
    p = OrbitParams()
    thetas, prof = radius_profile(casualty_grid(), CX, CY, p)
    min_body, max_err, swept = simulate(prof, thetas, start_theta=0.0, clockwise=False,
                                        sweep_deg=180.0)
    assert swept >= 179.0 and min_body >= 2.4 and max_err < 0.1
    # Backing out (v < 0) while off the path must steer back towards it.
    _, w_fwd, _ = tracker_command(3.2, 0.0, -math.pi / 2, 0, 0, 2.9, 0.0, True, 0.3, TrackerGains())
    _, w_rev, _ = tracker_command(3.2, 0.0, -math.pi / 2, 0, 0, 2.9, 0.0, True, -0.3, TrackerGains())
    assert w_fwd < 0 < w_rev


def test_sample_angles_count():
    assert len(sample_angles(3.0)) == 120


def test_orbit_from_far_standby_uses_longer_spiral():
    """Regression: Nav2's 0.5 m goal tolerance can leave the rover at 3.75 m. A fixed
    25 deg spiral would cut inward ~40 deg and swing the corner inside 2.3 m; the
    adaptive spiral keeps the body clear."""
    from vortex_mission.orbit_geometry import spiral_angle
    p = OrbitParams()
    thetas, prof = radius_profile(casualty_grid(), CX, CY, p)
    start_r = 3.75
    spiral = spiral_angle(start_r - p.radius)
    assert spiral > math.radians(60)
    g = TrackerGains()
    th0 = math.pi
    x, y, yaw = start_r * math.cos(th0), start_r * math.sin(th0), th0 - math.pi / 2
    swept, prev, total, min_body = 0.0, th0, 2 * math.pi, float('inf')
    for _ in range(40000):
        th = math.atan2(y, x)
        swept += abs(wrap_angle(th - prev))
        prev = th
        if swept >= total:
            break
        r_p, dr_p = profile_at(thetas, prof, th)
        r_t, dr_t = blended_target(r_p, dr_p, start_r, swept, total - swept, spiral, True,
                                   end_radius=3.3, spiral_out=spiral_angle(3.3 - p.radius))
        v, w, _ = tracker_command(x, y, yaw, CX, CY, r_t, dr_t, True, 0.3, g)
        x += v * math.cos(yaw) * 0.05
        y += v * math.sin(yaw) * 0.05
        yaw += w * 0.05
        min_body = min(min_body, body_distance(CX, CY, x, y, yaw, REAL_L, REAL_W))
    assert swept >= total - 0.02
    assert min_body >= 2.4, min_body


def add_hollow_box(g, x0, y0, x1, y1):
    """Only the outline is lethal: what a lidar-fed costmap holds for a big obstacle."""
    add_box(g, x0, y0, x1, y0 + 0.1)
    add_box(g, x0, y1 - 0.1, x1, y1)
    add_box(g, x0, y0, x0 + 0.1, y1)
    add_box(g, x1 - 0.1, y0, x1, y1)


def test_hollow_wall_no_island_and_no_leak():
    """Regression (sim, 2026-10-11): the empty inside of a hollow wall looked like a
    detour 'island'; its large radius leaked through the blocked stretch into the real
    arc, and the resulting ramps fragmented the arc so the orbit refused to move."""
    p = OrbitParams()
    g = casualty_grid()
    add_hollow_box(g, -6.0, 2.6, 6.0, 7.0)
    thetas, prof = radius_profile(g, CX, CY, p)
    arcs = free_arcs(prof)
    assert len(arcs) == 1, [(round(math.degrees(thetas[a.start])), a.span_deg()) for a in arcs]
    assert arcs[0].span_deg() > 200.0
    i180 = theta_to_index(math.pi, len(thetas))
    assert prof[i180] == pytest.approx(p.radius), 'no ramp leaked onto the free side'
    assert np.isnan(prof[theta_to_index(math.pi / 2, len(thetas))])


def test_envelope_does_not_cross_blocked_samples():
    from vortex_mission.orbit_geometry import _outward_envelope
    raw = np.array([2.9, 2.9, np.nan, 4.0, 4.0, np.nan, 2.9, 2.9])
    env = _outward_envelope(raw, 0.05)
    assert env[0] == pytest.approx(2.9) and env[7] == pytest.approx(2.9)
    assert env[1] == pytest.approx(2.9) and env[6] == pytest.approx(2.9)


# ---------------------------------------------------------------- real drivetrain

def drivetrain(v_c, w_c, v, w, dt, v_dead=0.38, w_dead=0.28, tau_v=0.4, tau_w=0.35):
    """Model of this rover from nav2_params4 + new_omega_fixed5's tables: commands below
    ~0.4 m/s / ~0.3 rad/s are not executed, and it lags ~1 s (first-order taus)."""
    v_t = v_c if abs(v_c) >= v_dead else 0.0
    w_t = w_c if abs(w_c) >= w_dead else 0.0
    v += (v_t - v) * min(1.0, dt / tau_v)
    w += (w_t - w) * min(1.0, dt / tau_w)
    return v, w


def lap_with_drivetrain(grid, speed, start_r=3.3, dt=0.02):
    p = OrbitParams()
    thetas, prof = radius_profile(grid, CX, CY, p)
    g = TrackerGains()
    th0 = math.pi
    x, y, yaw = start_r * math.cos(th0), start_r * math.sin(th0), th0 - math.pi / 2
    va = wa = 0.0
    from vortex_mission.orbit_geometry import spiral_angle
    sp_in, sp_out = spiral_angle(start_r - p.radius), spiral_angle(3.3 - p.radius)
    swept, prev, total, t, min_body = 0.0, th0, 2 * math.pi, 0.0, float('inf')
    while swept < total and t < 200:
        th = math.atan2(y, x)
        swept += abs(wrap_angle(th - prev))
        prev = th
        r_p, dr_p = profile_at(thetas, prof, th)
        r_t, dr_t = blended_target(r_p, dr_p, start_r, swept, total - swept, sp_in, True,
                                   end_radius=3.3, spiral_out=sp_out)
        v_c, w_c, _ = tracker_command(x, y, yaw, CX, CY, r_t, dr_t, True, speed, g)
        va, wa = drivetrain(max(v_c, 0.4), w_c, va, wa, dt)   # plugin floors v at 0.4
        x += va * math.cos(yaw) * dt
        y += va * math.sin(yaw) * dt
        yaw += wa * dt
        t += dt
        min_body = min(min_body, body_distance(CX, CY, x, y, yaw, REAL_L, REAL_W))
    return swept, min_body, t


def test_real_drivetrain_full_lap_at_mppi_minimum_speed():
    """At 0.4 m/s (MPPI VelocityDeadbandCritic) the orbit completes even though the circle
    'needs' only 0.14 rad/s, below the rover's ~0.3 rad/s yaw deadband."""
    swept, min_body, t = lap_with_drivetrain(casualty_grid(), 0.4)
    assert swept >= 2 * math.pi - 0.02
    assert min_body >= 2.4, min_body
    assert t < 70.0


def test_real_drivetrain_rock_detour():
    g = casualty_grid()
    add_box(g, -0.25, 2.7, 0.25, 3.1)
    swept, min_body, _ = lap_with_drivetrain(g, 0.4)
    assert swept >= 2 * math.pi - 0.02
    assert min_body >= 2.4, min_body


def rasterized_disc_grid(cx, cy, radius=2.3, spacing=0.05, res=0.1, half=10.0):
    """Keepout disc the way the costmap holds it: every cell that contains one of
    casualty_manager's disc points (filled disc of points every 5 cm) is lethal."""
    g = make_grid(res, half)
    r = np.arange(-radius, radius + 1e-9, spacing)
    gx, gy = np.meshgrid(r, r)
    m = gx * gx + gy * gy <= radius * radius
    px, py = gx[m] + cx, gy[m] + cy
    i = np.floor((px - g.origin_x) / res).astype(int)
    j = np.floor((py - g.origin_y) / res).astype(int)
    g.data[j, i] = LETHAL
    return g


def test_own_keepout_disc_cells_never_split_the_circle():
    """Regression (sim 2026-10-12): the disc's edge cells reach ~2.37-2.42 m, the padded
    side passes at 2.45 m, so at some grid alignments ONE sample read as blocked and the
    orbit stopped and reversed (~140 deg coverage on an empty circle)."""
    p = OrbitParams()
    bad = []
    for ox in np.linspace(0.0, 0.09, 4):
        for oy in np.linspace(0.0, 0.09, 4):
            for rock in (False, True):
                g = rasterized_disc_grid(ox, oy)
                if rock:      # detour ramps tilt the rover: the case that failed in the sim
                    add_box(g, ox - 0.25, oy + 2.7, ox + 0.25, oy + 3.1)
                _, prof = radius_profile(g, ox, oy, p)
                if np.isnan(prof).any():
                    bad.append((round(ox, 2), round(oy, 2), rock, int(np.isnan(prof).sum())))
    assert not bad, f'(offset x, offset y, rock, blocked samples) on a passable circle: {bad}'
