"""Orbit geometry around a casualty, shared by casualty_manager and the tests.

Pure Python + numpy, no ROS imports, so every function here can be unit-tested
offline. The C++ OrbitCasualty behavior (vortex_nav_plugins) implements the same
algorithms; keep the two in step.

Conventions
-----------
* Angles `theta` are the rover's polar angle around the casualty, map frame,
  radians, counter-clockwise positive (standard math convention).
* A clockwise orbit moves towards decreasing theta and keeps the casualty on the
  rover's right.
* Costs are nav2 costmap values: 0..252 free/inflated, 253 inscribed,
  254 lethal, 255 unknown. A footprint "collides" only if it covers a lethal cell
  (inflation is irrelevant when the whole footprint polygon is checked).
"""

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np

LETHAL = 254
NO_INFORMATION = 255


def wrap_angle(a: float) -> float:
    """Wrap to (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


# --------------------------------------------------------------------------
# Costmap grid
# --------------------------------------------------------------------------

@dataclass
class Grid:
    """A costmap snapshot. data[row, col] with row = y index, col = x index."""
    resolution: float
    origin_x: float
    origin_y: float
    data: np.ndarray  # shape (size_y, size_x), dtype uint8

    @property
    def size_x(self) -> int:
        return self.data.shape[1]

    @property
    def size_y(self) -> int:
        return self.data.shape[0]


def grid_from_costmap_msg(msg) -> Grid:
    """nav2_msgs/msg/Costmap -> Grid (data is row-major, x fastest)."""
    md = msg.metadata
    data = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(md.size_y, md.size_x)
    return Grid(md.resolution, md.origin.position.x, md.origin.position.y, data)


def footprint_sample_points(length: float, width: float, step: float) -> np.ndarray:
    """Points covering a length x width rectangle centred on the rover, rover frame.

    `step` should be at most half the costmap resolution so no covered cell is missed.
    """
    nx = max(2, int(math.ceil(length / step)) + 1)
    ny = max(2, int(math.ceil(width / step)) + 1)
    xs = np.linspace(-length / 2.0, length / 2.0, nx)
    ys = np.linspace(-width / 2.0, width / 2.0, ny)
    gx, gy = np.meshgrid(xs, ys)
    return np.stack([gx.ravel(), gy.ravel()], axis=1)


def footprint_collides(grid: Grid, x: float, y: float, yaw: float, pts: np.ndarray,
                       unknown_is_lethal: bool = False,
                       outside_is_lethal: bool = True) -> bool:
    """True if the footprint at (x, y, yaw) covers any lethal cell."""
    c, s = math.cos(yaw), math.sin(yaw)
    wx = x + c * pts[:, 0] - s * pts[:, 1]
    wy = y + s * pts[:, 0] + c * pts[:, 1]
    mx = np.floor((wx - grid.origin_x) / grid.resolution).astype(np.int64)
    my = np.floor((wy - grid.origin_y) / grid.resolution).astype(np.int64)
    inside = (mx >= 0) & (mx < grid.size_x) & (my >= 0) & (my < grid.size_y)
    if not inside.all() and outside_is_lethal:
        return True
    costs = grid.data[my[inside], mx[inside]]
    if unknown_is_lethal:
        return bool(np.any(costs >= LETHAL))
    return bool(np.any(costs == LETHAL))


def mask_own_disc(grid: Grid, cx: float, cy: float, radius: float) -> Grid:
    """Copy of `grid` with the casualty's OWN keepout-disc cells cleared.

    The keepout is enforced exactly by body_distance() (padded body >= keepout + guard).
    In the costmap the disc is rasterised: its edge cells reach ~0.07-0.1 m beyond its
    radius, and a rover tilted on a detour ramp then 'hits' its own casualty's disc although
    the real geometry is fine -- splitting the circle into false arcs (sim 2026-10-12).
    Only cells whose centre lies inside the disc (+ 0.75 cell) are cleared: real obstacles
    and OTHER casualties' discs stay.
    """
    out = Grid(grid.resolution, grid.origin_x, grid.origin_y, grid.data.copy())
    r = radius + 0.75 * grid.resolution
    i0 = max(0, int((cx - r - grid.origin_x) / grid.resolution) - 1)
    i1 = min(grid.size_x, int((cx + r - grid.origin_x) / grid.resolution) + 2)
    j0 = max(0, int((cy - r - grid.origin_y) / grid.resolution) - 1)
    j1 = min(grid.size_y, int((cy + r - grid.origin_y) / grid.resolution) + 2)
    if i1 <= i0 or j1 <= j0:
        return out
    jj, ii = np.mgrid[j0:j1, i0:i1]
    wx = grid.origin_x + (ii + 0.5) * grid.resolution
    wy = grid.origin_y + (jj + 0.5) * grid.resolution
    inside = (wx - cx) ** 2 + (wy - cy) ** 2 <= r * r
    sub = out.data[j0:j1, i0:i1]
    sub[inside & (sub == LETHAL)] = 0
    return out


def body_distance(px: float, py: float, x: float, y: float, yaw: float,
                  length: float, width: float) -> float:
    """Shortest distance from point (px, py) to a length x width rectangle at (x, y, yaw).

    Zero if the point is inside the rectangle. This is the number the 2.2 m rule is about.
    """
    dx, dy = px - x, py - y
    c, s = math.cos(yaw), math.sin(yaw)
    lx = c * dx + s * dy          # point in the rover frame
    ly = -s * dx + c * dy
    ex = max(abs(lx) - length / 2.0, 0.0)
    ey = max(abs(ly) - width / 2.0, 0.0)
    return math.hypot(ex, ey)


# --------------------------------------------------------------------------
# Radius profile r(theta) with outward detours
# --------------------------------------------------------------------------

@dataclass
class OrbitParams:
    radius: float = 2.9                # orbit radius of the rover centre
    max_detour_radius: float = 4.0     # widest outward detour
    detour_step: float = 0.1           # radius search step
    sample_deg: float = 3.0            # angular sample spacing
    max_radius_slope: float = 0.05     # max radius change between neighbouring samples (m).
                                       # 0.05 m per 3 deg tilts the rover ~18 deg off the tangent;
                                       # steeper tilts swing the inner rear corner inside 2.3 m.
    keepout_radius: float = 2.2        # rule: no part of the body closer than this
    keepout_guard: float = 0.1         # planning keeps the body >= keepout + guard
    footprint_length: float = 1.0      # padded footprint used for planning
    footprint_width: float = 0.9
    edge_margin_deg: float = 6.0       # stay this far from a blocked edge
    unknown_is_lethal: bool = False


def sample_angles(sample_deg: float) -> np.ndarray:
    n = int(round(360.0 / sample_deg))
    return np.arange(n) * (2.0 * math.pi / n)


def path_heading(theta: float, r: float, dr_dtheta: float, clockwise: bool) -> float:
    """Heading of a rover driving along r(theta) in the given direction."""
    d = -1.0 if clockwise else 1.0
    tx = d * (dr_dtheta * math.cos(theta) - r * math.sin(theta))
    ty = d * (dr_dtheta * math.sin(theta) + r * math.cos(theta))
    return math.atan2(ty, tx)


def pose_on_profile(cx: float, cy: float, theta: float, r: float, dr_dtheta: float,
                    clockwise: bool) -> Tuple[float, float, float]:
    return (cx + r * math.cos(theta), cy + r * math.sin(theta),
            path_heading(theta, r, dr_dtheta, clockwise))


def _pose_ok(grid: Grid, cx: float, cy: float, x: float, y: float, yaw: float,
             pts: np.ndarray, p: OrbitParams) -> bool:
    if body_distance(cx, cy, x, y, yaw, p.footprint_length, p.footprint_width) \
            < p.keepout_radius + p.keepout_guard:
        return False
    return not footprint_collides(grid, x, y, yaw, pts, p.unknown_is_lethal)


def _outward_envelope(raw: np.ndarray, slope: float) -> np.ndarray:
    """Smallest profile >= raw whose neighbours differ by at most `slope` (circular).

    Blocked samples (NaN) stay NaN and act as BARRIERS: a detour on one side of a blocked
    stretch never raises the profile on the other side. This is what makes a detour start
    before an obstacle and end after it, never cutting inward.
    """
    n = len(raw)
    blocked = np.isnan(raw)
    env = np.where(blocked, -np.inf, raw)
    for _ in range(2):                      # two laps settle the circular wrap-around
        for i in range(n):
            if not blocked[i]:
                env[i] = max(env[i], env[i - 1] - slope)
        for i in range(n - 1, -1, -1):
            if not blocked[i]:
                env[i] = max(env[i], env[(i + 1) % n] - slope)
    return np.where(blocked, np.nan, env)


def drop_islands(profile: np.ndarray, radius: float, eps: float = 0.05) -> np.ndarray:
    """Blank out free stretches that never come back to the orbit circle.

    A real detour leaves the circle and returns to it. A free stretch whose radius is
    above `radius` everywhere is a pocket the rover can't get into -- typically the
    unseen inside of a hollow obstacle (lidar only sees surfaces).
    """
    prof = profile.copy()
    for a in free_arcs(prof):
        idx = a.indices()
        if np.nanmin(prof[idx]) > radius + eps:
            prof[idx] = np.nan
    return prof


def radius_profile(grid: Grid, cx: float, cy: float, p: OrbitParams,
                   thetas: Optional[np.ndarray] = None) -> Tuple[np.ndarray, np.ndarray]:
    """r(theta): smallest radius >= p.radius at which the rover fits, NaN where blocked.

    Detours only go outward: r is never below p.radius. After smoothing, every sample
    (and the midpoint to its neighbour) is re-checked with the real path heading.
    """
    if thetas is None:
        thetas = sample_angles(p.sample_deg)
    grid = mask_own_disc(grid, cx, cy, p.keepout_radius + p.keepout_guard)
    pts = footprint_sample_points(p.footprint_length, p.footprint_width, grid.resolution / 2.0)
    radii = np.arange(p.radius, p.max_detour_radius + 1e-6, p.detour_step)
    raw = np.full(len(thetas), np.nan)
    for i, th in enumerate(thetas):
        for r in radii:
            x, y, yaw = pose_on_profile(cx, cy, th, r, 0.0, clockwise=True)
            if _pose_ok(grid, cx, cy, x, y, yaw, pts, p):
                raw[i] = r
                break
    # Smooth, then verify the smoothed path (samples and midpoints, with the real heading).
    # A sample whose smoothed pose collides -- e.g. tilted at the top of a detour so a corner
    # clips the obstacle -- is pushed OUT by one detour_step and everything is re-smoothed.
    # Only when it can't go further out (max_detour_radius) is it marked blocked.
    n = len(thetas)
    dth = 2.0 * math.pi / n
    for _ in range(int(round((p.max_detour_radius - p.radius) / p.detour_step)) + 2):
        prof = _outward_envelope(raw, p.max_radius_slope)
        failed = []
        for i in range(n):
            if np.isnan(prof[i]):
                continue
            j = (i + 1) % n
            r_i = prof[i]
            r_j = prof[j] if not np.isnan(prof[j]) else r_i
            slope = (r_j - r_i) / dth
            for frac in (0.0, 0.5):
                th = thetas[i] + frac * dth
                r = r_i + frac * (r_j - r_i)
                x, y, yaw = pose_on_profile(cx, cy, th, r, slope, clockwise=True)
                if not _pose_ok(grid, cx, cy, x, y, yaw, pts, p):
                    failed.append(i)
                    break
        if not failed:
            break
        for i in failed:
            raw[i] = max(raw[i], prof[i]) + p.detour_step
            if raw[i] > p.max_detour_radius + 1e-6:
                raw[i] = np.nan
    prof = _outward_envelope(raw, p.max_radius_slope)
    # Final check: anything still failing (loop ran out) is blocked -- never return an
    # unverified pose.
    ok = ~np.isnan(prof)
    for i in range(n):
        if not ok[i]:
            continue
        j = (i + 1) % n
        r_i = prof[i]
        r_j = prof[j] if not np.isnan(prof[j]) else r_i
        slope = (r_j - r_i) / dth
        for frac in (0.0, 0.5):
            x, y, yaw = pose_on_profile(cx, cy, thetas[i] + frac * dth, r_i + frac * (r_j - r_i),
                                        slope, clockwise=True)
            if not _pose_ok(grid, cx, cy, x, y, yaw, pts, p):
                ok[i] = False
                break
    prof[~ok] = np.nan
    return thetas, drop_islands(prof, p.radius)


# --------------------------------------------------------------------------
# Free arcs
# --------------------------------------------------------------------------

@dataclass
class Arc:
    start: int     # index of the first free sample (lowest theta, counter-clockwise order)
    length: int    # number of free samples
    n: int         # total samples on the circle

    @property
    def full(self) -> bool:
        return self.length >= self.n

    def indices(self) -> List[int]:
        return [(self.start + k) % self.n for k in range(self.length)]

    def span_deg(self) -> float:
        return 360.0 if self.full else self.length * 360.0 / self.n


def free_arcs(profile: np.ndarray) -> List[Arc]:
    """Contiguous runs of non-NaN samples on the circle, largest first."""
    n = len(profile)
    free = ~np.isnan(profile)
    if free.all():
        return [Arc(0, n, n)]
    if not free.any():
        return []
    first_blocked = int(np.argmin(free))         # rotate so we start just after a blocked sample
    arcs = []
    k = 0
    while k < n:
        i = (first_blocked + k) % n
        if free[i]:
            start = i
            length = 0
            while k < n and free[(first_blocked + k) % n]:
                length += 1
                k += 1
            arcs.append(Arc(start, length, n))
        else:
            k += 1
    arcs.sort(key=lambda a: a.length, reverse=True)
    return arcs


def arc_containing(arcs: Sequence[Arc], idx: int) -> Optional[Arc]:
    for a in arcs:
        if idx in a.indices():
            return a
    return None


def theta_to_index(theta: float, n: int) -> int:
    return int(round((theta % (2.0 * math.pi)) / (2.0 * math.pi) * n)) % n


def clockwise_start_theta(arc: Arc, thetas: np.ndarray, edge_margin_deg: float) -> float:
    """Where to park so that one clockwise sweep covers the whole arc.

    Clockwise = decreasing theta, so start at the arc's counter-clockwise end
    (its highest-theta sample), backed off from the blocked edge by the margin.
    """
    last = (arc.start + arc.length - 1) % arc.n
    return float(thetas[last]) - math.radians(edge_margin_deg)


# --------------------------------------------------------------------------
# Reference path-following law (the C++ behavior uses the same equations)
# --------------------------------------------------------------------------

@dataclass
class TrackerGains:
    k_lateral: float = 1.5        # 1/m: how hard to steer back to the path
    k_heading: float = 2.0        # 1/s
    max_correction_deg: float = 30.0
    max_angular: float = 0.8      # rad/s


def tracker_command(rover_x: float, rover_y: float, rover_yaw: float,
                    cx: float, cy: float, r_target: float, dr_dtheta: float,
                    clockwise: bool, v: float, g: TrackerGains) -> Tuple[float, float, float]:
    """Unicycle command (v, w) that follows the circle-like path r_target(theta).

    `v` may be negative (backing out along the path). Returns (v, w, lateral_error),
    where lateral_error = rover distance - r_target (positive = outside the path).
    """
    theta = math.atan2(rover_y - cy, rover_x - cx)
    d = math.hypot(rover_x - cx, rover_y - cy)
    e_out = d - r_target
    # Left of the travel direction is outward for a clockwise orbit, inward otherwise.
    e_left = e_out if clockwise else -e_out
    psi_path = path_heading(theta, r_target, dr_dtheta, clockwise)
    corr = math.atan(g.k_lateral * e_left)
    corr = max(-math.radians(g.max_correction_deg), min(math.radians(g.max_correction_deg), corr))
    sign_v = 1.0 if v >= 0.0 else -1.0
    psi_des = psi_path - sign_v * corr
    kappa = (-1.0 if clockwise else 1.0) / max(r_target, 0.1)
    w = kappa * v + g.k_heading * wrap_angle(psi_des - rover_yaw)
    w = max(-g.max_angular, min(g.max_angular, w))
    return v, w, e_out


def profile_at(thetas: np.ndarray, profile: np.ndarray, theta: float) -> Tuple[float, float]:
    """Linear interpolation of r(theta) on the circle. Returns (r, dr/dtheta); NaN if blocked."""
    n = len(thetas)
    dth = 2.0 * math.pi / n
    t = (theta % (2.0 * math.pi)) / dth
    i = int(math.floor(t)) % n
    j = (i + 1) % n
    f = t - math.floor(t)
    r_i, r_j = profile[i], profile[j]
    if np.isnan(r_i) or np.isnan(r_j):
        return float('nan'), 0.0
    return float(r_i + f * (r_j - r_i)), float((r_j - r_i) / dth)


def blended_target(r_profile: float, dr_profile: float, start_radius: float,
                   swept: float, remaining: float, spiral: float,
                   clockwise: bool, end_radius: Optional[float] = None,
                   spiral_out: Optional[float] = None) -> Tuple[float, float]:
    """Target radius (and its d/dtheta) while moving along a leg.

    Ramps from `start_radius` to the profile over the first `spiral` radians of the
    leg, and back out to `end_radius` (default: start_radius) over the last
    `spiral_out` radians (default: spiral). `swept` / `remaining` are the angles
    already covered / still to go on this leg. Smoothstep keeps it gentle; the
    returned slope includes the ramp so the tracker can feed it forward.
    Same as blendedTarget() in the C++ plugin.
    """
    def smooth(u: float) -> Tuple[float, float]:
        u = max(0.0, min(1.0, u))
        return u * u * (3.0 - 2.0 * u), 6.0 * u * (1.0 - u)

    if end_radius is None:
        end_radius = start_radius
    if spiral_out is None:
        spiral_out = spiral
    dswept_dtheta = -1.0 if clockwise else 1.0   # swept grows as theta decreases when CW
    w_in, dw_in = smooth(swept / spiral) if spiral > 0 else (1.0, 0.0)
    w_out, dw_out = smooth(remaining / spiral_out) if spiral_out > 0 else (1.0, 0.0)
    if w_in <= w_out:
        r = start_radius + w_in * (r_profile - start_radius)
        dr = w_in * dr_profile + (r_profile - start_radius) * (dw_in / max(spiral, 1e-9)) \
            * dswept_dtheta
    else:
        r = end_radius + w_out * (r_profile - end_radius)
        dr = w_out * dr_profile - (r_profile - end_radius) * (dw_out / max(spiral_out, 1e-9)) \
            * dswept_dtheta
    return r, dr


def spiral_angle(dr: float, max_slope: float = 0.6, min_spiral: float = math.radians(25.0)) -> float:
    """Angle needed to change radius by `dr` (smoothstep peaks at 1.5x its mean slope)."""
    return max(min_spiral, 1.5 * abs(dr) / max(max_slope, 1e-3))
