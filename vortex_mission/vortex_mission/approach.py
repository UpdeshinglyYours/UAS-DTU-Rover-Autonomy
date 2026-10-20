"""Where to park (STANDBY) next to a casualty. Pure functions, unit-tested offline.

Rules (see the plan, section 2.2):
* whole orbit circle free (or no orbit possible at all): any reachable point on the
  approach circle -- casualty_manager ranks these by planned path length;
* part of the circle blocked: the counter-clockwise end of the largest free arc, so a
  single clockwise sweep covers it; if that end can't be reached, step clockwise
  along the arc.
"""

import math
from dataclasses import dataclass
from typing import List, Optional

import numpy as np

from vortex_mission.orbit_geometry import (
    Arc, Grid, OrbitParams, body_distance, clockwise_start_theta, footprint_collides,
    footprint_sample_points, free_arcs, radius_profile)


@dataclass
class Candidate:
    theta: float      # angle of the parking spot around the casualty (rad, map frame)
    x: float
    y: float
    yaw: float        # facing the casualty
    on_arc: bool      # True if this is an arc-end candidate (partial circle)


@dataclass
class ApproachPlan:
    candidates: List[Candidate]   # in order of preference
    arcs: List[Arc]
    profile: np.ndarray
    thetas: np.ndarray

    @property
    def circle_free(self) -> bool:
        return bool(self.arcs) and self.arcs[0].full

    @property
    def largest_arc_deg(self) -> float:
        return self.arcs[0].span_deg() if self.arcs else 0.0


def _candidate(cx: float, cy: float, theta: float, radius: float, on_arc: bool) -> Candidate:
    x = cx + radius * math.cos(theta)
    y = cy + radius * math.sin(theta)
    return Candidate(theta, x, y, math.atan2(cy - y, cx - x), on_arc)


def candidate_ok(grid: Grid, cx: float, cy: float, c: Candidate, p: OrbitParams) -> bool:
    """Padded footprint free and outside keepout + guard when parked facing the casualty."""
    if body_distance(cx, cy, c.x, c.y, c.yaw, p.footprint_length, p.footprint_width) \
            < p.keepout_radius + p.keepout_guard:
        return False
    pts = footprint_sample_points(p.footprint_length, p.footprint_width, grid.resolution / 2.0)
    return not footprint_collides(grid, c.x, c.y, c.yaw, pts, p.unknown_is_lethal)


def plan_approach(grid: Grid, cx: float, cy: float, rover_x: float, rover_y: float,
                  p: OrbitParams, approach_radius: float, n_candidates: int = 16,
                  arc_step_deg: float = 15.0) -> ApproachPlan:
    thetas, prof = radius_profile(grid, cx, cy, p)
    arcs = free_arcs(prof)
    cands: List[Candidate] = []
    if arcs and not arcs[0].full:
        arc = arcs[0]
        start = clockwise_start_theta(arc, thetas, p.edge_margin_deg)
        span = math.radians(arc.span_deg() - 2.0 * p.edge_margin_deg)
        k = 0
        while k * math.radians(arc_step_deg) <= max(span, 0.0):
            cands.append(_candidate(cx, cy, start - k * math.radians(arc_step_deg),
                                    approach_radius, True))
            k += 1
    else:
        # Full circle free, or nothing free (still park for STANDBY): nearest first.
        rover_theta = math.atan2(rover_y - cy, rover_x - cx)
        ring = [_candidate(cx, cy, 2.0 * math.pi * i / n_candidates, approach_radius, False)
                for i in range(n_candidates)]
        ring.sort(key=lambda c: abs(math.atan2(math.sin(c.theta - rover_theta),
                                               math.cos(c.theta - rover_theta))))
        cands = ring
    cands = [c for c in cands if candidate_ok(grid, cx, cy, c, p)]
    return ApproachPlan(cands, arcs, prof, thetas)


def face_yaw_delta(rover_x: float, rover_y: float, rover_yaw: float,
                   cx: float, cy: float) -> float:
    """Signed turn (rad, shortest way) that makes the rover face the casualty."""
    target = math.atan2(cy - rover_y, cx - rover_x)
    return math.atan2(math.sin(target - rover_yaw), math.cos(target - rover_yaw))


def keepout_disc_points(centres, radius: float, spacing: float = 0.05,
                        z: float = 0.5) -> np.ndarray:
    """Filled discs of points (one per casualty) for the costmap's casualty_layer."""
    r = np.arange(-radius, radius + 1e-9, spacing)
    gx, gy = np.meshgrid(r, r)
    mask = gx * gx + gy * gy <= radius * radius
    disc = np.stack([gx[mask], gy[mask]], axis=1)
    out = [np.column_stack([disc[:, 0] + cx, disc[:, 1] + cy, np.full(len(disc), z)])
           for cx, cy in centres]
    return np.vstack(out).astype(np.float32) if out else np.zeros((0, 3), np.float32)


def nearest_index(points, x: float, y: float) -> Optional[int]:
    if not points:
        return None
    d = [math.hypot(px - x, py - y) for px, py in points]
    return int(np.argmin(d))
