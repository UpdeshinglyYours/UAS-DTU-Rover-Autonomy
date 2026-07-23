"""Efficient bounded endpoint-to-obstacle clearance checks."""

import math
from dataclasses import dataclass
from typing import Iterable, Optional, Tuple

import numpy as np

from .grid_snapshot import GridSnapshot


@dataclass(frozen=True)
class ClearanceResult:
    available: bool
    distance: float = math.inf
    source: str = ''

    def is_safe(self, required_clearance: float) -> bool:
        return self.available and self.distance > required_clearance


def obstacle_clearance_in_grid(
    grid: GridSnapshot,
    x: float,
    y: float,
    occupied_threshold: int,
    search_radius_m: float,
) -> Optional[float]:
    """
    Return clearance to occupied cell area, or infinity if none is nearby.

    Only a bounded window around the endpoint is inspected. Unknown cells are
    negative and are therefore never counted as obstacles.
    """
    if grid.resolution <= 0.0 or grid.world_to_cell(x, y) is None:
        return None

    dx = x - grid.origin_x
    dy = y - grid.origin_y
    cos_yaw = math.cos(grid.origin_yaw)
    sin_yaw = math.sin(grid.origin_yaw)
    local_x = cos_yaw * dx + sin_yaw * dy
    local_y = -sin_yaw * dx + cos_yaw * dy

    center_x = int(math.floor(local_x / grid.resolution))
    center_y = int(math.floor(local_y / grid.resolution))
    radius_cells = int(math.ceil(
        max(0.0, search_radius_m) / grid.resolution
    )) + 1
    min_x = max(0, center_x - radius_cells)
    max_x = min(grid.width - 1, center_x + radius_cells)
    min_y = max(0, center_y - radius_cells)
    max_y = min(grid.height - 1, center_y + radius_cells)

    values = np.frombuffer(grid.data, dtype=np.int8).reshape(
        grid.height,
        grid.width,
    )
    window = values[min_y:max_y + 1, min_x:max_x + 1]
    rows, columns = np.nonzero(window >= occupied_threshold)
    if rows.size == 0:
        return math.inf

    obstacle_x = columns.astype(float) + min_x
    obstacle_y = rows.astype(float) + min_y
    left = obstacle_x * grid.resolution
    right = (obstacle_x + 1.0) * grid.resolution
    bottom = obstacle_y * grid.resolution
    top = (obstacle_y + 1.0) * grid.resolution
    distance_x = np.maximum.reduce((
        left - local_x,
        np.zeros_like(left),
        local_x - right,
    ))
    distance_y = np.maximum.reduce((
        bottom - local_y,
        np.zeros_like(bottom),
        local_y - top,
    ))
    distances = np.hypot(distance_x, distance_y)
    return float(np.min(distances))


def combined_clearance(
    checks: Iterable[Tuple[str, GridSnapshot, float, float, int]],
    search_radius_m: float,
) -> ClearanceResult:
    """
    Combine already frame-aligned map and costmap endpoint checks.

    Each tuple contains (source, grid, x_in_grid_frame, y_in_grid_frame,
    occupied_threshold).
    """
    distances = []
    for source, grid, x, y, threshold in checks:
        distance = obstacle_clearance_in_grid(
            grid,
            x,
            y,
            threshold,
            search_radius_m,
        )
        if distance is not None:
            distances.append((distance, source))
    if not distances:
        return ClearanceResult(False)
    distance, source = min(distances, key=lambda item: item[0])
    return ClearanceResult(True, distance, source)
