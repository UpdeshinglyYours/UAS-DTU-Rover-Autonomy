#!/usr/bin/env python3
"""Report repeatable geometry/cost metrics for DTU reference and Nav2 paths."""

from __future__ import annotations

import json
import math
from pathlib import Path as FilePath
import time

from nav_msgs.msg import OccupancyGrid, Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


Point = tuple[float, float]


def path_points(message: Path) -> list[Point]:
    return [(pose.pose.position.x, pose.pose.position.y) for pose in message.poses]


def path_length(points: list[Point]) -> float:
    return sum(math.dist(start, end) for start, end in zip(points, points[1:]))


def point_segment_distance(point: Point, start: Point, end: Point) -> float:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    denominator = dx * dx + dy * dy
    if denominator == 0.0:
        return math.dist(point, start)
    fraction = max(
        0.0,
        min(
            1.0,
            ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy)
            / denominator,
        ),
    )
    projected = (start[0] + fraction * dx, start[1] + fraction * dy)
    return math.dist(point, projected)


def point_polyline_distance(point: Point, line: list[Point]) -> float:
    return min(
        point_segment_distance(point, start, end)
        for start, end in zip(line, line[1:])
    )


def cell_for(grid: OccupancyGrid, point: Point) -> tuple[int, int] | None:
    column = math.floor((point[0] - grid.info.origin.position.x) / grid.info.resolution)
    row = math.floor((point[1] - grid.info.origin.position.y) / grid.info.resolution)
    if 0 <= column < grid.info.width and 0 <= row < grid.info.height:
        return int(column), int(row)
    return None


def value_at(grid: OccupancyGrid, point: Point) -> int:
    cell = cell_for(grid, point)
    if cell is None:
        return 100
    column, row = cell
    return int(grid.data[row * grid.info.width + column])


def lethal_cells_crossed(
    grid: OccupancyGrid, points: list[Point]
) -> set[tuple[int, int]]:
    crossed: set[tuple[int, int]] = set()
    for point in densify(points, grid.info.resolution / 2.0):
        cell = cell_for(grid, point)
        if cell is None or value_at(grid, point) >= 100:
            crossed.add(cell if cell is not None else (-1, -1))
    return crossed


def densify(points: list[Point], spacing: float) -> list[Point]:
    output = [points[0]]
    for start, end in zip(points, points[1:]):
        length = math.dist(start, end)
        steps = max(1, math.ceil(length / spacing))
        for step in range(1, steps + 1):
            fraction = step / steps
            output.append(
                (
                    start[0] + fraction * (end[0] - start[0]),
                    start[1] + fraction * (end[1] - start[1]),
                )
            )
    return output


def minimum_lethal_clearance(grid: OccupancyGrid, points: list[Point]) -> float | None:
    resolution = grid.info.resolution
    origin_x = grid.info.origin.position.x
    origin_y = grid.info.origin.position.y
    min_x = min(point[0] for point in points)
    max_x = max(point[0] for point in points)
    min_y = min(point[1] for point in points)
    max_y = max(point[1] for point in points)
    array = np.asarray(grid.data, dtype=np.int16).reshape(
        (grid.info.height, grid.info.width)
    )
    occupied_xy = None
    for margin in (5.0, 10.0, 20.0, 50.0):
        column_min = max(0, math.floor((min_x - margin - origin_x) / resolution))
        column_max = min(
            grid.info.width,
            math.ceil((max_x + margin - origin_x) / resolution) + 1,
        )
        row_min = max(0, math.floor((min_y - margin - origin_y) / resolution))
        row_max = min(
            grid.info.height,
            math.ceil((max_y + margin - origin_y) / resolution) + 1,
        )
        rows, columns = np.nonzero(
            array[row_min:row_max, column_min:column_max] >= 100
        )
        if rows.size:
            occupied_xy = np.column_stack(
                (
                    origin_x + (columns + column_min + 0.5) * resolution,
                    origin_y + (rows + row_min + 0.5) * resolution,
                )
            )
            break
    if occupied_xy is None:
        return None
    minimum = math.inf
    sampled = densify(points, max(0.1, resolution))
    for offset in range(0, len(sampled), 128):
        block = np.asarray(sampled[offset : offset + 128], dtype=np.float64)
        distances_squared = (
            (block[:, None, 0] - occupied_xy[None, :, 0]) ** 2
            + (block[:, None, 1] - occupied_xy[None, :, 1]) ** 2
        )
        minimum = min(minimum, float(np.sqrt(distances_squared.min())))
    # Occupied coordinates are cell centres; subtract the cell circumradius to
    # conservatively approximate clearance to the occupied-cell boundary.
    return max(0.0, minimum - resolution * math.sqrt(2.0) / 2.0)


def compute_metrics(
    actual_message: Path,
    reference_message: Path,
    static_map: OccupancyGrid,
    road_mask: OccupancyGrid,
    combined_costmap: OccupancyGrid | None,
) -> dict:
    actual = path_points(actual_message)
    reference = path_points(reference_message)
    if len(actual) < 2 or len(reference) < 2:
        raise RuntimeError("Both paths need at least two poses")

    total_length = path_length(actual)
    close_lengths = {0.5: 0.0, 1.0: 0.0, 2.0: 0.0}
    road_length = 0.0
    offroad_length = 0.0
    for start, end in zip(actual, actual[1:]):
        length = math.dist(start, end)
        midpoint = ((start[0] + end[0]) / 2.0, (start[1] + end[1]) / 2.0)
        distance = point_polyline_distance(midpoint, reference)
        for threshold in close_lengths:
            if distance <= threshold:
                close_lengths[threshold] += length
        if value_at(road_mask, midpoint) == 0:
            road_length += length
        else:
            offroad_length += length

    clearance_grid = combined_costmap or static_map
    clearance = minimum_lethal_clearance(clearance_grid, actual)
    combined_crossings = lethal_cells_crossed(clearance_grid, actual)
    static_crossings = lethal_cells_crossed(static_map, actual)
    return {
        "actual_path_length_m": round(total_length, 3),
        "reference_route_length_m": round(path_length(reference), 3),
        "actual_within_reference_percent": {
            f"{threshold:.1f}_m": round(
                100.0 * length / total_length if total_length else 0.0, 2
            )
            for threshold, length in close_lengths.items()
        },
        "minimum_obstacle_clearance_m": None if clearance is None else round(clearance, 3),
        "clearance_source": (
            "/global_costmap/costmap" if combined_costmap is not None else "/dtu_static_map"
        ),
        "lethal_cells_crossed": len(combined_crossings),
        "static_lethal_cells_crossed": len(static_crossings),
        "actual_path_surface_percent": {
            "road": round(100.0 * road_length / total_length, 2),
            "offroad": round(100.0 * offroad_length / total_length, 2),
        },
        "actual_pose_count": len(actual),
        "reference_pose_count": len(reference),
        "actual_start": [round(actual[0][0], 6), round(actual[0][1], 6)],
        "actual_goal": [round(actual[-1][0], 6), round(actual[-1][1], 6)],
    }


class EvidenceCollector(Node):
    def __init__(self) -> None:
        super().__init__("dtu_path_metrics")
        self.declare_parameter("timeout_sec", 20.0)
        self.declare_parameter("settle_time_sec", 1.5)
        self.declare_parameter("output_file", "")
        self.messages: dict[str, object] = {}
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(Path, "/plan", lambda msg: self._save("actual", msg), 10)
        self.create_subscription(
            Path, "/dtu_reference_route", lambda msg: self._save("reference", msg), latched
        )
        self.create_subscription(
            OccupancyGrid, "/dtu_static_map", lambda msg: self._save("static", msg), latched
        )
        self.create_subscription(
            OccupancyGrid, "/dtu_road_mask", lambda msg: self._save("road", msg), latched
        )
        self.create_subscription(
            OccupancyGrid,
            "/global_costmap/costmap",
            lambda msg: self._save("combined", msg),
            latched,
        )

    def _save(self, key: str, message: object) -> None:
        self.messages[key] = message


def main() -> None:
    rclpy.init()
    node = EvidenceCollector()
    timeout = float(node.get_parameter("timeout_sec").value)
    settle_time = float(node.get_parameter("settle_time_sec").value)
    if settle_time < 0.0:
        raise SystemExit("settle_time_sec cannot be negative")
    deadline = time.monotonic() + timeout
    required = {"actual", "reference", "static", "road"}
    ready_since = None
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if required <= node.messages.keys():
            ready_since = ready_since or time.monotonic()
            if time.monotonic() - ready_since >= settle_time:
                break
    missing = sorted(required - node.messages.keys())
    if missing:
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit(f"Timed out waiting for: {', '.join(missing)}")
    metrics = compute_metrics(
        node.messages["actual"],
        node.messages["reference"],
        node.messages["static"],
        node.messages["road"],
        node.messages.get("combined"),
    )
    report = json.dumps(metrics, indent=2, sort_keys=True)
    print(report)
    output_file = str(node.get_parameter("output_file").value)
    if output_file:
        FilePath(output_file).write_text(report + "\n", encoding="utf-8")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
