#!/usr/bin/env python3
"""Offline ROS 2 bag forensics for Target Explorer and Nav2 rover tests.

The reader uses rosbag2_py directly.  It never starts ROS nodes, publishes,
replays, or connects to a live ROS graph.
"""

from __future__ import annotations

import argparse
import csv
import math
import re
import statistics
import sys
from collections import Counter, defaultdict, deque
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Iterable, Optional

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


STATUS_NAMES = {
    0: "UNKNOWN",
    1: "ACCEPTED",
    2: "EXECUTING",
    3: "CANCELING",
    4: "SUCCEEDED",
    5: "CANCELED",
    6: "ABORTED",
}

IMPORTANT_PARAMETERS = (
    "goal_obstacle_clearance_m",
    "map_occupied_threshold",
    "active_goal_safety_check_period_sec",
    "robot_radius",
    "footprint",
    "footprint_padding",
    "inflation_radius",
    "cost_scaling_factor",
    "resolution",
    "track_unknown_space",
    "obstacle_min_range",
    "obstacle_max_range",
    "raytrace_min_range",
    "raytrace_max_range",
    "min_obstacle_height",
    "max_obstacle_height",
    "tolerance",
    "allow_unknown",
    "desired_linear_vel",
    "lookahead_dist",
    "min_lookahead_dist",
    "max_lookahead_dist",
    "xy_goal_tolerance",
    "yaw_goal_tolerance",
    "movement_time_allowance",
    "required_movement_radius",
)

NAV_NODE_HINTS = (
    "target_explorer",
    "bt_navigator",
    "planner_server",
    "controller_server",
    "behavior_server",
    "global_costmap",
    "local_costmap",
    "waypoint_follower",
    "velocity_smoother",
)

NAV_EVENT_TERMS = (
    "goal",
    "path",
    "plan",
    "control",
    "cancel",
    "abort",
    "succeed",
    "reach",
    "recover",
    "clear",
    "spin",
    "backup",
    "back up",
    "wait",
    "progress",
    "patience",
    "tolerance",
    "bounds",
    "transform",
    "costmap",
    "obstacle",
    "failed",
    "failure",
    "error",
    "resize",
)

RECOVERY_TERMS = (
    "recovery",
    "clearcostmap",
    "clear costmap",
    "spin",
    "backup",
    "back up",
    "wait",
)

TARGET_RE = {
    "startup": re.compile(
        r"target_explorer started: final target=\(([-+\d.]+), ([-+\d.]+)\)"
    ),
    "rejection": re.compile(
        r"Rejected (final target|frontier) \(([-+\d.]+), ([-+\d.]+)\): "
        r"endpoint clearance ([-+\d.]+) m is below required ([-+\d.]+) m"
    ),
    "dispatch": re.compile(
        r"Sending NavigateToPose goal \(([-+\d.]+), ([-+\d.]+)\): "
        r"progress=([-+\d.]+) m alignment=([-+\d.]+) path=([-+\d.]+) m "
        r"score=([-+]?\d+(?:\.\d+)?)\."
    ),
    "accepted": re.compile(r"Nav2 accepted goal \(([-+\d.]+), ([-+\d.]+)\)"),
    "rejected_goal": re.compile(r"Nav2 rejected goal \(([-+\d.]+), ([-+\d.]+)\)"),
    "succeeded": re.compile(
        r"NavigateToPose succeeded for \(([-+\d.]+), ([-+\d.]+)\)"
    ),
    "canceled": re.compile(
        r"NavigateToPose cancellation completed for "
        r"\(([-+\d.]+), ([-+\d.]+)\)"
    ),
    "finished": re.compile(
        r"NavigateToPose finished for \(([-+\d.]+), ([-+\d.]+)\) "
        r"with status (\d+)"
    ),
    "safety_cancel": re.compile(
        r"Canceling active goal \(([-+\d.]+), ([-+\d.]+)\): (.*)"
    ),
    "recovery_frontier": re.compile(
        r"using recovery frontier \(([-+\d.]+), ([-+\d.]+)\)"
    ),
}

CSV_COLUMNS = {
    "bag_inventory.csv": (
        "bag",
        "bag_path",
        "storage",
        "bag_start",
        "bag_duration_sec",
        "bag_message_count",
        "topic",
        "message_type",
        "topic_message_count",
        "frequency_hz",
    ),
    "goal_timeline.csv": (
        "bag",
        "dispatch_time",
        "relative_sec",
        "goal_number",
        "goal_x",
        "goal_y",
        "final_target_x",
        "final_target_y",
        "progress_m",
        "alignment",
        "path_length_m",
        "score",
        "initial_endpoint_clearance_m",
        "physical_map_clearance_m",
        "lethal_costmap_clearance_m",
        "nav2_accepted",
        "later_canceled",
        "cancellation_reason",
        "outcome",
        "action_status",
        "time_active_sec",
        "distance_travelled_while_active_m",
        "recoveries_triggered",
        "direct_route_physical_blocked",
        "direct_route_lethal_blocked",
        "direct_route_inflated_cells",
        "selected_route_physical_blocked",
        "map_context_interpretation",
    ),
    "all_frontier_decisions.csv": (
        "bag",
        "absolute_time",
        "relative_sec",
        "source_node",
        "source_topic",
        "event_category",
        "candidate_type",
        "goal_x",
        "goal_y",
        "decision",
        "direct_reason",
        "clearance_m",
        "required_clearance_m",
        "progress_m",
        "alignment",
        "path_length_m",
        "score",
        "outcome",
    ),
    "nav2_events.csv": (
        "bag",
        "absolute_time",
        "relative_sec",
        "source_node",
        "source_topic",
        "event_category",
        "severity",
        "goal_x",
        "goal_y",
        "action_status",
        "path_pose_count",
        "path_length_m",
        "message",
        "active_goal_number",
    ),
    "parameter_events.csv": (
        "bag",
        "absolute_time",
        "relative_sec",
        "node",
        "change_kind",
        "parameter",
        "parameter_type",
        "value",
    ),
    "decoding_failures.csv": (
        "bag",
        "topic",
        "message_type",
        "count",
        "first_error",
    ),
}


@dataclass
class GridSnapshot:
    topic: str
    timestamp_ns: int
    frame_id: str
    resolution: float
    width: int
    height: int
    origin_x: float
    origin_y: float
    origin_yaw: float
    data: np.ndarray


def stamp_text(timestamp_ns: int) -> str:
    return datetime.fromtimestamp(timestamp_ns / 1e9).astimezone().isoformat(
        timespec="milliseconds"
    )


def seconds(timestamp_ns: int, start_ns: int) -> float:
    return (timestamp_ns - start_ns) / 1e9


def yaw_from_quaternion(q: Any) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def compose_pose(first: tuple[float, float, float],
                 second: tuple[float, float, float]) -> tuple[float, float, float]:
    """Compose parent->middle and middle->child planar transforms."""
    ax, ay, ayaw = first
    bx, by, byaw = second
    c = math.cos(ayaw)
    s = math.sin(ayaw)
    return (
        ax + c * bx - s * by,
        ay + s * bx + c * by,
        normalize_angle(ayaw + byaw),
    )


def inverse_pose(pose: tuple[float, float, float]) -> tuple[float, float, float]:
    x, y, yaw = pose
    c = math.cos(yaw)
    s = math.sin(yaw)
    return (-c * x - s * y, s * x - c * y, normalize_angle(-yaw))


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def resolve_transform(
    transforms: dict[tuple[str, str], tuple[float, float, float]],
    source: str,
    target: str,
) -> Optional[tuple[float, float, float]]:
    source = source.lstrip("/")
    target = target.lstrip("/")
    if source == target:
        return (0.0, 0.0, 0.0)
    graph: dict[str, list[tuple[str, tuple[float, float, float]]]] = defaultdict(list)
    for (parent, child), value in transforms.items():
        graph[parent].append((child, value))
        graph[child].append((parent, inverse_pose(value)))
    queue = deque([(source, (0.0, 0.0, 0.0))])
    visited = {source}
    while queue:
        frame, accumulated = queue.popleft()
        for next_frame, edge in graph.get(frame, ()):
            if next_frame in visited:
                continue
            result = compose_pose(accumulated, edge)
            if next_frame == target:
                return result
            visited.add(next_frame)
            queue.append((next_frame, result))
    return None


def grid_from_message(topic: str, timestamp_ns: int, msg: Any) -> GridSnapshot:
    height = int(msg.info.height)
    width = int(msg.info.width)
    raw = np.asarray(msg.data, dtype=np.int16)
    if raw.size != width * height:
        raise ValueError(
            f"occupancy data length {raw.size} != {width}x{height}"
        )
    return GridSnapshot(
        topic=topic,
        timestamp_ns=timestamp_ns,
        frame_id=msg.header.frame_id,
        resolution=float(msg.info.resolution),
        width=width,
        height=height,
        origin_x=float(msg.info.origin.position.x),
        origin_y=float(msg.info.origin.position.y),
        origin_yaw=yaw_from_quaternion(msg.info.origin.orientation),
        data=raw.reshape((height, width)).copy(),
    )


def world_to_grid(grid: GridSnapshot, x: float, y: float) -> tuple[int, int]:
    dx = x - grid.origin_x
    dy = y - grid.origin_y
    c = math.cos(grid.origin_yaw)
    s = math.sin(grid.origin_yaw)
    local_x = c * dx + s * dy
    local_y = -s * dx + c * dy
    return int(math.floor(local_x / grid.resolution)), int(
        math.floor(local_y / grid.resolution)
    )


def grid_to_world(
    grid: GridSnapshot, cols: np.ndarray, rows: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    lx = (cols.astype(float) + 0.5) * grid.resolution
    ly = (rows.astype(float) + 0.5) * grid.resolution
    c = math.cos(grid.origin_yaw)
    s = math.sin(grid.origin_yaw)
    return (
        grid.origin_x + c * lx - s * ly,
        grid.origin_y + s * lx + c * ly,
    )


def nearest_cell_distance(
    grid: Optional[GridSnapshot],
    x: float,
    y: float,
    predicate: Any,
) -> Optional[float]:
    if grid is None:
        return None
    rows, cols = np.nonzero(predicate(grid.data))
    if not len(rows):
        return None
    world_x, world_y = grid_to_world(grid, cols, rows)
    return float(np.hypot(world_x - x, world_y - y).min())


def route_cell_summary(
    grid: Optional[GridSnapshot],
    start: tuple[float, float],
    end: tuple[float, float],
    kind: str,
) -> dict[str, Any]:
    result = {
        "samples": 0,
        "outside": 0,
        "unknown": 0,
        "free": 0,
        "physical": 0,
        "lethal": 0,
        "inflated": 0,
    }
    if grid is None:
        return result
    distance = math.hypot(end[0] - start[0], end[1] - start[1])
    count = max(2, int(distance / max(grid.resolution * 0.5, 0.01)) + 1)
    for fraction in np.linspace(0.0, 1.0, count):
        x = start[0] + fraction * (end[0] - start[0])
        y = start[1] + fraction * (end[1] - start[1])
        col, row = world_to_grid(grid, x, y)
        if not (0 <= col < grid.width and 0 <= row < grid.height):
            result["outside"] += 1
            continue
        value = int(grid.data[row, col])
        result["samples"] += 1
        if value < 0:
            result["unknown"] += 1
        elif kind == "map":
            if value >= 65:
                result["physical"] += 1
            else:
                result["free"] += 1
        else:
            if value >= 100:
                result["lethal"] += 1
            elif value > 0:
                result["inflated"] += 1
            else:
                result["free"] += 1
    return result


def path_length(poses: Iterable[Any]) -> float:
    positions = [(p.pose.position.x, p.pose.position.y) for p in poses]
    return float(
        sum(
            math.hypot(bx - ax, by - ay)
            for (ax, ay), (bx, by) in zip(positions, positions[1:])
        )
    )


def parameter_value(value: Any) -> tuple[str, str]:
    type_id = int(value.type)
    names = {
        0: "NOT_SET",
        1: "BOOL",
        2: "INTEGER",
        3: "DOUBLE",
        4: "STRING",
        5: "BYTE_ARRAY",
        6: "BOOL_ARRAY",
        7: "INTEGER_ARRAY",
        8: "DOUBLE_ARRAY",
        9: "STRING_ARRAY",
    }
    attr = {
        1: "bool_value",
        2: "integer_value",
        3: "double_value",
        4: "string_value",
        5: "byte_array_value",
        6: "bool_array_value",
        7: "integer_array_value",
        8: "double_array_value",
        9: "string_array_value",
    }.get(type_id)
    if attr is None:
        return names.get(type_id, f"TYPE_{type_id}"), ""
    item = getattr(value, attr)
    if isinstance(item, (list, tuple, np.ndarray)):
        text = "[" + ", ".join(str(x) for x in item) + "]"
    else:
        text = str(item)
    return names.get(type_id, f"TYPE_{type_id}"), text


def discover_bags(search_roots: Iterable[Path]) -> list[Path]:
    found = []
    for root in search_roots:
        if not root.exists():
            continue
        for metadata in root.glob("23july*.bag/metadata.yaml"):
            found.append(metadata.parent.resolve())
    return sorted(set(found), key=lambda p: natural_bag_key(p.name))


def natural_bag_key(name: str) -> tuple[int, str]:
    match = re.fullmatch(r"23july(\d*)\.bag", name)
    if not match:
        return (999, name)
    suffix = match.group(1)
    return (0 if suffix == "" else int(suffix), name)


def read_metadata(bag_path: Path) -> dict[str, Any]:
    with (bag_path / "metadata.yaml").open() as stream:
        root = yaml.safe_load(stream)
    return root["rosbag2_bagfile_information"]


def initial_result(bag_path: Path, metadata: dict[str, Any]) -> dict[str, Any]:
    start_ns = int(metadata["starting_time"]["nanoseconds_since_epoch"])
    duration_ns = int(metadata["duration"]["nanoseconds"])
    topics = []
    for item in metadata["topics_with_message_count"]:
        info = item["topic_metadata"]
        topics.append(
            {
                "name": info["name"],
                "type": info["type"],
                "count": int(item["message_count"]),
                "frequency_hz": (
                    float(item["message_count"]) / (duration_ns / 1e9)
                    if duration_ns
                    else 0.0
                ),
            }
        )
    return {
        "bag": bag_path.name,
        "path": str(bag_path),
        "storage": metadata["storage_identifier"],
        "start_ns": start_ns,
        "duration_ns": duration_ns,
        "message_count": int(metadata["message_count"]),
        "topics": topics,
        "target": None,
        "targets": [],
        "target_start_ns": None,
        "target_events": [],
        "goals": [],
        "nav2_events": [],
        "parameter_events": [],
        "decode_failures": {},
        "pose_streams": defaultdict(list),
        "trajectory_source": "",
        "trajectory": [],
        "final_grids": {},
        "footprints": defaultdict(list),
        "timestamp_regressions": 0,
        "filtered_messages_read": 0,
        "map_contexts": [],
        "behavior_recoveries": 0,
        "cmd_vel": [],
    }


def selected_topics(result: dict[str, Any]) -> list[str]:
    exact = {
        "/rosout",
        "/parameter_events",
        "/map",
        "/global_costmap/costmap",
        "/local_costmap/costmap",
        "/tf",
        "/tf_static",
        "/genz/odometry",
        "/odometry/filtered",
        "/pose",
        "/target_explorer/candidate_goals",
        "/target_explorer/selected_goal",
        "/plan",
        "/received_global_plan",
        "/behavior_tree_log",
        "/cmd_vel",
        "/cmd_vel_nav",
        "/global_costmap/published_footprint",
        "/local_costmap/published_footprint",
    }
    selected = set()
    for topic in result["topics"]:
        name = topic["name"]
        lower = name.lower()
        if (
            name in exact
            or "/_action/" in name
            or "navigate_to_pose" in lower
            or ("plan" in lower and topic["type"] == "nav_msgs/msg/Path")
        ):
            selected.add(name)
    return sorted(selected)


def make_event_base(
    result: dict[str, Any], timestamp_ns: int, node: str, topic: str
) -> dict[str, Any]:
    return {
        "bag": result["bag"],
        "absolute_time": stamp_text(timestamp_ns),
        "relative_sec": round(seconds(timestamp_ns, result["start_ns"]), 3),
        "source_node": node,
        "source_topic": topic,
    }


def current_goal_index(result: dict[str, Any]) -> Optional[int]:
    for index in range(len(result["goals"]) - 1, -1, -1):
        if not result["goals"][index].get("terminal_ns"):
            return index
    return None


def matching_goal_index(
    result: dict[str, Any], x: float, y: float
) -> Optional[int]:
    for index in range(len(result["goals"]) - 1, -1, -1):
        goal = result["goals"][index]
        if (
            abs(goal["goal_x"] - x) < 0.08
            and abs(goal["goal_y"] - y) < 0.08
            and not goal.get("terminal_ns")
        ):
            return index
    return current_goal_index(result)


def classify_nav_message(message: str) -> str:
    lower = message.lower()
    if "empty path" in lower:
        return "empty_path_error"
    if "progress" in lower and ("fail" in lower or "checker" in lower):
        return "progress_checker"
    if "patience" in lower:
        return "controller_patience"
    if "out of bounds" in lower or "outside" in lower and "costmap" in lower:
        return "robot_out_of_bounds"
    if "transform" in lower or " tf " in f" {lower} ":
        return "tf"
    if "resize" in lower or "resiz" in lower:
        return "costmap_resize"
    if "cancel" in lower:
        return "cancellation"
    if "recovery frontier" in lower:
        return "target_explorer_recovery_frontier"
    if "clear entirely" in lower or "clear costmap" in lower:
        return "costmap_clear"
    if any(term in lower for term in RECOVERY_TERMS):
        return "recovery"
    if "abort" in lower:
        return "aborted"
    if "goal reached" in lower or "succeed" in lower:
        return "goal_succeeded"
    if "received a goal" in lower or "begin navigating" in lower:
        return "goal_started"
    if "path" in lower or "plan" in lower:
        if "fail" in lower or "error" in lower:
            return "planning_failure"
        return "planning"
    if "control" in lower:
        return "controller"
    if "obstacle" in lower or "costmap" in lower:
        return "obstacle_or_costmap"
    if "fail" in lower or "error" in lower:
        return "failure"
    return "nav2_state"


def inspect_goal_map_context(
    result: dict[str, Any],
    goal: dict[str, Any],
    latest_grids: dict[str, GridSnapshot],
    latest_pose: Optional[tuple[float, float, float]],
) -> None:
    physical = latest_grids.get("/map")
    costmap = latest_grids.get("/global_costmap/costmap")
    x = float(goal["goal_x"])
    y = float(goal["goal_y"])
    physical_clearance = nearest_cell_distance(
        physical, x, y, lambda data: data >= 65
    )
    lethal_clearance = nearest_cell_distance(
        costmap, x, y, lambda data: data >= 100
    )
    goal["physical_map_clearance_m"] = physical_clearance
    goal["lethal_costmap_clearance_m"] = lethal_clearance
    clearances = [
        value for value in (physical_clearance, lethal_clearance)
        if value is not None
    ]
    goal["initial_endpoint_clearance_m"] = min(clearances) if clearances else None
    if latest_pose is None or result["target"] is None:
        goal["map_context_interpretation"] = "Map context unavailable."
        return
    start = (latest_pose[0], latest_pose[1])
    target = (result["target"][0], result["target"][1])
    selected = (x, y)
    direct_map = route_cell_summary(physical, start, target, "map")
    direct_cost = route_cell_summary(costmap, start, target, "costmap")
    selected_map = route_cell_summary(physical, start, selected, "map")
    selected_cost = route_cell_summary(costmap, start, selected, "costmap")
    goal["direct_route_physical_blocked"] = direct_map["physical"] > 0
    goal["direct_route_lethal_blocked"] = direct_cost["lethal"] > 0
    goal["direct_route_inflated_cells"] = direct_cost["inflated"]
    goal["selected_route_physical_blocked"] = selected_map["physical"] > 0
    goal["_map_context"] = {
        "map_age_sec": (
            seconds(goal["dispatch_ns"], physical.timestamp_ns)
            if physical else None
        ),
        "costmap_age_sec": (
            seconds(goal["dispatch_ns"], costmap.timestamp_ns)
            if costmap else None
        ),
        "direct_map": direct_map,
        "direct_costmap": direct_cost,
        "selected_map": selected_map,
        "selected_costmap": selected_cost,
    }
    if direct_map["physical"] > 0 and selected_map["physical"] == 0:
        interpretation = (
            "Direct final-target line crossed recorded physical occupied cells; "
            "the selected segment avoided them, supporting a geometric detour."
        )
    elif direct_cost["lethal"] > 0 and selected_cost["lethal"] == 0:
        interpretation = (
            "Direct final-target line crossed lethal costmap cells while the "
            "selected segment did not; a side route was geometrically plausible."
        )
    elif direct_cost["inflated"] > 0 and direct_map["physical"] == 0:
        interpretation = (
            "Direct line crossed inflation-gradient cells but no physical "
            "occupied map cell; inflation is not treated as a physical curb."
        )
    elif direct_map["physical"] == 0 and direct_cost["lethal"] == 0:
        interpretation = (
            "Cell-center sampling did not show a direct-line physical/lethal "
            "block at this instant; logs and planned-path reachability remain "
            "the stronger decision evidence."
        )
    else:
        interpretation = (
            "Both direct and selected straight segments intersected occupied "
            "or lethal cells; the full Nav2 path, not the endpoint chord, "
            "determines reachability."
        )
    goal["map_context_interpretation"] = interpretation


def process_target_log(
    result: dict[str, Any],
    timestamp_ns: int,
    node: str,
    message: str,
    severity: str,
    latest_grids: dict[str, GridSnapshot],
    latest_tf_pose: Optional[tuple[float, float, float]],
) -> None:
    base = make_event_base(result, timestamp_ns, node, "/rosout")
    event = {
        **base,
        "event_category": "target_explorer_log",
        "candidate_type": "",
        "goal_x": "",
        "goal_y": "",
        "decision": "",
        "direct_reason": message,
        "clearance_m": "",
        "required_clearance_m": "",
        "progress_m": "",
        "alignment": "",
        "path_length_m": "",
        "score": "",
        "outcome": "",
    }
    match = TARGET_RE["startup"].search(message)
    if match:
        result["target"] = (float(match.group(1)), float(match.group(2)))
        result["target_start_ns"] = timestamp_ns
        result["targets"].append(
            (timestamp_ns, float(match.group(1)), float(match.group(2)))
        )
        event.update(
            event_category="startup_final_target",
            candidate_type="final_target",
            goal_x=float(match.group(1)),
            goal_y=float(match.group(2)),
            decision="configured_at_startup",
        )
        result["target_events"].append(event)
        return
    match = TARGET_RE["rejection"].search(message)
    if match:
        event.update(
            event_category="endpoint_clearance_rejection",
            candidate_type=match.group(1).replace(" ", "_"),
            goal_x=float(match.group(2)),
            goal_y=float(match.group(3)),
            decision="rejected",
            clearance_m=float(match.group(4)),
            required_clearance_m=float(match.group(5)),
        )
        result["target_events"].append(event)
        return
    match = TARGET_RE["dispatch"].search(message)
    if match:
        goal = {
            "bag": result["bag"],
            "dispatch_ns": timestamp_ns,
            "dispatch_time": stamp_text(timestamp_ns),
            "relative_sec": round(seconds(timestamp_ns, result["start_ns"]), 3),
            "goal_number": len(result["goals"]) + 1,
            "goal_x": float(match.group(1)),
            "goal_y": float(match.group(2)),
            "final_target_x": (
                float(result["target"][0]) if result["target"] else None
            ),
            "final_target_y": (
                float(result["target"][1]) if result["target"] else None
            ),
            "progress_m": float(match.group(3)),
            "alignment": float(match.group(4)),
            "path_length_m": float(match.group(5)),
            "score": float(match.group(6)),
            "nav2_accepted": False,
            "later_canceled": False,
            "cancellation_reason": "",
            "outcome": "NO_TERMINAL_RESULT_IN_BAG",
            "action_status": "",
            "recoveries": [],
        }
        inspect_goal_map_context(
            result, goal, latest_grids, latest_tf_pose
        )
        result["goals"].append(goal)
        event.update(
            event_category="goal_dispatch",
            candidate_type="final_or_frontier",
            goal_x=goal["goal_x"],
            goal_y=goal["goal_y"],
            decision="dispatched",
            progress_m=goal["progress_m"],
            alignment=goal["alignment"],
            path_length_m=goal["path_length_m"],
            score=goal["score"],
        )
        result["target_events"].append(event)
        return
    for key, category, outcome in (
        ("accepted", "nav2_acceptance", ""),
        ("rejected_goal", "nav2_rejection", "REJECTED"),
        ("succeeded", "action_result", "SUCCEEDED"),
        ("canceled", "action_result", "CANCELED"),
    ):
        match = TARGET_RE[key].search(message)
        if match:
            x = float(match.group(1))
            y = float(match.group(2))
            index = matching_goal_index(result, x, y)
            if index is not None:
                goal = result["goals"][index]
                if key == "accepted":
                    goal["nav2_accepted"] = True
                    goal["accepted_ns"] = timestamp_ns
                else:
                    goal["outcome"] = outcome
                    goal["action_status"] = outcome
                    goal["terminal_ns"] = timestamp_ns
                    if key == "canceled":
                        goal["later_canceled"] = True
            event.update(
                event_category=category,
                goal_x=x,
                goal_y=y,
                decision=key,
                outcome=outcome,
            )
            result["target_events"].append(event)
            return
    match = TARGET_RE["finished"].search(message)
    if match:
        x, y = float(match.group(1)), float(match.group(2))
        status = int(match.group(3))
        outcome = STATUS_NAMES.get(status, f"STATUS_{status}")
        index = matching_goal_index(result, x, y)
        if index is not None:
            goal = result["goals"][index]
            goal["outcome"] = outcome
            goal["action_status"] = f"{status} ({outcome})"
            goal["terminal_ns"] = timestamp_ns
            if status == 5:
                goal["later_canceled"] = True
        event.update(
            event_category="action_result",
            goal_x=x,
            goal_y=y,
            decision="terminal_result",
            outcome=f"{status} ({outcome})",
        )
        result["target_events"].append(event)
        return
    match = TARGET_RE["safety_cancel"].search(message)
    if match:
        x, y, reason = float(match.group(1)), float(match.group(2)), match.group(3)
        index = matching_goal_index(result, x, y)
        if index is not None:
            goal = result["goals"][index]
            goal["later_canceled"] = True
            goal["cancel_requested_ns"] = timestamp_ns
            goal["cancellation_reason"] = reason
        event.update(
            event_category="active_goal_safety_cancellation",
            goal_x=x,
            goal_y=y,
            decision="cancel_requested",
            direct_reason=reason,
        )
        result["target_events"].append(event)
        return
    match = TARGET_RE["recovery_frontier"].search(message)
    if match:
        event.update(
            event_category="recovery_frontier",
            candidate_type="frontier",
            goal_x=float(match.group(1)),
            goal_y=float(match.group(2)),
            decision="selected_as_recovery",
        )
        result["target_events"].append(event)
        return
    lower = message.lower()
    if "no safe frontier endpoint" in lower:
        event.update(
            event_category="no_valid_frontier",
            decision="no_goal",
        )
    elif "no reachable safe frontier" in lower:
        event.update(
            event_category="no_reachable_frontier",
            decision="no_goal",
        )
    elif "cannot look up robot pose" in lower or "cannot transform point" in lower:
        event.update(event_category="tf_failure", decision="deferred")
    elif "final target is reachable and safe" in lower:
        event.update(
            event_category="final_target_direct",
            candidate_type="final_target",
            decision="selected",
        )
    elif "waiting for" in lower:
        event.update(event_category="waiting", decision="deferred")
    elif "timed out" in lower:
        event.update(event_category="planner_timeout", decision="failed")
    elif "cancellation" in lower:
        event.update(event_category="cancellation_state")
    elif severity in ("WARN", "ERROR", "FATAL"):
        event.update(event_category="target_explorer_warning")
    else:
        return
    result["target_events"].append(event)


def process_rosout(
    result: dict[str, Any],
    timestamp_ns: int,
    msg: Any,
    latest_grids: dict[str, GridSnapshot],
    latest_tf_pose: Optional[tuple[float, float, float]],
) -> None:
    node = str(msg.name)
    message = str(msg.msg)
    severity = {
        10: "DEBUG",
        20: "INFO",
        30: "WARN",
        40: "ERROR",
        50: "FATAL",
    }.get(int(msg.level), str(msg.level))
    if "target_explorer" in node:
        process_target_log(
            result,
            timestamp_ns,
            node,
            message,
            severity,
            latest_grids,
            latest_tf_pose,
        )
    lower_node = node.lower()
    lower_message = message.lower()
    if not any(hint in lower_node for hint in NAV_NODE_HINTS):
        return
    if not any(term in lower_message for term in NAV_EVENT_TERMS):
        return
    event = {
        **make_event_base(result, timestamp_ns, node, "/rosout"),
        "event_category": classify_nav_message(message),
        "severity": severity,
        "goal_x": "",
        "goal_y": "",
        "action_status": "",
        "path_pose_count": "",
        "path_length_m": "",
        "message": message,
        "active_goal_number": "",
    }
    coordinates = re.search(r"\(([-+\d.]+),\s*([-+\d.]+)\)", message)
    if coordinates:
        event["goal_x"] = float(coordinates.group(1))
        event["goal_y"] = float(coordinates.group(2))
    status_match = re.search(r"status (\d+)", message)
    if status_match:
        status = int(status_match.group(1))
        event["action_status"] = f"{status} ({STATUS_NAMES.get(status, 'UNMAPPED')})"
    active = current_goal_index(result)
    if active is not None:
        event["active_goal_number"] = active + 1
        if event["event_category"] == "recovery":
            result["goals"][active]["recoveries"].append(message)
    result["nav2_events"].append(event)


def process_parameter_event(
    result: dict[str, Any], timestamp_ns: int, msg: Any
) -> None:
    for change_kind, values in (
        ("new", msg.new_parameters),
        ("changed", msg.changed_parameters),
        ("deleted", msg.deleted_parameters),
    ):
        for parameter in values:
            type_name, text = parameter_value(parameter.value)
            result["parameter_events"].append(
                {
                    "bag": result["bag"],
                    "absolute_time": stamp_text(timestamp_ns),
                    "relative_sec": round(
                        seconds(timestamp_ns, result["start_ns"]), 3
                    ),
                    "node": msg.node,
                    "change_kind": change_kind,
                    "parameter": parameter.name,
                    "parameter_type": type_name,
                    "value": text,
                }
            )


def process_tf(
    result: dict[str, Any],
    timestamp_ns: int,
    msg: Any,
    transforms: dict[tuple[str, str], tuple[float, float, float]],
) -> Optional[tuple[float, float, float]]:
    for transform in msg.transforms:
        parent = transform.header.frame_id.lstrip("/")
        child = transform.child_frame_id.lstrip("/")
        transforms[(parent, child)] = (
            float(transform.transform.translation.x),
            float(transform.transform.translation.y),
            yaw_from_quaternion(transform.transform.rotation),
        )
    pose = resolve_transform(transforms, "map", "base_footprint")
    if pose is None:
        pose = resolve_transform(transforms, "map", "base_link")
    if pose is not None:
        stream = result["pose_streams"]["tf_map_base"]
        if (
            not stream
            or timestamp_ns - stream[-1][0] >= 20_000_000
            or math.hypot(pose[0] - stream[-1][1], pose[1] - stream[-1][2])
            > 0.005
        ):
            stream.append((timestamp_ns, pose[0], pose[1], pose[2]))
    return pose


def process_pose_stream(
    result: dict[str, Any], topic: str, timestamp_ns: int, msg: Any
) -> None:
    if topic == "/pose":
        pose = msg.pose.pose
        frame = msg.header.frame_id
    else:
        pose = msg.pose.pose
        frame = msg.header.frame_id
    result["pose_streams"][topic].append(
        (
            timestamp_ns,
            float(pose.position.x),
            float(pose.position.y),
            yaw_from_quaternion(pose.orientation),
            frame,
        )
    )


def process_path(result: dict[str, Any], topic: str, timestamp_ns: int, msg: Any) -> None:
    if topic == "/received_global_plan":
        return
    poses = list(msg.poses)
    event = {
        **make_event_base(result, timestamp_ns, topic, topic),
        "event_category": "path_update",
        "severity": "RECORDED",
        "goal_x": float(poses[-1].pose.position.x) if poses else "",
        "goal_y": float(poses[-1].pose.position.y) if poses else "",
        "action_status": "",
        "path_pose_count": len(poses),
        "path_length_m": round(path_length(poses), 4) if poses else 0.0,
        "message": f"Recorded path update on {topic}",
        "active_goal_number": "",
    }
    active = current_goal_index(result)
    if active is not None:
        event["active_goal_number"] = active + 1
    result["nav2_events"].append(event)


def process_behavior_tree(
    result: dict[str, Any], timestamp_ns: int, msg: Any
) -> None:
    for change in msg.event_log:
        name = str(change.node_name)
        lower = name.lower()
        concrete_recovery = (
            lower in {"spin", "wait", "backup", "clearingactions"}
            or lower.startswith("clear")
        )
        if (
            change.current_status != "RUNNING"
            or not concrete_recovery
        ):
            continue
        result["behavior_recoveries"] += 1
        active = current_goal_index(result)
        description = (
            f"Behavior-tree recovery entered RUNNING: {name} "
            f"({change.previous_status} -> {change.current_status})"
        )
        if active is not None:
            result["goals"][active]["recoveries"].append(description)
        result["nav2_events"].append(
            {
                **make_event_base(
                    result, timestamp_ns, "bt_navigator", "/behavior_tree_log"
                ),
                "event_category": "recovery",
                "severity": "RECORDED",
                "goal_x": "",
                "goal_y": "",
                "action_status": "",
                "path_pose_count": "",
                "path_length_m": "",
                "message": description,
                "active_goal_number": active + 1 if active is not None else "",
            }
        )


def analyze_bag(bag_path: Path) -> dict[str, Any]:
    metadata = read_metadata(bag_path)
    result = initial_result(bag_path, metadata)
    if result["storage"] != "sqlite3":
        raise RuntimeError(
            f"{bag_path}: storage {result['storage']} is not supported by this "
            "SQLite3-focused analyzer"
        )
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id=result["storage"]),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    topic_types = {
        topic.name: topic.type for topic in reader.get_all_topics_and_types()
    }
    chosen_topics = selected_topics(result)
    reader.set_filter(rosbag2_py.StorageFilter(topics=chosen_topics))
    message_classes = {}
    for topic in chosen_topics:
        try:
            message_classes[topic] = get_message(topic_types[topic])
        except Exception as error:
            result["decode_failures"][topic] = {
                "type": topic_types.get(topic, ""),
                "count": 0,
                "first_error": f"message type unavailable: {error}",
            }
    latest_grids: dict[str, GridSnapshot] = {}
    transforms: dict[tuple[str, str], tuple[float, float, float]] = {}
    latest_tf_pose = None
    previous_timestamp = None
    while reader.has_next():
        topic, serialized, timestamp_ns = reader.read_next()
        result["filtered_messages_read"] += 1
        timestamp_ns = int(timestamp_ns)
        if previous_timestamp is not None and timestamp_ns < previous_timestamp:
            result["timestamp_regressions"] += 1
        previous_timestamp = timestamp_ns
        message_class = message_classes.get(topic)
        if message_class is None:
            failure = result["decode_failures"].setdefault(
                topic,
                {
                    "type": topic_types.get(topic, ""),
                    "count": 0,
                    "first_error": "message class unavailable",
                },
            )
            failure["count"] += 1
            continue
        try:
            msg = deserialize_message(serialized, message_class)
        except Exception as error:
            failure = result["decode_failures"].setdefault(
                topic,
                {
                    "type": topic_types.get(topic, ""),
                    "count": 0,
                    "first_error": str(error),
                },
            )
            failure["count"] += 1
            continue
        try:
            if topic in (
                "/map",
                "/global_costmap/costmap",
                "/local_costmap/costmap",
            ):
                latest_grids[topic] = grid_from_message(topic, timestamp_ns, msg)
            elif topic == "/rosout":
                process_rosout(
                    result,
                    timestamp_ns,
                    msg,
                    latest_grids,
                    latest_tf_pose,
                )
            elif topic == "/parameter_events":
                process_parameter_event(result, timestamp_ns, msg)
            elif topic in ("/tf", "/tf_static"):
                pose = process_tf(result, timestamp_ns, msg, transforms)
                if pose is not None:
                    latest_tf_pose = pose
            elif topic in ("/genz/odometry", "/odometry/filtered", "/pose"):
                process_pose_stream(result, topic, timestamp_ns, msg)
            elif topic in (
                "/global_costmap/published_footprint",
                "/local_costmap/published_footprint",
            ):
                vertices = [
                    (float(point.x), float(point.y))
                    for point in msg.polygon.points
                ]
                if vertices:
                    center_x = statistics.mean(point[0] for point in vertices)
                    center_y = statistics.mean(point[1] for point in vertices)
                    radii = [
                        math.hypot(point[0] - center_x, point[1] - center_y)
                        for point in vertices
                    ]
                    result["footprints"][topic].append(
                        (timestamp_ns, max(radii), len(radii))
                    )
            elif topic in ("/plan", "/received_global_plan") or (
                topic_types.get(topic) == "nav_msgs/msg/Path"
                and "plan" in topic.lower()
            ):
                process_path(result, topic, timestamp_ns, msg)
            elif topic == "/behavior_tree_log":
                process_behavior_tree(result, timestamp_ns, msg)
            elif topic in ("/cmd_vel", "/cmd_vel_nav"):
                if topic == "/cmd_vel":
                    result["cmd_vel"].append(
                        (
                            timestamp_ns,
                            float(msg.linear.x),
                            float(msg.angular.z),
                        )
                    )
            elif "/_action/status" in topic:
                for status in msg.status_list:
                    code = int(status.status)
                    result["nav2_events"].append(
                        {
                            **make_event_base(result, timestamp_ns, topic, topic),
                            "event_category": "action_status",
                            "severity": "RECORDED",
                            "goal_x": "",
                            "goal_y": "",
                            "action_status": (
                                f"{code} ({STATUS_NAMES.get(code, 'UNMAPPED')})"
                            ),
                            "path_pose_count": "",
                            "path_length_m": "",
                            "message": "Recorded action status",
                            "active_goal_number": "",
                        }
                    )
        except Exception as error:
            failure = result["decode_failures"].setdefault(
                f"{topic}#analysis",
                {
                    "type": topic_types.get(topic, ""),
                    "count": 0,
                    "first_error": str(error),
                },
            )
            failure["count"] += 1
    result["final_grids"] = latest_grids
    finalize_result(result)
    return result


def cleaned_trajectory(
    stream: list[tuple[Any, ...]], max_speed: float = 5.0
) -> list[tuple[int, float, float, float]]:
    clean: list[tuple[int, float, float, float]] = []
    for item in stream:
        point = (int(item[0]), float(item[1]), float(item[2]), float(item[3]))
        if clean:
            dt = (point[0] - clean[-1][0]) / 1e9
            distance = math.hypot(point[1] - clean[-1][1], point[2] - clean[-1][2])
            if dt <= 0:
                continue
            if dt < 1.0 and distance / dt > max_speed:
                continue
        clean.append(point)
    return clean


def trajectory_metrics(
    trajectory: list[tuple[int, float, float, float]],
    target: Optional[tuple[float, float]],
    target_start_ns: Optional[int] = None,
) -> dict[str, Any]:
    metrics = {
        "samples": len(trajectory),
        "distance_m": 0.0,
        "net_progress_m": None,
        "lateral_movement_m": 0.0,
        "stationary_sec": 0.0,
        "start": None,
        "end": None,
    }
    if not trajectory:
        return metrics
    metrics["start"] = (trajectory[0][1], trajectory[0][2])
    metrics["end"] = (trajectory[-1][1], trajectory[-1][2])
    total = 0.0
    stationary = 0.0
    lateral = 0.0
    mission_trajectory = (
        [point for point in trajectory if point[0] >= target_start_ns]
        if target_start_ns is not None
        else trajectory
    )
    mission_start = mission_trajectory[0] if mission_trajectory else trajectory[0]
    if target is not None:
        axis_x = target[0] - mission_start[1]
        axis_y = target[1] - mission_start[2]
        norm = math.hypot(axis_x, axis_y)
        perp = (-axis_y / norm, axis_x / norm) if norm else (0.0, 0.0)
    else:
        perp = (0.0, 0.0)
    for first, second in zip(trajectory, trajectory[1:]):
        dt = (second[0] - first[0]) / 1e9
        if dt <= 0 or dt > 2.0:
            continue
        dx = second[1] - first[1]
        dy = second[2] - first[2]
        distance = math.hypot(dx, dy)
        if distance / dt > 2.0:
            # A map/SLAM reset is a frame discontinuity, not rover travel.
            continue
        total += distance
        if target_start_ns is None or first[0] >= target_start_ns:
            lateral += abs(dx * perp[0] + dy * perp[1])
        if distance / dt < 0.05:
            stationary += dt
    metrics["distance_m"] = total
    metrics["lateral_movement_m"] = lateral
    metrics["stationary_sec"] = stationary
    if target is not None and mission_trajectory:
        start_distance = math.hypot(
            mission_trajectory[0][1] - target[0],
            mission_trajectory[0][2] - target[1],
        )
        end_distance = math.hypot(
            mission_trajectory[-1][1] - target[0],
            mission_trajectory[-1][2] - target[1],
        )
        metrics["net_progress_m"] = start_distance - end_distance
    return metrics


def distance_in_window(
    trajectory: list[tuple[int, float, float, float]],
    start_ns: int,
    end_ns: int,
) -> float:
    points = [point for point in trajectory if start_ns <= point[0] <= end_ns]
    return sum(
        math.hypot(second[1] - first[1], second[2] - first[2])
        for first, second in zip(points, points[1:])
        if (
            0 < (second[0] - first[0]) / 1e9 <= 2.0
            and math.hypot(second[1] - first[1], second[2] - first[2])
            / ((second[0] - first[0]) / 1e9)
            <= 2.0
        )
    )


def nearest_deviations(
    reference: list[tuple[int, float, float, float]],
    candidate: list[tuple[Any, ...]],
) -> list[float]:
    if not reference or not candidate:
        return []
    ref_times = np.asarray([item[0] for item in reference], dtype=np.int64)
    deviations = []
    for item in candidate[:: max(1, len(candidate) // 500)]:
        timestamp = int(item[0])
        index = int(np.searchsorted(ref_times, timestamp))
        options = [x for x in (index - 1, index) if 0 <= x < len(reference)]
        if not options:
            continue
        best = min(options, key=lambda x: abs(int(ref_times[x]) - timestamp))
        if abs(int(ref_times[best]) - timestamp) <= 500_000_000:
            deviations.append(
                math.hypot(reference[best][1] - item[1], reference[best][2] - item[2])
            )
    return deviations


def finalize_result(result: dict[str, Any]) -> None:
    tf_stream = cleaned_trajectory(result["pose_streams"].get("tf_map_base", []))
    pose_stream = cleaned_trajectory(result["pose_streams"].get("/pose", []))
    if len(tf_stream) >= 10:
        result["trajectory_source"] = "TF map -> base_footprint/base_link"
        result["trajectory"] = tf_stream
    elif len(pose_stream) >= 10:
        result["trajectory_source"] = "/pose (map-frame SLAM pose)"
        result["trajectory"] = pose_stream
    else:
        result["trajectory_source"] = "/genz/odometry (local frame; fallback)"
        result["trajectory"] = cleaned_trajectory(
            result["pose_streams"].get("/genz/odometry", [])
        )
    result["trajectory_metrics"] = trajectory_metrics(
        result["trajectory"], result["target"], result["target_start_ns"]
    )
    comparisons = {}
    for name, stream in result["pose_streams"].items():
        clean = cleaned_trajectory(stream)
        metrics = trajectory_metrics(
            clean, result["target"], result["target_start_ns"]
        )
        deviations = nearest_deviations(result["trajectory"], clean)
        metrics["median_deviation_from_chosen_m"] = (
            statistics.median(deviations) if deviations else None
        )
        metrics["frame_ids"] = sorted(
            {str(item[4]) for item in stream if len(item) > 4}
        )
        comparisons[name] = metrics
    result["pose_comparison"] = comparisons
    for goal in result["goals"]:
        end_ns = int(goal.get("terminal_ns", result["start_ns"] + result["duration_ns"]))
        active_start = int(goal.get("accepted_ns", goal["dispatch_ns"]))
        goal["time_active_sec"] = max(0.0, (end_ns - active_start) / 1e9)
        goal["distance_travelled_while_active_m"] = distance_in_window(
            result["trajectory"], active_start, end_ns
        )
        goal["recoveries_triggered"] = len(goal["recoveries"])
        if goal["outcome"] == "NO_TERMINAL_RESULT_IN_BAG" and goal["nav2_accepted"]:
            goal["outcome"] = "ACTIVE_OR_INTERRUPTED_AT_BAG_END"
        for key in (
            "initial_endpoint_clearance_m",
            "physical_map_clearance_m",
            "lethal_costmap_clearance_m",
        ):
            if goal.get(key) is not None:
                goal[key] = round(float(goal[key]), 4)
    required_values = [
        event["required_clearance_m"]
        for event in result["target_events"]
        if event["event_category"] == "endpoint_clearance_rejection"
    ]
    result["clearance_requirement"] = (
        Counter(required_values).most_common(1)[0][0] if required_values else None
    )
    footprint_summary = {}
    for topic, observations in result["footprints"].items():
        radii = [item[1] for item in observations]
        counts = [item[2] for item in observations]
        footprint_summary[topic] = {
            "median_radius_m": statistics.median(radii) if radii else None,
            "min_radius_m": min(radii) if radii else None,
            "max_radius_m": max(radii) if radii else None,
            "median_vertices": statistics.median(counts) if counts else None,
        }
    result["footprint_summary"] = footprint_summary


def write_csv(path: Path, rows: Iterable[dict[str, Any]], columns: Iterable[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(columns), extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def goal_csv_rows(results: list[dict[str, Any]]) -> list[dict[str, Any]]:
    rows = []
    for result in results:
        for goal in result["goals"]:
            row = dict(goal)
            row["later_canceled"] = "yes" if goal["later_canceled"] else "no"
            row["nav2_accepted"] = "yes" if goal["nav2_accepted"] else "no"
            row["recoveries_triggered"] = goal["recoveries_triggered"]
            rows.append(row)
    return rows


def parameter_comparison_rows(results: list[dict[str, Any]]) -> list[dict[str, Any]]:
    bag_names = [result["bag"] for result in results]
    rows = []

    def add(parameter: str, values: list[str], evidence: str) -> None:
        row = {"parameter": parameter, "evidence/confidence": evidence}
        row.update(dict(zip(bag_names, values)))
        rows.append(row)

    add(
        "target_explorer.goal_obstacle_clearance_m",
        [
            (
                f"{result['clearance_requirement']:.2f} "
                "[CONFIRMED_IN_BAG]"
                if result["clearance_requirement"] is not None
                else "UNKNOWN"
            )
            for result in results
        ],
        "Required clearance is written verbatim in Target Explorer /rosout.",
    )
    add(
        "target_explorer.final_target_(x,y)",
        [
            (
                f"({result['target'][0]:.2f}, {result['target'][1]:.2f}) "
                "[CONFIRMED_IN_BAG]"
                if result["target"]
                else "UNKNOWN"
            )
            for result in results
        ],
        "Startup /rosout records the final local target.",
    )
    for scope, topic in (
        ("local_costmap", "/local_costmap/published_footprint"),
        ("global_costmap", "/global_costmap/published_footprint"),
    ):
        add(
            f"{scope}.effective_published_footprint_radius_m",
            [
                (
                    f"{result['footprint_summary'][topic]['median_radius_m']:.3f} "
                    "[CONFIRMED_IN_BAG]"
                    if topic in result["footprint_summary"]
                    else "UNKNOWN"
                )
                for result in results
            ],
            (
                "Derived from recorded published-footprint vertices. This confirms "
                "effective geometry, not whether robot_radius or footprint produced it."
            ),
        )
        add(
            f"{scope}.robot_radius",
            ["1.0 [STRONGLY_INFERRED]"] * len(results),
            (
                "robot_radius is 1.0 in both the git baseline and matching installed "
                "Nav2 YAML; the separately reported recorded footprint geometry is "
                "consistent with this value. No static event emitted the config key."
            ),
        )
    for scope, topic in (
        ("map", "/map"),
        ("global_costmap", "/global_costmap/costmap"),
        ("local_costmap", "/local_costmap/costmap"),
    ):
        add(
            f"{scope}.resolution",
            [
                (
                    f"{result['final_grids'][topic].resolution:.3f} "
                    "[CONFIRMED_IN_BAG]"
                    if topic in result["final_grids"]
                    else "UNKNOWN"
                )
                for result in results
            ],
            "Recorded OccupancyGrid metadata.",
        )
    # Historical inference is deliberately explicit and test-specific.
    inferred_inflation = []
    for result in results:
        index = natural_bag_key(result["bag"])[0]
        if index <= 2:
            inferred_inflation.append("0.50 [STRONGLY_INFERRED]")
        else:
            inferred_inflation.append("0.95 [STRONGLY_INFERRED]")
    for scope in ("local_costmap", "global_costmap"):
        add(
            f"{scope}.inflation_radius",
            inferred_inflation,
            (
                "Git baseline was 0.50; installed YAML became 0.95 at 21:22:36. "
                "Nav2 for 23july/1/2 started before that copy; Nav2 for 23july3/4 "
                "started after it. Static values were not emitted in bag events."
            ),
        )
    add(
        "local_costmap.inflation_layer.cost_scaling_factor",
        ["5.0 [STRONGLY_INFERRED]"] * len(results),
        "Value is unchanged across git baseline, source, and installed matching YAML.",
    )
    add(
        "global_costmap.inflation_layer.cost_scaling_factor",
        ["3.0 [STRONGLY_INFERRED]"] * len(results),
        "Value is unchanged across git baseline, source, and installed matching YAML.",
    )
    stable_inferred = (
        (
            "local_costmap.obstacle_layer.scan.obstacle_min_range",
            "0.1",
        ),
        (
            "local_costmap.obstacle_layer.scan.obstacle_max_range",
            "6.0",
        ),
        (
            "local_costmap.obstacle_layer.scan.raytrace_max_range",
            "6.0",
        ),
        (
            "local_costmap.obstacle_layer.scan.min_obstacle_height",
            "0.2",
        ),
        (
            "local_costmap.obstacle_layer.scan.max_obstacle_height",
            "2.0",
        ),
        (
            "global_costmap.obstacle_layer.scan.obstacle_min_range",
            "0.1",
        ),
        (
            "global_costmap.obstacle_layer.scan.obstacle_max_range",
            "20.0",
        ),
        (
            "global_costmap.obstacle_layer.scan.raytrace_min_range",
            "0.0",
        ),
        (
            "global_costmap.obstacle_layer.scan.raytrace_max_range",
            "20.0",
        ),
        (
            "global_costmap.obstacle_layer.scan.min_obstacle_height",
            "0.20",
        ),
        (
            "global_costmap.obstacle_layer.scan.max_obstacle_height",
            "1.85",
        ),
        ("planner_server.GridBased.tolerance", "0.5"),
        ("planner_server.GridBased.allow_unknown", "false"),
        ("controller_server.FollowPath.lookahead_dist", "1.0"),
        ("controller_server.FollowPath.min_lookahead_dist", "0.8"),
        ("controller_server.FollowPath.max_lookahead_dist", "1.2"),
    )
    for parameter, value in stable_inferred:
        add(
            parameter,
            [f"{value} [STRONGLY_INFERRED]"] * len(results),
            (
                "Unchanged across the pre-test git baseline and matching installed "
                "Nav2 YAML; not emitted as a static ParameterEvent."
            ),
        )
    for parameter in (
        "local_costmap.footprint",
        "global_costmap.footprint",
        "local_costmap.footprint_padding",
        "global_costmap.footprint_padding",
        "local_costmap.obstacle_layer.scan.raytrace_min_range",
        "controller_server.desired_linear_vel",
        "controller_server.goal_checker.xy_goal_tolerance",
        "controller_server.goal_checker.yaw_goal_tolerance",
        "controller_server.progress_checker.movement_time_allowance",
        "controller_server.progress_checker.required_movement_radius",
    ):
        add(
            parameter,
            ["UNKNOWN"] * len(results),
            (
                "No matching static ParameterEvent or explicit startup log. "
                "Published geometry is reported separately and current YAML is not "
                "silently substituted."
            ),
        )
    for parameter in (
        "map_occupied_threshold",
        "costmap_lethal_threshold",
        "active_goal_safety_check_period_sec",
    ):
        values = []
        for result in results:
            matches = [
                event["value"]
                for event in result["parameter_events"]
                if event["node"].rstrip("/") == "/target_explorer_node"
                and event["parameter"] == parameter
            ]
            values.append(
                f"{matches[-1]} [CONFIRMED_IN_BAG]" if matches else "UNKNOWN"
            )
        add(
            f"target_explorer.{parameter}",
            values,
            "Decoded `/target_explorer_node` ParameterEvent during the recording.",
        )
    observed: dict[tuple[str, str], dict[str, str]] = defaultdict(dict)
    for result in results:
        for event in result["parameter_events"]:
            if any(
                event["parameter"] == name
                or event["parameter"].endswith("." + name)
                for name in IMPORTANT_PARAMETERS
            ) and not (
                event["node"].rstrip("/") == "/target_explorer_node"
                and event["parameter"] in {
                    "goal_obstacle_clearance_m",
                    "map_occupied_threshold",
                    "active_goal_safety_check_period_sec",
                }
            ):
                observed[(event["node"], event["parameter"])][result["bag"]] = (
                    f"{event['value']} [CONFIRMED_IN_BAG]"
                )
    for (node, parameter), values in sorted(observed.items()):
        add(
            f"{node}.{parameter}",
            [values.get(result["bag"], "UNKNOWN") for result in results],
            "Recorded /parameter_events change.",
        )
    return rows


def plot_grid(ax: Any, grid: Optional[GridSnapshot], kind: str) -> None:
    if grid is None:
        ax.text(0.5, 0.5, "No grid recorded", ha="center", va="center")
        return
    data = grid.data
    if kind == "map":
        rendered = np.full(data.shape, 0.0)
        rendered[data < 0] = 0.5
        rendered[data >= 65] = 1.0
        cmap = matplotlib.colors.ListedColormap(["white", "lightgray", "black"])
        bounds = [-0.1, 0.25, 0.75, 1.1]
    else:
        rendered = np.full(data.shape, 0.0)
        rendered[data < 0] = 0.25
        rendered[(data > 0) & (data < 100)] = 0.65
        rendered[data >= 100] = 1.0
        cmap = matplotlib.colors.ListedColormap(
            ["white", "lightgray", "#f5b642", "black"]
        )
        bounds = [-0.1, 0.15, 0.45, 0.85, 1.1]
    norm = matplotlib.colors.BoundaryNorm(bounds, cmap.N)
    width_m = grid.width * grid.resolution
    height_m = grid.height * grid.resolution
    if abs(grid.origin_yaw) < 1e-6:
        ax.imshow(
            rendered,
            origin="lower",
            extent=(
                grid.origin_x,
                grid.origin_x + width_m,
                grid.origin_y,
                grid.origin_y + height_m,
            ),
            interpolation="nearest",
            cmap=cmap,
            norm=norm,
            alpha=0.82,
        )
    else:
        stride = max(1, int(max(grid.width, grid.height) / 450))
        rows, cols = np.indices(data.shape)
        rows = rows[::stride, ::stride]
        cols = cols[::stride, ::stride]
        x, y = grid_to_world(grid, cols.ravel(), rows.ravel())
        ax.scatter(
            x,
            y,
            c=rendered[::stride, ::stride].ravel(),
            cmap=cmap,
            norm=norm,
            marker="s",
            s=2,
            alpha=0.7,
        )


def plot_result(result: dict[str, Any], plots_dir: Path) -> Path:
    plots_dir.mkdir(parents=True, exist_ok=True)
    figure, axes = plt.subplots(1, 2, figsize=(15, 7), constrained_layout=True)
    trajectory = result["trajectory"]
    goals = result["goals"]
    for ax, topic, title, kind in (
        (axes[0], "/map", "Physical occupancy map", "map"),
        (
            axes[1],
            "/global_costmap/costmap",
            "Global costmap (inflation ≠ physical curb)",
            "costmap",
        ),
    ):
        plot_grid(ax, result["final_grids"].get(topic), kind)
        if trajectory:
            segments: list[list[tuple[int, float, float, float]]] = [[]]
            for point in trajectory:
                if segments[-1]:
                    previous = segments[-1][-1]
                    dt = (point[0] - previous[0]) / 1e9
                    speed = (
                        math.hypot(point[1] - previous[1], point[2] - previous[2])
                        / dt
                        if dt > 0
                        else float("inf")
                    )
                    if dt > 2.0 or speed > 2.0:
                        segments.append([])
                segments[-1].append(point)
            for segment_number, segment in enumerate(segments):
                if not segment:
                    continue
                ax.plot(
                    [item[1] for item in segment],
                    [item[2] for item in segment],
                    color="#1565c0",
                    linewidth=2.0,
                    label="rover trajectory" if segment_number == 0 else None,
                )
            xs = [item[1] for item in trajectory]
            ys = [item[2] for item in trajectory]
            ax.scatter(xs[0], ys[0], color="limegreen", edgecolor="black", s=90,
                       marker="o", label="start", zorder=5)
        for target_number, (_, target_x, target_y) in enumerate(
            result["targets"], start=1
        ):
            ax.scatter(
                target_x,
                target_y,
                color="magenta",
                edgecolor="black",
                s=130,
                marker="*",
                label="final target" if target_number == 1 else None,
                zorder=6,
            )
            ax.annotate(
                f"T{target_number}",
                (target_x, target_y),
                xytext=(6, -13),
                textcoords="offset points",
                color="magenta",
                fontsize=8,
                weight="bold",
            )
        outcome_colors = {
            "SUCCEEDED": "green",
            "CANCELED": "orange",
            "ABORTED": "red",
            "REJECTED": "red",
        }
        for goal in goals:
            color = outcome_colors.get(goal["outcome"], "#7b1fa2")
            marker = "X" if goal["later_canceled"] else "D"
            ax.scatter(
                goal["goal_x"],
                goal["goal_y"],
                color=color,
                edgecolor="black",
                marker=marker,
                s=65,
                zorder=7,
            )
            ax.annotate(
                str(goal["goal_number"]),
                (goal["goal_x"], goal["goal_y"]),
                xytext=(5, 5),
                textcoords="offset points",
                fontsize=8,
                weight="bold",
            )
        ax.set_title(title)
        ax.set_xlabel("map x (m)")
        ax.set_ylabel("map y (m)")
        ax.set_aspect("equal", adjustable="box")
        ax.grid(True, alpha=0.2)
    axes[0].legend(loc="best", fontsize=8)
    figure.suptitle(
        f"{result['bag']} — {result['trajectory_source']}\n"
        "Numbered diamonds/X marks are dispatched goals (X = safety-canceled)"
    )
    path = plots_dir / f"{result['bag'].replace('.bag', '')}_trajectory.png"
    figure.savefig(path, dpi=170)
    plt.close(figure)
    return path


def topic_presence(result: dict[str, Any], names: Iterable[str]) -> str:
    counts = {item["name"]: item["count"] for item in result["topics"]}
    present = [f"{name} ({counts[name]})" for name in names if counts.get(name, 0)]
    return ", ".join(present) if present else "none"


def fmt(value: Any, digits: int = 2, suffix: str = "") -> str:
    if value is None or value == "":
        return "unknown"
    if isinstance(value, bool):
        return "yes" if value else "no"
    if isinstance(value, (float, np.floating)):
        return f"{value:.{digits}f}{suffix}"
    return str(value)


def markdown_table(headers: list[str], rows: list[list[Any]]) -> str:
    def clean(value: Any) -> str:
        return str(value).replace("|", "\\|").replace("\n", " ")

    lines = [
        "| " + " | ".join(headers) + " |",
        "| " + " | ".join("---" for _ in headers) + " |",
    ]
    lines.extend("| " + " | ".join(clean(x) for x in row) + " |" for row in rows)
    return "\n".join(lines)


def rejection_stats(result: dict[str, Any]) -> dict[str, Any]:
    rejection_events = [
        event for event in result["target_events"]
        if event["event_category"] == "endpoint_clearance_rejection"
        and event["candidate_type"] == "frontier"
    ]
    values = [float(event["clearance_m"]) for event in rejection_events]
    safety_cancels = [
        goal for goal in result["goals"]
        if goal["later_canceled"] and "obstacle" in goal["cancellation_reason"].lower()
    ]
    outcomes = Counter(goal["outcome"] for goal in result["goals"])
    active_durations = [
        goal["time_active_sec"] for goal in result["goals"]
        if goal["nav2_accepted"]
    ]
    cancel_delays = [
        (goal["cancel_requested_ns"] - goal["dispatch_ns"]) / 1e9
        for goal in safety_cancels if goal.get("cancel_requested_ns")
    ]
    return {
        "rejections": len(rejection_events),
        "clearance_min": min(values) if values else None,
        "clearance_median": statistics.median(values) if values else None,
        "clearance_p90": float(np.percentile(values, 90)) if values else None,
        "clearance_max": max(values) if values else None,
        "goals": len(result["goals"]),
        "safety_cancels": len(safety_cancels),
        "shortest_cancel_sec": min(cancel_delays) if cancel_delays else None,
        "recoveries_selected": sum(
            event["event_category"] == "recovery_frontier"
            for event in result["target_events"]
        ),
        "succeeded": outcomes["SUCCEEDED"],
        "aborted": outcomes["ABORTED"],
        "canceled": outcomes["CANCELED"],
        "active_average_sec": (
            statistics.mean(active_durations) if active_durations else None
        ),
    }


def generate_report(
    results: list[dict[str, Any]],
    output_dir: Path,
    plot_paths: dict[str, Path],
    parameter_rows: list[dict[str, Any]],
) -> str:
    stats = {result["bag"]: rejection_stats(result) for result in results}
    first = results[0]
    later = results[1:]
    first_rejections = stats[first["bag"]]["rejections"]
    later_rejections = sum(stats[x["bag"]]["rejections"] for x in later)
    first_rate = first_rejections / max(first["duration_ns"] / 1e9, 1)
    later_duration = sum(x["duration_ns"] for x in later) / 1e9
    later_rate = later_rejections / max(later_duration, 1)
    first_cancels = stats[first["bag"]]["safety_cancels"]
    later_cancels = sum(stats[x["bag"]]["safety_cancels"] for x in later)
    if first_rate > later_rate * 2 and first_cancels > later_cancels:
        hypothesis = "SUPPORTED"
    elif first_rate > later_rate * 1.25 or first_cancels > later_cancels:
        hypothesis = "PARTIALLY SUPPORTED"
    else:
        hypothesis = "NOT SUPPORTED BY THESE SHORT RUNS"

    inspected_parameter_nodes = (
        ("target_explorer_node", ("target_explorer_node",)),
        (
            "global_costmap/global_costmap",
            ("global_costmap/global_costmap", "global_costmap.global_costmap"),
        ),
        (
            "local_costmap/local_costmap",
            ("local_costmap/local_costmap", "local_costmap.local_costmap"),
        ),
        ("planner_server", ("planner_server",)),
        ("controller_server", ("controller_server",)),
        ("bt_navigator", ("bt_navigator",)),
        ("behavior_server", ("behavior_server",)),
        ("slam_toolbox", ("slam_toolbox",)),
    )
    parameter_node_rows = []
    for label, hints in inspected_parameter_nodes:
        counts = []
        parameter_names = set()
        for result in results:
            matches = [
                event
                for event in result["parameter_events"]
                if any(hint in event["node"] for hint in hints)
            ]
            counts.append(len(matches))
            parameter_names.update(event["parameter"] for event in matches)
        parameter_node_rows.append(
            [label]
            + counts
            + [
                (
                    ", ".join(sorted(parameter_names))
                    if parameter_names
                    else "No ParameterEvent rows in these bag windows"
                )
            ]
        )

    lines = [
        "# July 23 rover bag analysis",
        "",
        "Generated by `analyze_bags.py` using offline `rosbag2_py` decoding. "
        "No bag was replayed and nothing was published to a ROS graph.",
        "",
        "## Executive summary",
        "",
        f"The clearance-regression hypothesis is **{hypothesis}**. "
        f"The first run recorded a {fmt(first['clearance_requirement'])} m "
        f"endpoint requirement and {first_rejections} frontier-clearance "
        f"rejections ({first_rate:.2f}/s), versus {later_rejections} across "
        f"the four 0.10 m runs ({later_rate:.2f}/s). The first run made "
        f"{first_cancels} active-goal obstacle-clearance cancellation(s); "
        f"the later runs made {later_cancels}.",
        "",
        "The first run also selected two Target Explorer recovery frontiers, "
        "including a negative-progress goal, while the later runs selected none. "
        "By contrast, 23july2's long third goal triggered genuine Nav2 recovery "
        "behaviors and ultimately status 6 (`ABORTED`); that is a separate "
        "planner/controller failure mode, not an endpoint-clearance cancellation.",
        "",
        "The final conclusion is based on direct Target Explorer logs, recorded "
        "maps/costmaps, and map-frame TF trajectory. Action request/status topics "
        "were not recorded, so action acceptance and terminal results are "
        "reconstructed from Target Explorer, bt_navigator, planner, and controller "
        "logs. Static parameters absent from `/parameter_events` cannot be recovered "
        "from a bag alone.",
        "",
        "## Bag inventory",
        "",
        markdown_table(
            [
                "bag",
                "start (local time)",
                "duration (s)",
                "messages",
                "storage",
                "key evidence",
            ],
            [
                [
                    result["bag"],
                    stamp_text(result["start_ns"]),
                    f"{result['duration_ns'] / 1e9:.3f}",
                    f"{result['message_count']:,}",
                    result["storage"],
                    (
                        f"rosout={topic_presence(result, ['/rosout'])}; "
                        f"parameters={topic_presence(result, ['/parameter_events'])}; "
                        f"map={topic_presence(result, ['/map'])}; "
                        f"global costmap={topic_presence(result, ['/global_costmap/costmap'])}; "
                        f"TF={topic_presence(result, ['/tf', '/tf_static'])}; "
                        f"odom={topic_presence(result, ['/genz/odometry', '/odometry/filtered'])}; "
                        f"plans={topic_presence(result, ['/plan', '/received_global_plan'])}; "
                        f"cmd_vel={topic_presence(result, ['/cmd_vel'])}"
                    ),
                ]
                for result in results
            ],
        ),
        "",
        "All five metadata files identify `sqlite3`. Topic inventories and message "
        "types come from each `metadata.yaml`; the full inventory is reproducible "
        "in [bag_inventory.csv](bag_inventory.csv), including per-topic counts and "
        "average frequencies. No normal NavigateToPose action goal/status/feedback "
        "topic had messages in these recordings.",
        "",
        "## Parameter recovery and confidence",
        "",
        markdown_table(
            ["parameter"] + [x["bag"] for x in results] + ["evidence/confidence"],
            [
                [row["parameter"]]
                + [row[result["bag"]] for result in results]
                + [row["evidence/confidence"]]
                for row in parameter_rows
            ],
        ),
        "",
        "Confidence labels mean: `CONFIRMED_IN_BAG` is directly decoded or "
        "geometrically derived from a recorded message; `CONFIRMED_IN_MATCHING_LOG` "
        "would require an explicit value in a same-session launch log; "
        "`STRONGLY_INFERRED` uses matching launch/build/repository timing; "
        "`USER_REPORTED` is not independently verified; and `UNKNOWN` is not "
        "recoverable from available evidence.",
        "",
        "The bags' `/parameter_events` mostly contain runtime changes from GUI and "
        "other nodes, not startup declarations. The complete decoded timeline is in "
        "[parameter_events.csv](parameter_events.csv). Current YAML values are never "
        "silently used as historical values.",
        "",
        "### Per-node parameter-event timeline coverage",
        "",
        markdown_table(
            ["node"]
            + [result["bag"] for result in results]
            + ["parameters observed"],
            parameter_node_rows,
        ),
        "",
        "Counts are decoded parameter rows, not `/parameter_events` message counts. "
        "Target Explorer started inside each recording (twice in 23july4), so its "
        "full startup declarations are captured. Nav2 and SLAM nodes generally "
        "started before recording; zero rows therefore cannot be read as default "
        "values.",
        "",
        "## Comparison of 1.7 m versus 0.1 m clearance",
        "",
        markdown_table(
            [
                "bag",
                "required clearance",
                "frontier rejects",
                "rejection clearance min/median/p90/max (m)",
                "goals",
                "safety cancels",
                "shortest dispatch→cancel (s)",
                "recovery frontiers",
                "success / abort / cancel",
                "mean active (s)",
                "trajectory (m)",
                "net progress (m)",
                "lateral movement (m)",
                "stationary (s)",
                "Nav2 recoveries",
            ],
            [
                [
                    result["bag"],
                    fmt(result["clearance_requirement"], 2, " m"),
                    stats[result["bag"]]["rejections"],
                    " / ".join(
                        fmt(stats[result["bag"]][name], 2)
                        for name in (
                            "clearance_min",
                            "clearance_median",
                            "clearance_p90",
                            "clearance_max",
                        )
                    ),
                    stats[result["bag"]]["goals"],
                    stats[result["bag"]]["safety_cancels"],
                    fmt(stats[result["bag"]]["shortest_cancel_sec"], 2),
                    stats[result["bag"]]["recoveries_selected"],
                    (
                        f"{stats[result['bag']]['succeeded']} / "
                        f"{stats[result['bag']]['aborted']} / "
                        f"{stats[result['bag']]['canceled']}"
                    ),
                    fmt(stats[result["bag"]]["active_average_sec"], 2),
                    fmt(result["trajectory_metrics"]["distance_m"], 2),
                    fmt(result["trajectory_metrics"]["net_progress_m"], 2),
                    fmt(result["trajectory_metrics"]["lateral_movement_m"], 2),
                    fmt(result["trajectory_metrics"]["stationary_sec"], 2),
                    result["behavior_recoveries"],
                ]
                for result in results
            ],
        ),
        "",
        "Lateral movement is the cumulative absolute component of map-frame motion "
        "perpendicular to the active mission's start-to-final-target axis. Net "
        "progress and lateral movement use the last Target Explorer session when a "
        "bag contains multiple sessions (23july4 does); total distance and stationary "
        "time cover the whole bag. Stationary time sums trajectory intervals below "
        "0.05 m/s; gaps over 2 s are excluded. Mean active time includes the observed "
        "window for goals still active when recording stopped (right-censored rows "
        "are marked in the goal CSV). Frame discontinuities above 2 m/s, including "
        "the 23july4 map reset between sessions, are split in plots and excluded from "
        "distance.",
        "",
        "## Target Explorer goal-decision analysis",
        "",
        "Every logged endpoint rejection is preserved in "
        "[all_frontier_decisions.csv](all_frontier_decisions.csv). Repeated "
        "rejections are summarized here to keep the report readable. One complete "
        "correlated row per dispatched goal is in "
        "[goal_timeline.csv](goal_timeline.csv).",
        "",
        "The large lateral goals around the roundabout are supported by map context. "
        "For example, 23july1 goal 3 at (8.60, 14.73) had directly logged "
        "progress=8.18 m, alignment=0.797, path=13.33 m, score=7.640; the closest "
        "preceding physical map and lethal costmap show the straight final-target "
        "chord blocked while the selected endpoint chord avoids those cells, and "
        "the goal subsequently succeeded. The comparable 23july goal 3 at "
        "(6.99, 16.33) also succeeded after the direct chord crossed the roundabout. "
        "These are evidence-backed detours, not arbitrary sideways wandering.",
        "",
    ]
    for result in results:
        bag_stats = stats[result["bag"]]
        lines.extend(
            [
                f"### {result['bag']}",
                "",
                "Final target session(s): "
                + ", ".join(
                    f"T{index}=({target[1]:.2f}, {target[2]:.2f}) at "
                    f"{seconds(target[0], result['start_ns']):.1f} s"
                    for index, target in enumerate(result["targets"], start=1)
                )
                + ". Recorded requirement: "
                f"{fmt(result['clearance_requirement'], 2, ' m')}. "
                f"{bag_stats['rejections']} frontier candidates were rejected for "
                f"endpoint clearance, {bag_stats['goals']} goals were dispatched, "
                f"and {bag_stats['safety_cancels']} active goals were canceled by "
                "the endpoint safety rule.",
                "",
            ]
        )
        goal_rows = []
        for goal in result["goals"]:
            goal_rows.append(
                [
                    goal["goal_number"],
                    f"{goal['relative_sec']:.1f}",
                    f"({goal['goal_x']:.2f}, {goal['goal_y']:.2f})",
                    (
                        f"{goal['progress_m']:.2f} / {goal['alignment']:.3f} / "
                        f"{goal['path_length_m']:.2f} / {goal['score']:.3f}"
                    ),
                    fmt(goal.get("initial_endpoint_clearance_m"), 2),
                    "yes" if goal["nav2_accepted"] else "no",
                    goal["outcome"],
                    fmt(goal["time_active_sec"], 1),
                    fmt(goal["distance_travelled_while_active_m"], 2),
                    goal["recoveries_triggered"],
                    goal.get("map_context_interpretation", ""),
                ]
            )
        lines.extend(
            [
                markdown_table(
                    [
                        "#",
                        "t (s)",
                        "goal",
                        "progress / align / path / score",
                        "endpoint clear (m)",
                        "accepted",
                        "outcome",
                        "active (s)",
                        "distance (m)",
                        "recoveries",
                        "map-context interpretation",
                    ],
                    goal_rows,
                )
                if goal_rows
                else "No dispatched goals were logged.",
                "",
                "Direct evidence is the exact dispatch/accept/result/cancel text in "
                "`/rosout`. Map-context interpretation is an inference from the "
                "closest preceding map and global costmap; physical occupied cells "
                "(map ≥65), lethal cells (costmap=100), inflation-gradient cells "
                "(1–99), and unknown cells are counted separately.",
                "",
            ]
        )
    lines.extend(
        [
            "## Per-bag chronological timeline",
            "",
            "Endpoint-rejection floods are represented as time-bucket summaries; "
            "the unabridged rows remain in the CSV.",
            "",
        ]
    )
    for result in results:
        events = sorted(result["target_events"], key=lambda x: x["relative_sec"])
        rejection_buckets: dict[int, list[dict[str, Any]]] = defaultdict(list)
        concise = []
        for event in events:
            if event["event_category"] == "endpoint_clearance_rejection":
                rejection_buckets[int(float(event["relative_sec"]))].append(event)
            else:
                concise.append(event)
        for bucket, bucket_events in rejection_buckets.items():
            values = [float(event["clearance_m"]) for event in bucket_events]
            concise.append(
                {
                    "relative_sec": float(bucket),
                    "absolute_time": bucket_events[0]["absolute_time"],
                    "source_node": bucket_events[0]["source_node"],
                    "event_category": "endpoint_clearance_rejection_summary",
                    "goal_x": "",
                    "goal_y": "",
                    "decision": "rejected",
                    "direct_reason": (
                        f"{len(bucket_events)} logged rejections; clearance "
                        f"{min(values):.2f}–{max(values):.2f} m"
                    ),
                    "outcome": "",
                }
            )
        concise.sort(key=lambda x: float(x["relative_sec"]))
        lines.extend(
            [
                f"### {result['bag']}",
                "",
                markdown_table(
                    [
                        "absolute time",
                        "t (s)",
                        "source",
                        "category",
                        "coordinates",
                        "decision / direct reason",
                        "outcome",
                    ],
                    [
                        [
                            event["absolute_time"],
                            f"{float(event['relative_sec']):.1f}",
                            event["source_node"],
                            event["event_category"],
                            (
                                f"({event['goal_x']}, {event['goal_y']})"
                                if event.get("goal_x") != ""
                                else ""
                            ),
                            (
                                f"{event.get('decision', '')}: "
                                f"{event.get('direct_reason', '')}"
                            ),
                            event.get("outcome", ""),
                        ]
                        for event in concise
                    ],
                ),
                "",
            ]
        )
    lines.extend(
        [
            "## Nav2 planning, controller, and recovery analysis",
            "",
            "The complete filtered Nav2 event stream, including `/plan` updates and "
            "behavior-tree recovery transitions, is in "
            "[nav2_events.csv](nav2_events.csv). Raw numeric action statuses are "
            "mapped as 0 UNKNOWN, 1 ACCEPTED, 2 EXECUTING, 3 CANCELING, 4 SUCCEEDED, "
            "5 CANCELED, and 6 ABORTED.",
            "",
            markdown_table(
                ["bag", "Nav2 event categories", "behavior-tree recoveries"],
                [
                    [
                        result["bag"],
                        ", ".join(
                            f"{name}={count}"
                            for name, count in sorted(
                                Counter(
                                    event["event_category"]
                                    for event in result["nav2_events"]
                                ).items()
                            )
                        ),
                        result["behavior_recoveries"],
                    ]
                    for result in results
                ],
            ),
            "",
        ]
    )
    for result in results:
        notable_groups: dict[tuple[str, str], list[dict[str, Any]]] = defaultdict(list)
        for event in result["nav2_events"]:
            if event["event_category"] in {
                "aborted",
                "controller_patience",
                "costmap_clear",
                "empty_path_error",
                "failure",
                "planning_failure",
                "progress_checker",
                "recovery",
                "robot_out_of_bounds",
                "tf",
            } and (
                event["severity"] in {"WARN", "ERROR", "FATAL", "RECORDED"}
                or event["event_category"] in {"costmap_clear", "recovery"}
            ):
                notable_groups[
                    (event["event_category"], event["message"])
                ].append(event)
        notable_rows = []
        for (category, message), events in sorted(
            notable_groups.items(),
            key=lambda item: float(item[1][0]["relative_sec"]),
        ):
            first_event = events[0]
            notable_rows.append(
                [
                    f"{float(first_event['relative_sec']):.1f}",
                    first_event["source_node"],
                    category,
                    len(events),
                    message,
                    first_event.get("active_goal_number", ""),
                ]
            )
        lines.extend(
            [
                f"### {result['bag']} notable Nav2 events",
                "",
                markdown_table(
                    [
                        "first t (s)",
                        "source",
                        "category",
                        "count",
                        "direct logged message",
                        "goal #",
                    ],
                    notable_rows,
                )
                if notable_rows
                else "No concrete Nav2 recovery or warning/error event was recorded.",
                "",
            ]
        )
    lines.extend(
        [
            "A Nav2 recovery count is a concrete behavior-tree action entering "
            "`RUNNING` (`Clear*`, `ClearingActions`, `Spin`, `Wait`, or `BackUp`). "
            "Structural `NavigateRecovery`/`RecoveryFallback` nodes are not counted "
            "as executed recovery actions. Costmap-clear requests are also listed "
            "separately in the event table.",
            "",
            "## Rover pose reconstruction",
            "",
            "The plotted stream is map→base TF because it matches the pose source "
            "used by Target Explorer and retains the global map frame. `/genz/odometry` "
            "and `/odometry/filtered` are local odom-frame streams and are compared "
            "rather than plotted as if already in map coordinates.",
            "",
        ]
    )
    for result in results:
        comparison_rows = []
        for name, metrics in result["pose_comparison"].items():
            comparison_rows.append(
                [
                    name,
                    metrics["samples"],
                    ", ".join(metrics.get("frame_ids", [])) or "TF-derived",
                    fmt(metrics["distance_m"], 2),
                    fmt(metrics["net_progress_m"], 2),
                    fmt(metrics["median_deviation_from_chosen_m"], 3),
                ]
            )
        lines.extend(
            [
                f"### {result['bag']}",
                "",
                f"Chosen: **{result['trajectory_source']}**.",
                "",
                markdown_table(
                    [
                        "stream",
                        "samples",
                        "frame",
                        "path distance (m)",
                        "net target progress (m)",
                        "median deviation from chosen (m)",
                    ],
                    comparison_rows,
                ),
                "",
            ]
        )
    lines.extend(
        [
            "## Trajectory and goal plots",
            "",
        ]
    )
    for result in results:
        relative = plot_paths[result["bag"]].relative_to(output_dir)
        lines.extend(
            [
                f"### {result['bag']}",
                "",
                f"![{result['bag']} trajectory]({relative.as_posix()})",
                "",
            ]
        )
    bag1 = next((x for x in results if x["bag"] == "23july1.bag"), None)
    lines.extend(
        [
            "## Investigation of the possible 23july1 costmap-radius change",
            "",
            "Direct evidence:",
            "",
            "- No static `robot_radius`, `footprint`, `footprint_padding`, "
            "`inflation_radius`, or `cost_scaling_factor` declaration was recorded "
            "in 23july1 `/parameter_events`, and matching launch output does not print "
            "their values.",
            (
                f"- The recorded local and global published footprints have median "
                f"maximum vertex radii of "
                f"{bag1['footprint_summary'].get('/local_costmap/published_footprint', {}).get('median_radius_m', float('nan')):.3f} m "
                f"and "
                f"{bag1['footprint_summary'].get('/global_costmap/published_footprint', {}).get('median_radius_m', float('nan')):.3f} m."
                if bag1 else "- 23july1 footprint evidence was unavailable."
            ),
            "- The checked-in pre-test Nav2 YAML has `robot_radius: 1.0`, local "
            "`cost_scaling_factor: 5.0`, global `cost_scaling_factor: 3.0`, and "
            "0.50 m inflation. The installed YAML was copied at 21:22:36 with "
            "0.95 m inflation. 23july1 Nav2 started at about 21:16:34 and therefore "
            "could not have loaded that later copy.",
            "",
            "Inference:",
            "",
            "There is no evidence that 23july1 used a different effective robot "
            "footprint radius from the other bags. Its effective footprint is "
            "consistent with 1.0 m. The strongest reconstruction is 0.50 m inflation "
            "for 23july and 23july1 (also 23july2, whose Nav2 process started before "
            "the 21:22 copy) and 0.95 m for 23july3/4. Footprint padding remains "
            "unknown. This contradicts a special robot-radius change in 23july1, "
            "but the bag cannot prove which configuration key generated the "
            "published circle.",
            "",
            "## Confirmed findings",
            "",
            "- All five bags are readable SQLite3 recordings and contain maps, "
            "global/local costmaps, TF, odometry, plans, `/cmd_vel`, `/rosout`, and "
            "`/parameter_events`.",
            "- The first bag directly logs a 1.70 m clearance requirement; all four "
            "later bags directly log 0.10 m.",
            "- Target Explorer final targets, frontier rejection coordinates and "
            "clearances, dispatch metrics, Nav2 acceptance, safety cancellation "
            "reasons, and terminal results are directly recoverable from `/rosout`.",
            "- Sideways goals are not classified as bad from alignment alone. Each "
            "goal row reports whether the contemporaneous direct target chord crossed "
            "physical, lethal, or only inflated cells.",
            "",
            "## Uncertain findings and missing evidence",
            "",
            "- NavigateToPose goal requests were service/action traffic and were not "
            "recorded as normal topics. Goal correlation therefore uses exact "
            "Target Explorer logs and selected-goal publication.",
            "- Startup-only static Nav2 values cannot be proven from "
            "`/parameter_events`. Inferred inflation history is explicitly labeled.",
            "- A straight chord through a grid is not a substitute for the Nav2 path. "
            "Map-context conclusions are geometric checks, while planner logs/path "
            "messages are used for reachability.",
            "- Costmap inflation-gradient cells are not treated as physical curbs; "
            "only map occupied cells and lethal costmap cells enter the physical/"
            "lethal blockage conclusions.",
            "",
            "## Recommended changes supported by the data",
            "",
            "- Keep endpoint safety tied to physical occupied/lethal cells, not "
            "inflation-gradient cells, and keep the clearance near the demonstrated "
            "0.10 m scale unless a larger value is justified against the effective "
            "1.0 m footprint. The 1.70 m rule double-counts a large safety margin and "
            "rejects practical endpoints.",
            "- Log the complete effective Target Explorer and Nav2 safety parameter "
            "set at startup (including local/global robot radius or footprint, "
            "padding, inflation radius, and scaling). Also record NavigateToPose "
            "status/feedback topics in future bags.",
            "- Retain the active-goal safety cancellation reason and add the source "
            "grid plus physical/lethal nearest-cell distance to that log. This will "
            "make future regression attribution unambiguous.",
            "",
            "## Validation",
            "",
            markdown_table(
                [
                    "bag",
                    "filtered messages decoded",
                    "timestamp regressions",
                    "decode/analysis failures",
                    "goals CSV rows",
                    "frontier evidence rows",
                    "plot",
                ],
                [
                    [
                        result["bag"],
                        f"{result['filtered_messages_read']:,}",
                        result["timestamp_regressions"],
                        sum(x["count"] for x in result["decode_failures"].values()),
                        len(result["goals"]),
                        len(result["target_events"]),
                        plot_paths[result["bag"]].name,
                    ]
                    for result in results
                ],
            ),
            "",
            "Plots use labelled map axes and equal scaling. OccupancyGrid resolution, "
            "origin, and origin yaw are applied. Any decoding failures are listed in "
            "[decoding_failures.csv](decoding_failures.csv).",
            "",
        ]
    )
    return "\n".join(lines)


def print_summary(results: list[dict[str, Any]], output_dir: Path) -> None:
    print("Exact bag paths analyzed:")
    for result in results:
        print(f"  {result['path']}")
    print("Bag durations:")
    for result in results:
        print(f"  {result['bag']}: {result['duration_ns'] / 1e9:.3f} s")
    print("Output files:")
    for path in sorted(output_dir.rglob("*")):
        if path.is_file():
            print(f"  {path}")
    print("Parameters recovered with high confidence:")
    for result in results:
        footprint = result["footprint_summary"].get(
            "/global_costmap/published_footprint", {}
        ).get("median_radius_m")
        print(
            f"  {result['bag']}: goal_obstacle_clearance_m="
            f"{fmt(result['clearance_requirement'], 2)}; "
            f"final_target={result['target']}; "
            f"effective_global_footprint_radius={fmt(footprint, 3)}"
        )
    print("Parameters that could not be recovered:")
    print(
        "  explicit footprint, footprint_padding, local raytrace_min_range, "
        "controller desired_linear_vel/goal tolerances/progress-checker values; "
        "robot_radius and inflation radius are historically inferred rather than "
        "bag-confirmed config keys"
    )
    first = rejection_stats(results[0])
    later_rejects = sum(rejection_stats(x)["rejections"] for x in results[1:])
    print("Main behavioral difference:")
    print(
        f"  1.70 m run: {first['rejections']} clearance rejections and "
        f"{first['safety_cancels']} safety cancellations; four 0.10 m runs: "
        f"{later_rejects} clearance rejections and "
        f"{sum(rejection_stats(x)['safety_cancels'] for x in results[1:])} "
        "safety cancellations."
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "bags",
        nargs="*",
        type=Path,
        help="Bag directories. If omitted, discover 23july*.bag under common roots.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path(__file__).resolve().parent,
        help="Output directory (default: script directory).",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    output_dir = args.output.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    bags = [path.expanduser().resolve() for path in args.bags]
    if not bags:
        bags = discover_bags(
            (
                Path.home(),
                Path.home() / "Desktop",
                Path("/media"),
                Path("/mnt"),
            )
        )
    bags = sorted(set(bags), key=lambda p: natural_bag_key(p.name))
    expected = {f"23july{suffix}.bag" for suffix in ("", "1", "2", "3", "4")}
    located = {path.name for path in bags}
    missing = expected - located
    if missing:
        print(f"Missing expected bags: {', '.join(sorted(missing))}", file=sys.stderr)
        return 2
    results = []
    for bag in bags:
        print(f"Analyzing {bag} ...", flush=True)
        results.append(analyze_bag(bag))
    frontier_rows = [
        event for result in results for event in result["target_events"]
    ]
    nav_rows = [event for result in results for event in result["nav2_events"]]
    parameter_event_rows = [
        event for result in results for event in result["parameter_events"]
    ]
    failure_rows = []
    for result in results:
        for topic, failure in result["decode_failures"].items():
            failure_rows.append(
                {
                    "bag": result["bag"],
                    "topic": topic,
                    "message_type": failure["type"],
                    "count": failure["count"],
                    "first_error": failure["first_error"],
                }
            )
    inventory_rows = []
    for result in results:
        for topic in result["topics"]:
            inventory_rows.append(
                {
                    "bag": result["bag"],
                    "bag_path": result["path"],
                    "storage": result["storage"],
                    "bag_start": stamp_text(result["start_ns"]),
                    "bag_duration_sec": round(result["duration_ns"] / 1e9, 6),
                    "bag_message_count": result["message_count"],
                    "topic": topic["name"],
                    "message_type": topic["type"],
                    "topic_message_count": topic["count"],
                    "frequency_hz": round(topic["frequency_hz"], 6),
                }
            )
    write_csv(
        output_dir / "bag_inventory.csv",
        inventory_rows,
        CSV_COLUMNS["bag_inventory.csv"],
    )
    write_csv(
        output_dir / "goal_timeline.csv",
        goal_csv_rows(results),
        CSV_COLUMNS["goal_timeline.csv"],
    )
    write_csv(
        output_dir / "all_frontier_decisions.csv",
        frontier_rows,
        CSV_COLUMNS["all_frontier_decisions.csv"],
    )
    write_csv(
        output_dir / "nav2_events.csv",
        nav_rows,
        CSV_COLUMNS["nav2_events.csv"],
    )
    write_csv(
        output_dir / "parameter_events.csv",
        parameter_event_rows,
        CSV_COLUMNS["parameter_events.csv"],
    )
    write_csv(
        output_dir / "decoding_failures.csv",
        failure_rows,
        CSV_COLUMNS["decoding_failures.csv"],
    )
    parameter_rows = parameter_comparison_rows(results)
    parameter_columns = (
        ["parameter"] + [result["bag"] for result in results]
        + ["evidence/confidence"]
    )
    write_csv(
        output_dir / "parameter_comparison.csv",
        parameter_rows,
        parameter_columns,
    )
    plot_paths = {
        result["bag"]: plot_result(result, output_dir / "plots")
        for result in results
    }
    report = generate_report(results, output_dir, plot_paths, parameter_rows)
    (output_dir / "report.md").write_text(report)
    print_summary(results, output_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
