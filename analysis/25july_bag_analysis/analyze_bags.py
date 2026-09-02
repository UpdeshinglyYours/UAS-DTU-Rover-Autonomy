#!/usr/bin/env python3
"""Read-only forensic analysis for the 25 July target_explorer ROS bags."""

from __future__ import annotations

import bisect
import csv
import json
import math
import re
import sqlite3
from collections import Counter, defaultdict, deque
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Iterable, Optional

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
import numpy as np
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


ROOT = Path("/home/aryaman/UAS-DTU-Rover-Autonomy")
OUTPUT = ROOT / "analysis" / "25july_bag_analysis"
PLOTS = OUTPUT / "plots"
BAGS = (ROOT / "25july.bag", ROOT / "25july1.bag", ROOT / "25july2.bag")

TARGET_RE = re.compile(
    r"target_explorer started: final target=\(([-+\d.]+), ([-+\d.]+)\)"
)
SELECTED_RE = re.compile(
    r"Frontier \(([-+\d.]+), ([-+\d.]+)\) selected with score "
    r"([-+\d.]+); safe interior goal is \(([-+\d.]+), ([-+\d.]+)\), "
    r"([-+\d.]+) m back .* rejecting (\d+) samples"
)
RECOVERY_RE = re.compile(
    r"No forward-progress frontier is reachable; using recovery frontier "
    r"\(([-+\d.]+), ([-+\d.]+)\)"
)
CORRECTED_RE = re.compile(
    r"Nav2 accepted corrected goal \(([-+\d.]+), ([-+\d.]+)\)"
)
ENDPOINT_REJECTION_RE = re.compile(
    r"Rejected (?:final target|frontier) \(([-+\d.]+), ([-+\d.]+)\): "
    r"endpoint clearance ([-+\d.]+) m is below required ([-+\d.]+) m"
)
SAFE_REJECTION_RE = re.compile(
    r"Rejected frontier \(([-+\d.]+), ([-+\d.]+)\): "
    r"no safe interior on-path goal; (\d+) rejected samples \((.*)\)"
)
REASON_RE = re.compile(r"([^,=]+)=(\d+)")
UNSAFE_RE = re.compile(
    r"Active goal unsafe check (\d+)/(\d+) .* maximum area cost=(\d+)"
)
OMEGA_RE = re.compile(r"target=([-+\d.]+) omega=([-+\d.]+)")


@dataclass
class Grid:
    timestamp_ns: int
    frame_id: str
    resolution: float
    width: int
    height: int
    origin_x: float
    origin_y: float
    origin_yaw: float
    data: np.ndarray


def yaw_from_quaternion(q: Any) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def compose_pose(
    first: tuple[float, float, float],
    second: tuple[float, float, float],
) -> tuple[float, float, float]:
    ax, ay, ayaw = first
    bx, by, byaw = second
    cosine = math.cos(ayaw)
    sine = math.sin(ayaw)
    return (
        ax + cosine * bx - sine * by,
        ay + sine * bx + cosine * by,
        normalize_angle(ayaw + byaw),
    )


def inverse_pose(
    pose: tuple[float, float, float],
) -> tuple[float, float, float]:
    x, y, yaw = pose
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return (
        -cosine * x - sine * y,
        sine * x - cosine * y,
        normalize_angle(-yaw),
    )


def resolve_transform(
    transforms: dict[tuple[str, str], tuple[float, float, float]],
    source: str,
    target: str,
) -> Optional[tuple[float, float, float]]:
    graph: dict[
        str, list[tuple[str, tuple[float, float, float]]]
    ] = defaultdict(list)
    for (parent, child), pose in transforms.items():
        graph[parent].append((child, pose))
        graph[child].append((parent, inverse_pose(pose)))
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


def parameter_value(value: Any) -> Any:
    if value.type == 1:
        return bool(value.bool_value)
    if value.type == 2:
        return int(value.integer_value)
    if value.type == 3:
        return float(value.double_value)
    if value.type == 4:
        return str(value.string_value)
    if value.type == 5:
        return list(value.byte_array_value)
    if value.type == 6:
        return list(value.bool_array_value)
    if value.type == 7:
        return list(value.integer_array_value)
    if value.type == 8:
        return list(value.double_array_value)
    if value.type == 9:
        return list(value.string_array_value)
    return None


def metadata_for(bag: Path) -> dict[str, Any]:
    with (bag / "metadata.yaml").open() as stream:
        return yaml.safe_load(stream)["rosbag2_bagfile_information"]


def topic_counts(metadata: dict[str, Any]) -> dict[str, int]:
    return {
        item["topic_metadata"]["name"]: int(item["message_count"])
        for item in metadata["topics_with_message_count"]
    }


def database_for(bag: Path) -> Path:
    files = list(bag.glob("*.db3"))
    if len(files) != 1:
        raise RuntimeError(f"{bag}: expected one db3 file, found {len(files)}")
    return files[0]


def sqlite_connection(database: Path) -> sqlite3.Connection:
    return sqlite3.connect(f"file:{database}?mode=ro", uri=True)


def topic_table(
    connection: sqlite3.Connection,
) -> dict[int, tuple[str, str]]:
    return {
        int(topic_id): (str(name), str(message_type))
        for topic_id, name, message_type in connection.execute(
            "SELECT id,name,type FROM topics"
        )
    }


def clean_trajectory(
    points: Iterable[tuple[int, float, float, float]],
    maximum_speed: float = 3.0,
) -> list[tuple[int, float, float, float]]:
    clean: list[tuple[int, float, float, float]] = []
    for point in sorted(points):
        if clean:
            dt = (point[0] - clean[-1][0]) / 1e9
            distance = math.hypot(
                point[1] - clean[-1][1],
                point[2] - clean[-1][2],
            )
            if dt <= 0.0:
                continue
            if dt < 1.0 and distance / dt > maximum_speed:
                continue
        clean.append(point)
    return clean


def nearest_pose(
    trajectory: list[tuple[int, float, float, float]],
    timestamp_ns: int,
) -> Optional[tuple[int, float, float, float]]:
    if not trajectory:
        return None
    stamps = [point[0] for point in trajectory]
    index = bisect.bisect_left(stamps, timestamp_ns)
    options = []
    if index < len(trajectory):
        options.append(trajectory[index])
    if index:
        options.append(trajectory[index - 1])
    return min(options, key=lambda point: abs(point[0] - timestamp_ns))


def points_between(
    trajectory: list[tuple[int, float, float, float]],
    start_ns: int,
    end_ns: int,
) -> list[tuple[int, float, float, float]]:
    return [
        point
        for point in trajectory
        if start_ns <= point[0] <= end_ns
    ]


def travelled_distance(
    points: Iterable[tuple[int, float, float, float]],
) -> float:
    values = list(points)
    distance = 0.0
    for first, second in zip(values, values[1:]):
        dt = (second[0] - first[0]) / 1e9
        step = math.hypot(second[1] - first[1], second[2] - first[2])
        if 0.0 < dt <= 1.0 and step / dt <= 3.0:
            distance += step
    return distance


def target_distance(
    point: tuple[int, float, float, float],
    target: tuple[float, float],
) -> float:
    return math.hypot(point[1] - target[0], point[2] - target[1])


def grid_from_message(timestamp_ns: int, message: Any) -> Grid:
    width = int(message.info.width)
    height = int(message.info.height)
    return Grid(
        timestamp_ns=timestamp_ns,
        frame_id=message.header.frame_id,
        resolution=float(message.info.resolution),
        width=width,
        height=height,
        origin_x=float(message.info.origin.position.x),
        origin_y=float(message.info.origin.position.y),
        origin_yaw=yaw_from_quaternion(message.info.origin.orientation),
        data=np.asarray(message.data, dtype=np.int16).reshape(height, width),
    )


def latest_grid(
    connection: sqlite3.Connection,
    topics: dict[int, tuple[str, str]],
    topic_name: str,
) -> Optional[Grid]:
    match = next(
        (
            (topic_id, message_type)
            for topic_id, (name, message_type) in topics.items()
            if name == topic_name
        ),
        None,
    )
    if match is None:
        return None
    row = connection.execute(
        "SELECT timestamp,data FROM messages WHERE topic_id=? "
        "ORDER BY timestamp DESC LIMIT 1",
        (match[0],),
    ).fetchone()
    if row is None:
        return None
    message = deserialize_message(row[1], get_message(match[1]))
    return grid_from_message(int(row[0]), message)


def parse_target_logs(
    result: dict[str, Any],
) -> None:
    target = result["target"]
    goals: list[dict[str, Any]] = []
    active: Optional[dict[str, Any]] = None
    recovery_coordinate: Optional[tuple[float, float]] = None
    rejection_reasons: Counter[str] = Counter()
    endpoint_rejections = 0
    no_safe_rejections = 0
    no_reachable_cycles = 0
    correction_attempts = 0

    for timestamp_ns, level, node, message in result["logs"]:
        if node != "target_explorer_node":
            continue
        startup = TARGET_RE.search(message)
        if startup:
            target = (float(startup.group(1)), float(startup.group(2)))
            result["target_start_ns"] = timestamp_ns
            continue
        recovery = RECOVERY_RE.search(message)
        if recovery:
            recovery_coordinate = (
                float(recovery.group(1)),
                float(recovery.group(2)),
            )
            continue
        selected = SELECTED_RE.search(message)
        if selected:
            frontier = (float(selected.group(1)), float(selected.group(2)))
            safe_goal = (float(selected.group(4)), float(selected.group(5)))
            active = {
                "bag": result["bag"],
                "goal_number": len(goals) + 1,
                "selection_ns": timestamp_ns,
                "frontier_x": frontier[0],
                "frontier_y": frontier[1],
                "safe_goal_x": safe_goal[0],
                "safe_goal_y": safe_goal[1],
                "score": float(selected.group(3)),
                "standoff_path_m": float(selected.group(6)),
                "rejected_samples_before_goal": int(selected.group(7)),
                "recovery_logged": (
                    recovery_coordinate is not None
                    and math.hypot(
                        recovery_coordinate[0] - frontier[0],
                        recovery_coordinate[1] - frontier[1],
                    ) < 0.1
                ),
                "correction_count": 0,
                "correction_goals": [],
                "outcome": "ACTIVE_AT_BAG_END",
                "terminal_ns": None,
            }
            goals.append(active)
            recovery_coordinate = None
            continue
        corrected = CORRECTED_RE.search(message)
        if corrected and active is not None:
            active["correction_goals"].append(
                (float(corrected.group(1)), float(corrected.group(2)))
            )
            active["correction_count"] += 1
            continue
        if (
            "The active frontier goal has become unsafe" in message
            and active is not None
        ):
            correction_attempts += 1
            active["unsafe_correction_attempted"] = True
            continue
        if (
            "Active-goal correction skipped" in message
            and active is not None
        ):
            active["correction_skipped"] = message
            continue
        if (
            message in (
                "Nav2 reached the frontier goal.",
                "Nav2 reached the corrected goal.",
            )
            and active is not None
        ):
            active["terminal_ns"] = timestamp_ns
            active["outcome"] = message.removesuffix(".")
            active = None
            continue
        endpoint = ENDPOINT_REJECTION_RE.search(message)
        if endpoint:
            endpoint_rejections += 1
            continue
        safe_rejection = SAFE_REJECTION_RE.search(message)
        if safe_rejection:
            no_safe_rejections += 1
            for reason, count in REASON_RE.findall(safe_rejection.group(4)):
                rejection_reasons[reason.strip()] += int(count)
            continue
        if "Nav2 found no reachable safe frontier candidate" in message:
            no_reachable_cycles += 1

    result["target"] = target
    result["goals"] = goals
    result["rejections"] = {
        "endpoint_clearance_frontiers": endpoint_rejections,
        "no_safe_interior_frontiers": no_safe_rejections,
        "no_reachable_cycles": no_reachable_cycles,
        "sample_reasons": dict(rejection_reasons),
        "correction_attempts": correction_attempts,
    }


def enrich_goals(result: dict[str, Any]) -> None:
    trajectory = result["trajectory"]
    target = result["target"]
    bag_end = result["end_ns"]
    for goal in result["goals"]:
        start = nearest_pose(trajectory, goal["selection_ns"])
        end_ns = goal["terminal_ns"] or bag_end
        end = nearest_pose(trajectory, end_ns)
        goal["selection_relative_sec"] = (
            goal["selection_ns"] - result["start_ns"]
        ) / 1e9
        goal["terminal_relative_sec"] = (
            None
            if goal["terminal_ns"] is None
            else (goal["terminal_ns"] - result["start_ns"]) / 1e9
        )
        goal["duration_sec"] = (end_ns - goal["selection_ns"]) / 1e9
        goal["robot_start_x"] = None if start is None else start[1]
        goal["robot_start_y"] = None if start is None else start[2]
        goal["robot_end_x"] = None if end is None else end[1]
        goal["robot_end_y"] = None if end is None else end[2]
        if start is not None and target is not None:
            start_distance = target_distance(start, target)
            frontier_distance = math.hypot(
                goal["frontier_x"] - target[0],
                goal["frontier_y"] - target[1],
            )
            safe_distance = math.hypot(
                goal["safe_goal_x"] - target[0],
                goal["safe_goal_y"] - target[1],
            )
            goal["frontier_progress_m"] = start_distance - frontier_distance
            goal["safe_goal_progress_m"] = start_distance - safe_distance
            dx = goal["safe_goal_x"] - start[1]
            dy = goal["safe_goal_y"] - start[2]
            tx = target[0] - start[1]
            ty = target[1] - start[2]
            denominator = math.hypot(dx, dy) * math.hypot(tx, ty)
            goal["safe_goal_alignment"] = (
                0.0 if denominator <= 1e-9 else (dx * tx + dy * ty) / denominator
            )
            goal["safe_goal_distance_from_robot_m"] = math.hypot(dx, dy)
        else:
            goal["frontier_progress_m"] = None
            goal["safe_goal_progress_m"] = None
            goal["safe_goal_alignment"] = None
            goal["safe_goal_distance_from_robot_m"] = None
        if start is not None and end is not None and target is not None:
            goal["actual_target_progress_m"] = (
                target_distance(start, target) - target_distance(end, target)
            )
        else:
            goal["actual_target_progress_m"] = None
        window = points_between(trajectory, goal["selection_ns"], end_ns)
        goal["rover_distance_travelled_m"] = travelled_distance(window)
        odometry_window = points_between(
            result["odom_trajectory"],
            goal["selection_ns"],
            end_ns,
        )
        goal["odometry_distance_travelled_m"] = travelled_distance(
            odometry_window
        )
        direct = goal["safe_goal_distance_from_robot_m"]
        goal["travel_to_direct_ratio"] = (
            None
            if direct is None or direct <= 1e-6
            else goal["rover_distance_travelled_m"] / direct
        )
        expected_endpoint = (
            goal["correction_goals"][-1]
            if goal["correction_goals"]
            else (goal["safe_goal_x"], goal["safe_goal_y"])
        )
        goal["final_expected_goal_x"] = expected_endpoint[0]
        goal["final_expected_goal_y"] = expected_endpoint[1]
        goal["end_goal_error_m"] = (
            None
            if end is None
            else math.hypot(
                end[1] - expected_endpoint[0],
                end[2] - expected_endpoint[1],
            )
        )
        command_source = (
            "/cmd_vel"
            if result["commands"].get("/cmd_vel")
            else "/cmd_vel_nav"
        )
        command_window = [
            command
            for command in result["commands"].get(command_source, [])
            if goal["selection_ns"] <= command[0] <= end_ns
        ]
        angular_signs = [
            1 if command[2] > 0.05 else -1
            for command in command_window
            if abs(command[2]) > 0.05
        ]
        goal["command_angular_sign_changes"] = sum(
            first != second
            for first, second in zip(angular_signs, angular_signs[1:])
        )
        goal["command_p95_abs_angular_rad_s"] = (
            None
            if not command_window
            else float(
                np.percentile(
                    np.abs([command[2] for command in command_window]),
                    95,
                )
            )
        )
        goal["command_full_speed_fraction"] = (
            None
            if not command_window
            else sum(command[1] >= 0.49 for command in command_window)
            / len(command_window)
        )
        omega_window = [
            item
            for item in result["omega_tracking"]
            if goal["selection_ns"] <= item[0] <= end_ns
        ]
        goal["omega_tracking_mean_abs_error_rad_s"] = (
            None
            if not omega_window
            else float(
                np.mean(
                    [abs(item[1] - item[2]) for item in omega_window]
                )
            )
        )
        eligible_omega = [
            item
            for item in omega_window
            if abs(item[1]) > 0.05 and abs(item[2]) > 0.05
        ]
        goal["omega_tracking_opposite_sign_fraction"] = (
            None
            if not eligible_omega
            else sum(item[1] * item[2] < 0.0 for item in eligible_omega)
            / len(eligible_omega)
        )
        goal["classification"] = (
            "BACKWARD_RECOVERY"
            if (
                goal["frontier_progress_m"] is not None
                and goal["frontier_progress_m"] < 0.0
            )
            else "FORWARD"
        )


def command_metrics(result: dict[str, Any]) -> dict[str, Any]:
    commands = result["commands"].get("/cmd_vel", [])
    if not commands:
        commands = result["commands"].get("/cmd_vel_nav", [])
    if not commands:
        return {
            "source": "",
            "samples": 0,
            "negative_linear_fraction": None,
            "angular_sign_changes": 0,
            "angular_sign_changes_per_moving_second": None,
            "p95_abs_angular": None,
            "max_abs_angular": None,
        }
    moving = [item for item in commands if abs(item[1]) > 0.03]
    negative = [item for item in moving if item[1] < -0.03]
    signs = [
        1 if item[2] > 0.05 else -1
        for item in moving
        if abs(item[2]) > 0.05
    ]
    sign_changes = sum(
        first != second for first, second in zip(signs, signs[1:])
    )
    moving_duration = (
        (moving[-1][0] - moving[0][0]) / 1e9 if len(moving) > 1 else 0.0
    )
    angular = np.abs([item[2] for item in commands])
    source = "/cmd_vel" if result["commands"].get("/cmd_vel") else "/cmd_vel_nav"
    return {
        "source": source,
        "samples": len(commands),
        "negative_linear_fraction": (
            len(negative) / len(moving) if moving else 0.0
        ),
        "angular_sign_changes": sign_changes,
        "angular_sign_changes_per_moving_second": (
            sign_changes / moving_duration if moving_duration > 0.0 else 0.0
        ),
        "p95_abs_angular": float(np.percentile(angular, 95)),
        "max_abs_angular": float(np.max(angular)),
    }


def analyze_bag(bag: Path) -> dict[str, Any]:
    metadata = metadata_for(bag)
    start_ns = int(metadata["starting_time"]["nanoseconds_since_epoch"])
    duration_ns = int(metadata["duration"]["nanoseconds"])
    result: dict[str, Any] = {
        "bag": bag.name,
        "path": str(bag),
        "start_ns": start_ns,
        "duration_ns": duration_ns,
        "end_ns": start_ns + duration_ns,
        "message_count": int(metadata["message_count"]),
        "topic_counts": topic_counts(metadata),
        "target": None,
        "target_start_ns": None,
        "logs": [],
        "parameters": {},
        "candidate_cycles": [],
        "selected_frontier_topic": [],
        "selected_goal_topic": [],
        "paths": [],
        "commands": defaultdict(list),
        "tf_trajectory": [],
        "pose_trajectory": [],
        "odom_trajectory": [],
        "omega_tracking": [],
    }
    connection = sqlite_connection(database_for(bag))
    topics = topic_table(connection)
    selected_names = {
        "/rosout",
        "/parameter_events",
        "/tf",
        "/tf_static",
        "/pose",
        "/genz/odometry",
        "/target_explorer/candidate_goals",
        "/target_explorer/selected_frontier",
        "/target_explorer/selected_goal",
        "/plan",
        "/cmd_vel",
        "/cmd_vel_nav",
    }
    chosen = {
        topic_id: (name, message_type)
        for topic_id, (name, message_type) in topics.items()
        if name in selected_names
    }
    classes = {
        topic_id: get_message(message_type)
        for topic_id, (_, message_type) in chosen.items()
    }
    placeholders = ",".join("?" for _ in chosen)
    transforms: dict[
        tuple[str, str], tuple[float, float, float]
    ] = {}
    query = (
        "SELECT topic_id,timestamp,data FROM messages "
        f"WHERE topic_id IN ({placeholders}) ORDER BY timestamp"
    )
    for topic_id, timestamp_ns, serialized in connection.execute(
        query, tuple(chosen)
    ):
        timestamp_ns = int(timestamp_ns)
        topic_name = chosen[int(topic_id)][0]
        message = deserialize_message(serialized, classes[int(topic_id)])
        if topic_name == "/rosout":
            result["logs"].append(
                (
                    timestamp_ns,
                    int(message.level),
                    str(message.name),
                    str(message.msg),
                )
            )
            if message.name == "omega_controller":
                match = OMEGA_RE.search(message.msg)
                if match:
                    result["omega_tracking"].append(
                        (
                            timestamp_ns,
                            float(match.group(1)),
                            float(match.group(2)),
                        )
                    )
        elif topic_name == "/parameter_events":
            if "target_explorer" not in message.node:
                continue
            for parameter in (
                list(message.new_parameters)
                + list(message.changed_parameters)
            ):
                result["parameters"][parameter.name] = parameter_value(
                    parameter.value
                )
        elif topic_name in ("/tf", "/tf_static"):
            for transform in message.transforms:
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
                stream = result["tf_trajectory"]
                if (
                    not stream
                    or timestamp_ns - stream[-1][0] >= 50_000_000
                ):
                    stream.append((timestamp_ns, *pose))
        elif topic_name == "/pose":
            pose = message.pose.pose
            result["pose_trajectory"].append(
                (
                    timestamp_ns,
                    float(pose.position.x),
                    float(pose.position.y),
                    yaw_from_quaternion(pose.orientation),
                )
            )
        elif topic_name == "/genz/odometry":
            pose = message.pose.pose
            result["odom_trajectory"].append(
                (
                    timestamp_ns,
                    float(pose.position.x),
                    float(pose.position.y),
                    yaw_from_quaternion(pose.orientation),
                )
            )
        elif topic_name in (
            "/target_explorer/selected_frontier",
            "/target_explorer/selected_goal",
        ):
            pose = message.pose
            item = (
                timestamp_ns,
                float(pose.position.x),
                float(pose.position.y),
                yaw_from_quaternion(pose.orientation),
            )
            key = (
                "selected_frontier_topic"
                if topic_name.endswith("selected_frontier")
                else "selected_goal_topic"
            )
            result[key].append(item)
        elif topic_name == "/target_explorer/candidate_goals":
            candidates = [
                (
                    float(marker.pose.position.x),
                    float(marker.pose.position.y),
                    float(marker.color.r),
                    float(marker.color.g),
                )
                for marker in message.markers
                if marker.action == marker.ADD
                and marker.ns == "target_explorer_candidates"
            ]
            result["candidate_cycles"].append((timestamp_ns, candidates))
        elif topic_name == "/plan":
            poses = message.poses
            path = [
                (
                    float(pose.pose.position.x),
                    float(pose.pose.position.y),
                )
                for pose in poses
            ]
            result["paths"].append((timestamp_ns, path))
        elif topic_name in ("/cmd_vel", "/cmd_vel_nav"):
            result["commands"][topic_name].append(
                (
                    timestamp_ns,
                    float(message.linear.x),
                    float(message.angular.z),
                )
            )

    result["map"] = latest_grid(connection, topics, "/map")
    result["global_costmap"] = latest_grid(
        connection, topics, "/global_costmap/costmap"
    )
    connection.close()

    tf_trajectory = clean_trajectory(result["tf_trajectory"])
    pose_trajectory = clean_trajectory(result["pose_trajectory"])
    result["odom_trajectory"] = clean_trajectory(result["odom_trajectory"])
    if len(tf_trajectory) >= max(10, len(pose_trajectory) // 2):
        result["trajectory"] = tf_trajectory
        result["trajectory_source"] = "TF map→base_footprint/base_link"
    else:
        result["trajectory"] = pose_trajectory
        result["trajectory_source"] = "/pose"
    parse_target_logs(result)
    enrich_goals(result)
    result["command_metrics"] = command_metrics(result)
    result["navigation_log_metrics"] = navigation_log_metrics(result)
    result["trajectory_metrics"] = trajectory_summary(result)
    associate_paths(result)
    return result


def trajectory_summary(result: dict[str, Any]) -> dict[str, Any]:
    trajectory = result["trajectory"]
    target = result["target"]
    summary = {
        "source": result["trajectory_source"],
        "samples": len(trajectory),
        "travelled_m": travelled_distance(trajectory),
        "start_x": None,
        "start_y": None,
        "end_x": None,
        "end_y": None,
        "start_target_distance_m": None,
        "end_target_distance_m": None,
        "net_target_progress_m": None,
        "maximum_5s_backward_progress_m": None,
    }
    if not trajectory:
        return summary
    summary.update(
        start_x=trajectory[0][1],
        start_y=trajectory[0][2],
        end_x=trajectory[-1][1],
        end_y=trajectory[-1][2],
    )
    if target is None:
        return summary
    distances = [target_distance(point, target) for point in trajectory]
    summary["start_target_distance_m"] = distances[0]
    summary["end_target_distance_m"] = distances[-1]
    summary["net_target_progress_m"] = distances[0] - distances[-1]
    worst = 0.0
    left = 0
    for right, point in enumerate(trajectory):
        while point[0] - trajectory[left][0] > 5_000_000_000:
            left += 1
        worst = max(worst, distances[right] - distances[left])
    summary["maximum_5s_backward_progress_m"] = worst
    return summary


def navigation_log_metrics(result: dict[str, Any]) -> dict[str, int]:
    metrics = Counter()
    for _, _, node, message in result["logs"]:
        if "failed to create plan" in message:
            metrics["planner_failed_to_create_plan"] += 1
        if "Failed to make progress" in message:
            metrics["controller_failed_to_make_progress"] += 1
        if "Control loop missed" in message:
            metrics["controller_missed_rate"] += 1
        if "Received request to clear entirely" in message:
            metrics["costmap_clears"] += 1
        if node == "bt_navigator" and message == "Goal succeeded":
            metrics["nav2_goals_succeeded"] += 1
    return dict(metrics)


def associate_paths(result: dict[str, Any]) -> None:
    paths = result["paths"]
    for goal in result["goals"]:
        candidates = []
        for timestamp_ns, path in paths:
            if not path:
                continue
            if timestamp_ns < goal["selection_ns"] - 500_000_000:
                continue
            if timestamp_ns > goal["selection_ns"] + 2_000_000_000:
                continue
            endpoint_error = math.hypot(
                path[-1][0] - goal["safe_goal_x"],
                path[-1][1] - goal["safe_goal_y"],
            )
            candidates.append((endpoint_error, abs(timestamp_ns - goal["selection_ns"]), path))
        goal["_path"] = min(candidates)[2] if candidates else []


def grid_extent(grid: Grid) -> tuple[float, float, float, float]:
    return (
        grid.origin_x,
        grid.origin_x + grid.width * grid.resolution,
        grid.origin_y,
        grid.origin_y + grid.height * grid.resolution,
    )


def occupancy_colors(grid: Grid, costmap: bool = False) -> np.ndarray:
    data = grid.data
    image = np.ones((*data.shape, 3), dtype=float)
    image[data < 0] = (0.55, 0.55, 0.55)
    if costmap:
        known = data >= 0
        intensity = np.clip(data / 100.0, 0.0, 1.0)
        image[known, 0] = 1.0
        image[known, 1] = 1.0 - 0.9 * intensity[known]
        image[known, 2] = 1.0 - 0.9 * intensity[known]
    else:
        occupied = data >= 65
        uncertain = (data > 10) & (data < 65)
        image[occupied] = (0.05, 0.05, 0.05)
        image[uncertain] = (0.75, 0.75, 0.75)
    return image


def add_time_trajectory(
    axis: Any,
    trajectory: list[tuple[int, float, float, float]],
    start_ns: int,
) -> None:
    if len(trajectory) < 2:
        return
    points = np.asarray([[point[1], point[2]] for point in trajectory])
    segments = np.stack((points[:-1], points[1:]), axis=1)
    times = np.asarray(
        [(point[0] - start_ns) / 1e9 for point in trajectory[:-1]]
    )
    collection = LineCollection(
        segments,
        cmap="viridis",
        array=times,
        linewidth=2.4,
        zorder=4,
    )
    axis.add_collection(collection)
    plt.colorbar(collection, ax=axis, label="bag time (s)", fraction=0.045)


def plot_spatial(result: dict[str, Any]) -> Path:
    figure, axis = plt.subplots(figsize=(11, 9), constrained_layout=True)
    grid = result["map"]
    if grid is not None and abs(grid.origin_yaw) < 1e-6:
        axis.imshow(
            occupancy_colors(grid),
            origin="lower",
            extent=grid_extent(grid),
            interpolation="nearest",
            zorder=0,
        )
    add_time_trajectory(axis, result["trajectory"], result["start_ns"])
    for goal in result["goals"]:
        number = goal["goal_number"]
        recovery = goal["classification"] == "BACKWARD_RECOVERY"
        color = "crimson" if recovery else "tab:orange"
        axis.scatter(
            goal["frontier_x"],
            goal["frontier_y"],
            marker="x",
            s=95,
            linewidth=2.5,
            color=color,
            zorder=7,
        )
        axis.scatter(
            goal["safe_goal_x"],
            goal["safe_goal_y"],
            marker="o",
            s=55,
            facecolor="deepskyblue",
            edgecolor="navy",
            zorder=8,
        )
        axis.annotate(
            str(number),
            (goal["safe_goal_x"], goal["safe_goal_y"]),
            xytext=(5, 6),
            textcoords="offset points",
            fontsize=9,
            weight="bold",
            zorder=9,
        )
        for corrected_x, corrected_y in goal["correction_goals"]:
            axis.scatter(
                corrected_x,
                corrected_y,
                marker="D",
                s=55,
                color="magenta",
                zorder=8,
            )
        path = goal.get("_path", [])
        if path:
            axis.plot(
                [point[0] for point in path],
                [point[1] for point in path],
                color=color,
                alpha=0.35,
                linewidth=1.0,
                zorder=3,
            )
    if result["trajectory"] and result["target"] is not None:
        last = result["trajectory"][-1]
        dx = result["target"][0] - last[1]
        dy = result["target"][1] - last[2]
        norm = math.hypot(dx, dy)
        if norm:
            axis.arrow(
                last[1],
                last[2],
                3.0 * dx / norm,
                3.0 * dy / norm,
                width=0.08,
                color="royalblue",
                zorder=8,
            )
            axis.annotate(
                f"final target direction\n({result['target'][0]:.0f}, "
                f"{result['target'][1]:.0f})",
                (last[1] + 3.0 * dx / norm, last[2] + 3.0 * dy / norm),
                fontsize=9,
                color="royalblue",
            )
    axis.set_title(
        f"{result['bag']}: trajectory, selected frontiers, and sent goals\n"
        "orange X=forward frontier, red X=negative-progress recovery, "
        "blue circle=safe goal, magenta diamond=correction"
    )
    axis.set_xlabel("map x (m)")
    axis.set_ylabel("map y (m)")
    axis.set_aspect("equal", adjustable="box")
    axis.grid(alpha=0.18)
    path = PLOTS / f"{result['bag'].replace('.bag', '')}_spatial.png"
    figure.savefig(path, dpi=180)
    plt.close(figure)
    return path


def plot_progress(result: dict[str, Any]) -> Optional[Path]:
    trajectory = result["trajectory"]
    target = result["target"]
    if not trajectory or target is None:
        return None
    times = np.asarray(
        [(point[0] - result["start_ns"]) / 1e9 for point in trajectory]
    )
    distances = np.asarray([target_distance(point, target) for point in trajectory])
    figure, axes = plt.subplots(
        2, 1, figsize=(12, 8), sharex=True, constrained_layout=True
    )
    axes[0].plot(times, distances, color="black", linewidth=1.8)
    axes[0].set_ylabel("distance to final target (m)\n(up = backwards)")
    axes[0].grid(alpha=0.25)
    axes[1].plot(
        times,
        [point[1] for point in trajectory],
        label="map x",
        linewidth=1.5,
    )
    axes[1].plot(
        times,
        [point[2] for point in trajectory],
        label="map y",
        linewidth=1.5,
    )
    for goal in result["goals"]:
        time = goal["selection_relative_sec"]
        color = (
            "crimson"
            if goal["classification"] == "BACKWARD_RECOVERY"
            else "tab:green"
        )
        for axis in axes:
            axis.axvline(time, color=color, alpha=0.8, linewidth=1.2)
        axes[0].annotate(
            f"G{goal['goal_number']}",
            (time, np.interp(time, times, distances)),
            xytext=(3, 5),
            textcoords="offset points",
            color=color,
            weight="bold",
        )
    axes[1].set_xlabel("bag time (s)")
    axes[1].set_ylabel("map coordinate (m)")
    axes[1].legend()
    axes[1].grid(alpha=0.25)
    axes[0].set_title(
        f"{result['bag']}: actual rover progress toward final target "
        "(green=forward goal, red=negative-progress recovery)"
    )
    path = PLOTS / f"{result['bag'].replace('.bag', '')}_progress.png"
    figure.savefig(path, dpi=180)
    plt.close(figure)
    return path


def plot_goal_metrics(result: dict[str, Any]) -> Optional[Path]:
    goals = result["goals"]
    if not goals:
        return None
    numbers = np.arange(1, len(goals) + 1)
    frontier_progress = [
        goal["frontier_progress_m"] or 0.0 for goal in goals
    ]
    safe_progress = [
        goal["safe_goal_progress_m"] or 0.0 for goal in goals
    ]
    actual_progress = [
        goal["actual_target_progress_m"] or 0.0 for goal in goals
    ]
    standoffs = [goal["standoff_path_m"] for goal in goals]
    scores = [goal["score"] for goal in goals]
    figure, axes = plt.subplots(
        3, 1, figsize=(12, 10), sharex=True, constrained_layout=True
    )
    width = 0.36
    axes[0].bar(
        numbers - width / 2,
        frontier_progress,
        width,
        label="original frontier predicted progress",
        color="tab:orange",
    )
    axes[0].bar(
        numbers + width / 2,
        safe_progress,
        width,
        label="safe sent goal predicted progress",
        color="deepskyblue",
    )
    axes[0].axhline(0.0, color="black", linewidth=1)
    axes[0].set_ylabel("predicted progress (m)")
    axes[0].legend()
    axes[0].grid(axis="y", alpha=0.25)
    colors = ["crimson" if value < 0.0 else "tab:green" for value in actual_progress]
    axes[1].bar(numbers, actual_progress, color=colors)
    axes[1].axhline(0.0, color="black", linewidth=1)
    axes[1].set_ylabel("actual progress during goal (m)")
    axes[1].grid(axis="y", alpha=0.25)
    axes[2].bar(numbers, standoffs, color="slateblue", label="path standoff")
    score_axis = axes[2].twinx()
    score_axis.plot(
        numbers,
        scores,
        marker="o",
        color="darkorange",
        label="frontier score",
    )
    axes[2].set_ylabel("standoff (m)")
    score_axis.set_ylabel("score")
    axes[2].set_xlabel("goal number")
    axes[2].set_xticks(numbers)
    axes[2].grid(axis="y", alpha=0.25)
    figure.suptitle(
        f"{result['bag']}: goal direction, actual outcome, standoff, and score"
    )
    path = PLOTS / f"{result['bag'].replace('.bag', '')}_goals.png"
    figure.savefig(path, dpi=180)
    plt.close(figure)
    return path


def plot_goal_zooms(result: dict[str, Any]) -> Optional[Path]:
    goals = result["goals"]
    if not goals:
        return None
    columns = 2
    rows = int(math.ceil(len(goals) / columns))
    figure, axes = plt.subplots(
        rows,
        columns,
        figsize=(14, 5.5 * rows),
        constrained_layout=True,
    )
    axis_list = list(np.asarray(axes).reshape(-1))
    grid = result["map"]
    for axis, goal in zip(axis_list, goals):
        end_ns = goal["terminal_ns"] or result["end_ns"]
        trajectory = points_between(
            result["trajectory"],
            goal["selection_ns"],
            end_ns,
        )
        context_x = [
            goal["frontier_x"],
            goal["safe_goal_x"],
            goal["final_expected_goal_x"],
        ] + [point[1] for point in trajectory]
        context_y = [
            goal["frontier_y"],
            goal["safe_goal_y"],
            goal["final_expected_goal_y"],
        ] + [point[2] for point in trajectory]
        minimum_x, maximum_x = min(context_x) - 1.2, max(context_x) + 1.2
        minimum_y, maximum_y = min(context_y) - 1.2, max(context_y) + 1.2
        if grid is not None and abs(grid.origin_yaw) < 1e-6:
            axis.imshow(
                occupancy_colors(grid),
                origin="lower",
                extent=grid_extent(grid),
                interpolation="nearest",
                zorder=0,
            )
        path = goal.get("_path", [])
        if path:
            axis.plot(
                [point[0] for point in path],
                [point[1] for point in path],
                linestyle="--",
                color="royalblue",
                linewidth=1.3,
                label="Nav2 plan",
                zorder=2,
            )
        if trajectory:
            axis.plot(
                [point[1] for point in trajectory],
                [point[2] for point in trajectory],
                color="black",
                linewidth=2.0,
                label="actual rover trace",
                zorder=4,
            )
            axis.scatter(
                trajectory[0][1],
                trajectory[0][2],
                marker="s",
                s=65,
                color="limegreen",
                edgecolor="black",
                label="start",
                zorder=7,
            )
            axis.scatter(
                trajectory[-1][1],
                trajectory[-1][2],
                marker="*",
                s=120,
                color="gold",
                edgecolor="black",
                label="end",
                zorder=7,
            )
        axis.scatter(
            goal["frontier_x"],
            goal["frontier_y"],
            marker="x",
            s=100,
            linewidth=2.5,
            color="crimson",
            label="original frontier",
            zorder=8,
        )
        axis.scatter(
            goal["safe_goal_x"],
            goal["safe_goal_y"],
            marker="o",
            s=65,
            color="deepskyblue",
            edgecolor="navy",
            label="sent safe goal",
            zorder=8,
        )
        for corrected_x, corrected_y in goal["correction_goals"]:
            axis.scatter(
                corrected_x,
                corrected_y,
                marker="D",
                s=60,
                color="magenta",
                label="corrected goal",
                zorder=8,
            )
        ratio = goal["travel_to_direct_ratio"]
        ratio_text = "n/a" if ratio is None else f"{ratio:.1f}×"
        axis.set_title(
            f"G{goal['goal_number']} {goal['classification']}: "
            f"travel {goal['rover_distance_travelled_m']:.2f} m, "
            f"direct {goal['safe_goal_distance_from_robot_m']:.2f} m "
            f"({ratio_text}), target progress "
            f"{goal['actual_target_progress_m']:+.2f} m"
        )
        axis.set_xlim(minimum_x, maximum_x)
        axis.set_ylim(minimum_y, maximum_y)
        axis.set_aspect("equal", adjustable="box")
        axis.grid(alpha=0.18)
        axis.set_xlabel("map x (m)")
        axis.set_ylabel("map y (m)")
    for axis in axis_list[len(goals):]:
        axis.set_visible(False)
    handles, labels = axis_list[0].get_legend_handles_labels()
    unique = dict(zip(labels, handles))
    figure.legend(
        unique.values(),
        unique.keys(),
        loc="outside upper center",
        ncol=min(6, len(unique)),
    )
    figure.suptitle(
        f"{result['bag']}: per-goal path efficiency and rover motion",
        fontsize=16,
    )
    path = PLOTS / f"{result['bag'].replace('.bag', '')}_goal_zooms.png"
    figure.savefig(path, dpi=180)
    plt.close(figure)
    return path


def plot_commands(result: dict[str, Any]) -> Optional[Path]:
    source = result["command_metrics"]["source"]
    commands = result["commands"].get(source, [])
    if not commands:
        return None
    times = [(item[0] - result["start_ns"]) / 1e9 for item in commands]
    figure, axes = plt.subplots(
        2, 1, figsize=(12, 7), sharex=True, constrained_layout=True
    )
    axes[0].plot(times, [item[1] for item in commands], linewidth=0.9)
    axes[0].axhline(0.0, color="black", linewidth=0.8)
    axes[0].set_ylabel("linear x (m/s)")
    axes[0].grid(alpha=0.25)
    axes[1].plot(
        times,
        [item[2] for item in commands],
        linewidth=0.9,
        color="tab:purple",
    )
    axes[1].axhline(0.0, color="black", linewidth=0.8)
    axes[1].set_ylabel("angular z (rad/s)")
    axes[1].set_xlabel("bag time (s)")
    axes[1].grid(alpha=0.25)
    axes[0].set_title(f"{result['bag']}: recorded velocity commands on {source}")
    path = PLOTS / f"{result['bag'].replace('.bag', '')}_commands.png"
    figure.savefig(path, dpi=180)
    plt.close(figure)
    return path


def plot_costmap(result: dict[str, Any]) -> Optional[Path]:
    grid = result["global_costmap"]
    if grid is None or abs(grid.origin_yaw) >= 1e-6:
        return None
    figure, axis = plt.subplots(figsize=(11, 9), constrained_layout=True)
    axis.imshow(
        occupancy_colors(grid, costmap=True),
        origin="lower",
        extent=grid_extent(grid),
        interpolation="nearest",
    )
    add_time_trajectory(axis, result["trajectory"], result["start_ns"])
    for goal in result["goals"]:
        axis.scatter(
            goal["safe_goal_x"],
            goal["safe_goal_y"],
            marker="o",
            s=45,
            color=(
                "crimson"
                if goal["classification"] == "BACKWARD_RECOVERY"
                else "deepskyblue"
            ),
            edgecolor="black",
            zorder=8,
        )
        axis.annotate(
            str(goal["goal_number"]),
            (goal["safe_goal_x"], goal["safe_goal_y"]),
            xytext=(4, 4),
            textcoords="offset points",
            weight="bold",
        )
    axis.set_title(
        f"{result['bag']}: final recorded global costmap and sent goals\n"
        "white=cost 0, red gradient=inflation/lethal, grey=unknown"
    )
    axis.set_xlabel("map x (m)")
    axis.set_ylabel("map y (m)")
    axis.set_aspect("equal", adjustable="box")
    path = PLOTS / f"{result['bag'].replace('.bag', '')}_costmap.png"
    figure.savefig(path, dpi=180)
    plt.close(figure)
    return path


def write_csv(
    path: Path,
    rows: list[dict[str, Any]],
    fields: list[str],
) -> None:
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def serializable_result(result: dict[str, Any]) -> dict[str, Any]:
    return {
        "bag": result["bag"],
        "path": result["path"],
        "start_time": datetime.fromtimestamp(
            result["start_ns"] / 1e9
        ).astimezone().isoformat(),
        "duration_sec": result["duration_ns"] / 1e9,
        "message_count": result["message_count"],
        "target": result["target"],
        "parameters": result["parameters"],
        "trajectory_metrics": result["trajectory_metrics"],
        "command_metrics": result["command_metrics"],
        "navigation_log_metrics": result["navigation_log_metrics"],
        "rejections": result["rejections"],
        "goals": [
            {
                key: value
                for key, value in goal.items()
                if key not in {"_path", "selection_ns", "terminal_ns"}
            }
            for goal in result["goals"]
        ],
        "candidate_cycle_count": len(result["candidate_cycles"]),
        "selected_frontier_topic_count": len(
            result["selected_frontier_topic"]
        ),
        "selected_goal_topic_count": len(result["selected_goal_topic"]),
        "map": (
            None
            if result["map"] is None
            else {
                "frame": result["map"].frame_id,
                "resolution": result["map"].resolution,
                "width": result["map"].width,
                "height": result["map"].height,
                "origin": (
                    result["map"].origin_x,
                    result["map"].origin_y,
                    result["map"].origin_yaw,
                ),
            }
        ),
    }


def main() -> int:
    OUTPUT.mkdir(parents=True, exist_ok=True)
    PLOTS.mkdir(parents=True, exist_ok=True)
    results = []
    for bag in BAGS:
        print(f"Analyzing {bag}...", flush=True)
        results.append(analyze_bag(bag))
    plot_manifest = {}
    for result in results:
        paths = [
            plot_spatial(result),
            plot_progress(result),
            plot_goal_metrics(result),
            plot_goal_zooms(result),
            plot_commands(result),
            plot_costmap(result),
        ]
        plot_manifest[result["bag"]] = [
            str(path.relative_to(OUTPUT)) for path in paths if path is not None
        ]
    summary = {
        "generated_at": datetime.now().astimezone().isoformat(),
        "bags": [serializable_result(result) for result in results],
        "plots": plot_manifest,
    }
    (OUTPUT / "summary.json").write_text(json.dumps(summary, indent=2))

    goal_rows = []
    for result in results:
        for goal in result["goals"]:
            goal_rows.append(
                {
                    key: (
                        json.dumps(value)
                        if isinstance(value, (list, tuple, dict))
                        else value
                    )
                    for key, value in goal.items()
                    if key not in {"_path", "selection_ns", "terminal_ns"}
                }
            )
    goal_fields = sorted({key for row in goal_rows for key in row})
    write_csv(OUTPUT / "goal_episodes.csv", goal_rows, goal_fields)

    bag_rows = []
    for result in results:
        row = {
            "bag": result["bag"],
            "duration_sec": result["duration_ns"] / 1e9,
            "messages": result["message_count"],
            "goals": len(result["goals"]),
            **result["trajectory_metrics"],
            **{
                f"command_{key}": value
                for key, value in result["command_metrics"].items()
            },
            **{
                f"navigation_{key}": value
                for key, value in result["navigation_log_metrics"].items()
            },
            **{
                f"rejection_{key}": (
                    json.dumps(value) if isinstance(value, dict) else value
                )
                for key, value in result["rejections"].items()
            },
        }
        bag_rows.append(row)
    bag_fields = sorted({key for row in bag_rows for key in row})
    write_csv(OUTPUT / "bag_summary.csv", bag_rows, bag_fields)

    parameter_rows = []
    for result in results:
        for name, value in sorted(result["parameters"].items()):
            parameter_rows.append(
                {"bag": result["bag"], "parameter": name, "value": value}
            )
    write_csv(
        OUTPUT / "target_explorer_parameters.csv",
        parameter_rows,
        ["bag", "parameter", "value"],
    )

    log_rows = []
    for result in results:
        for timestamp_ns, level, node, message in result["logs"]:
            if node == "target_explorer_node":
                log_rows.append(
                    {
                        "bag": result["bag"],
                        "relative_sec": (timestamp_ns - result["start_ns"]) / 1e9,
                        "level": level,
                        "message": message,
                    }
                )
    write_csv(
        OUTPUT / "target_explorer_logs.csv",
        log_rows,
        ["bag", "relative_sec", "level", "message"],
    )
    navigation_rows = []
    navigation_nodes = (
        "planner_server",
        "controller_server",
        "bt_navigator",
        "global_costmap",
        "local_costmap",
    )
    navigation_terms = (
        "goal",
        "plan",
        "progress",
        "cancel",
        "clear entirely",
        "missed",
    )
    for result in results:
        for timestamp_ns, level, node, message in result["logs"]:
            if (
                any(hint in node for hint in navigation_nodes)
                and (
                    level >= 30
                    or any(term in message.lower() for term in navigation_terms)
                )
            ):
                navigation_rows.append(
                    {
                        "bag": result["bag"],
                        "relative_sec": (
                            timestamp_ns - result["start_ns"]
                        ) / 1e9,
                        "level": level,
                        "node": node,
                        "message": message,
                    }
                )
    write_csv(
        OUTPUT / "navigation_events.csv",
        navigation_rows,
        ["bag", "relative_sec", "level", "node", "message"],
    )
    print(f"Wrote analysis to {OUTPUT}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
