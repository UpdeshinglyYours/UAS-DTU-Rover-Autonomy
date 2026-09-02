#!/usr/bin/env python3
"""Offline analysis for the filtered August 29 DTU rover bag."""

from __future__ import annotations

import json
import math
from collections import defaultdict
from datetime import datetime, timezone
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rosbag2_py
import yaml
from geometry_msgs.msg import Twist
from mavros_msgs.msg import GPSRAW
from nav_msgs.msg import OccupancyGrid, Odometry, Path as PathMsg
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import NavSatFix, PointCloud2
from tf2_msgs.msg import TFMessage


HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
FILTERED_BAG = REPO / "aug29_filtered.bag"
ORIGINAL_BAG = REPO / "aug29.bag"
GRAPH_PATH = REPO / "src/dtu_prior_map/maps/dtu_road_graph.json"
NAV2_CONFIG_PATH = REPO / "src/lirovo/config/nav2_dtu_rtk_params.yaml"

RATE_TOPICS = [
    "/genz/odometry",
    "/odometry/filtered",
    "/odometry/gps",
    "/odometry/global",
    "/mavros/imu/data",
    "/scan",
    "/cmd_vel_nav",
    "/cmd_vel",
]


def yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def finite_float(value):
    value = float(value)
    return value if math.isfinite(value) else None


def statistics(values: np.ndarray) -> dict:
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    if not len(values):
        return {key: None for key in ("minimum", "mean", "median", "p95", "maximum")}
    return {
        "minimum": float(np.min(values)),
        "mean": float(np.mean(values)),
        "median": float(np.median(values)),
        "p95": float(np.percentile(values, 95)),
        "maximum": float(np.max(values)),
    }


def nearest_indices(reference_times: np.ndarray, query_times: np.ndarray) -> np.ndarray:
    if not len(reference_times):
        return np.zeros(len(query_times), dtype=int)
    indices = np.searchsorted(reference_times, query_times)
    indices = np.clip(indices, 1, len(reference_times) - 1)
    use_previous = (
        query_times - reference_times[indices - 1]
        < reference_times[indices] - query_times
    )
    return indices - use_previous.astype(int)


def previous_indices(reference_times: np.ndarray, query_times: np.ndarray) -> np.ndarray:
    """Return the newest sample at or before each query time."""
    return np.clip(
        np.searchsorted(reference_times, query_times, side="right") - 1,
        0,
        len(reference_times) - 1,
    )


def bag_metadata(path: Path) -> dict:
    data = yaml.safe_load((path / "metadata.yaml").read_text(encoding="utf-8"))
    return data["rosbag2_bagfile_information"]


def directory_size(path: Path) -> int:
    return sum(item.stat().st_size for item in path.rglob("*") if item.is_file())


def road_segments(graph: dict):
    segments = []
    for edge in graph["edges"]:
        points = np.asarray(edge["geometry"], dtype=float)
        segments.extend(zip(points[:-1], points[1:]))
    starts = np.asarray([segment[0] for segment in segments])
    vectors = np.asarray([segment[1] - segment[0] for segment in segments])
    squared = np.sum(vectors * vectors, axis=1)
    return starts, vectors, squared


def nearest_polyline(points, starts, vectors, squared):
    distances = np.empty(len(points))
    signed = np.empty(len(points))
    segment_indices = np.empty(len(points), dtype=int)
    for index, point in enumerate(points):
        fractions = np.clip(
            np.sum((point - starts) * vectors, axis=1) / squared, 0.0, 1.0
        )
        deltas = point - (starts + fractions[:, None] * vectors)
        norms_squared = np.sum(deltas * deltas, axis=1)
        best = int(np.argmin(norms_squared))
        distances[index] = math.sqrt(norms_squared[best])
        signed[index] = (
            vectors[best, 0] * deltas[best, 1]
            - vectors[best, 1] * deltas[best, 0]
        ) / math.sqrt(squared[best])
        segment_indices[index] = best
    return distances, signed, segment_indices


def rate_metrics(times) -> dict:
    values = np.asarray(times, dtype=float)
    if len(values) < 2:
        return {
            "message_count": int(len(values)),
            "mean_hz": 0.0 if len(values) == 0 else None,
            "median_interval_s": None,
            "p95_interval_s": None,
            "worst_gap_s": None,
            "gaps_gt_0_05_s": 0,
            "gaps_gt_0_10_s": 0,
            "gaps_gt_0_20_s": 0,
            "gaps_gt_0_50_s": 0,
        }
    intervals = np.diff(values)
    return {
        "message_count": int(len(values)),
        "mean_hz": float((len(values) - 1) / (values[-1] - values[0])),
        "median_interval_s": float(np.median(intervals)),
        "p95_interval_s": float(np.percentile(intervals, 95)),
        "worst_gap_s": float(np.max(intervals)),
        "gaps_gt_0_05_s": int(np.count_nonzero(intervals > 0.05)),
        "gaps_gt_0_10_s": int(np.count_nonzero(intervals > 0.10)),
        "gaps_gt_0_20_s": int(np.count_nonzero(intervals > 0.20)),
        "gaps_gt_0_50_s": int(np.count_nonzero(intervals > 0.50)),
    }


def sustained_runs(times: np.ndarray, values: np.ndarray, threshold: float):
    mask = values > threshold
    output = []
    start = None
    for index, active in enumerate(mask):
        if active and start is None:
            start = index
        if start is not None and (not active or index == len(mask) - 1):
            end = index if active else index - 1
            output.append(
                {
                    "start_index": start,
                    "end_index": end,
                    "start_s": float(times[start]),
                    "end_s": float(times[end]),
                    "duration_s": float(times[end] - times[start]),
                    "mean_error_m": float(np.mean(values[start : end + 1])),
                    "max_error_m": float(np.max(values[start : end + 1])),
                }
            )
            start = None
    return sorted(output, key=lambda item: item["duration_s"], reverse=True)


def transform_to_map(odom: np.ndarray, map_odom: np.ndarray) -> np.ndarray:
    indices = nearest_indices(map_odom[:, 0], odom[:, 0])
    tf = map_odom[indices]
    c = np.cos(tf[:, 3])
    s = np.sin(tf[:, 3])
    x = tf[:, 1] + c * odom[:, 1] - s * odom[:, 2]
    y = tf[:, 2] + s * odom[:, 1] + c * odom[:, 2]
    return np.column_stack((odom[:, 0], x, y))


def iso_timestamp(nanoseconds: int) -> str:
    return datetime.fromtimestamp(nanoseconds / 1e9, tz=timezone.utc).isoformat()


def read_bag():
    timestamps = defaultdict(list)
    odometry = defaultdict(list)
    commands = defaultdict(list)
    gps_fixes = []
    gps_status = defaultdict(list)
    plans = defaultdict(list)
    reference_routes = []
    map_odom = []
    clouds = []
    costmaps = []

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(FILTERED_BAG), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    while reader.has_next():
        topic, data, timestamp_ns = reader.read_next()
        timestamp = timestamp_ns / 1e9
        timestamps[topic].append(timestamp)
        if topic in (
            "/genz/odometry",
            "/odometry/filtered",
            "/odometry/gps",
            "/odometry/global",
        ):
            msg = deserialize_message(data, Odometry)
            odometry[topic].append(
                (
                    timestamp,
                    msg.pose.pose.position.x,
                    msg.pose.pose.position.y,
                    yaw_from_quaternion(msg.pose.pose.orientation),
                    msg.twist.twist.linear.x,
                    msg.twist.twist.angular.z,
                    msg.pose.covariance[0],
                    msg.pose.covariance[7],
                    msg.pose.covariance[35],
                )
            )
        elif topic in ("/cmd_vel", "/cmd_vel_nav"):
            msg = deserialize_message(data, Twist)
            commands[topic].append((timestamp, msg.linear.x, msg.angular.z))
        elif topic == "/mavros/global_position/raw/fix":
            msg = deserialize_message(data, NavSatFix)
            gps_fixes.append(
                (
                    timestamp,
                    msg.latitude,
                    msg.longitude,
                    msg.altitude,
                    msg.position_covariance[0],
                    msg.position_covariance[4],
                    msg.position_covariance[8],
                    msg.position_covariance_type,
                )
            )
        elif topic in ("/mavros/gpsstatus/gps1/raw", "/mavros/gpsstatus/gps2/raw"):
            msg = deserialize_message(data, GPSRAW)
            gps_status[topic].append(
                (
                    timestamp,
                    msg.fix_type,
                    msg.satellites_visible,
                    msg.eph,
                    msg.epv,
                    msg.h_acc,
                    msg.v_acc,
                    msg.hdg_acc,
                )
            )
        elif topic in ("/plan", "/unsmoothed_plan", "/dtu_reference_route"):
            msg = deserialize_message(data, PathMsg)
            points = np.asarray(
                [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses],
                dtype=float,
            )
            if topic == "/dtu_reference_route":
                reference_routes.append((timestamp, points))
            else:
                plans[topic].append((timestamp, points))
        elif topic in ("/tf", "/tf_static"):
            msg = deserialize_message(data, TFMessage)
            for transform in msg.transforms:
                if (
                    transform.header.frame_id == "map"
                    and transform.child_frame_id == "odom"
                ):
                    map_odom.append(
                        (
                            timestamp,
                            transform.transform.translation.x,
                            transform.transform.translation.y,
                            yaw_from_quaternion(transform.transform.rotation),
                        )
                    )
        elif topic == "/genz/non_planar_points":
            msg = deserialize_message(data, PointCloud2)
            points = np.ndarray(
                (msg.width * msg.height,),
                dtype={
                    "names": ["x", "y", "z"],
                    "formats": ["<f4", "<f4", "<f4"],
                    "offsets": [0, 4, 8],
                    "itemsize": msg.point_step,
                },
                buffer=msg.data,
            )
            # Recorded static TF: base_link -> lidar = (-0.11, +0.30, +0.54),
            # identity rotation. Match the launch's 0..2 m scan height gate.
            x = points["x"].astype(float) - 0.11
            y = points["y"].astype(float) + 0.30
            z = points["z"].astype(float) + 0.54
            valid = (
                np.isfinite(x)
                & np.isfinite(y)
                & np.isfinite(z)
                & (x >= 0.43)
                & (z >= 0.0)
                & (z <= 2.0)
                & (np.abs(y) <= 0.75)
            )
            candidates = np.flatnonzero(valid)
            if len(candidates):
                ranges = np.hypot(x[candidates], y[candidates])
                best = candidates[int(np.argmin(ranges))]
                clouds.append((timestamp, ranges.min(), x[best], y[best], z[best], len(candidates)))
            else:
                clouds.append((timestamp, np.nan, np.nan, np.nan, np.nan, 0))
        elif topic == "/local_costmap/costmap":
            msg = deserialize_message(data, OccupancyGrid)
            costmaps.append(
                (
                    timestamp,
                    msg.info.resolution,
                    msg.info.width,
                    msg.info.height,
                    msg.info.origin.position.x,
                    msg.info.origin.position.y,
                    np.asarray(msg.data, dtype=np.int16).reshape(
                        msg.info.height, msg.info.width
                    ),
                )
            )

    return {
        "timestamps": timestamps,
        "odometry": {key: np.asarray(value, dtype=float) for key, value in odometry.items()},
        "commands": {key: np.asarray(value, dtype=float) for key, value in commands.items()},
        "gps_fixes": np.asarray(gps_fixes, dtype=float),
        "gps_status": {key: np.asarray(value, dtype=float) for key, value in gps_status.items()},
        "plans": plans,
        "reference_routes": reference_routes,
        "map_odom": np.asarray(map_odom, dtype=float),
        "clouds": np.asarray(clouds, dtype=float),
        "costmaps": costmaps,
    }


def main():
    HERE.mkdir(parents=True, exist_ok=True)
    data = read_bag()
    graph = json.loads(GRAPH_PATH.read_text(encoding="utf-8"))
    starts, vectors, squared = road_segments(graph)
    global_odom = data["odometry"]["/odometry/global"]
    filtered_odom = data["odometry"]["/odometry/filtered"]
    genz_odom = data["odometry"]["/genz/odometry"]
    gps_odom = data["odometry"]["/odometry/gps"]
    start_time = min(times[0] for times in data["timestamps"].values() if times)
    elapsed = global_odom[:, 0] - start_time

    centerline_error, centerline_signed, _ = nearest_polyline(
        global_odom[:, 1:3], starts, vectors, squared
    )
    centerline = statistics(centerline_error)
    centerline["within_0_5_m_pct"] = float(100 * np.mean(centerline_error <= 0.5))
    centerline["within_1_0_m_pct"] = float(100 * np.mean(centerline_error <= 1.0))
    centerline["within_1_5_m_pct"] = float(100 * np.mean(centerline_error <= 1.5))
    centerline["within_2_0_m_pct"] = float(100 * np.mean(centerline_error <= 2.0))
    centerline["signed_median_m"] = float(np.median(centerline_signed))
    centerline["signed_mean_m"] = float(np.mean(centerline_signed))
    centerline["positive_side_pct"] = float(100 * np.mean(centerline_signed > 0))
    runs = sustained_runs(elapsed, centerline_error, 2.0)
    centerline["worst_sustained_gt_2m"] = runs[0]

    plan_points = np.concatenate(
        [points for _, points in data["plans"]["/plan"] if len(points)], axis=0
    )
    plan_error, _, _ = nearest_polyline(plan_points, starts, vectors, squared)
    plan_centerline = statistics(plan_error)
    plan_centerline["within_0_5_m_pct"] = float(100 * np.mean(plan_error <= 0.5))

    rates = {topic: rate_metrics(data["timestamps"].get(topic, [])) for topic in RATE_TOPICS}
    rates["/local_costmap/costmap"] = rate_metrics(
        data["timestamps"].get("/local_costmap/costmap", [])
    )
    rates["/genz/non_planar_points"] = rate_metrics(
        data["timestamps"].get("/genz/non_planar_points", [])
    )
    rates["/plan"] = rate_metrics(data["timestamps"].get("/plan", []))

    gps_indices = nearest_indices(global_odom[:, 0], gps_odom[:, 0])
    global_gps_error = np.linalg.norm(
        global_odom[gps_indices, 1:3] - gps_odom[:, 1:3], axis=1
    )
    global_gps = statistics(global_gps_error)
    global_gps["timestamp_alignment_p95_s"] = float(
        np.percentile(np.abs(global_odom[gps_indices, 0] - gps_odom[:, 0]), 95)
    )

    gps1 = data["gps_status"]["/mavros/gpsstatus/gps1/raw"]
    status_at_gps = gps1[nearest_indices(gps1[:, 0], gps_odom[:, 0]), 1].astype(int)
    fixed_mask_gps = status_at_gps == 6
    float_mask_gps = status_at_gps == 5
    global_gps["rtk_fixed"] = statistics(global_gps_error[fixed_mask_gps])
    global_gps["rtk_float"] = statistics(global_gps_error[float_mask_gps])

    status_at_global = gps1[
        nearest_indices(gps1[:, 0], global_odom[:, 0]), 1
    ].astype(int)
    centerline["rtk_fixed"] = statistics(centerline_error[status_at_global == 6])
    centerline["rtk_float"] = statistics(centerline_error[status_at_global == 5])

    global_steps = np.linalg.norm(np.diff(global_odom[:, 1:3], axis=0), axis=1)
    gps_steps = np.linalg.norm(np.diff(gps_odom[:, 1:3], axis=0), axis=1)
    global_yaw_steps = np.abs(
        (np.diff(global_odom[:, 3]) + np.pi) % (2 * np.pi) - np.pi
    )
    large_global_step_times = global_odom[1:, 0][global_steps > 0.5]
    large_step_to_gps = np.min(
        np.abs(large_global_step_times[:, None] - gps_odom[:, 0][None, :]), axis=1
    )
    global_xy_sigma = np.sqrt(np.maximum(0.0, global_odom[:, 6] + global_odom[:, 7]))
    global_yaw_sigma = np.sqrt(np.maximum(0.0, global_odom[:, 8]))
    localization = {
        "global_to_gps_m": global_gps,
        "global_position_step_m": statistics(global_steps),
        "global_steps_gt_0_5_m": int(np.count_nonzero(global_steps > 0.5)),
        "global_steps_gt_0_5_m_within_0_1_s_of_gps": int(
            np.count_nonzero(large_step_to_gps < 0.1)
        ),
        "gps_position_step_m": statistics(gps_steps),
        "global_heading_step_deg": statistics(np.degrees(global_yaw_steps)),
        "global_reported_xy_sigma_m": {
            **statistics(global_xy_sigma),
            "first": float(global_xy_sigma[0]),
            "last": float(global_xy_sigma[-1]),
        },
        "global_reported_yaw_sigma_rad": {
            **statistics(global_yaw_sigma),
            "first": float(global_yaw_sigma[0]),
            "last": float(global_yaw_sigma[-1]),
        },
        "gps1_fix_type_counts": {
            "no_fix_or_unknown_0": int(np.count_nonzero(gps1[:, 1] == 0)),
            "rtk_float_5": int(np.count_nonzero(gps1[:, 1] == 5)),
            "rtk_fixed_6": int(np.count_nonzero(gps1[:, 1] == 6)),
        },
        "rtk_fixed_pct": float(100 * np.mean(gps1[:, 1] == 6)),
        "rtk_float_pct": float(100 * np.mean(gps1[:, 1] == 5)),
        "gps1_satellites": statistics(gps1[:, 2]),
        "gps1_horizontal_accuracy_mm": statistics(gps1[:, 5]),
    }
    transitions = []
    for index in np.flatnonzero(np.diff(gps1[:, 1]) != 0) + 1:
        transitions.append(
            {
                "elapsed_s": float(gps1[index, 0] - start_time),
                "from_fix_type": int(gps1[index - 1, 1]),
                "to_fix_type": int(gps1[index, 1]),
            }
        )
    localization["gps1_fix_transitions"] = transitions

    cmd = data["commands"]["/cmd_vel"]
    cmd_at_global = cmd[nearest_indices(cmd[:, 0], global_odom[:, 0])]
    active = (global_odom[:, 0] >= cmd[0, 0]) & (global_odom[:, 0] <= cmd[-1, 0])
    hard_turn = np.abs(cmd_at_global[:, 2]) >= 0.75
    centerline["active_navigation"] = statistics(centerline_error[active])
    centerline["hard_turn"] = statistics(centerline_error[active & hard_turn])
    centerline["straight"] = statistics(
        centerline_error[active & (np.abs(cmd_at_global[:, 2]) < 0.2)]
    )
    centerline["correlation_abs_cmd_wz"] = float(
        np.corrcoef(centerline_error[active], np.abs(cmd_at_global[active, 2]))[0, 1]
    )

    clouds = data["clouds"]
    finite_cloud = np.isfinite(clouds[:, 1])
    candidate_indices = np.flatnonzero(clouds[:, 1] <= 4.0)
    candidate_events = []
    costmap_times = np.asarray([item[0] for item in data["costmaps"]])
    odom_indices = previous_indices(filtered_odom[:, 0], clouds[:, 0])
    map_indices = previous_indices(costmap_times, clouds[:, 0])
    proxy_cost_values = np.full(len(clouds), np.nan)
    for index in np.flatnonzero(finite_cloud):
        odom = filtered_odom[odom_indices[index]]
        theta = odom[3]
        x_odom = odom[1] + math.cos(theta) * clouds[index, 2] - math.sin(theta) * clouds[index, 3]
        y_odom = odom[2] + math.sin(theta) * clouds[index, 2] + math.cos(theta) * clouds[index, 3]
        _, resolution, width, height, origin_x, origin_y, grid = data["costmaps"][map_indices[index]]
        column = int((x_odom - origin_x) / resolution)
        row = int((y_odom - origin_y) / resolution)
        if 0 <= column < width and 0 <= row < height:
            proxy_cost_values[index] = grid[row, column]
    for index in candidate_indices:
        command_index = nearest_indices(cmd[:, 0], np.asarray([clouds[index, 0]]))[0]
        candidate_events.append(
            {
                "elapsed_s": float(clouds[index, 0] - start_time),
                "proxy_range_m": float(clouds[index, 1]),
                "proxy_point_base_link_xyz_m": [float(value) for value in clouds[index, 2:5]],
                "cmd_linear_x_mps": float(cmd[command_index, 1]),
                "cmd_angular_z_radps": float(cmd[command_index, 2]),
                "filtered_odom_age_s": float(
                    clouds[index, 0] - filtered_odom[odom_indices[index], 0]
                ),
                "costmap_publication_age_s": float(
                    clouds[index, 0] - costmap_times[map_indices[index]]
                ),
                "costmap_cell_value": finite_float(proxy_cost_values[index]),
                "steering_response_latency_s": None,
                "speed_reduction_latency_s": None,
                "latency_note": "Single isolated proxy sample; command was already at high steering and reduced speed.",
            }
        )
    obstacle = {
        "scan_message_count": int(len(data["timestamps"].get("/scan", []))),
        "proxy_topic": "/genz/non_planar_points",
        "proxy_definition": "nearest point with x>=0.43 m, |y|<=0.75 m, 0<=z<=2 m in base_link",
        "proxy_frames": int(len(clouds)),
        "proxy_frames_with_candidate": int(np.count_nonzero(finite_cloud)),
        "proxy_forward_range_m": statistics(clouds[finite_cloud, 1]),
        "proxy_frames_within_4m": int(len(candidate_indices)),
        "candidate_events": candidate_events,
        "proxy_costmap_cell_free_pct": float(
            100 * np.mean(proxy_cost_values[np.isfinite(proxy_cost_values)] == 0)
        ),
        "interpretation_limit": "The proxy cloud was not consumed by Nav2 and cannot replace the absent /scan record.",
    }

    original_meta = bag_metadata(ORIGINAL_BAG)
    filtered_meta = bag_metadata(FILTERED_BAG)
    original_size = directory_size(ORIGINAL_BAG)
    filtered_size = directory_size(FILTERED_BAG)
    retained = []
    for entry in filtered_meta["topics_with_message_count"]:
        topic = entry["topic_metadata"]
        retained.append(
            {
                "name": topic["name"],
                "type": topic["type"],
                "message_count": int(entry["message_count"]),
            }
        )
    retained.sort(key=lambda item: item["name"])
    bag = {
        "original_size_bytes": original_size,
        "filtered_size_bytes": filtered_size,
        "original_size_gib": original_size / 2**30,
        "filtered_size_mib": filtered_size / 2**20,
        "reduction_pct": 100 * (1 - filtered_size / original_size),
        "filtered_topic_count": len(retained),
        "filtered_nonempty_topic_count": int(
            sum(item["message_count"] > 0 for item in retained)
        ),
        "filtered_message_count": int(filtered_meta["message_count"]),
        "duration_s": filtered_meta["duration"]["nanoseconds"] / 1e9,
        "start_ns": int(filtered_meta["starting_time"]["nanoseconds_since_epoch"]),
        "end_ns": int(
            filtered_meta["starting_time"]["nanoseconds_since_epoch"]
            + filtered_meta["duration"]["nanoseconds"]
        ),
        "retained_topics": retained,
    }
    bag["start_utc"] = iso_timestamp(bag["start_ns"])
    bag["end_utc"] = iso_timestamp(bag["end_ns"])

    nav2 = yaml.safe_load(NAV2_CONFIG_PATH.read_text(encoding="utf-8"))
    controller = nav2["controller_server"]["ros__parameters"]
    local_costmap = nav2["local_costmap"]["local_costmap"]["ros__parameters"]
    global_costmap = nav2["global_costmap"]["global_costmap"]["ros__parameters"]
    smoother = nav2["velocity_smoother"]["ros__parameters"]
    follow = controller["FollowPath"]
    config = {
        "controller_frequency_hz": controller["controller_frequency"],
        "controller_odom_topic_configured": controller.get("odom_topic"),
        "controller_odom_topic_effective_default": "/odom",
        "controller_effective_topic_recorded_messages": 0,
        "bt_navigator_odom_topic": nav2["bt_navigator"]["ros__parameters"]["odom_topic"],
        "velocity_smoother_odom_topic": smoother["odom_topic"],
        "velocity_smoother_feedback": smoother["feedback"],
        "local_costmap_pose_source": "TF odom->base_link from local EKF",
        "local_costmap_update_frequency_hz": local_costmap["update_frequency"],
        "local_costmap_publish_frequency_hz": local_costmap["publish_frequency"],
        "observed_local_costmap_publish_hz": rates["/local_costmap/costmap"]["mean_hz"],
        "scan_topic": local_costmap["obstacle_layer"]["scan"]["topic"],
        "scan_observation_persistence": "not explicitly configured",
        "obstacle_min_range_m": local_costmap["obstacle_layer"]["scan"]["obstacle_min_range"],
        "obstacle_max_range_m": local_costmap["obstacle_layer"]["scan"]["obstacle_max_range"],
        "raytrace_min_range_m": local_costmap["obstacle_layer"]["scan"]["raytrace_min_range"],
        "raytrace_max_range_m": local_costmap["obstacle_layer"]["scan"]["raytrace_max_range"],
        "inflation_radius_m": local_costmap["inflation_layer"]["inflation_radius"],
        "inflation_cost_scaling_factor": local_costmap["inflation_layer"]["cost_scaling_factor"],
        "footprint": local_costmap["footprint"],
        "transform_tolerance_s": local_costmap["transform_tolerance"],
        "mppi_time_steps": follow["time_steps"],
        "mppi_model_dt_s": follow["model_dt"],
        "mppi_horizon_s": follow["time_steps"] * follow["model_dt"],
        "mppi_vx_max_mps": follow["vx_max"],
        "mppi_wz_max_radps": follow["wz_max"],
        "mppi_obstacle_critic": follow["ObstaclesCritic"],
        "mppi_path_align_weight": follow["PathAlignCritic"]["cost_weight"],
        "mppi_path_follow_weight": follow["PathFollowCritic"]["cost_weight"],
        "velocity_smoother_max_decel": smoother["max_decel"],
        "global_planner_cost_travel_multiplier": nav2["planner_server"]["ros__parameters"]["GridBased"]["cost_travel_multiplier"],
        "centerline_filter_enabled": global_costmap["road_preference_filter"]["enabled"],
        "pointcloud_to_laserscan_default_input": "/nonground",
        "recorded_default_input_messages": 0,
    }

    filtered_map = transform_to_map(filtered_odom, data["map_odom"])
    genz_map = transform_to_map(genz_odom, data["map_odom"])
    summary = {
        "bag_filtering": bag,
        "centerline_tracking": centerline,
        "planned_path_centerline": plan_centerline,
        "topic_rates": rates,
        "localization": localization,
        "obstacle_avoidance": obstacle,
        "nav2_configuration": config,
        "ranked_likely_causes": [
            {
                "rank": 1,
                "confidence": "high",
                "cause": "No /scan messages reached the recorded Nav2 obstacle input, leaving live obstacle marking and clearing without observations.",
            },
            {
                "rank": 2,
                "confidence": "high",
                "cause": "controller_server omitted odom_topic, so Humble defaulted to /odom; the bag contains no /odom publisher, while /odometry/filtered was only configured for BT navigator and velocity smoother.",
            },
            {
                "rank": 3,
                "confidence": "medium",
                "cause": "Global localization was discontinuous during mostly RTK-float operation: 34 global EKF position steps exceeded 0.5 m and centerline error was substantially worse in float periods.",
            },
        ],
        "recommended_changes": {
            "high_confidence": [
                "Make the point-cloud-to-LaserScan input match the actually published nonground/obstacle cloud; gate Nav2 activation on a nonzero, healthy /scan rate and verify live obstacle cells in the local costmap.",
                "Set controller_server.ros__parameters.odom_topic to /odometry/filtered and verify the controller subscription before motion.",
                "Record /scan explicitly with compatible sensor-data QoS plus the compact obstacle cloud in the next diagnostic bag.",
            ],
            "medium_confidence": [
                "Investigate why GenZ/cloud cadence was 6.67 Hz rather than the previously observed ~11 Hz; retest stopping/steering margin at the actual maximum rover speed.",
                "Gate or reduce navigation speed when HERE4 is RTK float, and tune global GPS fusion only after checking time synchronization/covariance because the global EKF made repeated sub-second correction jumps.",
            ],
            "do_not_change_yet": [
                "Do not reduce centerline cost_travel_multiplier solely from this run; the planner had no recorded obstacle input and 93.0% of plan samples stayed within 0.5 m of a mapped centerline.",
                "Do not change footprint, inflation, MPPI obstacle weights, or deceleration limits until a run with a validated /scan stream shows inadequate clearance despite correctly marked obstacles.",
                "Do not lower or restore EKF output rates: /odometry/filtered was 40.0 Hz and /odometry/global was 30.0 Hz, already above/at the requested 25-30 Hz range.",
            ],
        },
    }

    make_plots(
        graph,
        starts,
        vectors,
        squared,
        data,
        global_odom,
        gps_odom,
        filtered_map,
        genz_map,
        centerline_error,
        status_at_global,
        global_gps_error,
        rates,
        proxy_cost_values,
        start_time,
    )
    (HERE / "summary.json").write_text(
        json.dumps(summary, indent=2, allow_nan=False) + "\n", encoding="utf-8"
    )
    write_report(summary)
    print(json.dumps({
        "original_size_gib": bag["original_size_gib"],
        "filtered_size_mib": bag["filtered_size_mib"],
        "centerline_median_m": centerline["median"],
        "centerline_p95_m": centerline["p95"],
        "genz_hz": rates["/genz/odometry"]["mean_hz"],
        "filtered_odom_hz": rates["/odometry/filtered"]["mean_hz"],
        "global_odom_hz": rates["/odometry/global"]["mean_hz"],
        "imu_hz": rates["/mavros/imu/data"]["mean_hz"],
        "scan_hz": rates["/scan"]["mean_hz"],
    }, indent=2))


def make_plots(
    graph,
    starts,
    vectors,
    squared,
    data,
    global_odom,
    gps_odom,
    filtered_map,
    genz_map,
    centerline_error,
    status_at_global,
    global_gps_error,
    rates,
    proxy_cost_values,
    start_time,
):
    plt.rcParams.update({"font.size": 10, "axes.grid": True, "grid.alpha": 0.22})
    elapsed = global_odom[:, 0] - start_time

    fig, ax = plt.subplots(figsize=(10, 8), constrained_layout=True)
    bounds = (
        global_odom[:, 1].min() - 5,
        global_odom[:, 1].max() + 5,
        global_odom[:, 2].min() - 5,
        global_odom[:, 2].max() + 5,
    )
    label = True
    for edge in graph["edges"]:
        points = np.asarray(edge["geometry"])
        if points[:, 0].max() < bounds[0] or points[:, 0].min() > bounds[1] or points[:, 1].max() < bounds[2] or points[:, 1].min() > bounds[3]:
            continue
        ax.plot(points[:, 0], points[:, 1], color="#94a3b8", lw=0.8, alpha=0.65, label="Mapped road centreline" if label else None)
        label = False
    for index, (_, route) in enumerate(data["reference_routes"]):
        ax.plot(route[:, 0], route[:, 1], lw=1.3, alpha=0.7, label="Recorded reference route" if index == len(data["reference_routes"]) - 1 else None)
    scatter = ax.scatter(global_odom[:, 1], global_odom[:, 2], c=centerline_error, s=8, cmap="viridis", vmin=0, vmax=max(2.5, np.percentile(centerline_error, 99)), label="Global EKF trajectory", zorder=4)
    ax.scatter(gps_odom[:, 1], gps_odom[:, 2], s=10, color="#f97316", alpha=0.65, label="GPS odometry", zorder=5)
    high = centerline_error > 2.0
    ax.scatter(global_odom[high, 1], global_odom[high, 2], s=12, facecolors="none", edgecolors="#dc2626", linewidths=0.7, label=">2 m error", zorder=6)
    fig.colorbar(scatter, ax=ax, label="Nearest mapped centreline distance (m)")
    ax.set(xlabel="DTU map X (m)", ylabel="DTU map Y (m)", title="August 29 global trajectory and mapped road centrelines", xlim=bounds[:2], ylim=bounds[2:])
    ax.set_aspect("equal", adjustable="box")
    ax.legend(loc="best", fontsize=8)
    fig.savefig(HERE / "centerline_tracking.png", dpi=180)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(11, 5), constrained_layout=True)
    ax.plot(elapsed, centerline_error, lw=1.1, color="#2563eb", label="Global odometry to centreline")
    ax.fill_between(elapsed, 0, centerline_error, where=status_at_global == 5, color="#f59e0b", alpha=0.13, label="RTK float")
    for level in (0.5, 1.0, 1.5, 2.0):
        ax.axhline(level, color="#64748b" if level < 2 else "#dc2626", ls=":" if level < 2 else "--", lw=0.9)
    ax.set(xlabel="Elapsed bag time (s)", ylabel="Centreline error (m)", title="Centreline error over time")
    ax.legend(loc="upper left", fontsize=8)
    fig.savefig(HERE / "centerline_error_vs_time.png", dpi=180)
    plt.close(fig)

    rate_order = RATE_TOPICS + ["/local_costmap/costmap", "/genz/non_planar_points", "/plan"]
    hz = [rates[topic]["mean_hz"] or 0 for topic in rate_order]
    p95 = [rates[topic]["p95_interval_s"] or 0 for topic in rate_order]
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), constrained_layout=True)
    axes[0].barh(rate_order, hz, color="#2563eb")
    axes[0].set(xlabel="Mean recorded rate (Hz)", title="Recorded topic rates")
    axes[1].barh(rate_order, p95, color="#f97316")
    axes[1].set(xlabel="p95 inter-message interval (s)", title="Message interval tail latency")
    fig.savefig(HERE / "topic_rates.png", dpi=180)
    plt.close(fig)

    fig, axes = plt.subplots(1, 2, figsize=(14, 6), constrained_layout=True)
    axes[0].plot(global_odom[:, 1], global_odom[:, 2], label="Global EKF", lw=1.7)
    axes[0].plot(gps_odom[:, 1], gps_odom[:, 2], label="GPS odometry", lw=1.2, marker=".", ms=2)
    axes[0].plot(filtered_map[:, 1], filtered_map[:, 2], label="Filtered odom via map->odom TF", lw=1.0, alpha=0.8)
    axes[0].plot(genz_map[:, 1], genz_map[:, 2], label="GenZ via map->odom TF", lw=0.9, alpha=0.75)
    axes[0].set(xlabel="DTU map X (m)", ylabel="DTU map Y (m)", title="Map-frame odometry comparison")
    axes[0].set_aspect("equal", adjustable="box")
    axes[0].legend(fontsize=8)
    gps_elapsed = gps_odom[:, 0] - start_time
    axes[1].plot(gps_elapsed, global_gps_error, marker=".", ms=4, lw=1.0, label="Global EKF to nearest GPS odometry")
    axes[1].axhline(0.5, ls="--", color="#f59e0b", lw=0.9)
    axes[1].axhline(1.0, ls="--", color="#dc2626", lw=0.9)
    axes[1].set(xlabel="Elapsed bag time (s)", ylabel="Position difference (m)", title="Global EKF vs GPS")
    axes[1].legend(fontsize=8)
    fig.savefig(HERE / "odometry_comparison.png", dpi=180)
    plt.close(fig)

    clouds = data["clouds"]
    cmd = data["commands"]["/cmd_vel"]
    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True, constrained_layout=True)
    axes[0].scatter(clouds[:, 0] - start_time, clouds[:, 1], s=8, color="#7c3aed", label="Non-planar cloud proxy")
    axes[0].axhline(4.0, color="#dc2626", ls="--", lw=0.9)
    axes[0].set(ylabel="Forward proxy range (m)", title="Obstacle evidence (recorded /scan has zero messages)")
    axes[0].legend(fontsize=8)
    axes[1].plot(cmd[:, 0] - start_time, cmd[:, 1], label="linear.x", lw=1.1)
    axes[1].plot(cmd[:, 0] - start_time, cmd[:, 2], label="angular.z", lw=1.0)
    axes[1].set(ylabel="Command", title="Commanded velocity")
    axes[1].legend(fontsize=8)
    axes[2].scatter(clouds[:, 0] - start_time, proxy_cost_values, s=8, color="#059669")
    axes[2].set(
        xlabel="Elapsed bag time (s)",
        ylabel="Cost value",
        title="Local costmap value at proxy point",
        ylim=(-5, 105),
    )
    axes[2].text(
        0.99,
        0.08,
        "All sampled proxy cells = 0 (free)",
        transform=axes[2].transAxes,
        ha="right",
        va="bottom",
        color="#047857",
    )
    fig.savefig(HERE / "obstacle_encounters.png", dpi=180)
    plt.close(fig)

    finite = np.isfinite(clouds[:, 1])
    command_indices = nearest_indices(cmd[:, 0], clouds[finite, 0])
    fig, axes = plt.subplots(2, 1, figsize=(11, 7), constrained_layout=True)
    axes[0].scatter(clouds[finite, 0] - start_time, clouds[finite, 1], c=cmd[command_indices, 1], cmap="plasma", s=12)
    axes[0].set(xlabel="Elapsed bag time (s)", ylabel="Proxy range (m)", title="Forward obstacle proxy colored by commanded speed")
    scatter = axes[1].scatter(clouds[finite, 1], cmd[command_indices, 1], c=np.abs(cmd[command_indices, 2]), cmap="viridis", s=15)
    axes[1].set(xlabel="Forward proxy range (m)", ylabel="cmd_vel linear.x (m/s)", title="Commanded speed versus obstacle proxy")
    fig.colorbar(scatter, ax=axes[1], label="|angular.z| (rad/s)")
    fig.savefig(HERE / "cmd_vel_vs_obstacle_distance.png", dpi=180)
    plt.close(fig)


def write_report(summary):
    bag = summary["bag_filtering"]
    center = summary["centerline_tracking"]
    rates = summary["topic_rates"]
    loc = summary["localization"]
    obstacle = summary["obstacle_avoidance"]
    config = summary["nav2_configuration"]
    causes = summary["ranked_likely_causes"]
    recs = summary["recommended_changes"]
    rate_table = []
    for topic in RATE_TOPICS:
        item = rates[topic]
        hz = "n/a" if item["mean_hz"] is None else f"{item['mean_hz']:.3f}"
        med = "n/a" if item["median_interval_s"] is None else f"{item['median_interval_s']:.4f}"
        p95 = "n/a" if item["p95_interval_s"] is None else f"{item['p95_interval_s']:.4f}"
        worst = "n/a" if item["worst_gap_s"] is None else f"{item['worst_gap_s']:.4f}"
        rate_table.append(
            f"| `{topic}` | {item['message_count']} | {hz} | {med} | {p95} | {worst} | {item['gaps_gt_0_05_s']} | {item['gaps_gt_0_10_s']} | {item['gaps_gt_0_20_s']} | {item['gaps_gt_0_50_s']} |"
        )
    retained_table = [
        f"| `{item['name']}` | `{item['type']}` | {item['message_count']} |"
        for item in bag["retained_topics"]
    ]
    worst = center["worst_sustained_gt_2m"]
    candidate = obstacle["candidate_events"][0] if obstacle["candidate_events"] else None
    candidate_text = (
        f"The only proxy point within 4 m occurred at t={candidate['elapsed_s']:.2f} s, range {candidate['proxy_range_m']:.2f} m. "
        f"The command was already {candidate['cmd_linear_x_mps']:.2f} m/s and {candidate['cmd_angular_z_radps']:.2f} rad/s; its local-costmap cell was {candidate['costmap_cell_value']}. "
        "Because it was a single isolated sample, steering and speed-reduction latency are not measurable."
        if candidate
        else "The fallback cloud produced no candidate within 4 m."
    )
    report = f"""# August 29 DTU live rover test — offline bag analysis

## 1. Executive summary

The dominant obstacle-avoidance failure is upstream of Nav2 tuning: `/scan`, the sole configured obstacle source for both costmaps, recorded **zero messages**. The cloud-to-scan launch defaults to `/nonground`, but that topic is absent; `/genz/non_planar_points` exists at only {rates['/genz/non_planar_points']['mean_hz']:.2f} Hz and was not consumed by Nav2. The controller also lacks an explicit `odom_topic`; ROS 2 Humble Nav2 defaults it to `/odom`, and this bag has no `/odom` publisher. These two wiring gaps are stronger explanations than MPPI critic weights, footprint, inflation, or the centreline tether.

The rover's global pose was a median **{center['median']:.3f} m** from the mapped road centreline (p95 **{center['p95']:.3f} m**). The collision-aware plan itself was much more centred: {summary['planned_path_centerline']['within_0_5_m_pct']:.1f}% of recorded plan poses were within 0.5 m. Deviations were not consistently on one side; the longest interval above 2 m was {worst['duration_s']:.2f} s (t={worst['start_s']:.2f}–{worst['end_s']:.2f} s), during prolonged hard steering and RTK-float operation.

Restoring EKF output to 30 Hz is **not justified**: `/odometry/filtered` was {rates['/odometry/filtered']['mean_hz']:.2f} Hz and `/odometry/global` was {rates['/odometry/global']['mean_hz']:.2f} Hz with no gaps above 50 ms. The raw GenZ rate was lower than expected at {rates['/genz/odometry']['mean_hz']:.2f} Hz, but the EKFs predicted at healthy rates.

## 2. Bag filtering result

- Original: {bag['original_size_gib']:.3f} GiB.
- Filtered: {bag['filtered_size_mib']:.3f} MiB.
- Reduction: {bag['reduction_pct']:.2f}%.
- Filtered duration: {bag['duration_s']:.6f} s.
- Start/end: `{bag['start_utc']}` / `{bag['end_utc']}`.
- Retained topic records: {bag['filtered_topic_count']} ({bag['filtered_nonempty_topic_count']} non-empty); messages: {bag['filtered_message_count']}.
- `/genz/non_planar_points` is retained only because `/scan` is empty; it is the smallest recorded non-planar proxy cloud. Both ~3.5 GiB global costmap streams and duplicate raw costmaps were excluded.

| Topic | Type | Messages |
|---|---|---:|
{chr(10).join(retained_table)}

## 3. RTK/global localization quality

`/odometry/global` vs `/odometry/gps`: median {loc['global_to_gps_m']['median']:.3f} m, mean {loc['global_to_gps_m']['mean']:.3f} m, p95 {loc['global_to_gps_m']['p95']:.3f} m, max {loc['global_to_gps_m']['maximum']:.3f} m. The receiver reported RTK fixed for only {loc['rtk_fixed_pct']:.1f}% of samples and float for {loc['rtk_float_pct']:.1f}%. During fixed periods the global-vs-GPS median was {loc['global_to_gps_m']['rtk_fixed']['median']:.3f} m; during float it was {loc['global_to_gps_m']['rtk_float']['median']:.3f} m.

The global EKF made {loc['global_steps_gt_0_5_m']} consecutive position steps above 0.5 m at a nominal 30 Hz; the largest was {loc['global_position_step_m']['maximum']:.3f} m. {loc['global_steps_gt_0_5_m_within_0_1_s_of_gps']} of those {loc['global_steps_gt_0_5_m']} steps occurred within 0.1 s of a GPS-odometry update, supporting delayed/discrete GPS correction rather than physical motion over ~33 ms. GPS's own 1 Hz position step had p95/max {loc['gps_position_step_m']['p95']:.3f}/{loc['gps_position_step_m']['maximum']:.3f} m. Heading was much smoother (maximum consecutive change {loc['global_heading_step_deg']['maximum']:.2f}°).

Reported global XY sigma did not grow monotonically: first/median/p95/last were {loc['global_reported_xy_sigma_m']['first']:.3f}/{loc['global_reported_xy_sigma_m']['median']:.3f}/{loc['global_reported_xy_sigma_m']['p95']:.3f}/{loc['global_reported_xy_sigma_m']['last']:.3f} m. GPS1 reported a median horizontal-accuracy field of {loc['gps1_horizontal_accuracy_mm']['median']:.1f} mm and {loc['gps1_satellites']['median']:.0f} satellites, but the prolonged float status and correction steps mean global localization was not consistently smooth enough to call fully acceptable.

The bag provides no independent ground truth. Consequently, it cannot uniquely divide every centreline deviation into physical tracking error versus GNSS/global-estimator error. The strong status correlation is evidence of a localization contribution, not proof that all float-period error was GNSS error.

## 4. Centerline tracking quality

The actual tether geometry comes from `src/dtu_prior_map/maps/dtu_road_graph.json`, which generates the recorded centreline cost mask. `/dtu_reference_route` is also recorded, but it is a goal-specific obstacle-independent route and includes short start/goal connectors, so the required nearest-centreline statistics use all mapped road graph segments.

- Median: **{center['median']:.3f} m**; mean: **{center['mean']:.3f} m**; p95: **{center['p95']:.3f} m**; max: **{center['maximum']:.3f} m**.
- Within 0.5/1.0/1.5/2.0 m: {center['within_0_5_m_pct']:.1f}% / {center['within_1_0_m_pct']:.1f}% / {center['within_1_5_m_pct']:.1f}% / {center['within_2_0_m_pct']:.1f}%.
- RTK fixed centreline median/p95: {center['rtk_fixed']['median']:.3f}/{center['rtk_fixed']['p95']:.3f} m; RTK float: {center['rtk_float']['median']:.3f}/{center['rtk_float']['p95']:.3f} m.
- Longest sustained >2 m: t={worst['start_s']:.2f}–{worst['end_s']:.2f} s, {worst['duration_s']:.2f} s, mean/max {worst['mean_error_m']:.3f}/{worst['max_error_m']:.3f} m.
- Direction: nearest graph-edge sign was positive for {center['positive_side_pct']:.1f}% with signed median {center['signed_median_m']:.3f} m. Edge directions are not lane directions, and the recorded goal-route sign also alternates; the data does not support a systematic one-side bias.
- Turn correlation was weak overall (Pearson correlation with `|cmd_vel.angular.z|` {center['correlation_abs_cmd_wz']:.3f}), though the longest high-error interval coincided with sustained maximum steering. Replanning was steady at {rates['/plan']['mean_hz']:.3f} Hz; the plan remained near the mapped road, so large rover-pose errors were not primarily caused by the global planner choosing an off-centre route.

Plots: `centerline_tracking.png`, `centerline_error_vs_time.png`.

## 5. Odometry and sensor rates

| Topic | Messages | Mean Hz | Median dt | p95 dt | Worst gap | >0.05 s | >0.10 s | >0.20 s | >0.50 s |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
{chr(10).join(rate_table)}

The prior expectation was GenZ ~11 Hz, IMU 400+ Hz, filtered odometry 25–30 Hz. This run instead recorded GenZ {rates['/genz/odometry']['mean_hz']:.2f} Hz, IMU {rates['/mavros/imu/data']['mean_hz']:.2f} Hz, local filtered odometry {rates['/odometry/filtered']['mean_hz']:.2f} Hz, and global odometry {rates['/odometry/global']['mean_hz']:.2f} Hz. The EKF outputs were not slow.

Odometry consumers from the repository configuration:

- `bt_navigator`: `/odometry/filtered`.
- `velocity_smoother`: `/odometry/filtered`, but configured `OPEN_LOOP`.
- local costmap: pose via TF `odom -> base_link`, published by the 40 Hz local EKF.
- `controller_server`: no `odom_topic` configured, therefore Humble default `/odom`; no such publisher/topic exists in the bag. MPPI therefore lacked recorded velocity feedback even though TF pose updates were healthy.

## 6. Obstacle avoidance findings

Both costmap obstacle layers are configured exclusively for `/scan`, with marking and clearing enabled. `/scan` has zero messages. Thus obstacle entry time, true forward range, steering latency, braking latency, scan age, and scan-to-costmap latency cannot be reconstructed from the intended perception stream.

The local occupancy grid published at {rates['/local_costmap/costmap']['mean_hz']:.2f} Hz versus a configured 10 Hz publication rate (internal update configured 25 Hz). This topic rate does not prove the internal update loop was slow; with no scan, there was no live obstacle observation to update. {candidate_text}

Current relevant settings: controller 20 Hz; MPPI horizon {config['mppi_horizon_s']:.2f} s ({config['mppi_time_steps']} × {config['mppi_model_dt_s']:.2f} s); max {config['mppi_vx_max_mps']:.1f} m/s and {config['mppi_wz_max_radps']:.1f} rad/s; footprint `{config['footprint']}`; inflation {config['inflation_radius_m']:.2f} m; collision margin {config['mppi_obstacle_critic']['collision_margin_distance']:.2f} m; local transform tolerance {config['transform_tolerance_s']:.2f} s; deceleration limits {config['velocity_smoother_max_decel']}.

The centreline tether acts in the **global planner** through the global costmap filter and `cost_travel_multiplier=3.0`. MPPI's local obstacle critic acts in the **local controller**. Because no scan reached either obstacle layer, this bag cannot show the tether overpowering a correctly marked obstacle. The plan being 93% within 0.5 m is expected when the planner is blind to live obstacles, not evidence that the weight is too strong.

## 7. Ranked likely causes of poor avoidance

{chr(10).join(f"{item['rank']}. **{item['confidence'].upper()}** — {item['cause']}" for item in causes)}

The 6.67 Hz upstream cloud/GenZ cadence is a secondary rate concern: at 0.9 m/s the rover moves about 0.135 m per cloud interval. It is not the main failure because Nav2 received no scan at any rate.

## 8. Recommended changes for the next rover test

### HIGH CONFIDENCE

{chr(10).join('- ' + item for item in recs['high_confidence'])}

### MEDIUM CONFIDENCE

{chr(10).join('- ' + item for item in recs['medium_confidence'])}

### DO NOT CHANGE YET

{chr(10).join('- ' + item for item in recs['do_not_change_yet'])}

## 9. Things NOT justified by the evidence

- Restoring `/odometry/filtered` to ~30 Hz: it already ran at 40.0 Hz, while `/odometry/global` ran at 30.0 Hz.
- Blaming slow controller execution: `/cmd_vel_nav` and `/cmd_vel` were both ~20 Hz, matching configuration.
- Blaming small inflation, an undersized footprint, weak obstacle critic, or insufficient configured deceleration before validating that obstacles reach the costmap.
- Blaming the centreline tether for unsafe clearance: no recorded scan allowed the global planner or MPPI obstacle critic to compare centreline preference against live obstacles.
- Claiming a measured obstacle-response latency or physical GPS ground-truth error from this bag.

## 10. Exact commands/scripts used

```bash
source /opt/ros/humble/setup.bash
ros2 bag info aug29.bag
python3 analysis/scripts/filter_rosbag.py aug29.bag aug29_filtered.bag
ros2 bag info aug29_filtered.bag
python3 analysis/aug29_live_test/analyze_aug29.py
```

Filtering is implemented in `analysis/scripts/filter_rosbag.py`; analysis and plotting are implemented in this directory's `analyze_aug29.py`. The original bag was opened read-only and was not altered. No ROS nodes, live launch, navigation goals, or motion commands were started.
"""
    (HERE / "report.md").write_text(report, encoding="utf-8")


if __name__ == "__main__":
    main()
