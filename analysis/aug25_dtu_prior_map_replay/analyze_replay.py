#!/usr/bin/env python3
"""Analyze the August 25 sensor bag through the August 27 DTU RTK pipeline."""

import csv
import json
import math
import sqlite3
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
from pyproj import Transformer
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
RAW_BAG = Path("/home/aryaman/Desktop/aug25.bag/aug25.bag_0.db3")
OUTPUT_BAG = HERE / "current_pipeline_output_1x/current_pipeline_output_1x_0.db3"
MAPS = REPO / "src/dtu_prior_map/maps"
ORIGIN_E = 706231.750
ORIGIN_N = 3181578.500
RESOLUTION = 0.25


def read_topic(database: Path, topic: str):
    connection = sqlite3.connect(database)
    row = connection.execute(
        "SELECT id, type FROM topics WHERE name = ?", (topic,)
    ).fetchone()
    if row is None:
        raise RuntimeError(f"Topic {topic} is absent from {database}")
    topic_id, type_name = row
    message_type = get_message(type_name)
    messages = [
        deserialize_message(data, message_type)
        for (data,) in connection.execute(
            "SELECT data FROM messages WHERE topic_id = ? ORDER BY timestamp",
            (topic_id,),
        )
    ]
    connection.close()
    return messages


def stamp_seconds(message) -> float:
    return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9


def road_segments(graph):
    segments = []
    for edge in graph["edges"]:
        points = edge["geometry"]
        segments.extend(zip(points, points[1:]))
    starts = np.asarray([segment[0] for segment in segments], dtype=float)
    ends = np.asarray([segment[1] for segment in segments], dtype=float)
    vectors = ends - starts
    squared_lengths = np.sum(vectors * vectors, axis=1)
    return starts, vectors, squared_lengths


def centreline_distances(points, starts, vectors, squared_lengths):
    distances = []
    for point in points:
        fractions = np.clip(
            np.sum((point - starts) * vectors, axis=1) / squared_lengths,
            0.0,
            1.0,
        )
        projections = starts + fractions[:, None] * vectors
        distances.append(np.min(np.linalg.norm(point - projections, axis=1)))
    return np.asarray(distances)


def percentiles(values):
    return {
        "minimum": float(np.min(values)),
        "mean": float(np.mean(values)),
        "median": float(np.median(values)),
        "p95": float(np.quantile(values, 0.95)),
        "maximum": float(np.max(values)),
        "rms": float(np.sqrt(np.mean(values * values))),
    }


def map_values(points, image):
    height, width = image.shape
    columns = np.floor(points[:, 0] / RESOLUTION).astype(int)
    rows = height - 1 - np.floor(points[:, 1] / RESOLUTION).astype(int)
    inside = (
        (columns >= 0) & (columns < width) & (rows >= 0) & (rows < height)
    )
    values = np.full(len(points), -1, dtype=int)
    values[inside] = image[rows[inside], columns[inside]]
    return values


def add_map_background(axis, static_image, bounds):
    x_min, x_max, y_min, y_max = bounds
    height, width = static_image.shape
    col0 = max(0, int(math.floor(x_min / RESOLUTION)))
    col1 = min(width, int(math.ceil(x_max / RESOLUTION)))
    bottom_row = max(0, int(math.floor(y_min / RESOLUTION)))
    top_row = min(height, int(math.ceil(y_max / RESOLUTION)))
    row0 = height - top_row
    row1 = height - bottom_row
    crop = static_image[row0:row1, col0:col1]
    display = np.where(crop >= 65, 0.25, 0.97)
    axis.imshow(
        display,
        cmap="gray",
        vmin=0.0,
        vmax=1.0,
        extent=(
            col0 * RESOLUTION,
            col1 * RESOLUTION,
            bottom_row * RESOLUTION,
            top_row * RESOLUTION,
        ),
        origin="upper",
        interpolation="nearest",
        zorder=0,
    )


def add_centrelines(axis, graph, bounds):
    x_min, x_max, y_min, y_max = bounds
    label_used = False
    for edge in graph["edges"]:
        geometry = np.asarray(edge["geometry"])
        if (
            np.max(geometry[:, 0]) < x_min
            or np.min(geometry[:, 0]) > x_max
            or np.max(geometry[:, 1]) < y_min
            or np.min(geometry[:, 1]) > y_max
        ):
            continue
        axis.plot(
            geometry[:, 0],
            geometry[:, 1],
            color="#111827",
            linewidth=0.9,
            alpha=0.7,
            label="Mapped road centreline" if not label_used else None,
            zorder=2,
        )
        label_used = True


def main():
    graph = json.loads((MAPS / "dtu_road_graph.json").read_text())
    starts, vectors, squared_lengths = road_segments(graph)

    transformer = Transformer.from_crs(4326, 32643, always_xy=True)
    raw_messages = read_topic(RAW_BAG, "/mavros/global_position/raw/fix")
    raw_rows = []
    for message in raw_messages:
        easting, northing = transformer.transform(
            message.longitude, message.latitude
        )
        raw_rows.append(
            (
                stamp_seconds(message),
                easting - ORIGIN_E,
                northing - ORIGIN_N,
                message.position_covariance[0],
                message.position_covariance[4],
            )
        )
    raw = np.asarray(raw_rows)

    global_messages = read_topic(OUTPUT_BAG, "/odometry/global")
    global_track = np.asarray(
        [
            (
                stamp_seconds(message),
                message.pose.pose.position.x,
                message.pose.pose.position.y,
            )
            for message in global_messages
        ]
    )
    navsat_messages = read_topic(OUTPUT_BAG, "/odometry/gps")
    navsat_track = np.asarray(
        [
            (
                stamp_seconds(message),
                message.pose.pose.position.x,
                message.pose.pose.position.y,
            )
            for message in navsat_messages
        ]
    )

    direct_centreline = centreline_distances(
        raw[:, 1:3], starts, vectors, squared_lengths
    )
    global_centreline = centreline_distances(
        global_track[:, 1:3], starts, vectors, squared_lengths
    )
    navsat_centreline = centreline_distances(
        navsat_track[:, 1:3], starts, vectors, squared_lengths
    )

    interpolated_direct = np.column_stack(
        (
            np.interp(global_track[:, 0], raw[:, 0], raw[:, 1]),
            np.interp(global_track[:, 0], raw[:, 0], raw[:, 2]),
        )
    )
    global_gps_error = np.linalg.norm(
        global_track[:, 1:3] - interpolated_direct, axis=1
    )
    navsat_gps_error = np.linalg.norm(
        navsat_track[:, 1:3] - raw[:, 1:3], axis=1
    )

    dot = np.sum(raw[:, 1] * navsat_track[:, 1] + raw[:, 2] * navsat_track[:, 2])
    cross = np.sum(
        raw[:, 1] * navsat_track[:, 2] - raw[:, 2] * navsat_track[:, 1]
    )
    navsat_rotation = math.atan2(cross, dot)
    rotation = np.asarray(
        [
            [math.cos(navsat_rotation), -math.sin(navsat_rotation)],
            [math.sin(navsat_rotation), math.cos(navsat_rotation)],
        ]
    )
    rotation_residual = np.linalg.norm(
        raw[:, 1:3] @ rotation.T - navsat_track[:, 1:3], axis=1
    )

    static_image = np.asarray(Image.open(MAPS / "dtu_static.pgm"))
    centre_cost_image = np.asarray(Image.open(MAPS / "dtu_centerline_cost.pgm"))
    direct_static_values = map_values(raw[:, 1:3], static_image)
    direct_centre_cost = map_values(raw[:, 1:3], centre_cost_image)
    global_static_values = map_values(global_track[:, 1:3], static_image)
    global_occupied = global_static_values >= 65

    end_time = raw[-1, 0]
    last_60 = (global_track[:, 0] >= end_time - 60.0) & (
        global_track[:, 0] <= end_time
    )
    last_10 = (global_track[:, 0] >= end_time - 10.0) & (
        global_track[:, 0] <= end_time
    )

    summary = {
        "bag_duration_seconds": float(raw[-1, 0] - raw[0, 0]),
        "sample_counts": {
            "raw_gps": len(raw),
            "navsat_odometry_gps": len(navsat_track),
            "global_ekf": len(global_track),
        },
        "direct_utm_gps_to_centreline_m": percentiles(direct_centreline),
        "direct_utm_gps_static_free_samples": int(
            np.count_nonzero(direct_static_values == 0)
        ),
        "direct_utm_gps_offroad_cost_samples": int(
            np.count_nonzero(direct_centre_cost == 70)
        ),
        "global_ekf_static_occupied_samples": int(np.count_nonzero(global_occupied)),
        "global_ekf_last_occupied_elapsed_s": float(
            np.max(global_track[global_occupied, 0]) - global_track[0, 0]
        ),
        "navsat_to_direct_utm_m": percentiles(navsat_gps_error),
        "navsat_to_centreline_m": percentiles(navsat_centreline),
        "navsat_exact_rotation_deg": math.degrees(navsat_rotation),
        "navsat_rotation_fit_max_residual_m": float(np.max(rotation_residual)),
        "global_ekf_to_direct_utm_last_60_s_m": percentiles(
            global_gps_error[last_60]
        ),
        "global_ekf_to_centreline_last_60_s_m": percentiles(
            global_centreline[last_60]
        ),
        "global_ekf_to_direct_utm_last_10_s_m": percentiles(
            global_gps_error[last_10]
        ),
        "global_ekf_to_centreline_last_10_s_m": percentiles(
            global_centreline[last_10]
        ),
        "initial": {
            "direct_utm_xy_m": raw[0, 1:3].tolist(),
            "global_ekf_xy_m": global_track[0, 1:3].tolist(),
            "separation_m": float(global_gps_error[0]),
        },
        "final": {
            "direct_utm_xy_m": raw[-1, 1:3].tolist(),
            "global_ekf_xy_m": global_track[-1, 1:3].tolist(),
            "global_to_direct_utm_m": float(global_gps_error[-1]),
            "global_to_centreline_m": float(global_centreline[-1]),
        },
    }
    (HERE / "summary.json").write_text(
        json.dumps(summary, indent=2) + "\n", encoding="utf-8"
    )

    with (HERE / "gps_samples.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            [
                "stamp_s",
                "direct_utm_x_m",
                "direct_utm_y_m",
                "navsat_x_m",
                "navsat_y_m",
                "navsat_to_direct_m",
                "direct_to_centreline_m",
                "navsat_to_centreline_m",
                "gps_variance_x_m2",
                "gps_variance_y_m2",
            ]
        )
        for index in range(len(raw)):
            writer.writerow(
                [
                    raw[index, 0],
                    raw[index, 1],
                    raw[index, 2],
                    navsat_track[index, 1],
                    navsat_track[index, 2],
                    navsat_gps_error[index],
                    direct_centreline[index],
                    navsat_centreline[index],
                    raw[index, 3],
                    raw[index, 4],
                ]
            )

    with (HERE / "global_ekf_samples.csv").open(
        "w", newline="", encoding="utf-8"
    ) as stream:
        writer = csv.writer(stream)
        writer.writerow(
            [
                "stamp_s",
                "elapsed_s",
                "global_x_m",
                "global_y_m",
                "interpolated_direct_utm_x_m",
                "interpolated_direct_utm_y_m",
                "global_to_direct_utm_m",
                "global_to_centreline_m",
            ]
        )
        for index in range(len(global_track)):
            writer.writerow(
                [
                    global_track[index, 0],
                    global_track[index, 0] - raw[0, 0],
                    global_track[index, 1],
                    global_track[index, 2],
                    interpolated_direct[index, 0],
                    interpolated_direct[index, 1],
                    global_gps_error[index],
                    global_centreline[index],
                ]
            )

    full_bounds = (-15.0, 470.0, -15.0, 420.0)
    zoom_bounds = (
        float(np.min(raw[:, 1]) - 15.0),
        float(np.max(raw[:, 1]) + 15.0),
        float(np.min(raw[:, 2]) - 15.0),
        float(np.max(raw[:, 2]) + 15.0),
    )
    figure, axes = plt.subplots(1, 3, figsize=(20, 7), constrained_layout=True)

    for axis, bounds, title in (
        (axes[0], full_bounds, "Full relocation produced by current EKF"),
        (axes[1], zoom_bounds, "Zoom at the recorded rover route"),
    ):
        add_map_background(axis, static_image, bounds)
        add_centrelines(axis, graph, bounds)
        axis.plot(
            global_track[:, 1],
            global_track[:, 2],
            color="#f97316",
            linewidth=2.2,
            label="Current global EKF",
            zorder=4,
        )
        axis.plot(
            navsat_track[:, 1],
            navsat_track[:, 2],
            color="#2563eb",
            linewidth=2.0,
            label="navsat_transform GPS",
            zorder=5,
        )
        axis.plot(
            raw[:, 1],
            raw[:, 2],
            color="#16a34a",
            linewidth=2.2,
            label="Direct GPS in EPSG:32643",
            zorder=6,
        )
        axis.scatter(
            [global_track[0, 1]],
            [global_track[0, 2]],
            marker="x",
            s=80,
            color="#dc2626",
            label="EKF start" if axis is axes[0] else None,
            zorder=7,
        )
        axis.set_xlim(bounds[0], bounds[1])
        axis.set_ylim(bounds[2], bounds[3])
        axis.set_aspect("equal", adjustable="box")
        axis.set_title(title)
        axis.set_xlabel("DTU map X (m, east)")
        axis.set_ylabel("DTU map Y (m, north)")
        axis.grid(alpha=0.2)

    handles, labels = axes[1].get_legend_handles_labels()
    unique = dict(zip(labels, handles))
    axes[1].legend(unique.values(), unique.keys(), loc="best", fontsize=8)

    elapsed_global = global_track[:, 0] - raw[0, 0]
    elapsed_gps = raw[:, 0] - raw[0, 0]
    axes[2].semilogy(
        elapsed_global,
        global_gps_error,
        color="#f97316",
        linewidth=2.0,
        label="Global EKF to direct GPS",
    )
    axes[2].semilogy(
        elapsed_global,
        global_centreline,
        color="#7c3aed",
        linewidth=1.6,
        label="Global EKF to centreline",
    )
    axes[2].semilogy(
        elapsed_gps,
        navsat_gps_error,
        color="#2563eb",
        linewidth=1.8,
        label="navsat GPS to direct GPS",
    )
    axes[2].scatter(
        elapsed_gps,
        direct_centreline,
        color="#16a34a",
        s=12,
        label="Direct GPS to centreline",
        zorder=5,
    )
    axes[2].axhline(1.0, color="#6b7280", linestyle=":", linewidth=1.0)
    axes[2].set_title("Position error through the 111.8 s bag")
    axes[2].set_xlabel("Elapsed bag time (s)")
    axes[2].set_ylabel("Distance (m, logarithmic scale)")
    axes[2].set_xlim(0.0, raw[-1, 0] - raw[0, 0])
    axes[2].grid(which="both", alpha=0.25)
    axes[2].legend(loc="best", fontsize=8)

    figure.suptitle(
        "August 25 bag over DTU prior map — current August 27 RTK pipeline",
        fontsize=15,
    )
    figure.savefig(HERE / "aug25_dtu_overlay.png", dpi=180)
    plt.close(figure)

    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
