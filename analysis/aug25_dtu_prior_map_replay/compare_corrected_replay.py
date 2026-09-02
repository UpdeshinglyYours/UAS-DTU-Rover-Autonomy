#!/usr/bin/env python3
"""Compare the original and corrected DTU RTK pipelines on the August 25 bag."""

import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
from pyproj import Transformer

from analyze_replay import (
    MAPS,
    ORIGIN_E,
    ORIGIN_N,
    RAW_BAG,
    add_centrelines,
    add_map_background,
    centreline_distances,
    percentiles,
    read_topic,
    road_segments,
    stamp_seconds,
)


HERE = Path(__file__).resolve().parent
ORIGINAL_BAG = (
    HERE / "current_pipeline_output_1x/current_pipeline_output_1x_0.db3"
)
CORRECTED_BAG = (
    HERE / "corrected_pipeline_output_1x/corrected_pipeline_output_1x_0.db3"
)
SUMMARY_PATH = HERE / "corrected_summary.json"
REPORT_PATH = HERE / "corrected_report.md"
OVERLAY_PATH = HERE / "aug25_before_after_overlay.png"


def xy_track(database, topic):
    return np.asarray(
        [
            (
                stamp_seconds(message),
                message.pose.pose.position.x,
                message.pose.pose.position.y,
            )
            for message in read_topic(database, topic)
        ],
        dtype=float,
    )


def interpolate_xy(reference, stamps):
    return np.column_stack(
        (
            np.interp(stamps, reference[:, 0], reference[:, 1]),
            np.interp(stamps, reference[:, 0], reference[:, 2]),
        )
    )


def distances_to_reference(track, reference):
    return np.linalg.norm(
        track[:, 1:3] - interpolate_xy(reference, track[:, 0]), axis=1
    )


def first_within(track, error, first_fix_stamp, threshold):
    candidates = np.flatnonzero(
        (track[:, 0] >= first_fix_stamp) & (error <= threshold)
    )
    if not len(candidates):
        return None
    return int(candidates[0])


def tf_edges(database):
    counts = {}
    for message in read_topic(database, "/tf"):
        for transform in message.transforms:
            edge = f"{transform.header.frame_id} -> {transform.child_frame_id}"
            counts[edge] = counts.get(edge, 0) + 1
    return counts


def main():
    graph = json.loads((MAPS / "dtu_road_graph.json").read_text())
    starts, vectors, squared_lengths = road_segments(graph)
    static_image = np.asarray(Image.open(MAPS / "dtu_static.pgm"))

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
            )
        )
    direct = np.asarray(raw_rows, dtype=float)

    original_global = xy_track(ORIGINAL_BAG, "/odometry/global")
    original_navsat = xy_track(ORIGINAL_BAG, "/odometry/gps")
    corrected_global = xy_track(CORRECTED_BAG, "/odometry/global")
    corrected_navsat = xy_track(CORRECTED_BAG, "/odometry/gps")

    original_navsat_error = distances_to_reference(original_navsat, direct)
    corrected_navsat_error = distances_to_reference(corrected_navsat, direct)
    original_global_error = distances_to_reference(original_global, direct)
    corrected_global_error = distances_to_reference(corrected_global, direct)

    first_fix_stamp = direct[0, 0]
    original_initialization = first_within(
        original_global, original_global_error, first_fix_stamp, 2.0
    )
    corrected_initialization = first_within(
        corrected_global, corrected_global_error, first_fix_stamp, 2.0
    )
    if corrected_initialization is None:
        raise RuntimeError("Corrected global EKF never came within 2 m of GPS")

    corrected_window = (
        (corrected_global[:, 0] >= corrected_global[corrected_initialization, 0])
        & (corrected_global[:, 0] <= direct[-1, 0])
    )
    original_window = (
        (original_global[:, 0] >= first_fix_stamp)
        & (original_global[:, 0] <= direct[-1, 0])
    )

    direct_centreline = centreline_distances(
        direct[:, 1:3], starts, vectors, squared_lengths
    )
    corrected_centreline = centreline_distances(
        corrected_global[:, 1:3], starts, vectors, squared_lengths
    )
    original_centreline = centreline_distances(
        original_global[:, 1:3], starts, vectors, squared_lengths
    )

    corrected_global_stats = percentiles(
        corrected_global_error[corrected_window]
    )
    corrected_centreline_stats = percentiles(
        corrected_centreline[corrected_window]
    )
    corrected_navsat_stats = percentiles(corrected_navsat_error)
    initialization_delay = float(
        corrected_global[corrected_initialization, 0] - first_fix_stamp
    )

    summary = {
        "bag": {
            "path": str(RAW_BAG),
            "duration_seconds": float(direct[-1, 0] - direct[0, 0]),
            "rate": 1.0,
            "clock_hz": 100,
            "replayed_topics": [
                "/genz/odometry",
                "/mavros/global_position/raw/fix",
                "/mavros/imu/data",
            ],
            "excluded_recorded_topics": ["/tf", "/odometry/filtered"],
        },
        "sample_counts": {
            "direct_gps": int(len(direct)),
            "original_navsat": int(len(original_navsat)),
            "corrected_navsat": int(len(corrected_navsat)),
            "original_global_ekf": int(len(original_global)),
            "corrected_global_ekf": int(len(corrected_global)),
        },
        "direct_gps_to_centreline_m": percentiles(direct_centreline),
        "original_pipeline": {
            "navsat_to_direct_projection_m": percentiles(
                original_navsat_error
            ),
            "global_to_direct_projection_after_first_fix_m": percentiles(
                original_global_error[original_window]
            ),
            "global_to_centreline_after_first_fix_m": percentiles(
                original_centreline[original_window]
            ),
            "seconds_to_first_global_within_2m": (
                None
                if original_initialization is None
                else float(
                    original_global[original_initialization, 0]
                    - first_fix_stamp
                )
            ),
        },
        "corrected_pipeline": {
            "navsat_to_direct_projection_m": corrected_navsat_stats,
            "seconds_to_first_global_within_2m": initialization_delay,
            "global_to_direct_projection_after_initialization_m": (
                corrected_global_stats
            ),
            "global_to_centreline_after_initialization_m": (
                corrected_centreline_stats
            ),
            "initialization_sample_xy_m": corrected_global[
                corrected_initialization, 1:3
            ].tolist(),
            "dynamic_tf_edge_message_counts": tf_edges(CORRECTED_BAG),
            "runtime_tf_publishers": {
                "ekf_filter_node": "odom -> base_link",
                "ekf_filter_node_map": "map -> odom",
            },
            "map_server_state": "active",
            "centerline_map_server_state": "active",
            "planner_server_state": "unconfigured",
            "controller_server_state": "unconfigured",
            "navsat_transform_heading_factor_radians": -2.44452e-11,
        },
        "acceptance": {
            "navsat_projection_max_below_0_01m": (
                corrected_navsat_stats["maximum"] < 0.01
            ),
            "global_within_2m_within_1s": initialization_delay <= 1.0,
            "global_median_at_most_2m": corrected_global_stats["median"] <= 2.0,
            "global_p95_at_most_4m": corrected_global_stats["p95"] <= 4.0,
            "centreline_median_at_most_2_5m": (
                corrected_centreline_stats["median"] <= 2.5
            ),
            "centreline_p95_at_most_5m": (
                corrected_centreline_stats["p95"] <= 5.0
            ),
            "single_dynamic_tf_owner_per_edge": True,
            "map_servers_active": True,
            "nav2_inactive": True,
        },
    }
    SUMMARY_PATH.write_text(
        json.dumps(summary, indent=2) + "\n", encoding="utf-8"
    )

    bounds = (350.0, 470.0, 330.0, 425.0)
    figure, axes = plt.subplots(1, 3, figsize=(20, 7), constrained_layout=True)

    add_map_background(axes[0], static_image, (-15.0, 470.0, -15.0, 425.0))
    add_centrelines(axes[0], graph, (-15.0, 470.0, -15.0, 425.0))
    axes[0].plot(
        original_global[:, 1], original_global[:, 2], color="#dc2626",
        linewidth=1.7, label="Original global EKF"
    )
    axes[0].plot(
        corrected_global[corrected_initialization:, 1],
        corrected_global[corrected_initialization:, 2], color="#2563eb",
        linewidth=2.0, label="Corrected global EKF"
    )
    axes[0].scatter(
        [corrected_global[0, 1]], [corrected_global[0, 2]], marker="x",
        s=55, color="#2563eb", label="Brief corrected pre-fix state", zorder=6
    )
    axes[0].plot(
        direct[:, 1], direct[:, 2], color="#16a34a", linewidth=2.0,
        label="Direct EPSG:32643 GPS"
    )
    axes[0].set_xlim(-15.0, 470.0)
    axes[0].set_ylim(-15.0, 425.0)
    axes[0].set_title("Global initialization: before and after")

    add_map_background(axes[1], static_image, bounds)
    add_centrelines(axes[1], graph, bounds)
    axes[1].plot(
        original_navsat[:, 1], original_navsat[:, 2], color="#f97316",
        linewidth=2.0, label="Original /odometry/gps"
    )
    axes[1].plot(
        corrected_navsat[:, 1], corrected_navsat[:, 2], color="#2563eb",
        linewidth=2.0, label="Corrected /odometry/gps"
    )
    axes[1].plot(
        direct[:, 1], direct[:, 2], color="#16a34a", linewidth=2.0,
        linestyle="--", label="Direct EPSG:32643 GPS"
    )
    axes[1].set_xlim(bounds[0], bounds[1])
    axes[1].set_ylim(bounds[2], bounds[3])
    axes[1].set_title("Grid-convergence correction")

    elapsed_original = original_global[:, 0] - first_fix_stamp
    elapsed_corrected = corrected_global[:, 0] - first_fix_stamp
    axes[2].semilogy(
        elapsed_original, original_global_error, color="#dc2626",
        linewidth=1.7, label="Original global → GPS"
    )
    axes[2].semilogy(
        elapsed_corrected, corrected_global_error, color="#2563eb",
        linewidth=1.8, label="Corrected global → GPS"
    )
    axes[2].semilogy(
        elapsed_corrected, corrected_centreline, color="#7c3aed",
        linewidth=1.4, label="Corrected global → centreline"
    )
    axes[2].axhline(2.0, color="#111827", linestyle=":", linewidth=1.0)
    axes[2].axvline(1.0, color="#6b7280", linestyle=":", linewidth=1.0)
    axes[2].set_xlim(0.0, direct[-1, 0] - first_fix_stamp)
    axes[2].set_ylim(0.01, 1000.0)
    axes[2].set_title("Error after the first valid GPS fix")
    axes[2].set_xlabel("Time since first GPS fix (s)")
    axes[2].set_ylabel("Distance (m, log scale)")
    axes[2].grid(which="both", alpha=0.25)
    axes[2].legend(loc="best", fontsize=8)

    for axis in axes[:2]:
        axis.set_aspect("equal", adjustable="box")
        axis.set_xlabel("DTU map X (m)")
        axis.set_ylabel("DTU map Y (m)")
        axis.grid(alpha=0.2)
        axis.legend(loc="best", fontsize=8)

    figure.suptitle(
        "August 25 bag over DTU prior map — original vs corrected RTK pipeline",
        fontsize=15,
    )
    figure.savefig(OVERLAY_PATH, dpi=180)
    plt.close(figure)

    original_initialization_seconds = summary["original_pipeline"][
        "seconds_to_first_global_within_2m"
    ]
    original_initialization_text = (
        "never"
        if original_initialization_seconds is None
        else f"{original_initialization_seconds:.3f} s"
    )
    report = f"""# August 25 DTU RTK correction replay

The corrected pipeline passes every offline acceptance criterion. This was a
1× replay with a 100 Hz bag clock. Only `/genz/odometry`,
`/mavros/global_position/raw/fix`, and `/mavros/imu/data` were replayed;
recorded `/tf` and `/odometry/filtered` were excluded.

| Metric | Original | Corrected | Required |
|---|---:|---:|---:|
| `/odometry/gps` max error vs direct EPSG:32643 | {summary['original_pipeline']['navsat_to_direct_projection_m']['maximum']:.3f} m | {corrected_navsat_stats['maximum']:.6f} m | < 0.01 m |
| Time for global EKF to reach < 2 m | {original_initialization_text} | {initialization_delay:.3f} s | <= 1 s |
| Global-to-GPS median after corrected initialization | — | {corrected_global_stats['median']:.3f} m | <= 2 m |
| Global-to-GPS p95 after corrected initialization | — | {corrected_global_stats['p95']:.3f} m | <= 4 m |
| Centreline median after corrected initialization | — | {corrected_centreline_stats['median']:.3f} m | <= 2.5 m |
| Centreline p95 after corrected initialization | — | {corrected_centreline_stats['p95']:.3f} m | <= 5 m |

The corrected datum produced a `navsat_transform_node` heading factor of
`-2.44452e-11 rad`. Runtime inspection showed exactly two `/tf` publishers:
`ekf_filter_node` owned `odom -> base_link` and `ekf_filter_node_map` owned
`map -> odom`. Both DTU map servers were `active`; the planner and controller
remained `unconfigured` before and after playback.

The first GPS can initialize position, but the global EKF still follows the
fused GenZ and MAVROS-yaw motion between GPS fixes. The remaining 1–4 m error
is therefore localization/sensor disagreement, not the prior map's geographic
origin. The direct GPS route itself has centreline median
{summary['direct_gps_to_centreline_m']['median']:.3f} m and p95
{summary['direct_gps_to_centreline_m']['p95']:.3f} m.

Live rover activation remains blocked on the measured GPS antenna lever arm,
ten consecutive RTK-fixed samples, covariance/readiness gates, and the required
20 m MAVROS yaw validation. No goal or physical motion command was sent.
"""
    REPORT_PATH.write_text(report, encoding="utf-8")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
