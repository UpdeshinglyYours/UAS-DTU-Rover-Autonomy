#!/usr/bin/env python3
"""Copy selected serialized ROS 2 topics without changing their timestamps."""

from __future__ import annotations

import argparse
import shutil
from collections import Counter
from pathlib import Path

import rosbag2_py


DEFAULT_TOPICS = [
    "/mavros/imu/data",
    "/scan",
    # /scan has no messages in this recording.  This is the smallest recorded
    # non-planar cloud and is retained as an offline proximity proxy.
    "/genz/non_planar_points",
    "/mavros/global_position/raw/fix",
    "/mavros/global_position/raw/gps_vel",
    "/mavros/global_position/raw/satellites",
    "/mavros/global_position/compass_hdg",
    "/mavros/gpsstatus/gps1/raw",
    "/mavros/gpsstatus/gps1/rtk",
    "/mavros/gpsstatus/gps2/raw",
    "/genz/odometry",
    "/odometry/filtered",
    "/odometry/gps",
    "/odometry/global",
    "/gps/filtered",
    "/plan",
    "/unsmoothed_plan",
    "/plan_smoothed",
    "/cmd_vel_nav",
    "/cmd_vel",
    "/goal_pose",
    "/local_costmap/costmap",
    "/local_costmap/published_footprint",
    "/tf",
    "/tf_static",
    "/dtu_reference_route",
    "/dtu_centerline_filter_info",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--topics", nargs="+", default=DEFAULT_TOPICS)
    parser.add_argument(
        "--replace",
        action="store_true",
        help="replace an existing output directory (never touches the input)",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    input_path = args.input.resolve()
    output_path = args.output.resolve()
    if input_path == output_path:
        raise SystemExit("input and output paths must differ")
    if not input_path.is_dir():
        raise SystemExit(f"input bag directory does not exist: {input_path}")
    if output_path.exists():
        if not args.replace:
            raise SystemExit(f"output already exists: {output_path}")
        shutil.rmtree(output_path)

    storage = rosbag2_py.StorageOptions(uri=str(input_path), storage_id="sqlite3")
    converter = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage, converter)

    metadata = {topic.name: topic for topic in reader.get_all_topics_and_types()}
    requested = list(dict.fromkeys(args.topics))
    missing = [topic for topic in requested if topic not in metadata]
    if missing:
        raise SystemExit("requested topics absent from bag: " + ", ".join(missing))

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(output_path), storage_id="sqlite3"),
        converter,
    )
    for topic in requested:
        writer.create_topic(metadata[topic])

    selected = set(requested)
    counts: Counter[str] = Counter()
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic in selected:
            writer.write(topic, data, timestamp)
            counts[topic] += 1

    # Ensure writer destruction flushes metadata before printing completion.
    del writer
    print(f"created {output_path}")
    for topic in requested:
        print(f"{counts[topic]:8d}  {topic}")


if __name__ == "__main__":
    main()
