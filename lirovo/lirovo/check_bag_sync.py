#!/usr/bin/env python3
"""Check that a rover bag's camera and LiDAR topics share one clock.

    python3 check_bag_sync.py <bag dir> [--camera TOPIC] [--lidar TOPIC ...]

For every topic: message count, rate, and the stamp lag -- the bag's
receive time minus the message's header stamp. On one clock that lag is the
sensor's real latency: small, positive and steady (tens of ms for a
camera, up to a scan period for a LiDAR). A topic stamped on another clock
-- a device clock, sim time, a second machine -- shows a lag of seconds or
years, or a negative one. Then, for each LiDAR topic, how far each camera
frame is from its nearest scan, which is what a camera/LiDAR fusion sees.

Exits 1 if a stamped topic looks like it is on another clock, if stamps go
backwards, or if a camera or LiDAR topic named on the command line is
empty. Reads header stamps straight out of the CDR bytes, so a bag of
point clouds is checked in seconds.
"""
import argparse
import bisect
import os
import statistics
import struct
import sys

import yaml

#: Topics whose first field is a std_msgs/Header or builtin_interfaces/Time.
STAMPED_TYPES = {
    "sensor_msgs/msg/CompressedImage", "sensor_msgs/msg/Image",
    "sensor_msgs/msg/CameraInfo", "sensor_msgs/msg/PointCloud2",
    "sensor_msgs/msg/LaserScan", "sensor_msgs/msg/Imu",
    "sensor_msgs/msg/NavSatFix", "sensor_msgs/msg/BatteryState",
    "geometry_msgs/msg/PoseStamped", "geometry_msgs/msg/TwistStamped",
    "nav_msgs/msg/Odometry", "mavros_msgs/msg/RCIn", "mavros_msgs/msg/State",
    "mavros_msgs/msg/VfrHud", "fcb_orin_msgs/msg/Status",
}
#: Lag beyond which a topic is taken to be on another clock.
MAX_LAG_S = 1.0
#: How far a stamp may run ahead of its receive time (clock granularity).
MIN_LAG_S = -0.005

DEFAULT_CAMERA = "/fcb_driver/image_raw/compressed"
DEFAULT_LIDARS = ("/bf_lidar/point_cloud_out", "/lightwarelidar/pointcloud")


def header_stamp_ns(data):
    """Nanoseconds of the leading Time in a CDR message, or None."""
    if len(data) < 12:
        return None
    endian = "<" if data[1] == 1 else ">"
    sec, nsec = struct.unpack_from(endian + "iI", data, 4)
    return sec * 1_000_000_000 + nsec


def open_reader(uri):
    import rosbag2_py
    with open(os.path.join(uri, "metadata.yaml")) as fh:
        meta = yaml.safe_load(fh)["rosbag2_bagfile_information"]
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=uri, storage_id=meta["storage_identifier"]),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    return reader, types


def percentile(values, q):
    values = sorted(values)
    return values[min(len(values) - 1, int(round(q * (len(values) - 1))))]


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("bag")
    ap.add_argument("--camera", default=DEFAULT_CAMERA)
    ap.add_argument("--lidar", action="append", default=None,
                    help=f"LiDAR topic, repeatable (default: {', '.join(DEFAULT_LIDARS)})")
    ap.add_argument("--require", action="store_true",
                    help="fail if the camera or any LiDAR topic has no messages")
    args = ap.parse_args()
    lidars = args.lidar or list(DEFAULT_LIDARS)

    reader, types = open_reader(args.bag)
    recv = {}      # topic -> [receive ns]
    stamps = {}    # topic -> [(stamp ns, receive ns)]
    while reader.has_next():
        topic, data, t = reader.read_next()
        recv.setdefault(topic, []).append(t)
        if types.get(topic) in STAMPED_TYPES:
            stamp = header_stamp_ns(data)
            if stamp:
                stamps.setdefault(topic, []).append((stamp, t))

    problems = []
    print(f"{'topic':44} {'msgs':>6} {'Hz':>7} {'lag med':>9} {'lag p95':>9} {'lag max':>9}  backwards")
    for topic in sorted(types):
        times = recv.get(topic, [])
        n = len(times)
        hz = (n - 1) / ((times[-1] - times[0]) / 1e9) if n > 1 and times[-1] > times[0] else 0.0
        row = f"{topic:44} {n:6d} {hz:7.2f}"
        pairs = stamps.get(topic)
        if pairs:
            lags = [(r - s) / 1e9 for s, r in pairs]
            backwards = sum(1 for a, b in zip(pairs, pairs[1:]) if b[0] < a[0])
            med, p95, worst = statistics.median(lags), percentile(lags, 0.95), max(lags, key=abs)
            row += f" {med*1e3:7.1f}ms {p95*1e3:7.1f}ms {worst*1e3:7.1f}ms  {backwards}"
            if not MIN_LAG_S <= med <= MAX_LAG_S:
                problems.append(f"{topic}: median stamp lag {med:.3f} s -- stamped on another clock")
            if backwards:
                problems.append(f"{topic}: {backwards} stamps went backwards")
        print(row)

    def stamp_list(topic):
        return sorted(s for s, _ in stamps.get(topic, []))

    camera = stamp_list(args.camera)
    print()
    if not camera:
        print(f"camera {args.camera}: no messages")
        if args.require:
            problems.append(f"{args.camera}: no messages")
    for lidar in lidars:
        scans = stamp_list(lidar)
        if not scans:
            print(f"lidar {lidar}: no messages")
            if args.require:
                problems.append(f"{lidar}: no messages")
            continue
        if not camera:
            continue
        gaps = []
        for s in camera:
            i = bisect.bisect_left(scans, s)
            near = [abs(s - scans[j]) for j in (i - 1, i) if 0 <= j < len(scans)]
            gaps.append(min(near) / 1e9)
        overlap = (max(camera[0], scans[0]), min(camera[-1], scans[-1]))
        print(f"camera -> nearest {lidar} scan: median {statistics.median(gaps)*1e3:.1f} ms, "
              f"p95 {percentile(gaps, 0.95)*1e3:.1f} ms, max {max(gaps)*1e3:.1f} ms; "
              f"overlap {max(0, overlap[1]-overlap[0])/1e9:.1f} s")
        if overlap[1] <= overlap[0]:
            problems.append(f"{args.camera} and {lidar} do not overlap in time")

    print()
    if problems:
        print("FAIL")
        for p in problems:
            print("  " + p)
        return 1
    print("OK -- every stamped topic is on the recording machine's clock")
    return 0


if __name__ == "__main__":
    sys.exit(main())
