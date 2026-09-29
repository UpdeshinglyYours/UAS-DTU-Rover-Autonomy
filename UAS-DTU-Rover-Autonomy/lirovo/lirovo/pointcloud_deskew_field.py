#!/usr/bin/env python3
"""
Republishes the simulated 3D lidar's sensor_msgs/PointCloud2 with an added per-point
"point_time_offset" field (float64), which is the exact field name your genz-icp fork's
GetTimestampField() looks for (see genz-icp/ros/ros2/Utils.hpp) alongside the stock
"t"/"timestamp"/"time" names. genz_icp normalizes whatever it reads to 0.0-1.0 internally
(NormalizeTimestamps), so this node just writes that same 0.0-1.0 fraction directly - no unit
conversion needed on either side.

Same simulation caveat as livox_deskew_converter.py: Gazebo Classic's ray sensor computes the
whole scan grid in a single physics step, so there is no genuine per-point capture time to
report, and no way to reproduce a true Blickfeld dual-MEMS-mirror Lissajous/rosette scan
pattern without a custom raycasting plugin. This synthesizes a serpentine (boustrophedon)
sweep over the sensor's fixed horizontal x vertical raster grid instead - closer to a real
MEMS fast-axis mirror's back-and-forth motion than either "everything simultaneous" or a
single-direction sweep, but still an approximation for deskew-compatible timestamps, not a
physically faithful MEMS simulation.
"""

import struct

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2


class PointCloudDeskewField(Node):
    def __init__(self):
        super().__init__('pointcloud_deskew_field')

        # Must match the sensor's actual <horizontal><samples> / <vertical><samples> config
        # in bcr_bot.xacro's three_d_lidar block, so the row/col reconstruction below lines
        # up with how the plugin actually iterates the raster grid. If they drift out of
        # sync this falls back to a flat linear mapping (still monotonic 0..1, just without
        # the serpentine shape).
        self.declare_parameter('horizontal_samples', 333)
        self.declare_parameter('vertical_lasers', 24)

        self.horizontal_samples = self.get_parameter('horizontal_samples').get_parameter_value().integer_value
        self.vertical_lasers = self.get_parameter('vertical_lasers').get_parameter_value().integer_value

        self.sub = self.create_subscription(PointCloud2, 'points_in', self.pc_callback, 10)
        self.pub = self.create_publisher(PointCloud2, 'points_out', 10)

    def pc_callback(self, msg):
        points = list(pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True))
        point_num = len(points)
        if point_num == 0:
            return

        expected_grid = self.horizontal_samples * self.vertical_lasers
        use_grid = (point_num == expected_grid)

        out = PointCloud2()
        out.header = msg.header
        out.height = 1
        out.width = point_num
        out.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='point_time_offset', offset=12, datatype=PointField.FLOAT64, count=1),
        ]
        out.is_bigendian = False
        out.point_step = 20  # 3 * float32 (12 bytes) + 1 * float64 (8 bytes)
        out.row_step = out.point_step * out.width
        out.is_dense = True

        buf = bytearray()
        for k, p in enumerate(points):
            if use_grid:
                row = k // self.horizontal_samples
                col = k % self.horizontal_samples
                # serpentine: alternate sweep direction each row, like a MEMS fast-axis
                # mirror flicking back and forth rather than snapping back to scan start
                effective_col = col if (row % 2 == 0) else (self.horizontal_samples - 1 - col)
                linear_index = row * self.horizontal_samples + effective_col
                fraction = linear_index / max(expected_grid - 1, 1)
            else:
                fraction = k / max(point_num - 1, 1)

            # explicit little-endian, no native struct padding (must match point_step=20 exactly)
            buf.extend(struct.pack('<fffd', float(p[0]), float(p[1]), float(p[2]), fraction))

        out.data = bytes(buf)
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudDeskewField()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
