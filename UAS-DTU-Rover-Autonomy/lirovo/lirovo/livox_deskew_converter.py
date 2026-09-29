#!/usr/bin/env python3
"""
Converts the simulated 3D lidar's plain sensor_msgs/PointCloud2 (from bcr_bot's
gazebo_ros_ray_sensor plugin) into a livox_ros_driver2/CustomMsg, synthesizing a per-point
offset_time field for deskewing.

IMPORTANT LIMITATION: Gazebo Classic's ray sensor computes every point in one grid in a single
physics step - there is no real per-point capture time to report, and no way to reproduce a
genuine Blickfeld-style dual-MEMS-mirror Lissajous/rosette scan pattern without a custom
raycasting plugin. This node approximates it instead: it treats the sensor's fixed
horizontal x vertical raster grid as if a mirror swept it back-and-forth (serpentine /
boustrophedon - left-to-right on one row, right-to-left on the next, alternating), which is
closer to how a real MEMS fast-axis mirror actually moves than either "everything is
simultaneous" or a naive single-direction sweep. It is a best-effort synthetic timestamp for
deskew-compatible downstream code, not a physically faithful MEMS simulation.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from livox_ros_driver2.msg import CustomMsg, CustomPoint


class LivoxDeskewConverter(Node):
    def __init__(self):
        super().__init__('livox_deskew_converter')

        # Must match the sensor's actual <horizontal><samples> and <vertical><samples>
        # config in bcr_bot's gazebo.xacro/bcr_bot.xacro three_d_lidar block, so the
        # row/col reconstruction below lines up with how the plugin actually iterates the
        # raster grid. If they drift out of sync this falls back to a flat linear mapping
        # (still monotonic, just without the serpentine timing shape).
        self.declare_parameter('horizontal_samples', 333)
        self.declare_parameter('vertical_lasers', 24)
        # Full scan period in seconds - defaults to 1.0 / three_d_lidar_update_rate (11.1 Hz)
        self.declare_parameter('scan_period_s', 1.0 / 11.1)
        self.declare_parameter('lidar_id', 0)

        self.horizontal_samples = self.get_parameter('horizontal_samples').get_parameter_value().integer_value
        self.vertical_lasers = self.get_parameter('vertical_lasers').get_parameter_value().integer_value
        self.scan_period_s = self.get_parameter('scan_period_s').get_parameter_value().double_value
        self.lidar_id = self.get_parameter('lidar_id').get_parameter_value().integer_value

        self.sub = self.create_subscription(PointCloud2, 'points_in', self.pc_callback, 10)
        self.pub = self.create_publisher(CustomMsg, 'custom_out', 10)

    def pc_callback(self, msg):
        points = list(pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True))
        point_num = len(points)
        if point_num == 0:
            return

        scan_period_ns = self.scan_period_s * 1e9
        expected_grid = self.horizontal_samples * self.vertical_lasers
        use_grid = (point_num == expected_grid)

        custom_points = []
        for k, p in enumerate(points):
            if use_grid:
                row = k // self.horizontal_samples
                col = k % self.horizontal_samples
                # serpentine: alternate sweep direction each row, like a MEMS fast-axis
                # mirror flicking back and forth rather than snapping back to start
                effective_col = col if (row % 2 == 0) else (self.horizontal_samples - 1 - col)
                linear_index = row * self.horizontal_samples + effective_col
                line = row
            else:
                linear_index = k
                line = 0

            fraction = linear_index / max(expected_grid - 1, 1) if use_grid else k / max(point_num - 1, 1)
            offset_time = int(round(fraction * scan_period_ns))
            offset_time = max(0, min(offset_time, 0xFFFFFFFF))

            cp = CustomPoint()
            cp.offset_time = offset_time
            cp.x = float(p[0])
            cp.y = float(p[1])
            cp.z = float(p[2])
            cp.reflectivity = 0  # simulated sensor has no reflectivity/intensity data
            cp.tag = 0
            cp.line = line
            custom_points.append(cp)

        out = CustomMsg()
        out.header = msg.header
        out.timebase = int(msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec)
        out.point_num = point_num
        out.lidar_id = self.lidar_id
        out.rsvd = [0, 0, 0]
        out.points = custom_points
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = LivoxDeskewConverter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
