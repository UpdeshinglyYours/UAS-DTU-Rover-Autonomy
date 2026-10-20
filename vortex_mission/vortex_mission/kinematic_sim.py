#!/usr/bin/env python3
"""kinematic_sim: a Gazebo-free stand-in for the rover, for desk-testing the casualty pipeline.

* integrates /cmd_vel as a unicycle and broadcasts TF map -> base_link
  (+ /odometry/filtered and /odom)
* fakes the sensors nav2_params3/4 read, from boxes placed in the world:
    /patchwork/nonground and /bf_lidar/point_cloud_out  (PointCloud2, frame 'lidar')
    /sf45b/scan                                         (LaserScan,   frame 'sf45b')

Boxes: parameter `boxes` = flat list [x0, y0, x1, y1, ...] (map frame, metres).
The static TFs base_link->lidar and base_link->sf45b must be published as in lirovo4.
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from tf2_ros import TransformBroadcaster


class KinematicSim(Node):

    def __init__(self):
        super().__init__('kinematic_sim')
        self.x = self.declare_parameter('start_x', 0.0).value
        self.y = self.declare_parameter('start_y', 0.0).value
        self.yaw = self.declare_parameter('start_yaw', 0.0).value
        boxes = list(self.declare_parameter('boxes', [0.0]).value)
        self.boxes = [tuple(boxes[i:i + 4]) for i in range(0, len(boxes) - 3, 4)]
        self.body = (self.declare_parameter('real_footprint_length', 0.8).value,
                     self.declare_parameter('real_footprint_width', 0.7).value)
        self.min_clearance = float('inf')
        self.lidar_offset = list(self.declare_parameter('lidar_offset', [-0.17, 0.0, 0.54]).value)
        self.sf45b_offset = list(self.declare_parameter('sf45b_offset', [0.29, 0.0, 0.15]).value)
        self.max_range = self.declare_parameter('max_range', 15.0).value
        self.cmd_timeout = self.declare_parameter('cmd_timeout', 0.5).value
        # Drivetrain model of the real rover (nav2_params4 + new_omega_fixed5 tables):
        # commands below the deadbands are not executed, and it lags ~1 s.
        self.drivetrain = self.declare_parameter('drivetrain_model', True).value
        # angular 0.30 = velocity_smoother deadband (the rover ignores gentler yaw rates);
        # lags ~0.5 s time constant = the 0.8-1.6 s settle times in omega/linear_feedforward.yaml
        self.v_dead = self.declare_parameter('linear_deadband', 0.38).value
        self.w_dead = self.declare_parameter('angular_deadband', 0.30).value
        self.tau_v = self.declare_parameter('linear_lag', 0.5).value
        self.tau_w = self.declare_parameter('angular_lag', 0.5).value
        self.v_cmd = self.w_cmd = 0.0      # what was commanded
        self.v = self.w = 0.0              # what the wheels actually do
        self.last_cmd = self.get_clock().now()

        # Box SURFACES sampled every 2 cm (sides x heights 0.1..0.9 m), like a real lidar
        # return. nav2_params3/4 STVL uses a voxel filter with voxel_min_points: 3 on 4-5 cm
        # voxels, so a sparse cloud would be filtered away entirely.
        pts = []
        step = 0.02
        zs = np.arange(0.10, 0.90 + 1e-6, step)
        for x0, y0, x1, y1 in self.boxes:
            xa, xb, ya, yb = min(x0, x1), max(x0, x1), min(y0, y1), max(y0, y1)
            xs = np.arange(xa, xb + 1e-6, step)
            ys = np.arange(ya, yb + 1e-6, step)
            outline = np.vstack([np.column_stack([xs, np.full_like(xs, ya)]),
                                 np.column_stack([xs, np.full_like(xs, yb)]),
                                 np.column_stack([np.full_like(ys, xa), ys]),
                                 np.column_stack([np.full_like(ys, xb), ys])])
            for z in zs:
                pts.append(np.column_stack([outline, np.full(len(outline), z)]))
        self.world = np.vstack(pts) if pts else np.zeros((0, 3))
        self.get_logger().info(f'{len(boxes) // 4} box(es), {len(self.world)} obstacle points')

        self.tf = TransformBroadcaster(self)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.pub_odom = [self.create_publisher(Odometry, t, 10)
                         for t in ('/odometry/filtered', '/odom')]
        self.pub_cloud = [self.create_publisher(PointCloud2, t, 10)
                          for t in ('/patchwork/nonground', '/bf_lidar/point_cloud_out')]
        self.pub_scan = self.create_publisher(LaserScan, '/sf45b/scan', 10)
        self.dt = 0.02
        self.create_timer(self.dt, self.step)
        self.create_timer(0.2, self.sense)
        self.create_timer(0.1, self.check_collision)

    def on_cmd(self, msg: Twist):
        self.v_cmd, self.w_cmd = msg.linear.x, msg.angular.z
        self.last_cmd = self.get_clock().now()

    def step(self):
        if (self.get_clock().now() - self.last_cmd).nanoseconds * 1e-9 > self.cmd_timeout:
            self.v_cmd = self.w_cmd = 0.0
        if self.drivetrain:
            v_t = self.v_cmd if abs(self.v_cmd) >= self.v_dead else 0.0
            w_t = self.w_cmd if abs(self.w_cmd) >= self.w_dead else 0.0
            self.v += (v_t - self.v) * min(1.0, self.dt / self.tau_v)
            self.w += (w_t - self.w) * min(1.0, self.dt / self.tau_w)
        else:
            self.v, self.w = self.v_cmd, self.w_cmd
        self.x += self.v * math.cos(self.yaw) * self.dt
        self.y += self.v * math.sin(self.yaw) * self.dt
        self.yaw = math.atan2(math.sin(self.yaw + self.w * self.dt),
                              math.cos(self.yaw + self.w * self.dt))
        now = self.get_clock().now().to_msg()
        t = TransformStamped()
        t.header.stamp, t.header.frame_id, t.child_frame_id = now, 'map', 'base_link'
        t.transform.translation.x, t.transform.translation.y = self.x, self.y
        t.transform.rotation.z = math.sin(self.yaw / 2)
        t.transform.rotation.w = math.cos(self.yaw / 2)
        self.tf.sendTransform(t)
        o = Odometry()
        o.header.stamp, o.header.frame_id, o.child_frame_id = now, 'map', 'base_link'
        o.pose.pose.position.x, o.pose.pose.position.y = self.x, self.y
        o.pose.pose.orientation = t.transform.rotation
        o.twist.twist.linear.x, o.twist.twist.angular.z = self.v, self.w
        for p in self.pub_odom:
            p.publish(o)

    def check_collision(self):
        """Real-body clearance to every box; logs the session minimum, errors on contact."""
        if not self.boxes:
            return
        L, W = self.body
        xs, ys = np.meshgrid(np.linspace(-L / 2, L / 2, 9), np.linspace(-W / 2, W / 2, 9))
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        px, py = self.x + c * xs - s * ys, self.y + s * xs + c * ys
        gap = float('inf')
        for x0, y0, x1, y1 in self.boxes:
            dx = np.maximum(np.maximum(min(x0, x1) - px, px - max(x0, x1)), 0.0)
            dy = np.maximum(np.maximum(min(y0, y1) - py, py - max(y0, y1)), 0.0)
            gap = min(gap, float(np.min(np.hypot(dx, dy))))
        if gap < self.min_clearance:
            self.min_clearance = gap
            self.get_logger().info(f'closest approach to an obstacle so far: {gap:.2f} m')
        if gap <= 0.0:
            self.get_logger().error('COLLISION with an obstacle box', throttle_duration_sec=1.0)

    def _to_sensor(self, offset):
        """World obstacle points in a sensor frame mounted at `offset` on base_link (no rotation)."""
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        dx, dy = self.world[:, 0] - self.x, self.world[:, 1] - self.y
        bx, by = c * dx + s * dy, -s * dx + c * dy
        return np.column_stack([bx - offset[0], by - offset[1], self.world[:, 2] - offset[2]])

    def sense(self):
        stamp = self.get_clock().now().to_msg()
        # Blickfeld: forward ~70 deg cone, 1.5 .. max_range
        if len(self.world):
            p = self._to_sensor(self.lidar_offset)
            r = np.hypot(p[:, 0], p[:, 1])
            ang = np.arctan2(p[:, 1], p[:, 0])
            p = p[(r < self.max_range) & (np.abs(ang) < math.radians(35))]
        else:
            p = np.zeros((0, 3))
        cloud = PointCloud2()
        cloud.header.stamp, cloud.header.frame_id = stamp, 'lidar'
        cloud.height, cloud.width = 1, len(p)
        cloud.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                        for i, n in enumerate('xyz')]
        cloud.point_step, cloud.row_step = 12, 12 * len(p)
        cloud.is_dense = True
        cloud.data = p.astype(np.float32).tobytes()
        for pub in self.pub_cloud:
            pub.publish(cloud)
        # SF45/B: 2D scan, -90..90 deg, 0.4 .. 2.0 m
        scan = LaserScan()
        scan.header.stamp, scan.header.frame_id = stamp, 'sf45b'
        scan.angle_min, scan.angle_max = -math.pi / 2, math.pi / 2
        scan.angle_increment = math.radians(1.0)
        # Like the real driver: range_max is the sensor's (50 m), not the 2 m the costmap
        # uses. With inf_is_valid, nav2 maps inf -> range_max - 1e-4; a 2.0 range_max
        # would paint a phantom 2 m ring around the rover.
        scan.range_min, scan.range_max = 0.4, 50.0
        n = int(round((scan.angle_max - scan.angle_min) / scan.angle_increment)) + 1
        ranges = np.full(n, float('inf'))
        if len(self.world):
            q = self._to_sensor(self.sf45b_offset)
            q = q[np.abs(q[:, 2] - 0.0) < 0.3]
            r = np.hypot(q[:, 0], q[:, 1])
            a = np.arctan2(q[:, 1], q[:, 0])
            ok = (r >= scan.range_min) & (r <= 2.0) & (np.abs(a) <= math.pi / 2)
            idx = np.round((a[ok] - scan.angle_min) / scan.angle_increment).astype(int)
            np.minimum.at(ranges, idx, r[ok])
        scan.ranges = ranges.astype(np.float32).tolist()
        self.pub_scan.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = KinematicSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
