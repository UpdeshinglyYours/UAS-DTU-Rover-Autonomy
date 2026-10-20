#!/usr/bin/env python3
"""keepout_monitor: READ-ONLY check of the 2.2 m rule. Never sends velocity commands.

Every 50 ms: real-footprint distance from the rover body to every casualty.
Publishes the smallest on /keepout_monitor/min_distance, warns below keepout + 0.1,
errors below the keepout, and prints the closest approach of the session on exit.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Float32
from tf2_ros import Buffer, TransformException, TransformListener

from vortex_interfaces.msg import CasualtyArray

from vortex_mission.orbit_geometry import body_distance

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST)


class KeepoutMonitor(Node):

    def __init__(self):
        super().__init__('keepout_monitor')
        self.keepout = self.declare_parameter('keepout_radius', 2.2).value
        self.warn_margin = self.declare_parameter('warn_margin', 0.1).value
        self.length = self.declare_parameter('real_footprint_length', 0.8).value
        self.width = self.declare_parameter('real_footprint_width', 0.7).value
        self.global_frame = self.declare_parameter('global_frame', 'map').value
        self.robot_frame = self.declare_parameter('robot_frame', 'base_link').value
        self.casualties = []
        self.session_min = float('inf')
        self.session_min_id = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(CasualtyArray, '/casualties', self.on_casualties, LATCHED)
        self.pub = self.create_publisher(Float32, '~/min_distance', 10)
        self.create_timer(0.05, self.tick)

    def on_casualties(self, msg: CasualtyArray):
        self.casualties = [(c.id, c.pose.pose.position.x, c.pose.pose.position.y)
                           for c in msg.casualties]

    def tick(self):
        if not self.casualties:
            return
        try:
            t = self.tf_buffer.lookup_transform(self.global_frame, self.robot_frame, Time())
        except TransformException:
            return
        x, y = t.transform.translation.x, t.transform.translation.y
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        best, best_id = float('inf'), None
        for cid, cx, cy in self.casualties:
            d = body_distance(cx, cy, x, y, yaw, self.length, self.width)
            if d < best:
                best, best_id = d, cid
        self.pub.publish(Float32(data=float(best)))
        if best < self.session_min:
            self.session_min, self.session_min_id = best, best_id
        if best < self.keepout:
            self.get_logger().error(
                f'KEEPOUT BREACH: body {best:.2f} m from casualty {best_id} '
                f'(limit {self.keepout} m)', throttle_duration_sec=0.5)
        elif best < self.keepout + self.warn_margin:
            self.get_logger().warn(
                f'body {best:.2f} m from casualty {best_id}', throttle_duration_sec=1.0)

    def report(self):
        if self.session_min_id is not None:
            self.get_logger().info(f'closest approach this session: {self.session_min:.3f} m '
                                   f'to casualty {self.session_min_id}')


def main(args=None):
    rclpy.init(args=args)
    node = KeepoutMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.report()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
