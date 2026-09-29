#!/usr/bin/env python3
"""
Republishes bcr_bot's ground-truth Gazebo odometry (nav_msgs/Odometry, already correctly
tf_prefixed in its frame_id/child_frame_id - e.g. "rover1/odom" / "rover1/base_footprint")
as nav2's bt_navigator odom_topic (/odometry/filtered by default in nav2_params.yaml).

This does NOT touch TF - Gazebo's diff-drive plugin already publishes the
<namespace>/odom -> <namespace>/base_footprint transform directly, and robot_state_publisher
publishes the fixed base_footprint -> base_link joint from the URDF. This node only fills the
gap left by removing the EKF: nav2 still needs *some* topic at its configured odom_topic for
velocity/pose feedback (e.g. progress checking).
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class OdomRelay(Node):
    def __init__(self):
        super().__init__('odom_relay')
        self.pub = self.create_publisher(Odometry, 'odom_out', 10)
        self.sub = self.create_subscription(Odometry, 'odom_in', self.callback, 10)

    def callback(self, msg):
        # passthrough as-is - frame_id/child_frame_id are already correctly tf_prefixed by
        # bcr_bot's gazebo.xacro, nothing to rewrite
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdomRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
