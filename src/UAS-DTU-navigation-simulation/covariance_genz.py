#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from copy import deepcopy
from rclpy.qos import QoSProfile, ReliabilityPolicy

class GenzCovInjector(Node):
    def __init__(self):
        super().__init__('genz_cov_injector')

        # Hardcoded variance values
        self.pose_var_xy = 0.05 ** 2
        self.pose_var_z = 0.1 ** 2
        self.pose_var_yaw = 0.02 ** 2
        self.twist_var_vx = 0.02 ** 2
        self.twist_var_wz = 0.01 ** 2

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # Subscribers and publishers
        self.sub = self.create_subscription(Odometry, '/genz/odometry', self.odom_cb, qos)
        self.pub = self.create_publisher(Odometry, '/genz/odometry_cov', qos)

        # To log once
        self.logged_once = False

    def odom_cb(self, msg: Odometry):
        out = deepcopy(msg)

        # Set proper frame IDs
        out.header.frame_id = 'odom'
        out.child_frame_id = 'base_link'

        # Inject pose covariance if all values are zero
        if all(c == 0.0 for c in out.pose.covariance):
            out.pose.covariance[0] = self.pose_var_xy
            out.pose.covariance[7] = self.pose_var_xy
            out.pose.covariance[14] = self.pose_var_z
            out.pose.covariance[21] = self.pose_var_yaw
            out.pose.covariance[28] = self.pose_var_yaw
            out.pose.covariance[35] = self.pose_var_yaw

        # Inject twist covariance if all values are zero
        if all(c == 0.0 for c in out.twist.covariance):
            out.twist.covariance[0] = self.twist_var_vx
            out.twist.covariance[35] = self.twist_var_wz

        # Publish modified message
        self.pub.publish(out)

        if not self.logged_once:
            self.get_logger().info("Started publishing /genz/odometry_cov with injected covariance.")
            self.logged_once = True

def main(args=None):
    rclpy.init(args=args)
    node = GenzCovInjector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

