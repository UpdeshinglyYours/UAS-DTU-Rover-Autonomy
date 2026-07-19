#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist


class CmdVelFilter(Node):

    def __init__(self):
        super().__init__('cmd_vel_filter')

        self.threshold = 0.5  # rad/s

        self.sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.pub = self.create_publisher(
            Twist,
            '/mavros/setpoint_velocity/cmd_vel_unstamped',
            10
        )

        self.get_logger().info(
            f'CmdVel filter started. Angular threshold = {self.threshold} rad/s'
        )

    def cmd_vel_callback(self, msg):

        out_msg = Twist()

        # Copy linear velocities
        out_msg.linear.x = msg.linear.x
        out_msg.linear.y = msg.linear.y
        out_msg.linear.z = msg.linear.z

        # Copy angular velocities
        out_msg.angular.x = msg.angular.x
        out_msg.angular.y = msg.angular.y

        # Filter angular.z
        if abs(msg.angular.z) < self.threshold:
            out_msg.angular.z = 0.0
        else:
            out_msg.angular.z = msg.angular.z

        self.pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)

    node = CmdVelFilter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()