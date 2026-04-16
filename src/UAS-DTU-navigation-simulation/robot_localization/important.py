#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

from rclpy.qos import QoSProfile, ReliabilityPolicy


class GPSFixRelay(Node):

    def __init__(self):
        super().__init__('gps_fix_relay')

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT  # 🔥 CRITICAL

        self.get_logger().info("QoS set to BEST_EFFORT")

        self.sub = self.create_subscription(
            NavSatFix,
            '/mavros/global_position/raw/fix',
            self.callback,
            qos
        )

        self.pub = self.create_publisher(
            NavSatFix,
            '/gps/fix_corrected',
            qos
        )

    def callback(self, msg):
        msg.header.frame_id = "gps"
        self.pub.publish(msg)
        


def main():
    rclpy.init()
    node = GPSFixRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
