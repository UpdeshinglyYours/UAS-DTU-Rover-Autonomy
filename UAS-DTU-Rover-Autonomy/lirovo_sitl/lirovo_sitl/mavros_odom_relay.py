#!/usr/bin/env python3
"""
Same idea as lirovo's odom_relay.py (passthrough into nav2's bt_navigator odom_topic), but
subscribing with explicit best-effort QoS - MAVROS's local_position/odom publisher uses
SensorDataQoS (best-effort), and a plain default create_subscription (reliable) silently
drops every message from it ("New publisher discovered... incompatible QoS... RELIABILITY").
lirovo's own odom_relay doesn't opt into qos_overriding_options, so a launch-time
qos_overrides parameter has no effect on it either - hence this separate node instead of
patching that one.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry


class MavrosOdomRelay(Node):
    def __init__(self):
        super().__init__('mavros_odom_relay')
        self.pub = self.create_publisher(Odometry, 'odom_out', 10)
        self.sub = self.create_subscription(
            Odometry, 'odom_in', self.callback, qos_profile_sensor_data)

    def callback(self, msg):
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MavrosOdomRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
