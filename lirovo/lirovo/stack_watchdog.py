#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, PointCloud2

from tf2_ros import Buffer
from tf2_ros import TransformListener


class StackWatchdog(Node):

    def __init__(self):

        super().__init__('stack_watchdog')

        self.declare_parameter('odom_topic', '/genz/odometry')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('pc_topic', '/bf_lidar/point_cloud_out')

        self.declare_parameter('odom_timeout', 2.0)
        self.declare_parameter('scan_timeout', 2.0)
        self.declare_parameter('pc_timeout', 2.0)

        self.declare_parameter('check_frequency', 1.0)
        self.declare_parameter('check_tf', True)

        self.odom_topic = self.get_parameter(
            'odom_topic'
        ).value

        self.scan_topic = self.get_parameter(
            'scan_topic'
        ).value

        self.pc_topic = self.get_parameter(
            'pc_topic'
        ).value

        self.odom_timeout = self.get_parameter(
            'odom_timeout'
        ).value

        self.scan_timeout = self.get_parameter(
            'scan_timeout'
        ).value

        self.pc_timeout = self.get_parameter(
            'pc_timeout'
        ).value

        self.check_tf = self.get_parameter(
            'check_tf'
        ).value

        self.last_odom = None
        self.last_scan = None
        self.last_pc = None

        self.prev_state = {}

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_cb,
            10
        )

        self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_cb,
            10
        )

        self.create_subscription(
            PointCloud2,
            self.pc_topic,
            self.pc_cb,
            10
        )

        rate = self.get_parameter(
            'check_frequency'
        ).value

        self.timer = self.create_timer(
            1.0 / rate,
            self.check_stack
        )

        self.get_logger().info(
            "Stack watchdog started"
        )

    def odom_cb(self, msg):
        self.last_odom = time.time()

    def scan_cb(self, msg):
        self.last_scan = time.time()

    def pc_cb(self, msg):
        self.last_pc = time.time()

    def report_change(self, name, healthy):

        previous = self.prev_state.get(name)

        if previous is None:
            self.prev_state[name] = healthy
            return

        if previous == healthy:
            return

        self.prev_state[name] = healthy

        if healthy:
            self.get_logger().info(
                f"[RECOVERED] {name}"
            )
        else:
            self.get_logger().error(
                f"[FAILED] {name}"
            )

    def topic_alive(self, last_time, timeout):

        if last_time is None:
            return False

        return (
            time.time() - last_time
        ) < timeout

    def tf_alive(self):

        try:

            self.tf_buffer.lookup_transform(
                'map',
                'base_footprint',
                rclpy.time.Time()
            )

            return True

        except Exception:

            return False

    def check_stack(self):

        odom_ok = self.topic_alive(
            self.last_odom,
            self.odom_timeout
        )

        scan_ok = self.topic_alive(
            self.last_scan,
            self.scan_timeout
        )

        pc_ok = self.topic_alive(
            self.last_pc,
            self.pc_timeout
        )

        self.report_change(
            'Odometry',
            odom_ok
        )

        self.report_change(
            'LaserScan',
            scan_ok
        )

        self.report_change(
            'PointCloud',
            pc_ok
        )

        if self.check_tf:

            tf_ok = self.tf_alive()

            self.report_change(
                'TF map->base_footprint',
                tf_ok
            )
            


def main():

    rclpy.init()

    node = StackWatchdog()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()