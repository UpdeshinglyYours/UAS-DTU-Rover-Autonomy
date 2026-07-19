#!/usr/bin/env python3

import time

import rclpy

from rclpy.node import Node

from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid

from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2

from tf2_ros import Buffer
from tf2_ros import TransformListener

from lifecycle_msgs.srv import GetState

import os
import socket
from datetime import datetime

print("ROVER STARTUP REPORT")
print(datetime.now())
print(socket.gethostname())
print(f"ROS_DOMAIN_ID={os.getenv('ROS_DOMAIN_ID','0')}")


NAV2_NODES = [
    '/planner_server',
    '/controller_server',
    '/bt_navigator'
]


class StartupReport(Node):

    def __init__(self):

        super().__init__('startup_report')

        self.start_time = time.time()

        self.topic_stats = {
            '/bf_lidar/point_cloud_out': {
                'count': 0,
                'last_stamp': None,
                'last_msg_time': None
            },

            '/scan': {
                'count': 0,
                'last_stamp': None,
                'last_msg_time': None
            },

            '/genz/odometry': {
                'count': 0,
                'last_stamp': None,
                'last_msg_time': None
            },

            '/odometry/filtered': {
                'count': 0,
                'last_stamp': None,
                'last_msg_time': None
            },

            '/map': {
                'count': 0,
                'last_stamp': None,
                'last_msg_time': None
            }
        }

        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        self.create_subscription(
            PointCloud2,
            '/bf_lidar/point_cloud_out',
            self.pc_callback,
            10
        )

        self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.create_subscription(
            Odometry,
            '/genz/odometry',
            self.genz_odom_callback,
            10
        )

        self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.filtered_odom_callback,
            10
        )

        self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        self.timer = self.create_timer(
            5.0,
            self.generate_report
        )

        self.report_generated = False

    def update_topic(self, topic):

        now = time.time()

        self.topic_stats[topic]['count'] += 1
        self.topic_stats[topic]['last_msg_time'] = now

    def pc_callback(self, msg):

        self.update_topic(
            '/bf_lidar/point_cloud_out'
        )

    def scan_callback(self, msg):

        self.update_topic(
            '/scan'
        )

    def genz_odom_callback(self, msg):

        self.update_topic(
            '/genz/odometry'
        )

    def filtered_odom_callback(self, msg):

        self.update_topic(
            '/odometry/filtered'
        )

    def map_callback(self, msg):

        self.update_topic(
            '/map'
        )

    def check_tf(self, parent, child):

        try:

            self.tf_buffer.lookup_transform(
                parent,
                child,
                rclpy.time.Time()
            )

            return True

        except Exception:

            return False

    def nav2_state(self, node_name):

        try:

            client = self.create_client(
                GetState,
                f'{node_name}/get_state'
            )

            if not client.wait_for_service(
                timeout_sec=1.0
            ):
                return "UNAVAILABLE"

            req = GetState.Request()

            future = client.call_async(req)

            rclpy.spin_until_future_complete(
                self,
                future,
                timeout_sec=2.0
            )

            if future.result() is None:
                return "NO_RESPONSE"

            return future.result().current_state.label

        except Exception:

            return "ERROR"

    def generate_report(self):

        if self.report_generated:
            return

        self.report_generated = True

        elapsed = max(
            time.time() - self.start_time,
            1.0
        )

        print()
        print("=" * 70)
        print("                    ROVER STARTUP REPORT")
        print("=" * 70)

        print("\nTOPICS")
        print("-" * 20)

        for topic in self.topic_stats:

            if self.topic_stats[topic]['count'] > 0:
                print(f"✓ {topic}")
            else:
                print(f"✗ {topic}")

        print("\nTOPIC RATES")
        print("-" * 20)

        for topic, stats in self.topic_stats.items():

            rate = stats['count'] / elapsed

            print(
                f"{topic:<30} {rate:>6.2f} Hz"
            )

        print("\nTF TREE")
        print("-" * 20)

        tf_checks = [

            ('odom',
             'base_link'),

            ('map',
             'odom'),

            ('map',
             'base_footprint'),

            ('base_link',
             'base_footprint'),

            ('base_link',
             'lidar')
        ]

        for parent, child in tf_checks:

            ok = self.check_tf(
                parent,
                child
            )

            print(
                f"{'✓' if ok else '✗'} "
                f"{parent} -> {child}"
            )

        print("\nNAV2 LIFECYCLE")
        print("-" * 20)

        nav_ok = True

        for node in NAV2_NODES:

            state = self.nav2_state(node)

            if state.lower() == "active":

                print(
                    f"✓ {node} ACTIVE"
                )

            else:

                nav_ok = False

                print(
                    f"✗ {node} {state}"
                )

        print("\nDATA AGE")
        print("-" * 20)

        now = time.time()

        for topic, stats in self.topic_stats.items():

            if stats['last_msg_time'] is None:

                print(
                    f"{topic:<30} NO DATA"
                )

            else:

                age = (
                    now
                    - stats['last_msg_time']
                )

                print(
                    f"{topic:<30} "
                    f"{age:.2f} sec"
                )

        print("\nSYSTEM STATUS")
        print("-" * 20)

        localization_ready = (
            self.topic_stats['/genz/odometry']['count'] > 0
            and self.topic_stats['/odometry/filtered']['count']
            > 0
        )

        mapping_ready = (
            self.topic_stats['/map']['count']
            > 0
        )

        navigation_ready = nav_ok

        print(
            f"Localization: "
            f"{'READY' if localization_ready else 'FAIL'}"
        )

        print(
            f"Mapping: "
            f"{'READY' if mapping_ready else 'FAIL'}"
        )

        print(
            f"Navigation: "
            f"{'READY' if navigation_ready else 'FAIL'}"
        )

        print()
        print("=" * 70)

        if (
            localization_ready
            and mapping_ready
            and navigation_ready
        ):
            print("STACK READY")
        else:
            print("STACK NOT READY")

        print("=" * 70)
        print()

        self.destroy_node()


def main():

    rclpy.init()

    node = StartupReport()

    rclpy.spin(node)


if __name__ == '__main__':
    main()
