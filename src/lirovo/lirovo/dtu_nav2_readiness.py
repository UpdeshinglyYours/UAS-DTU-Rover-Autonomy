#!/usr/bin/env python3

"""Read-only DTU RTK Nav2 readiness check.

This utility observes the ROS graph for a bounded interval. It does not
publish, configure, activate, or otherwise change any rover component.
"""

import time
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan, NavSatFix
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener

try:
    from mavros_msgs.msg import GPSRAW
except ImportError:  # Optional when checking a non-MAVROS development host.
    GPSRAW = None


TOPICS = {
    '/scan': (LaserScan, 5.0, 1.0),
    '/odometry/filtered': (Odometry, 25.0, 0.25),
    '/odometry/global': (Odometry, 20.0, 0.25),
    '/mavros/imu/data': (Imu, None, 0.25),
    '/mavros/global_position/raw/fix': (NavSatFix, None, 2.0),
}

REQUIRED_TF_EDGES = (
    ('map', 'odom'),
    ('odom', 'base_link'),
    ('base_link', 'base_footprint'),
    ('base_footprint', 'gps_antenna'),
)


class TopicStats:
    """Receipt timing for one topic."""

    def __init__(self):
        self.receipts = []
        self.last_stamp = None

    def update(self, message):
        self.receipts.append(time.monotonic())
        header = getattr(message, 'header', None)
        if header is not None:
            stamp = header.stamp
            self.last_stamp = stamp.sec + stamp.nanosec * 1.0e-9

    @property
    def count(self):
        return len(self.receipts)

    @property
    def rate(self):
        if len(self.receipts) < 2:
            return 0.0
        duration = self.receipts[-1] - self.receipts[0]
        return (len(self.receipts) - 1) / duration if duration > 0 else 0.0

    @property
    def receipt_age(self):
        if not self.receipts:
            return float('inf')
        return time.monotonic() - self.receipts[-1]


class DtuNav2Readiness(Node):
    """Collect and report read-only DTU RTK readiness evidence."""

    def __init__(self):
        super().__init__('dtu_nav2_readiness')
        self.declare_parameter('sample_seconds', 5.0)
        self.sample_seconds = float(
            self.get_parameter('sample_seconds').value
        )
        self.stats = {topic: TopicStats() for topic in TOPICS}
        self.direct_tf_edges = set()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._readiness_subscriptions = []
        for topic, (message_type, _, _) in TOPICS.items():
            subscription = self.create_subscription(
                message_type,
                topic,
                lambda message, name=topic: self.stats[name].update(message),
                qos_profile_sensor_data,
            )
            self._readiness_subscriptions.append(subscription)

        dynamic_tf_qos = QoSProfile(depth=100)
        self._readiness_subscriptions.append(
            self.create_subscription(
                TFMessage, '/tf', self.tf_callback, dynamic_tf_qos
            )
        )
        static_tf_qos = QoSProfile(depth=100)
        static_tf_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._readiness_subscriptions.append(
            self.create_subscription(
                TFMessage, '/tf_static', self.tf_callback, static_tf_qos
            )
        )

        self.rtk_fix_type = None
        if GPSRAW is not None:
            self._readiness_subscriptions.append(
                self.create_subscription(
                    GPSRAW,
                    '/mavros/gpsstatus/gps1/raw',
                    self.gpsraw_callback,
                    qos_profile_sensor_data,
                )
            )

    def tf_callback(self, message):
        for transform in message.transforms:
            self.direct_tf_edges.add(
                (transform.header.frame_id, transform.child_frame_id)
            )

    def gpsraw_callback(self, message):
        self.rtk_fix_type = int(message.fix_type)

    def topic_exists(self, topic):
        return bool(self.get_publishers_info_by_topic(topic))

    def topic_results(self):
        results = []
        for topic, (_, minimum_rate, maximum_age) in TOPICS.items():
            stats = self.stats[topic]
            exists = self.topic_exists(topic)
            fresh = stats.receipt_age <= maximum_age
            rate_ok = minimum_rate is None or stats.rate >= minimum_rate
            passed = exists and stats.count > 0 and fresh and rate_ok
            detail = (
                f'count={stats.count}, rate={stats.rate:.2f} Hz, '
                f'receipt_age={stats.receipt_age:.2f} s'
            )
            if minimum_rate is not None:
                detail += f', minimum={minimum_rate:.2f} Hz'
            results.append((passed, f'TOPIC {topic}', detail))
        return results

    def tf_results(self):
        results = []
        frames = yaml.safe_load(self.tf_buffer.all_frames_as_yaml()) or {}
        for parent, child in REQUIRED_TF_EDGES:
            buffered_parent = frames.get(child, {}).get('parent')
            direct = (
                buffered_parent == parent
                or (parent, child) in self.direct_tf_edges
            )
            connected = self.tf_buffer.can_transform(
                parent, child, rclpy.time.Time()
            )
            results.append(
                (
                    direct and connected,
                    f'TF {parent} -> {child}',
                    (
                        f'direct_edge={direct}, connected={connected}, '
                        f'buffer_parent={buffered_parent}'
                    ),
                )
            )
        return results

    @staticmethod
    def load_nav2_config():
        share = Path(get_package_share_directory('lirovo'))
        path = share / 'config' / 'nav2_dtu_rtk_params.yaml'
        with path.open(encoding='utf-8') as stream:
            return yaml.safe_load(stream), path

    def configuration_results(self):
        config, path = self.load_nav2_config()
        controller = config['controller_server']['ros__parameters']
        local = config['local_costmap']['local_costmap']['ros__parameters']
        global_costmap = config['global_costmap']['global_costmap'][
            'ros__parameters'
        ]
        results = [
            (
                controller.get('odom_topic') == '/odometry/filtered',
                'CONFIG controller_server odom',
                str(controller.get('odom_topic')),
            ),
            (
                local['obstacle_layer']['scan'].get('topic') == '/scan',
                'CONFIG local obstacle source',
                str(local['obstacle_layer']['scan'].get('topic')),
            ),
            (
                global_costmap['obstacle_layer']['scan'].get('topic')
                == '/scan',
                'CONFIG global obstacle source',
                str(global_costmap['obstacle_layer']['scan'].get('topic')),
            ),
        ]
        for index, result in enumerate(results):
            passed, name, detail = result
            results[index] = (passed, name, f'{detail} ({path})')
        return results

    def controller_graph_result(self):
        filtered = self.get_subscriptions_info_by_topic(
            '/odometry/filtered'
        )
        default_odom = self.get_subscriptions_info_by_topic('/odom')

        def is_controller(endpoint):
            return endpoint.node_name == 'controller_server'

        filtered_controller = any(is_controller(item) for item in filtered)
        default_controller = any(is_controller(item) for item in default_odom)
        controller_present = any(
            name == 'controller_server'
            for name, _ in self.get_node_names_and_namespaces()
        )
        if filtered_controller and not default_controller:
            return (
                True,
                'GRAPH controller_server odom',
                '/odometry/filtered (no /odom subscription)',
            )
        if default_controller:
            return (
                False,
                'GRAPH controller_server odom',
                'unexpected /odom subscription detected',
            )
        state = 'unconfigured' if controller_present else 'not running'
        return (
            None,
            'GRAPH controller_server odom',
            f'{state}; static configuration checked above',
        )

    def rtk_status(self):
        labels = {5: 'RTK Float', 6: 'RTK Fixed'}
        if self.rtk_fix_type is None:
            return 'unavailable (GPSRAW is optional)'
        return labels.get(self.rtk_fix_type, f'fix_type={self.rtk_fix_type}')

    def report(self):
        results = (
            self.topic_results()
            + self.tf_results()
            + self.configuration_results()
            + [self.controller_graph_result()]
        )
        print('DTU RTK NAV2 READINESS (READ-ONLY)')
        print(f'Sampled for {self.sample_seconds:.1f} s')
        for passed, name, detail in results:
            label = 'PASS' if passed else 'FAIL'
            if passed is None:
                label = 'SKIP'
            print(f'{label:4} | {name:<38} | {detail}')
        tf_publishers = {
            topic: len(self.get_publishers_info_by_topic(topic))
            for topic in ('/tf', '/tf_static')
        }
        print(
            'INFO | TF publisher endpoints                  | '
            f"/tf={tf_publishers['/tf']}, "
            f"/tf_static={tf_publishers['/tf_static']}"
        )
        print(
            'INFO | TF ownership                            | '
            'direct edges checked; correlate publishers with nodes using '
            '`ros2 topic info -v /tf` when more than one endpoint exists'
        )
        print(f'INFO | RTK status                              | {self.rtk_status()}')
        failures = [result for result in results if result[0] is False]
        print('RESULT: PASS' if not failures else 'RESULT: FAIL')
        return not failures


def main(args=None):
    rclpy.init(args=args)
    node = DtuNav2Readiness()
    deadline = time.monotonic() + node.sample_seconds
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    passed = node.report()
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if passed else 1)


if __name__ == '__main__':
    main()
