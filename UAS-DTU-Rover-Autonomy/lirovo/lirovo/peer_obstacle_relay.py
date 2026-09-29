#!/usr/bin/env python3
"""
Relays this rover's own /scan to the peer rover's nav2 stack, UNCHANGED (same frame_id, e.g.
"rover1/base_link" - no coordinate transform applied here at all).

Deliberately does NOT pre-transform points into the peer's map frame. nav2's own ObstacleLayer
already does that itself via its own TF lookups - it resolves whatever frame_id the incoming
message carries against its own global_frame, exactly like it does for its own local sensors.
Since the peer's frame is reachable via the one-time static <own>/map <-> <peer>/map transform,
this works transparently, AND (this is the actual reason to relay raw, not pre-transformed)
ObstacleLayer's raytrace-based clearing computes its "sensor origin" from the incoming
message's own frame_id at each timestamp - if we pre-transform into the peer's map frame, that
origin resolves to a fixed point (the peer map's own origin), not rover1's actual moving
position, making clearing geometrically meaningless. Relaying raw keeps the origin correct, so
peer_obstacle_layer's clearing:true (see nav2_params.yaml) behaves like a real transient local
sensor feed instead of permanent global marks.

Only rate-limits and drops empty scans - no other processing.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class PeerObstacleRelay(Node):
    def __init__(self):
        super().__init__('peer_obstacle_relay')

        self.declare_parameter('publish_rate_hz', 5.0)
        self.publish_rate_hz = self.get_parameter('publish_rate_hz').get_parameter_value().double_value
        self._min_period = 1.0 / self.publish_rate_hz
        self._last_publish_time = None

        # qos_profile_sensor_data (BEST_EFFORT) - pointcloud_to_laserscan_node publishes /scan
        # as BEST_EFFORT; a default RELIABLE subscription is incompatible with that and ROS 2
        # silently delivers zero messages in that case rather than erroring.
        self.sub = self.create_subscription(
            LaserScan, 'scan_in', self.scan_callback, qos_profile_sensor_data)
        self.pub = self.create_publisher(LaserScan, 'scan_out', qos_profile_sensor_data)

    def scan_callback(self, msg):
        now = self.get_clock().now()
        if self._last_publish_time is not None:
            elapsed = (now - self._last_publish_time).nanoseconds / 1e9
            if elapsed < self._min_period:
                return
        self._last_publish_time = now
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PeerObstacleRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
