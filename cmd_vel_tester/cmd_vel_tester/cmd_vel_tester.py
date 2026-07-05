#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)


class CmdVelSequencer(Node):

    def __init__(self):
        super().__init__('cmd_vel_unstamped_sequencer')

        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.subscription = self.create_subscription(
            Imu,
            '/mavros/imu/data',
            self.sensor_callback,
            qos
        )

        self.timer = self.create_timer(0.02, self.timer_callback)

        self.start_time = self.get_clock().now()
        self.time = 4.0

        self.current_yaw_rate = 0.0
        self.last_stage = -1

        self.current_stage_samples = []
        self.results = []
        self.final_printed = False

    def sensor_callback(self, msg):
        self.current_yaw_rate = msg.angular_velocity.z

    def save_stage_stats(self):
        """Compute and save statistics for the current stage."""

        if len(self.current_stage_samples) == 0:
            return

        avg = sum(self.current_stage_samples) / len(self.current_stage_samples)
        minimum = min(self.current_stage_samples)
        maximum = max(self.current_stage_samples)

        self.results.append({
            'stage': self.last_stage,
            'samples': len(self.current_stage_samples),
            'mean': avg,
            'min': minimum,
            'max': maximum
        })

    def timer_callback(self):

        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1e9

        cmd = Twist()

        stage = int(elapsed // self.time)

        # Detect stage change
        if stage != self.last_stage:

            if self.last_stage != -1:
                self.save_stage_stats()

            self.current_stage_samples = []
            self.last_stage = stage

        # Stage 0
        if elapsed < self.time * 1.0:
            cmd.angular.z = 0.75
            self.current_stage_samples.append(self.current_yaw_rate)

        # Stage 1
        elif elapsed < self.time * 2.0:
            cmd.angular.z = -0.75
            self.current_stage_samples.append(self.current_yaw_rate)

        # Stage 2
        elif elapsed < self.time * 3.0:
            cmd.angular.z = 0.5
            self.current_stage_samples.append(self.current_yaw_rate)

        # Stage 3
        elif elapsed < self.time * 4.0:
            cmd.angular.z = -0.25
            self.current_stage_samples.append(self.current_yaw_rate)

        # Stage 4
        elif elapsed < self.time * 5.0:
            cmd.angular.z = 0.25
            self.current_stage_samples.append(self.current_yaw_rate)

        else:

            if not self.final_printed:

                # Save statistics for the final stage
                self.save_stage_stats()

                self.get_logger().info("")
                self.get_logger().info("========== FINAL RESULTS ==========")

                for result in self.results:

                    self.get_logger().info(
                        f"Stage {result['stage']} | "
                        f"Samples={result['samples']} | "
                        f"Mean={result['mean']:.4f} rad/s | "
                        f"Min={result['min']:.4f} rad/s | "
                        f"Max={result['max']:.4f} rad/s"
                    )

                self.get_logger().info("===================================")

                self.final_printed = True

            cmd = Twist()

        self.publisher.publish(cmd)


def main(args=None):

    rclpy.init(args=args)

    node = CmdVelSequencer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()