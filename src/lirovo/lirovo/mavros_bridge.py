#!/usr/bin/env python3

import rclpy
import dronekit
import time
from dronekit import connect,VehicleMode
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class MavrosOdomBridge(Node):
    def __init__(self):
        super().__init__('mavros_odom_bridge')
        self.latest_odom_msg = None
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.odom_pub = self.create_publisher(Odometry, '/odom', qos)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.subscription = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_callback,
            qos
        )

        timer_period = 0.1  # 50 ms = 20 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def odom_callback(self, odom_msg: Odometry):
        self.latest_odom_msg = odom_msg

    def timer_callback(self):
        if self.latest_odom_msg is None:
            return

        # Get current time for both /odom and /tf
        current_time = self.get_clock().now().to_msg()

        # Publish Odometry at fixed rate
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose = self.latest_odom_msg.pose
        odom.twist = self.latest_odom_msg.twist
        self.odom_pub.publish(odom)

        # Publish Transform at fixed rate
        tf = TransformStamped()
        tf.header.stamp = current_time
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_link'
        tf.transform.translation.x = odom.pose.pose.position.x
        tf.transform.translation.y = odom.pose.pose.position.y
        tf.transform.translation.z = odom.pose.pose.position.z
        tf.transform.rotation = odom.pose.pose.orientation

        self.tf_broadcaster.sendTransform(tf)


# class Nav2MavrosVelocity(Node):
#     def __init__(self):
#         super().__init__('nav2_mavros_velocity')
#         self.subscription = self.create_subscription(
#             Twist,
#             '/cmd_vel',
#             self.cmd_vel_callback,
#             10
#         )
#         self.mavros_pub = self.create_publisher(
#             Twist,
#             '/mavros/setpoint_velocity/cmd_vel_unstamped',
#             10
#         )

#     def cmd_vel_callback(self, msg: Twist):
#         twist_stamped = TwistStamped()
#         twist_stamped.header.stamp = self.get_clock().now().to_msg()
#         twist_stamped.header.frame_id = 'base_link'
#         twist_stamped.twist = msg
#         self.mavros_pub.publish(twist_stamped)


def main(args=None):
    rclpy.init(args=args)
    odom_bridge = MavrosOdomBridge()
    #vel_bridge = Nav2MavrosVelocity()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(odom_bridge)
    #executor.add_node(vel_bridge)

    try:
        executor.spin()
    except KeyboardInterrupt:
        print("Shutting down MAVROS Bridge...")
    finally:
        odom_bridge.destroy_node()
        #vel_bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
