#!/usr/bin/env python3

import rclpy
import dronekit
import time
from dronekit import connect,VehicleMode
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class MavrosOdomBridge(Node):
    def __init__(self):
        super().__init__('mavros_odom_bridge')
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.subscription = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_callback,
            10
        )

    def pose_callback(self, pose_msg: PoseStamped):
        odom = Odometry()
        odom.header = pose_msg.header
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose = pose_msg.pose
        self.odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header = pose_msg.header
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_link'
        tf.transform.translation.x = pose_msg.pose.position.x
        tf.transform.translation.y = pose_msg.pose.position.y
        tf.transform.translation.z = pose_msg.pose.position.z
        tf.transform.rotation = pose_msg.pose.orientation
        self.tf_broadcaster.sendTransform(tf)


class Nav2MavrosVelocity(Node):
    def __init__(self):
        super().__init__('nav2_mavros_velocity')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.mavros_pub = self.create_publisher(
            TwistStamped,
            '/mavros/setpoint_velocity/cmd_vel_unstamped',
            10
        )

    def cmd_vel_callback(self, msg: Twist):
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = 'base_link'
        twist_stamped.twist = msg
        self.mavros_pub.publish(twist_stamped)


def main(args=None):
    rclpy.init(args=args)
    odom_bridge = MavrosOdomBridge()
    vel_bridge = Nav2MavrosVelocity()

    # connection_string = '/dev/tty/ACM0'
    # vehicle = connect(connection_string, wait_ready=True)
    # print("✅ Vehicle connected.")

    # print("🛡️ Arming vehicle...")
    # vehicle.armed = True
    # while not vehicle.armed:
    #     print("⏳ Waiting for vehicle to arm...")
    #     time.sleep(1)

    # print("🚀 Vehicle is armed. Switching to GUIDED mode.")
    # vehicle.mode = VehicleMode('GUIDED')

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(odom_bridge)
    executor.add_node(vel_bridge)

    try:
        executor.spin()
    except KeyboardInterrupt:
        print("Shutting down MAVROS Bridge...")
    finally:
        odom_bridge.destroy_node()
        vel_bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()