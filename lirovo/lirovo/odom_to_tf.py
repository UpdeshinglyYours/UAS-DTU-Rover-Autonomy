#!/usr/bin/env python3
import rclpy
# Added: math module for quaternion to RPY and back conversion
import math
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class OdomToTf(Node):
    def __init__(self):
        super().__init__('odom_to_tf_node')
        
        # Create a TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Define QoS profile matching MAVROS (Best Effort)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribe to MAVROS odometry with Best Effort QoS
        self.subscription = self.create_subscription(
            Odometry,
            '/mavros/local_position/odom',
            self.odom_callback,
            qos_profile)
            
        self.get_logger().info("Odometry to TF bridge started. Listening to odometry...")

    def odom_callback(self, msg):
        t = TransformStamped()

        # Copy timestamp and frames
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'

        # Copy position
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        # Commented out dynamic Z:
        # t.transform.translation.z = msg.pose.pose.position.z
        # Added: Hardcode Z to 0.0
        t.transform.translation.z = 0.0

        # Copy orientation
        # Commented out original orientation:
        # t.transform.rotation = msg.pose.pose.orientation
        # Added: Convert quaternion to RPY, zero out roll and pitch, and convert back to quaternion
        q = msg.pose.pose.orientation
        t0 = +2.0 * (q.w * q.x + q.y * q.z)
        t1 = +1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(t0, t1)

        t2 = +2.0 * (q.w * q.y - q.z * q.x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        t3 = +2.0 * (q.w * q.z + q.x * q.y)
        t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(t3, t4)

        # Explicitly zero out roll and pitch
        roll = 0.0
        pitch = 0.0

        # Convert back from (roll=0, pitch=0, yaw) to quaternion
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        # Added: Match hemisphere sign of original quaternion to prevent antipodal jumps in TF when w < 0
        if q.w < 0.0:
            cy = -cy
            sy = -sy
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = sy
        t.transform.rotation.w = cy

        # Publish transform
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdomToTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
