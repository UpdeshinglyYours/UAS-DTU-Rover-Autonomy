#!/usr/bin/env python3
import rclpy
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
            '/bcr_bot/odom',
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
        t.transform.translation.z = msg.pose.pose.position.z

        # Copy orientation
        t.transform.rotation = msg.pose.pose.orientation

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
