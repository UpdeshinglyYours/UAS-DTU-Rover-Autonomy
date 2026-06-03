import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class GenzCovInjector(Node):
    def __init__(self):
        super().__init__('genz_cov_injector')

        self.subscription = self.create_subscription(
            Odometry,
            '/genz/odometry',
            self.odom_callback,
            10
        )
        self.publisher = self.create_publisher(
            Odometry,
            '/genz/odometry_cov',
            10
        )

    def odom_callback(self, msg):
        # Set child_frame_id so EKF can publish TF
        msg.child_frame_id = "base_link"

        # Inject pose covariance if all zero
        if all(c == 0.0 for c in msg.pose.covariance):
            msg.pose.covariance[0] = 0.05
            msg.pose.covariance[7] = 0.05
            msg.pose.covariance[14] = 0.1
            msg.pose.covariance[21] = 0.01
            msg.pose.covariance[28] = 0.01
            msg.pose.covariance[35] = 0.02

        # Inject twist covariance if all zero
        if all(c == 0.0 for c in msg.twist.covariance):
            msg.twist.covariance[0] = 0.05
            msg.twist.covariance[7] = 0.05
            msg.twist.covariance[14] = 0.1
            msg.twist.covariance[21] = 0.01
            msg.twist.covariance[28] = 0.01
            msg.twist.covariance[35] = 0.02

        # Publish modified message
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = GenzCovInjector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

