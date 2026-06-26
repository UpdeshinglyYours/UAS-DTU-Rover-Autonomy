import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class PrecisionInspector(Node):
    def __init__(self):
        super().__init__('precision_inspector')
        self.subscription = self.create_subscription(
            Odometry, '/genz/odometry', self.listener_callback, 10)

    def listener_callback(self, msg):
        # Print with 20 decimal places of precision
        self.get_logger().info(f"Position X: {msg.pose.pose.position.x:.20f}")
        # Print the first covariance value (index 0) to check precision there too
        self.get_logger().info(f"Covariance [0]: {msg.pose.covariance[0]:.20f}")

def main(args=None):
    rclpy.init(args=args)
    node = PrecisionInspector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()