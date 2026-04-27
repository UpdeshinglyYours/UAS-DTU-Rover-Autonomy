import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelAmplifier(Node):
    def __init__(self):
        super().__init__('cmd_vel_amplifier')
        
        self.pub = self.create_publisher(
            Twist,
            '/mavros/setpoint_velocity/cmd_vel_unstamped',
            10)

        self.sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)

    def cmd_vel_callback(self, msg):
        amplified_msg = Twist()

        # Multiply linear components
        amplified_msg.linear.y = msg.linear.x * 10
        amplified_msg.linear.x = msg.linear.y * 10
        amplified_msg.linear.z = msg.linear.z * 10

        # Multiply angular components
        amplified_msg.angular.x = msg.angular.x * 10
        amplified_msg.angular.y = msg.angular.y * 10
        amplified_msg.angular.z = msg.angular.z * 10

        self.pub.publish(amplified_msg)
        #self.get_logger().info(f"Amplified and published: {amplified_msg}")

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelAmplifier()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

