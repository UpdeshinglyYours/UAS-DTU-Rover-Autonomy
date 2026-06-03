import rclpy
from rclpy.node import Node
from mavros_msgs.msg import OverrideRCIn
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import math


class Converter_Node(Node):
    def __init__(self):
        super().__init__('converter_node')
        self.publisher = self.create_publisher(OverrideRCIn,'/mavros/rc/override',10)
        self.subscriber_cmdvel = self.create_subscription(Twist,'/cmd_vel',self.cmdvel_callback,QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, depth=10))
        self.get_logger().info("node started")
        self.timer = self.create_timer(0.25, self.converter_node)
        self.linear_x = 0.0
        self.angular_z = 0.0


    def cmdvel_callback(self,msg):
        self.linear_x = msg.linear.x * 5
        self.angular_z = msg.angular.z * 3 *-1


    def converter_node(self):
        rc_msg = OverrideRCIn()
        
        if self.linear_x>1:
            self.linear_x=1
        elif self.linear_x<-1:
            self.linear_x=-1

        if self.linear_x>0:
            pwm_linear=300*self.linear_x+1651
        elif self.linear_x<0:
            pwm_linear=300*self.linear_x+1351
        else:
            pwm_linear=1501

        if self.angular_z>1.5:
            self.angular_z=1.5
        elif self.angular_z<-1.5:
            self.angular_z=-1.5

        if self.angular_z>0:
            pwm_angular = 1576 + math.sqrt(5625 + 90000 * self.angular_z)
        elif self.angular_z<0:
            pwm_angular = 1426 - math.sqrt(5625 + 90000 * abs(self.angular_z))
        else:
            pwm_angular=1501

        #print(f"linear velocity is {self.linear_x}")
        #print(f"angular velocity is {self.angular_z}")
        print(f"pwm_linear is {pwm_linear}")
        print(f"pwm_angular is {pwm_angular}")

        for i in range(18):
            rc_msg.channels[i]=65535

        rc_msg.channels[1] = int(pwm_linear) 
        rc_msg.channels[0] = int(pwm_angular) 
        
        self.publisher.publish(rc_msg)

def main(args=None):
    rclpy.init(args=args)
    node = Converter_Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


            

        
        
        
