#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class AdaptiveVelocityController(Node):
    def __init__(self):
        super().__init__("adaptive_velocity_controller")
        qos=QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST,depth=10)
        self.create_subscription(Twist,"/cmd_vel",self.cmd_callback,10)
        self.create_subscription(Odometry,"/mavros/local_position/odom",self.odom_callback,qos)
        self.pub=self.create_publisher(Twist,"/mavros/setpoint_velocity/cmd_vel_unstamped",10)

        self.cmd=Twist()
        self.des_v=0.0
        self.des_w=0.0
        self.v_actual=None
        self.w_actual=None
        self.v_filt=0.0
        self.w_filt=0.0
        self.alpha=0.2

        self.prev_des_v = 0.0
        self.prev_des_w = 0.0

        self.initial_gain_v = 0.75
        self.initial_gain_left = 0.55
        self.initial_gain_right = 0.55

        self.reset_threshold_v = 0.4      # m/s
        self.reset_threshold_w = 0.4      # rad/s

        # Initialize gains
        self.gain_v = self.initial_gain_v
        self.gain_left = self.initial_gain_left
        self.gain_right = self.initial_gain_right

        self.lr=0.02
        self.dead_v=0.04
        self.dead_w=0.04
        self.min_gain=0.2
        self.max_gain=2.0
        self.max_v=2.0
        self.max_w=1.5
        self.prev_v=0.0
        self.prev_w=0.0
        self.dv_lim=0.05
        self.dw_lim=0.05
        self.count=0
        self.create_timer(0.01,self.control_loop)

    def cmd_callback(self, msg):

        # Check if desired velocity changed significantly
        # Check if desired velocity changed significantly
        
        if abs(msg.linear.x - self.prev_des_v) > self.reset_threshold_v:

            # Reset learned gain
            self.gain_v = self.initial_gain_v

            # Reset slew-rate memory
            self.prev_v = 0.0

            # Reset filtered odometry
            self.v_filt = 0.0

            self.get_logger().info(
                f"Linear gain reset "
                f"{self.prev_des_v:.2f} -> {msg.linear.x:.2f}"
            )


        # Check if desired yaw rate changed significantly
        
        if abs(msg.angular.z - self.prev_des_w) > self.reset_threshold_w:

            self.gain_left = self.initial_gain_left
            self.gain_right = self.initial_gain_right

            # Reset slew-rate memory
            self.prev_w = 0.0

            # Reset filtered odometry
            self.w_filt = 0.0
            self.w_actual = None

            self.get_logger().info(
                f"Angular gain reset "
                f"{self.prev_des_w:.2f} -> {msg.angular.z:.2f}"
            )

        self.prev_des_v = msg.linear.x

        self.prev_des_w = msg.angular.z

        self.cmd = msg
        self.des_v = msg.linear.x
        self.des_w = -1*msg.angular.z


    def odom_callback(self,msg):
        self.v_filt=self.alpha*msg.twist.twist.linear.x+(1-self.alpha)*self.v_filt
        self.v_actual=self.v_filt
        self.w_filt=self.alpha*msg.twist.twist.angular.z+(1-self.alpha)*self.w_filt
        self.w_actual=(-1)*self.w_filt



    def control_loop(self):

        rv = 1.0
        rw = 1.0
        
        if self.v_actual is None or self.w_actual is None:
            return
        gw=self.gain_right if self.des_w>=0 else self.gain_left
        v_cmd=self.gain_v*self.des_v


        v_cmd = self.gain_v * self.des_v

        w_cmd = gw * self.des_w

        # Slew-rate limiting
        v_cmd = self.prev_v + np.clip(
            v_cmd - self.prev_v,
            -self.dv_lim,
            self.dv_lim
        )

        w_cmd = self.prev_w + np.clip(
            w_cmd - self.prev_w,
            -self.dw_lim,
            self.dw_lim
        )

        self.prev_v = v_cmd
        self.prev_w = w_cmd

        v_cmd = float(np.clip(v_cmd, -self.max_v, self.max_v))
        w_cmd = float(np.clip(w_cmd, -self.max_w, self.max_w))
        
        w_cmd=gw*self.des_w      
        w_cmd=self.prev_w+np.clip(w_cmd-self.prev_w,-self.dw_lim,self.dw_lim)


        self.prev_v=v_cmd
        self.prev_w=w_cmd
        v_cmd=float(np.clip(v_cmd,-self.max_v,self.max_v))
        w_cmd=float(np.clip(w_cmd,-self.max_w,self.max_w))
        out=Twist()
        out.linear.y=v_cmd
        out.angular.z=w_cmd
        self.pub.publish(out)

        # ---------- Linear adaptation ----------
        if abs(self.des_v) > self.dead_v and abs(self.v_actual) > self.dead_v:

            rv = np.clip(
                abs(self.des_v) / abs(self.v_actual),
                0.8,
                1.2
            )

            self.gain_v *= (
                1.0 +
                self.lr * (rv - 1.0)
            )

            self.gain_v = float(np.clip(
                self.gain_v,
                self.min_gain,
                self.max_gain
            ))

        # ---------- Angular adaptation ----------
        if abs(self.des_w) > self.dead_w and abs(self.w_actual) > self.dead_w:

            rw = np.clip(
                abs(self.des_w) / abs(self.w_actual),
                0.8,
                1.2
            )

            if self.des_w >= 0:

                self.gain_right *= (
                    1.0 +
                    self.lr * (rw - 1.0)
                )

                self.gain_right = float(np.clip(
                    self.gain_right,
                    self.min_gain,
                    self.max_gain
                ))

            else:

                self.gain_left *= (
                    1.0 +
                    self.lr * (rw - 1.0)
                )

                self.gain_left = float(np.clip(
                    self.gain_left,
                    self.min_gain,
                    self.max_gain
                ))
        self.count+=1
        if self.count % 20 == 0:
            self.get_logger().info(
                f"Vd={self.des_v:.2f} "
                f"Vc={v_cmd:.2f} "
                f"Va={self.v_actual:.2f} "
                f"Rv={rv:.2f} "
                f"Gv={self.gain_v:.3f} | "
                f"Wd={self.des_w:.2f} "
                f"Wc={w_cmd:.2f} "
                f"Wa={self.w_actual:.2f} "
                f"Rw={rw:.2f} "
                f"GL={self.gain_left:.3f} "
                f"GR={self.gain_right:.3f}"
            )

def main(args=None):
    rclpy.init(args=args)
    node=AdaptiveVelocityController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()
