#!/usr/bin/env python3
"""
Relays cmd_vel (Twist, ROS's usual ENU/FLU convention - same message Gazebo's diff_drive
plugin consumes) into mavros' setpoint_velocity plugin's cmd_vel_unstamped topic. MAVROS
itself does the ENU->NED/FLU->FRD conversion before sending SET_POSITION_TARGET_LOCAL_NED to
the FCU - the same conversion a real rover's MAVROS would perform, so this drives the SITL
vehicle through the actual frame-conversion code path being validated, not a shortcut around
it.

Requires the vehicle to be ARMED and in GUIDED mode (ArduRover ignores velocity setpoints in
MANUAL/HOLD) - this node doesn't do that itself, do it once via the mavros arming/set_mode
services (see gazebo_sitl_demo.launch.py's docstring or just call them directly).
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelToMavros(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_mavros')
        self.pub = self.create_publisher(Twist, 'cmd_vel_out', 10)
        self.sub = self.create_subscription(Twist, 'cmd_vel_in', self.callback, 10)

    def callback(self, msg):
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToMavros()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
