#! /usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import Twist


def _peek_target_robot_namespace(argv, default='bcr_bot'):
    """Read target_robot_namespace out of raw argv before the Node exists, so the default
    rclpy node name (used when this script is run directly, not through a launch file that
    overrides the name via a __node:= remap) can reflect it too. Only handles the
    '-p target_robot_namespace:=value' / '--param target_robot_namespace:=value' forms; the
    declared ROS parameter is still the source of truth once the node is constructed."""
    for i, arg in enumerate(argv):
        for flag in ('-p', '--param'):
            if arg == flag and i + 1 < len(argv):
                key, sep, value = argv[i + 1].partition(':=')
                if sep and key == 'target_robot_namespace':
                    return value
    return default


class CmdVelRemapper(Node):
    def __init__(self):
        # target_robot_namespace is the Gazebo robot_namespace the bcr_bot instance was
        # spawned with (e.g. "rover1") - its diff-drive plugin only listens on
        # "<target_robot_namespace>/cmd_vel", not plain "cmd_vel". Default "bcr_bot" matches
        # bcr_bot_gazebo_spawn.launch.py's own default so single-robot use is unchanged.
        # When launched via lirovo.launch.py, the node's actual name is already overridden by
        # a launch-time __node:= remap; this peek only matters for direct `ros2 run` usage.
        namespace_hint = _peek_target_robot_namespace(sys.argv)
        super().__init__(f'cmd_vel_remapper_{namespace_hint}')

        self.declare_parameter('target_robot_namespace', 'bcr_bot')
        target_robot_namespace = self.get_parameter('target_robot_namespace').get_parameter_value().string_value

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )

        # Create a subscriber for the cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            qos_profile
        )
        self.subscription  # Prevent unused variable warning

        # Create a publisher for the "<target_robot_namespace>/cmd_vel" topic
        self.publish_topic = f'{target_robot_namespace}/cmd_vel'
        self.publisher = self.create_publisher(Twist, self.publish_topic, qos_profile)

    def cmd_vel_callback(self, msg):
        # Republish the received message to the target robot's cmd_vel topic
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    cmd_vel_remapper = CmdVelRemapper()

    try:
        rclpy.spin(cmd_vel_remapper)
    except KeyboardInterrupt:
        pass
    finally:
        cmd_vel_remapper.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

