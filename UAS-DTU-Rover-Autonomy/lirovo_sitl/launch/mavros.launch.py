"""
Two namespaced mavros_node instances, one per SITL vehicle brought up by sitl.launch.py in
this same package. Namespacing (rover1/rover2) mirrors lirovo's own convention so the topic
names this produces (e.g. /rover1/mavros/local_position/odom) are directly comparable to
what lirovo_hardware.launch.py and ekf_localization.yaml already assume.

fcu_url connects to each SITL instance's default MAVLink output port. sim_vehicle.py (this
ArduPilot checkout) only opens ONE default "--out" port per instance, at 14550 + 10*instance
(checked directly in Tools/autotest/sim_vehicle.py - there is no separate companion-computer
port unless you pass extra --out flags yourself):
  rover1 (SITL instance 0) -> udp://127.0.0.1:14550@127.0.0.1:14555
  rover2 (SITL instance 1) -> udp://127.0.0.1:14560@127.0.0.1:14565
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction


def generate_launch_description():
    # Several stock mavros plugins collide with each other on generic shared topic names
    # (cellular_status/sys_status both fighting over "~/status", companion_process_status/
    # debug_value both fighting over "~/send", ...) - each collision throws and kills the
    # whole node on startup ("invalid allocator, at ./src/rcl/subscription.c:219"), and
    # denylisting them one crash at a time is a losing game. Allowlist just what's needed:
    # sys_status (state/heartbeat, and SetMode), imu/local_position/global_position (the
    # odom/imu topic-mapping check this package started as), command (arming), and
    # setpoint_velocity - THE plugin that matters for the sim-to-real check: it converts a
    # ROS Twist (ENU/FLU, cmd_vel's normal convention) into MAVLink's NED/FRD
    # SET_POSITION_TARGET_LOCAL_NED before sending it to the FCU - the exact same conversion
    # code path a real rover's MAVROS would run, so driving SITL through it here is a real
    # test of the ENU<->NED handling, not just a topic-name check.
    common_params = {
        'plugin_allowlist': ['sys_status', 'imu', 'local_position', 'global_position',
                              'command', 'setpoint_velocity'],
    }

    rover1_mavros = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',
        namespace='rover1',
        output='screen',
        parameters=[{
            # remote host must be explicit - leaving it blank makes mavros try to send its
            # outgoing heartbeats to a broadcast address on a loopback-bound socket, which
            # throws "sendto: Invalid argument" and the link never comes up.
            'fcu_url': 'udp://127.0.0.1:14550@127.0.0.1:14555',
            'target_system_id': 1,
            'target_component_id': 1,
            **common_params,
        }],
    )

    rover2_mavros = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',
        namespace='rover2',
        output='screen',
        parameters=[{
            'fcu_url': 'udp://127.0.0.1:14560@127.0.0.1:14565',
            'target_system_id': 2,
            'target_component_id': 1,
            **common_params,
        }],
    )

    # SITL needs time to actually come up and start speaking MAVLink before mavros_node's
    # connection attempts will succeed - sitl.launch.py's rover2 is itself delayed 5s, so
    # give the whole thing a healthy head start.
    return LaunchDescription([
        TimerAction(period=15.0, actions=[rover1_mavros]),
        TimerAction(period=20.0, actions=[rover2_mavros]),
    ])
