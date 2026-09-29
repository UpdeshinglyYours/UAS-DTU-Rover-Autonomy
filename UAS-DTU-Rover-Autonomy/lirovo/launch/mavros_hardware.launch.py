"""
Namespaced mavros_node for real hardware. Created 2026-08-30 specifically to stop hand-typing/
pasting the long `ros2 run mavros mavros_node --ros-args -r __ns:=... -p ...` command each time
(kept getting mangled by line-wrapping when pasted) - this replaces all of that with one short
`ros2 launch lirovo mavros_hardware.launch.py robot_namespace:=rover1 fcu_url:=...` call.

Loads mavros/mavros/launch/apm_config.yaml and apm_pluginlists.yaml (this workspace's own,
built earlier tonight - not the /opt/ros/humble stock copy, verify with
`ros2 pkg prefix mavros` if anything looks off). apm_config.yaml's frame_id/tf.frame_id/
tf.child_frame_id fields use the <robot_ns_noslash> placeholder token (this project's own
convention, same as ekf_localization.yaml) - substituted here via ReplaceString before mavros
ever sees the file, exactly like lirovo_hardware.launch.py already does for its own config
files.

local_position.tf.send is TRUE in apm_config.yaml (flipped from the stock false), broadcasting
<ns>/odom -> <ns>/base_link - this OVERLAPS with EKF's own odom->base_link broadcast in
lirovo_hardware.launch.py if both run at once. Don't run this alongside a live EKF fusing real
mavros data unless you mean for mavros to be the one owning that TF instead of EKF - they will
fight over the same frame pair otherwise. Fine to run standalone (no EKF) for testing, which is
what this was built for tonight.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import SetRemap
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from nav2_common.launch import ReplaceString
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    robot_namespace = LaunchConfiguration('robot_namespace')
    fcu_url = LaunchConfiguration('fcu_url')
    tgt_system = LaunchConfiguration('tgt_system')
    use_sim_time = LaunchConfiguration('use_sim_time')

    pkg_mavros_dir = get_package_share_directory('mavros')
    apm_config_path = os.path.join(pkg_mavros_dir, 'launch', 'apm_config.yaml')
    apm_pluginlists_path = os.path.join(pkg_mavros_dir, 'launch', 'apm_pluginlists.yaml')

    apm_config_namespaced = ReplaceString(
        source_file=apm_config_path,
        replacements={'<robot_ns_noslash>': robot_namespace},
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'robot_namespace',
            default_value='rover1',
            description='This rover\'s namespace, e.g. "rover1" - must match whatever '
                        'lirovo_hardware.launch.py is using for the same rover.'
        ),
        DeclareLaunchArgument(
            'tgt_system',
            default_value='1',
            description='MAVLink system id of THIS rover\'s FCU (ArduPilot SYSID_THISMAV). '
                        'Only used to build the /uas<N> MAVLink bus topic names for the remap '
                        'below - the remap is what actually separates the two rovers, so this '
                        'can stay at 1 for both. Change it only if you also change the FCU\'s '
                        'SYSID_THISMAV, and keep the two in sync or the remap will target a '
                        'bus that does not exist.'
        ),
        DeclareLaunchArgument(
            'fcu_url',
            default_value='udp://127.0.0.1:14550@127.0.0.1:14557',
            description='mavros fcu_url - swap for the real pixhawk connection string, e.g. '
                        '"serial:///dev/ttyACM0:57600" once hardware is connected (verify the '
                        'actual device path live with `ls /dev/ttyACM*` first).'
        ),
        # 2026-09-08: same treatment as lirovo_hardware.launch.py - default True requires a
        # bag publishing /<robot_namespace>/clock (matching whatever robot_namespace this
        # instance uses), or mavros' own timers/time-sync plugin freeze at time zero. Pass
        # use_sim_time:=false for real-FCU runs with no bag.
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Requires a bag publishing /<robot_namespace>/clock or mavros freezes '
                        'at time zero. Pass use_sim_time:=false for real-FCU runs with no bag.'
        ),

        # Absolute topic (leading slash), same reason /uas<N> needs its own explicit remaps
        # below instead of relying on __ns:= - must be remapped per-node here too so mavros
        # actually follows THIS rover's bag clock, not a shared/plain /clock.
        SetRemap(src='/clock', dst=PythonExpression(['"/', robot_namespace, '/clock"'])),

        Node(
            package='mavros',
            executable='mavros_node',
            namespace=robot_namespace,
            # 2026-08-30: name='mavros' REMOVED. launch turns it into "-r __node:=mavros",
            # which in ROS 2 is a GLOBAL remap rule - it renames every node the process
            # creates, and each mavros plugin is a real node (plugin.hpp:86,
            # Node::make_shared(subnode, parent_fqn)). With it set, /<ns>/imu,
            # /<ns>/local_position, /<ns>/global_position, /<ns>/sys ... ALL collapsed onto
            # the single FQN /<ns>/mavros. Two things broke as a result:
            #   1. apm_config.yaml's per-plugin keys (/<ns>/imu:, /<ns>/local_position:)
            #      matched nothing, so every frame_id stayed at its stock un-namespaced
            #      default - verified live: imu messages were stamped "base_link", not
            #      "rover2/base_link", which makes the EKF drop them AND collides across
            #      rovers. It cannot be fixed by re-keying to /<ns>/mavros: either, because
            #      imu/local_position/global_position each declare their OWN "frame_id"
            #      (imu.cpp:82, local_position.cpp:62, global_position.cpp:71) needing three
            #      DIFFERENT values - one merged key can only hold one.
            #   2. topics flattened: "~/data" (imu.cpp:111) landed at <ns>/mavros/data
            #      instead of <ns>/mavros/imu/data.
            # Without it, plugins keep their constructor names -> /<ns>/imu, /<ns>/local_position
            # (the layout apm_config.yaml is already keyed for). The UAS node is still named
            # "mavros" by default, so nothing else needs renaming.
            output='screen',
            # 2026-08-30: THE ROUTE from nav2 to the FCU. nav2's velocity_smoother publishes
            # geometry_msgs/Twist on the namespaced "<ns>/cmd_vel"
            # (navigation_launch_shared_tf.py remaps cmd_vel_smoothed -> cmd_vel), while
            # mavros' setpoint_velocity plugin listens on its OWN node's
            # "~/cmd_vel_unstamped" = <ns>/setpoint_velocity/cmd_vel_unstamped
            # (setpoint_velocity.cpp:72-73). Nothing connected the two, so the rover planned
            # and never moved. Both names are written ABSOLUTELY on purpose: launch remap
            # rules are process-global (they hit every mavros plugin sub-node), so an
            # unqualified rule could match more than intended - the fully-qualified form
            # touches exactly this one topic.
            # NOTE cmd_vel_unstamped (Twist), NOT cmd_vel (TwistStamped) - nav2 publishes the
            # unstamped type. Twist has no header, so there is no frame_id to namespace here;
            # the velocity is body-frame by definition.
            #
            # 2026-08-30 BLOCKER FIX - the MAVLink bus. mavros_node.cpp:57 builds
            #     uas_url = format("/uas%d", tgt_system)
            # with a LEADING SLASH, so it is absolute and completely immune to __ns. The
            # router publishes/subscribes <uas_url>/mavlink_source|sink
            # (mavros_router.cpp:493-498) and the UAS node subscribes <uas_url>/mavlink_source
            # (mavros_uas.cpp:366-368). Both rovers default to tgt_system=1, so BOTH sat on
            # /uas1 - confirmed live, `ros2 topic list` showed a bare /uas1/mavlink_source and
            # /uas1/mavlink_sink with no namespace. Every raw MAVLink frame from rover1 was
            # therefore decoded by rover2's plugins and vice versa, undetectably.
            #
            # Remapping BOTH endpoints works because the router and the UAS node live in the
            # SAME process (mavros_node.cpp:66 and :83 are added to one executor), and launch
            # remap rules are process-global - so both ends move together and stay paired.
            #
            # Do NOT try to fix this by setting uas_url in a params file instead: the router's
            # "uas_urls" is overwritten programmatically by set_parameters() at
            # mavros_node.cpp:77-79, AFTER the params file is applied, so the router and the
            # UAS node would end up on different topics and the link would go dead.
            remappings=[
                (PythonExpression(
                    ['"/', robot_namespace, '/setpoint_velocity/cmd_vel_unstamped"']),
                 PythonExpression(['"/', robot_namespace, '/cmd_vel"'])),
                (PythonExpression(['"/uas', tgt_system, '/mavlink_source"']),
                 PythonExpression(
                     ['"/', robot_namespace, '/uas', tgt_system, '/mavlink_source"'])),
                (PythonExpression(['"/uas', tgt_system, '/mavlink_sink"']),
                 PythonExpression(
                     ['"/', robot_namespace, '/uas', tgt_system, '/mavlink_sink"'])),
            ],
            parameters=[
                apm_pluginlists_path,
                apm_config_namespaced,
                {'fcu_url': fcu_url},
                # int, not str - mavros_node.cpp:35,42 declares tgt_system as an integer, and a
                # LaunchConfiguration is always a string. Passed the same way fcu_url is (which
                # is likewise declared on mavros_node and demonstrably reaches it), so the value
                # the remap above is built from cannot drift from the one mavros actually uses.
                {'tgt_system': ParameterValue(tgt_system, value_type=int)},
                {'use_sim_time': use_sim_time},
            ],
        ),
    ])
