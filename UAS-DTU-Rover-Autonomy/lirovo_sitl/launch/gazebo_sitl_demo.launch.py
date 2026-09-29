"""
Full two-rover demo: bcr_bot in Gazebo (visuals, physics, the simulated 3D lidar) driven by
ArduRover SITL + MAVROS (real odom/imu source) instead of Gazebo's own ground-truth odom -
this is the "wire bcr_bot to SITL, use mavros as odom" setup.

Reuses everything that already exists rather than reimplementing it - nothing in `bcr_bot` or
`lirovo` is edited by this file:
  - bcr_bot/launch/gazebo.launch.py (empty world) + bcr_bot_gazebo_spawn.launch.py x2 (the
    actual robot models/physics/lidar - unchanged from the normal Gazebo-only demo)
  - lirovo_sitl/launch/sitl.launch.py + mavros.launch.py (this package, already built/tested)
  - lirovo's own nav2 fork (bringup_launch_shared_tf.py), its odom_relay/peer_obstacle_relay
    executables, and bcr_bot's remapper.py (cmd_vel bridge) - all included/run as they already
    exist, just pointed at different topics via remappings/launch args.

THE ONE SWAP: lirovo.launch.py's own odom_relay passes through bcr_bot's Gazebo ground-truth
odom (<ns>/odom). Here, odom_relay's `odom_in` is remapped to <ns>/mavros/odom instead - so
nav2's velocity/pose feedback (bt_navigator's odom_topic) now genuinely comes from MAVROS/
SITL, not Gazebo's ground truth. Gazebo still owns the physical TF chain (odom->base_footprint
via its diff-drive plugin) and the visuals/lidar - only the *data feeding nav2* changes source.
Caveat: mavros/odom's frame_id is "map"/"base_link" (unprefixed) rather than this project's
usual "<ns>/odom"/"<ns>/base_footprint" - harmless for nav2 (it only reads the twist/pose
values here, TF is unaffected), but worth knowing if you consume this topop elsewhere.

Costmap only - no real map.yaml, no SLAM (use_localization:=False), same as every other
launch file in this project so far.
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PythonExpression
from launch_ros.actions import Node, SetParameter
from nav2_common.launch import ReplaceString
from ament_index_python.packages import get_package_share_directory


def rover_bringup_nodes(namespace, peer_namespace, nav2_params_path):
    """Everything one rover needs on top of its already-spawned bcr_bot Gazebo model +
    already-running SITL/mavros instance: laserscan conversion, map->odom static TF,
    mavros-sourced odom relay, peer obstacle relay, cmd_vel bridge, and its own nav2 stack."""

    nav2_params_namespaced = ReplaceString(
        source_file=nav2_params_path,
        replacements={'<robot_ns_noslash>': namespace},
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('lirovo'), 'launch',
                         'bringup_launch_shared_tf.py')
        ),
        launch_arguments={
            'params_file': nav2_params_namespaced,
            'autostart': 'True',
            'map': 'map',
            'use_localization': 'False',
            'namespace': namespace,
            'use_namespace': 'True',
        }.items(),
    )

    return [
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name=PythonExpression(["'pointcloud_to_laserscan_' + '", namespace, "'"]),
            parameters=[{
                'target_frame': PythonExpression(['"', namespace, '/base_link"']),
                'transform_tolerance': 0.05,
                'min_height': 0.2,
                'max_height': 1.0,
                'angle_min': -0.6293,
                'angle_max': 0.6293,
                'angle_increment': 0.00872665,
                'scan_time': 0.1,
                'range_min': 1.5,
                'range_max': 41.0,
                'use_inf': True,
                'inf_epsilon': 1.0,
                'queue_size': 50,
                'use_sim_time': True,
            }],
            remappings=[
                ('cloud_in', PythonExpression(['"/', namespace, '/livox/lidar"'])),
                ('scan', PythonExpression(['"/', namespace, '/scan"'])),
            ],
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0',
                       PythonExpression(['"', namespace, '/map"']),
                       PythonExpression(['"', namespace, '/odom"'])],
            name=PythonExpression(["'static_tf_odom_' + '", namespace, "'"]),
        ),

        # THE SWAP - see module docstring. mavros_odom_relay (this package) rather than
        # lirovo's own odom_relay - MAVROS's local_position/odom publisher uses best-effort
        # QoS, and lirovo's odom_relay subscribes with the plain reliable default, which
        # silently drops every message ("incompatible QoS... RELIABILITY"); confirmed live
        # with this exact stack. mavros_odom_relay.py subscribes best-effort explicitly.
        Node(
            package='lirovo_sitl',
            executable='mavros_odom_relay',
            name=PythonExpression(["'odom_relay_mavros_' + '", namespace, "'"]),
            output='screen',
            remappings=[
                ('odom_in', PythonExpression(['"/', namespace, '/mavros/odom"'])),
                ('odom_out', PythonExpression(['"/', namespace, '/odometry/filtered"'])),
            ],
        ),

        Node(
            package='lirovo',
            executable='peer_obstacle_relay',
            name=PythonExpression(["'peer_obstacle_relay_' + '", namespace, "'"]),
            output='screen',
            parameters=[{'publish_rate_hz': 5.0}],
            remappings=[
                ('scan_in', PythonExpression(['"/', namespace, '/scan"'])),
                ('scan_out', PythonExpression(['"/', peer_namespace, '/obstacles_in"'])),
            ],
        ),

        Node(
            package='bcr_bot',
            executable='remapper.py',
            name=PythonExpression(["'remapper_' + '", namespace, "'"]),
            output='screen',
            parameters=[{'target_robot_namespace': namespace}],
        ),

        nav2_launch,
    ]


def generate_launch_description():
    pkg_bcr_bot = get_package_share_directory('bcr_bot')
    pkg_lirovo_sitl = get_package_share_directory('lirovo_sitl')
    nav2_params_path = os.path.join(get_package_share_directory('lirovo'), 'config',
                                     'nav2_params.yaml')

    gazebo_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bcr_bot, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'spawn_robot': 'false'}.items(),
    )

    spawn_rover1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bcr_bot, 'launch', 'bcr_bot_gazebo_spawn.launch.py')
        ),
        launch_arguments={
            'robot_namespace': 'rover1',
            'position_x': '0.0',
            'position_y': '0.0',
            'use_sim_time': 'true',
        }.items(),
    )
    spawn_rover2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bcr_bot, 'launch', 'bcr_bot_gazebo_spawn.launch.py')
        ),
        launch_arguments={
            'robot_namespace': 'rover2',
            'position_x': '0.0',
            'position_y': '5.0',
            'use_sim_time': 'true',
        }.items(),
    )

    sitl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_lirovo_sitl, 'launch', 'sitl.launch.py')
        ),
    )
    mavros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_lirovo_sitl, 'launch', 'mavros.launch.py')
        ),
    )

    rviz_config = os.path.join(get_package_share_directory('bcr_bot'), 'rviz',
                                'two_rover_view.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),

        gazebo_world,
        sitl_launch,

        # Robots need the Gazebo world up first; SITL boots independently in parallel.
        TimerAction(period=4.0, actions=[spawn_rover1, spawn_rover2]),

        # mavros.launch.py has its own internal 15s/20s delays before connecting - starting
        # its include at t=4 puts the actual mavros_node starts at ~19s/24s, comfortably after
        # SITL's heartbeats are flowing (confirmed ~5-8s boot time earlier this session).
        TimerAction(period=4.0, actions=[mavros_launch]),

        # Everything above needs to be fully up (Gazebo models spawned, mavros connected)
        # before nav2/odom_relay/remapper start consuming their topics.
        TimerAction(period=30.0, actions=[
            *rover_bringup_nodes('rover1', 'rover2', nav2_params_path),
            *rover_bringup_nodes('rover2', 'rover1', nav2_params_path),
        ]),

        TimerAction(period=34.0, actions=[rviz]),
    ])
