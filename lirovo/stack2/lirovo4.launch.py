# lirovo4.launch.py -- copy of lirovo3.launch.py (same nodes and static TFs) with three changes:
#   * Nav2 starts only after the first odometry message (/genz/odometry or /mavros/local_position/odom)
#     plus nav2_delay seconds. Started together with GenZ (lirovo3), the costmaps got clouds before any
#     map->base_link existed; after that first "earlier than all the data in the transform cache" drop
#     the STVL subscription thread of a costmap hung for good (2026-10-18: its toggle_enabled service
#     timed out while get_state answered) and the costmap stayed empty.
#   * the GenZ pipeline gets use_sim_time:=false (its launch file defaults to true, which left the
#     deskewer clock at 0 so no cloud was ever processed).
#   * Nav2 reads nav2_params4.yaml (= nav2_params3 with the stvl_layer transform_tolerance 0.3 -> 1.0).
#   * nav2_nodes:=costmaps runs only controller_server (local_costmap) and planner_server
#     (global_costmap), each in its own process, with their own lifecycle manager; cmd_vel goes to
#     /costmap_test/cmd_vel_unused so nothing can drive. For the costmap freeze tests (2026-10-18).
#
# lirovo3.launch.py for the uas_rover container -- copy of ~/vortex_ws/lirovo/launch/lirovo3.launch.py
# (laptop stack used by start_hardware_stack2.sh). Same nodes, same static TFs, same nav2_params3.
# Only differences, both because the files live next to this launch file instead of in the
# lirovo share directory (run it by path, no colcon rebuild of lirovo needed):
#   * nav2_params3.yaml is read from this folder;
#   * default_nav_through_poses_bt_xml is rewritten at launch time to this folder's
#     vortex_custom_polygon_tree.xml (the YAML still holds the laptop path /home/kreacher/...).
# Odometry source for map->base_link (odom_to_tf), launch argument odom_source:
#   mavros (default)  /mavros/local_position/odom -- the Cube's EKF; needs an EKF origin (GPS fix)
#   genz              /genz/odometry -- GenZ-ICP lidar odometry; its pipeline is started here too
#
#   ros2 launch /workspace/rover_ws/src/UAS-DTU-navigation-simulation/lirovo/stack2/lirovo3.launch.py
#   ros2 launch .../lirovo/stack2/lirovo4.launch.py odom_source:=genz

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction, RegisterEventHandler, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from nav2_common.launch import RewrittenYaml

HERE = os.path.dirname(os.path.realpath(__file__))

ODOM_TOPICS = {'mavros': '/mavros/local_position/odom', 'genz': '/genz/odometry'}


def odometry_nodes(context):
    """odom_to_tf on the chosen odometry topic, plus the GenZ pipeline when odom_source:=genz."""
    source = LaunchConfiguration('odom_source').perform(context).strip().lower()
    if source not in ODOM_TOPICS:
        raise RuntimeError(f"odom_source must be one of {sorted(ODOM_TOPICS)} (got '{source}')")
    topic = ODOM_TOPICS[source]
    actions = [
        LogInfo(msg=f'odom_source={source}: map->base_link from {topic}'),
        Node(
            package='lirovo',
            executable='odom_to_tf',
            name='odom_to_tf',
            output='screen',
            parameters=[{'use_sim_time': False}],
            # odom_to_tf hard-codes its input (/mavros/local_position/odom in this repo,
            # /genz/odometry in the laptop's vortex_ws): remap whichever one it is.
            remappings=[(t, topic) for t in ODOM_TOPICS.values() if t != topic],
        ),
    ]
    if source == 'genz':
        try:
            pipeline = os.path.join(get_package_share_directory('genz_icp'), 'launch',
                                    'bf_lidar_genz_pipeline.launch.py')
        except PackageNotFoundError:
            pipeline = ''
        if os.path.exists(pipeline):
            # 3 s: GenZ must not start before the base_link->lidar static TF exists (it diverged
            # to roll 63 deg when it did, 2026-10-16).
            actions.append(TimerAction(period=3.0, actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(pipeline),
                launch_arguments={'visualize': 'false', 'use_sim_time': 'false'}.items())]))
        else:
            actions.append(LogInfo(msg='odom_source=genz but genz_icp has no bf_lidar_genz_pipeline.launch.py: '
                                       'start GenZ-ICP yourself (it must publish /genz/odometry)'))
    return actions


def costmap_servers(nav2_params):
    """controller_server + planner_server only (they hold the local and global costmap)."""
    return [
        Node(package='nav2_controller', executable='controller_server', name='controller_server',
             output='screen', parameters=[nav2_params],
             remappings=[('cmd_vel', '/costmap_test/cmd_vel_unused')]),
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             output='screen', parameters=[nav2_params]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_costmaps', output='screen',
             parameters=[{'autostart': True, 'bond_timeout': 0.0,
                          'node_names': ['controller_server', 'planner_server']}]),
    ]


def gated_nav2(context, nav2_launch, nav2_params):
    """Start Nav2 nav2_delay seconds after the first message on the odometry topic."""
    topic = ODOM_TOPICS[LaunchConfiguration('odom_source').perform(context).strip().lower()]
    delay = float(LaunchConfiguration('nav2_delay').perform(context))
    mode = LaunchConfiguration('nav2_nodes').perform(context).strip().lower()
    if mode not in ('full', 'costmaps'):
        raise RuntimeError(f"nav2_nodes must be 'full' or 'costmaps' (got '{mode}')")
    nav2_actions = [nav2_launch] if mode == 'full' else costmap_servers(nav2_params)
    wait_odom = ExecuteProcess(
        cmd=['ros2', 'topic', 'echo', '--once', '--field', 'header.stamp', topic, 'nav_msgs/msg/Odometry'],
        name='wait_for_odometry', output='log')
    return [
        LogInfo(msg=f'Nav2 waits for the first {topic} message, then {delay:g} s'),
        wait_odom,
        RegisterEventHandler(OnProcessExit(target_action=wait_odom, on_exit=[
            LogInfo(msg=f'{topic} is up: starting Nav2 ({mode}) in {delay:g} s'),
            TimerAction(period=delay, actions=nav2_actions)])),
    ]


def generate_launch_description():
    namePackage = 'lirovo'
    patchwork_params_path = os.path.join(get_package_share_directory(namePackage), 'config',
                                         'patchwork_plusplus_blickfeld.yaml')
    nav2_params = RewrittenYaml(
        source_file=os.path.join(HERE, 'nav2_params4.yaml'),
        param_rewrites={'default_nav_through_poses_bt_xml':
                        os.path.join(HERE, 'vortex_custom_polygon_tree.xml')},
        convert_types=True)

    pkg_nav2_dir = get_package_share_directory('nav2_bringup')
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'params_file': nav2_params,
            'autostart': 'True',
            'map': 'map',
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument('odom_source', default_value='mavros',
                              description='map->base_link source: mavros (/mavros/local_position/odom) '
                                          'or genz (/genz/odometry, starts the GenZ-ICP pipeline)'),
        DeclareLaunchArgument('nav2_nodes', default_value='full',
                              description='full (nav2_bringup) or costmaps (controller + planner server only)'),
        DeclareLaunchArgument('nav2_delay', default_value='3.0',
                              description='seconds between the first odometry message and the Nav2 start'),
        SetParameter(name='use_sim_time', value=False),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['-0.17', '0.0', '0.54', '0', '0', '0', 'base_link', 'lidar'],
            parameters=[{'use_sim_time': False}],
            name='static_tf_lidar'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.29', '0.0', '0.16', '0', '0', '0', 'base_link', 'camera'],
            parameters=[{'use_sim_time': False}],
            name='static_tf_camera'
        ),
        OpaqueFunction(function=odometry_nodes),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.29', '0.0', '0.15', '0', '0', '0', 'base_link', 'sf45b'],
            parameters=[{'use_sim_time': False}],
            name='static_tf_sf45b'
        ),
        Node(
            package='patchwork_plusplus',
            executable='patchwork_plusplus_exe',
            name='patchwork_plusplus',
            output='screen',
            parameters=[patchwork_params_path, {'use_sim_time': False}],
            remappings=[
                ('cloud', '/patchwork/cloud'),
                ('ground', '/patchwork/ground'),
                ('nonground', '/patchwork/nonground'),
            ],
        ),
        OpaqueFunction(function=gated_nav2, args=[nav2_launch, nav2_params]),
    ])
