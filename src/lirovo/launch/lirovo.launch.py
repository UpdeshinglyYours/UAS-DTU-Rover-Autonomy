import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    package_share = get_package_share_directory('lirovo')
    nav2_params_path = os.path.join(
        package_share, 'config', 'nav2_params.yaml'
    )
    dtu_nav2_params_path = os.path.join(
        package_share, 'config', 'nav2_dtu_rtk_params.yaml'
    )
    dtu_rtk_localization_path = os.path.join(
        package_share, 'config', 'dtu_rtk_localization.yaml'
    )
    dtu_rtk_gate_path = os.path.join(
        package_share, 'config', 'dtu_rtk_gate.yaml'
    )
    slam_params_path = os.path.join(
        package_share, 'config', 'slam_params.yaml'
    )
    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    dtu_prior_map_share = get_package_share_directory('dtu_prior_map')
    dtu_navsat_datum_path = os.path.join(
        dtu_prior_map_share, 'config', 'navsat_datum.yaml'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')
    global_localization = LaunchConfiguration('global_localization')
    dtu_nav2_autostart = LaunchConfiguration('dtu_nav2_autostart')
    enable_centerline_tether = LaunchConfiguration('enable_centerline_tether')
    cost_travel_multiplier = LaunchConfiguration('cost_travel_multiplier')

    slam_mode = PythonExpression(
        ["'", global_localization, "'.lower() == 'slam'"]
    )
    dtu_rtk_mode = PythonExpression(
        ["'", global_localization, "'.lower() == 'dtu_rtk'"]
    )
    reference_route_enabled = PythonExpression(
        [
            "'",
            global_localization,
            "'.lower() == 'dtu_rtk' and '",
            enable_centerline_tether,
            "'.lower() == 'true'",
        ]
    )

    configured_dtu_nav2_params = RewrittenYaml(
        source_file=dtu_nav2_params_path,
        param_rewrites={
            (
                'global_costmap.global_costmap.ros__parameters.'
                'road_preference_filter.enabled'
            ): enable_centerline_tether,
            (
                'planner_server.ros__parameters.GridBased.'
                'cost_travel_multiplier'
            ): cost_travel_multiplier,
        },
        convert_types=True,
    )

    arguments = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'global_localization',
            default_value='slam',
            choices=['slam', 'dtu_rtk'],
            description=(
                "Global localization owner: 'slam' preserves the existing "
                "pipeline; 'dtu_rtk' uses RTK against the DTU prior map."
            ),
        ),
        DeclareLaunchArgument(
            'dtu_nav2_autostart',
            default_value='false',
            description=(
                'Activate DTU Nav2 lifecycle nodes automatically. Keep false '
                'until RTK quality, heading, and TF ownership are verified.'
            ),
        ),
        DeclareLaunchArgument(
            'enable_centerline_tether',
            default_value='true',
            description='Apply the DTU centreline mask in dtu_rtk mode.',
        ),
        DeclareLaunchArgument(
            'cost_travel_multiplier',
            default_value='3.0',
            description='SmacPlanner2D traversal-cost weight in dtu_rtk mode.',
        ),
        DeclareLaunchArgument(
            'input_cloud_topic', default_value='/bf_lidar/point_cloud_deskewed'
        ),
        DeclareLaunchArgument('output_scan_topic', default_value='/scan'),
    ]

    slam_nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': nav2_params_path,
            'autostart': 'true',
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(slam_mode),
    )

    dtu_nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': configured_dtu_nav2_params,
            'autostart': dtu_nav2_autostart,
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(dtu_rtk_mode),
    )

    base_footprint_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_basefootprint',
        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0',
            '--roll', '0',
            '--pitch', '0',
            # GenZ base_link is already REP-103 (+X forward, +Y left).
            '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'base_footprint',
        ],
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool)
        }],
    )

    gps_antenna_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_gps_antenna',
        arguments=[
            '--x', '0.10',
            '--y', '0.05',
            '--z', '0.40',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '0',
            '--frame-id', 'base_footprint',
            '--child-frame-id', 'gps_antenna',
        ],
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool)
        }],
        condition=IfCondition(dtu_rtk_mode),
    )

    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        parameters=[{
            'target_frame': 'base_link',
            'transform_tolerance': 0.05,
            'min_height': 0.3,
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
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
        }],
        remappings=[
            ('cloud_in', LaunchConfiguration('input_cloud_topic')),
            ('scan', LaunchConfiguration('output_scan_topic')),
        ],
    )

    local_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(package_share, 'config', 'localization.yaml'),
            {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
        ],
    )

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_path,
            {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
        ],
        condition=IfCondition(slam_mode),
    )

    global_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_map',
        output='screen',
        parameters=[
            dtu_rtk_localization_path,
            {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
        ],
        remappings=[
            ('odometry/filtered', '/odometry/global'),
            ('set_pose', '/ekf_filter_node_map/set_pose'),
        ],
        condition=IfCondition(dtu_rtk_mode),
    )

    manual_global_alignment = Node(
        package='lirovo',
        executable='manual_global_alignment',
        name='manual_global_alignment',
        output='screen',
        parameters=[
            dtu_rtk_gate_path,
            {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
        ],
        condition=IfCondition(dtu_rtk_mode),
    )

    rtk_fix_gate = Node(
        package='lirovo',
        executable='rtk_fix_gate',
        name='rtk_fix_gate',
        output='screen',
        parameters=[
            dtu_rtk_gate_path,
            {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
        ],
        condition=IfCondition(dtu_rtk_mode),
    )

    navsat_transform = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        parameters=[
            dtu_navsat_datum_path,
            {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
        ],
        remappings=[
            ('gps/fix', '/mavros/global_position/rtk_fixed_only'),
            ('odometry/filtered', '/odometry/global'),
            ('odometry/gps', '/odometry/gps'),
            ('gps/filtered', '/gps/filtered'),
            ('fromLL', '/navsat_transform/fromLL'),
        ],
        condition=IfCondition(dtu_rtk_mode),
    )

    dtu_maps = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                dtu_prior_map_share, 'launch', 'dtu_prior_map.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'enable_centerline_cost': enable_centerline_tether,
        }.items(),
        condition=IfCondition(dtu_rtk_mode),
    )

    reference_route = Node(
        package='dtu_prior_map',
        executable='dtu_reference_route_publisher.py',
        name='dtu_reference_route_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
        }],
        condition=IfCondition(reference_route_enabled),
    )

    delayed_slam_toolbox = TimerAction(period=1.0, actions=[slam_toolbox])
    delayed_slam_nav2 = TimerAction(period=3.0, actions=[slam_nav2_launch])
    delayed_dtu_nav2 = TimerAction(period=3.0, actions=[dtu_nav2_launch])
    return LaunchDescription(
        arguments
        + [
            base_footprint_transform,
            gps_antenna_transform,
            pointcloud_to_laserscan,
            local_ekf,
            dtu_maps,
            global_ekf,
            manual_global_alignment,
            rtk_fix_gate,
            navsat_transform,
            reference_route,
            delayed_slam_toolbox,
            delayed_slam_nav2,
            delayed_dtu_nav2,
        ]
    )
