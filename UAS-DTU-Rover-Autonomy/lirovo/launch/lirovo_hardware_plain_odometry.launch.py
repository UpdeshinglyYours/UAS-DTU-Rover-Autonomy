
"""
Variant of lirovo_hardware.launch.py that includes genz_icp/launch/odometry.launch.py
DIRECTLY (odometry.launch.py's own internal deskew:=true), instead of
bf_lidar_genz_pipeline.launch.py (the external imu_rotation_deskew_node pipeline that
lirovo_hardware.launch.py switched to on 2026-09-01). Created 2026-09-01 at the user's request
to KEEP both options available side by side rather than replacing one with the other.

Everything else in this file is an exact copy of lirovo_hardware.launch.py - same multi-robot
architecture (namespacing, peer_obstacle_relay, the shared-/tf nav2 fork, EKF, static TFs,
laserscan). Only the genz_icp_launch block differs. See lirovo_hardware.launch.py's own
docstring/comments for the full rationale behind everything else in this file - not repeated
here to avoid the two drifting into contradicting copies of the same explanation.

Assumes a `mavros` node is already running separately (mavros_hardware.launch.py, same
robot_namespace), same as lirovo_hardware.launch.py.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import SetParameter
from launch_ros.actions import SetRemap
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import TimerAction
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch.conditions import IfCondition
from nav2_common.launch import ReplaceString
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    namePackage = 'lirovo'
    nav2_params_path = os.path.join(get_package_share_directory(namePackage), 'config', 'nav2_params.yaml')
    ekf_params_path = os.path.join(get_package_share_directory(namePackage), 'config', 'ekf_localization.yaml')

    robot_namespace = LaunchConfiguration('robot_namespace')
    peer_robot_namespace = LaunchConfiguration('peer_robot_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    lidar_topic = PythonExpression(['"/', robot_namespace, '/point_cloud_out"'])

    nav2_params_path_namespaced = ReplaceString(
        source_file=nav2_params_path,
        replacements={'<robot_ns_noslash>': robot_namespace},
    )

    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    rviz_config_path = PathJoinSubstitution(
        [get_package_share_directory(namePackage), 'rviz',
         PythonExpression(['"', rviz_config, '.rviz"'])])
    rviz_config_namespaced = ReplaceString(
        source_file=ReplaceString(
            source_file=rviz_config_path,
            replacements={'<robot_ns_noslash>': robot_namespace},
        ),
        replacements={'<robot_namespace>': ('/', robot_namespace)},
    )

    ekf_params_path_namespaced = ReplaceString(
        source_file=ReplaceString(
            source_file=ekf_params_path,
            replacements={'<robot_ns_noslash>': robot_namespace},
        ),
        replacements={'<robot_namespace>': ('/', robot_namespace)},
    )

    pkg_nav2_dir = get_package_share_directory('nav2_bringup')
    pkg_lirovo_dir = get_package_share_directory('lirovo')
    pkg_genz_icp_dir = get_package_share_directory('genz_icp')

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_lirovo_dir, 'launch', 'bringup_launch_shared_tf.py')
        ),
        launch_arguments={
            'params_file': nav2_params_path_namespaced,
            'autostart': 'True',
            'map': 'map',
            'use_localization': 'False',
            'namespace': robot_namespace,
            'use_namespace': 'True',
        }.items()
    )
    delayed_nav2_launch = TimerAction(period=3.0, actions=[nav2_launch])

    # PLAIN odometry.launch.py include (not bf_lidar_genz_pipeline.launch.py) - relies on
    # odometry.launch.py's own internal deskew:=true, no separate imu_rotation_deskew_node.
    # genz_icp reads the real lidar's own native pointcloud directly - deskew:=true works
    # correctly here because the real driver actually populates a genuine per-point timestamp
    # field (point_time_offset - see genz-icp/ros/ros2/Utils.hpp), unlike bcr_bot's simulated
    # sensor which needed a synthetic one manufactured by pointcloud_deskew_field.py.
    genz_icp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_genz_icp_dir, 'launch', 'odometry.launch.py')
        ),
        launch_arguments={
            'namespace': robot_namespace,
            'topic': lidar_topic,
            'odom_frame': PythonExpression(['"', robot_namespace, '/odom"']),
            'base_frame': PythonExpression(['"', robot_namespace, '/base_link"']),
            'publish_odom_tf': 'false',
            'deskew': 'true',
            'visualize': 'false',
        }.items(),
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'robot_namespace',
            default_value='',
            description='This rover\'s own namespace, e.g. "rover1". Empty = unnamespaced single-robot use.'
        ),
        DeclareLaunchArgument(
            'peer_robot_namespace',
            default_value='',
            description='The OTHER rover\'s namespace, e.g. rover2 when this instance is rover1. Empty (default) disables the peer obstacle relay.'
        ),

        DeclareLaunchArgument(
            'use_rviz',
            default_value='False',
            description='Start rviz2 for THIS rover. Off by default - two rovers means two '
                        'rviz2 instances, which is heavy; each is namespaced so they do not '
                        'collide on the node name.'
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value='nav2_costmap_view',
            description='Which config in lirovo/rviz to load, WITHOUT the .rviz suffix. '
                        'One of: nav2_costmap_view, genz_odometry_view.'
        ),

        # 2026-09-08: same treatment as lirovo_hardware.launch.py (see its own comment for the
        # full explanation) - default True requires a bag publishing /<robot_namespace>/clock,
        # or every node here freezes at time zero. Pass use_sim_time:=false for real-FCU runs.
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Requires a bag publishing /<robot_namespace>/clock or every node '
                        'freezes at time zero. Pass use_sim_time:=false for real-FCU runs '
                        'with no bag.'
        ),

        SetRemap(src='/clock', dst=PythonExpression(['"/', robot_namespace, '/clock"'])),
        SetParameter(name='use_sim_time', value=use_sim_time),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            namespace=robot_namespace,
            arguments=['0', '0', '0', '0', '0', '0',
                       PythonExpression(['"', robot_namespace, '/map"']),
                       PythonExpression(['"', robot_namespace, '/odom"'])],
            name='static_tf_odom',
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            namespace=robot_namespace,
            name='static_tf_lidar',
            arguments=['--x', '0', '--y', '0', '--z', '0',
                       '--yaw', '0', '--pitch', '0', '--roll', '0',
                       '--frame-id', PythonExpression(['"', robot_namespace, '/base_link"']),
                       '--child-frame-id', PythonExpression(['"', robot_namespace, '/lidar"'])],
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            namespace=robot_namespace,
            name='pointcloud_to_laserscan',
            parameters=[{
                'target_frame': PythonExpression(['"', robot_namespace, '/base_link"']),
                'transform_tolerance': 0.05,
                'min_height': -0.8,
                'max_height': 0.65,
                'angle_min': -0.55,
                'angle_max': 0.7,
                'angle_increment': 0.00872665,
                'scan_time': 0.1,
                'range_min': 1.5,
                'range_max': 41.0,
                'use_inf': True,
                'inf_epsilon': 1.0,
                'queue_size': 50,
                'use_sim_time': use_sim_time,
            }],
            remappings=[
                ('cloud_in', lidar_topic),
                ('scan', PythonExpression(['"/', robot_namespace, '/scan"'])),
            ],
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            namespace=robot_namespace,
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_params_path_namespaced],
        ),

        genz_icp_launch,

        Node(
            package='lirovo',
            executable='peer_obstacle_relay',
            namespace=robot_namespace,
            name='peer_obstacle_relay',
            output='screen',
            parameters=[{
                'publish_rate_hz': 5.0,
            }],
            remappings=[
                ('scan_in', PythonExpression(['"/', robot_namespace, '/scan"'])),
                ('scan_out', PythonExpression(['"/', peer_robot_namespace, '/obstacles_in"'])),
            ],
            condition=IfCondition(PythonExpression(['"', peer_robot_namespace, '" != ""'])),
        ),

        Node(
            condition=IfCondition(use_rviz),
            package='rviz2',
            executable='rviz2',
            namespace=robot_namespace,
            name='rviz2',
            arguments=['-d', rviz_config_namespaced],
            output='screen',
        ),

        delayed_nav2_launch,
    ])
