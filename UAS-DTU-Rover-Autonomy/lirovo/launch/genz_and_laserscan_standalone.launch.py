"""
Standalone test launch: ONLY pointcloud_to_laserscan + genz_icp's bf_lidar_genz_pipeline.launch.py,
namespaced. Created 2026-09-02 for bag-replay testing without bringing up the rest of
lirovo_hardware.launch.py (no EKF, no nav2, no peer_obstacle_relay, no static TFs of its own).

Matches the exact genz_icp include lirovo_hardware.launch.py currently uses:
    ros2 launch genz_icp bf_lidar_genz_pipeline.launch.py \
      namespace:=rover1 raw_cloud_topic:=/rover1/point_cloud_out \
      enable_static_tf:=false visualize:=false
except visualize is left at the pipeline's own default (true) here, so its namespaced
rviz2 (genz_icp_ros2.rviz, text-substituted for "/<ns>/genz/..." topics and
"Fixed Frame: <ns>/odom" - see bf_lidar_genz_pipeline.launch.py's own _launch_setup) comes up
automatically.

enable_static_tf is still false, same reason as lirovo_hardware.launch.py: this file does NOT
publish base_link->lidar itself, so you must run static_tf_lidar (and static_tf_odom, if
pointcloud_to_laserscan's target_frame lookup needs it) as separate standalone commands
yourself before/around this - same as the rest of the bag-replay test setup.

Assumes mavros_hardware.launch.py is already running separately for the same robot_namespace
(imu/data, needed by the deskewer and by odometry.launch.py's IMU-assisted prediction).
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    robot_namespace = LaunchConfiguration('robot_namespace')
    lidar_topic = PythonExpression(['"/', robot_namespace, '/point_cloud_out"'])

    pkg_genz_icp_dir = get_package_share_directory('genz_icp')

    genz_icp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_genz_icp_dir, 'launch', 'bf_lidar_genz_pipeline.launch.py')
        ),
        launch_arguments={
            'namespace': robot_namespace,
            'raw_cloud_topic': lidar_topic,
            'enable_static_tf': 'false',
            # true (pipeline default) - this is the whole point of this file: bring up
            # genz_icp's own namespaced rviz2, unlike lirovo_hardware.launch.py which sets
            # this false to avoid a duplicate rviz alongside its own nav2 costmap one.
            'visualize': 'true',
        }.items(),
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'robot_namespace',
            default_value='rover1',
            description='This rover\'s namespace, e.g. "rover1" - must match whatever '
                        'published /roverN/point_cloud_out and mavros_hardware.launch.py '
                        'are using.'
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
                'use_sim_time': False,
            }],
            remappings=[
                ('cloud_in', lidar_topic),
                ('scan', PythonExpression(['"/', robot_namespace, '/scan"'])),
            ],
        ),

        genz_icp_launch,
    ])
