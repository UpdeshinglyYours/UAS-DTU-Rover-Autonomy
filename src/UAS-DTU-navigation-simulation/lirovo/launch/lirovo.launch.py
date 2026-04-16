from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import TimerAction
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    namePackage = 'lirovo'
    slam_params_path = os.path.join(get_package_share_directory(namePackage),'config','slam_params.yaml')
    nav2_params_path = os.path.join(get_package_share_directory(namePackage),'config','nav2_params.yaml')
    print(f"SLAM params path: {slam_params_path}")

    pkg_nav2_dir = get_package_share_directory('nav2_bringup')

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'params_file': nav2_params_path,
            'autostart': 'True',
            'map': 'map',  
        }.items() 
    )
    delayed_nav2_launch = TimerAction(
    period=3.0,
    actions=[nav2_launch]
)

    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['2', '2', '2', '1.57', '0', '1.57', 'base_link', 'lidar'],
            parameters=[{'use_sim_time': False}],
            name='static_tf_lidar'
        ),
        Node(
            package='lirovo',
            executable='pointcloud_processor',
            name='pointcloud_processor',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            parameters=[{
                'target_frame': 'base_link',
                'transform_tolerance': 0.5,
                'min_height': -0.3,
                'max_height': 100.0,
                'angle_min': -3.14159,
                'angle_max': +3.14159,
                'angle_increment': 0.00872665,
                'scan_time': 0.8,
                'range_min': 0.1,
                'range_max': 1000.0,
                'use_inf': True,
                'inf_epsilon': 1.0,
                'queue_size': 50,
                'use_sim_time': False,
            }],
            remappings=[
                ('cloud_in', '/synced_pointcloud'),
                ('scan', '/scan'),
            ],
        ),
         Node(
             package='tf2_ros',
             executable='static_transform_publisher',
             arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
             name='static_tf_odom'
         ),
       # Node(
       #     package='lirovo',
       #     executable='mavros_bridge',
       #     name='mavros_bridge',
       #     output='screen',
       #     parameters=[{'use_sim_time': False}]
       # ),
        Node(
       package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(
            get_package_share_directory(namePackage),
            'config', 'localization.yaml')],
        ),
      #  Node(
      #      package='lirovo',
      #      executable='navigator',
      #      name='navigator',
      #      output='screen',
      #      parameters=[{'use_sim_time': False}]
      #  ),
      #  TimerAction(
      #      period=1.0,
      #      actions=[
      #          Node(
      #              package='slam_toolbox',
      #              executable='async_slam_toolbox_node',
      #              name='slam_toolbox',
      #              output='screen',
      #              parameters=[slam_params_path],
      #          ),
      #      ]
      #  ),        
      #  delayed_nav2_launch
    ])
