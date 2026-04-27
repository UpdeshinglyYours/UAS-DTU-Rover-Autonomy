from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    namePackage = 'lirovo'
    slam_params_path = os.path.join(get_package_share_directory(namePackage), 'config','slam_params.yaml')
    nav2_params_path = os.path.join(get_package_share_directory(namePackage),'config','nav2_params.yaml')

    return LaunchDescription([
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            parameters=[{
                'target_frame': 'body_link',
                'transform_tolerance': 0.01,
                'min_height': 0.0,
                'max_height': 1.0,
                'angle_min': -1.5708,
                'angle_max': 1.5708,
                'angle_increment': 0.0087,
                'scan_time': 0.3333,
                'range_min': 0.45,
                'range_max': 100.0,
                'use_inf': True,
                'inf_epsilon': 1.0,
                'queue_size': 10
            }],
            remappings=[
                ('cloud_in', '/bf_lidar/point_cloud_out'),
                ('scan', '/scan')
            ]
        ),
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox_node',
            output='screen',
            parameters=[slam_params_path]
        ),
        Node(
            package="nav2_costmap_2d",
            executable="costmap_2d_node",
            name="laser_costmap",
            output="screen",
            parameters=[{
                "use_sim_time": False,
                "global_frame": "map",
                "robot_base_frame": "base_link",
                "update_frequency": 5.0,
                "publish_frequency": 1.0,
                "rolling_window": True,
                "width": 10.0,
                "height": 10.0,
                "resolution": 0.05,
                "plugins": ["obstacle_layer", "inflation_layer"],

                "obstacle_layer": {
                    "plugin": "nav2_costmap_2d::ObstacleLayer",
                    "enabled": True,
                    "observation_sources": "scan",
                    "scan": {
                        "topic": "/scan",
                        "max_obstacle_height": 2.0,
                        "min_obstacle_height": 0.0,
                        "obstacle_range": 3.0,
                        "raytrace_range": 3.5,
                        "clearing": True,
                        "marking": True,
                        "data_type": "LaserScan"
                    }
                },

                "inflation_layer": {
                    "plugin": "nav2_costmap_2d::InflationLayer",
                    "inflation_radius": 0.55,
                    "cost_scaling_factor": 10.0
                }
            }],
            remappings=[
                ("/scan", "/scan")  # change this if needed
            ]
        ),
        Node(
            package='lirovo',
            executable='mavros_bridge.py',
            name='mavros_bridge',
            output='screen'
        ),
    ])
