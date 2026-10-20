"""Desk test of the whole casualty pipeline without Gazebo or the rover.

kinematic_sim (fake rover + fake lidars) + Nav2 (nav2_params4) + mission nodes.
Run in its own ROS domain so it can never talk to the real rover:

  ROS_DOMAIN_ID=77 ros2 launch vortex_mission sim_casualty.launch.py \
      boxes:="[9.8, 2.8, 10.3, 3.2]"          # optional obstacles, [x0, y0, x1, y1, ...]
  ROS_DOMAIN_ID=77 ros2 run vortex_mission add_casualty --x 10 --y 0
  ROS_DOMAIN_ID=77 ros2 service call /casualty_manager/start_orbit \
      vortex_interfaces/srv/StartOrbit "{casualty_id: 1}"
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def static_tf(name, xyz, child):
    return Node(package='tf2_ros', executable='static_transform_publisher', name=name,
                arguments=[str(xyz[0]), str(xyz[1]), str(xyz[2]), '0', '0', '0',
                           'base_link', child])


def generate_launch_description():
    default_params = os.path.expanduser('~/vortex_ws/lirovo/config/nav2_params4.yaml')
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')),
        launch_arguments={'params_file': LaunchConfiguration('params_file'),
                          'use_sim_time': 'False', 'autostart': 'True'}.items())
    # mission_launch:=mission_simple.launch.py -> stock-Nav2 waypoint orbit (casualty_manager_simple)
    mission = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            get_package_share_directory('vortex_mission'), 'launch',
            LaunchConfiguration('mission_launch')])))
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('boxes', default_value='[0.0]'),
        DeclareLaunchArgument('mission_launch', default_value='mission.launch.py'),
        DeclareLaunchArgument('start_x', default_value='0.0'),
        DeclareLaunchArgument('start_y', default_value='0.0'),
        DeclareLaunchArgument('start_yaw', default_value='0.0'),
        DeclareLaunchArgument('drivetrain_model', default_value='true',
                              description='model the real rover: 0.4 m/s / 0.3 rad/s deadbands + ~1 s lag'),
        static_tf('static_tf_lidar', (-0.17, 0.0, 0.54), 'lidar'),
        static_tf('static_tf_sf45b', (0.29, 0.0, 0.15), 'sf45b'),
        static_tf('static_tf_camera', (0.29, 0.0, 0.16), 'camera'),
        Node(package='vortex_mission', executable='kinematic_sim', name='kinematic_sim',
             output='screen',
             parameters=[{'boxes': LaunchConfiguration('boxes'),
                          'start_x': LaunchConfiguration('start_x'),
                          'start_y': LaunchConfiguration('start_y'),
                          'start_yaw': LaunchConfiguration('start_yaw'),
                          'drivetrain_model': LaunchConfiguration('drivetrain_model')}]),
        TimerAction(period=2.0, actions=[nav2]),
        mission,
    ])
