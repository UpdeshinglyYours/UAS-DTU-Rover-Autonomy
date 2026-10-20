"""casualty_manager + rc_orbit_trigger + keepout_monitor, with config/casualty_params.yaml.

Override the RC channel without editing the file:
  ros2 launch vortex_mission mission.launch.py orbit_channel:=7
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('vortex_mission'), 'config',
                          'casualty_params.yaml')
    orbit_channel = LaunchConfiguration('orbit_channel')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=params),
        DeclareLaunchArgument('orbit_channel', default_value='0',
                              description='1-based RC channel of the orbit switch'),
        Node(package='vortex_mission', executable='casualty_manager', name='casualty_manager',
             output='screen', parameters=[LaunchConfiguration('params_file')]),
        Node(package='vortex_mission', executable='keepout_monitor', name='keepout_monitor',
             output='screen', parameters=[LaunchConfiguration('params_file')]),
        Node(package='vortex_mission', executable='rc_orbit_trigger', name='rc_orbit_trigger',
             output='screen', parameters=[LaunchConfiguration('params_file'),
                                          {'orbit_channel': orbit_channel}]),
    ])
