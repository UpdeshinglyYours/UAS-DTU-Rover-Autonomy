"""Stock-Nav2 casualty mission: casualty_manager_simple + keepout_monitor + rc_orbit_trigger
+ casualty_models_bridge (frame extractor + audio model on the Orin host, over localhost HTTP).

  ros2 launch vortex_mission mission_simple.launch.py orbit_channel:=10
  ros2 launch vortex_mission mission_simple.launch.py models_bridge:=false   # RC / console only
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('vortex_mission'), 'config',
                          'casualty_params_simple.yaml')
    params_file = LaunchConfiguration('params_file')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=params),
        DeclareLaunchArgument('orbit_channel', default_value='10',
                              description='1-based RC channel of the orbit switch'),
        DeclareLaunchArgument('models_bridge', default_value='true',
                              description='run casualty_models_bridge (audio done -> orbit)'),
        Node(package='vortex_mission', executable='casualty_manager_simple',
             name='casualty_manager', output='screen', parameters=[params_file]),
        Node(package='vortex_mission', executable='keepout_monitor', name='keepout_monitor',
             output='screen', parameters=[params_file]),
        Node(package='vortex_mission', executable='rc_orbit_trigger', name='rc_orbit_trigger',
             output='screen', parameters=[params_file,
                                          {'orbit_channel': LaunchConfiguration('orbit_channel')}]),
        Node(package='vortex_mission', executable='casualty_models_bridge',
             name='casualty_models_bridge', output='screen', parameters=[params_file],
             condition=IfCondition(LaunchConfiguration('models_bridge'))),
    ])
