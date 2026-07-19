import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory('target_explorer')
    params_file = os.path.join(
        package_share,
        'config',
        'target_explorer_params.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'goal_mode',
            default_value='local',
            description='Compatibility selector; this launch runs local mode.',
        ),
        DeclareLaunchArgument(
            'local_goal_x',
            default_value='0.0',
            description='Absolute final map-frame X coordinate.',
        ),
        DeclareLaunchArgument(
            'local_goal_y',
            default_value='40.0',
            description='Absolute final map-frame Y coordinate.',
        ),
        Node(
            package='target_explorer',
            executable='target_explorer_node',
            name='target_explorer_node',
            output='screen',
            parameters=[
                params_file,
                {
                    'target_x': ParameterValue(
                        LaunchConfiguration('local_goal_x'),
                        value_type=float,
                    ),
                    'target_y': ParameterValue(
                        LaunchConfiguration('local_goal_y'),
                        value_type=float,
                    ),
                    'use_sim_time': False,
                },
            ],
        ),
    ])
