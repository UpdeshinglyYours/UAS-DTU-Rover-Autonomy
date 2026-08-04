import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('target_explorer')
    params_file = os.path.join(
        package_share,
        'config',
        'target_explorer_params.yaml',
    )

    return LaunchDescription([
        Node(
            package='target_explorer',
            executable='target_explorer_node',
            name='target_explorer_node',
            output='screen',
            parameters=[params_file],
        ),
    ])
