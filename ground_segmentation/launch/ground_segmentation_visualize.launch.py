import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_ground_segmentation = get_package_share_directory('ground_segmentation')
    base_launch = os.path.join(pkg_ground_segmentation, 'launch', 'ground_segmentation.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument(
            name='visualise',
            default_value='true',
            description='Visualization switch (defaults to true for visualize launch)'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(base_launch),
            launch_arguments={'visualise': LaunchConfiguration('visualise')}.items()
        )
    ])
