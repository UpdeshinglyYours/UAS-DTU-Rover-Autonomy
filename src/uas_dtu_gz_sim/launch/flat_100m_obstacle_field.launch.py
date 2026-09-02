#!/usr/bin/python3

from os.path import join

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    sim_path = get_package_share_directory("uas_dtu_gz_sim")
    bcr_bot_path = get_package_share_directory("bcr_bot")
    gz_sim_path = get_package_share_directory("ros_gz_sim")

    default_world = join(
        sim_path, "worlds", "flat_100m_obstacle_field.sdf"
    )
    world_file = LaunchConfiguration("world_file")
    position_x = LaunchConfiguration("position_x")
    position_y = LaunchConfiguration("position_y")
    orientation_yaw = LaunchConfiguration("orientation_yaw")
    camera_enabled = LaunchConfiguration("camera_enabled")
    stereo_camera_enabled = LaunchConfiguration("stereo_camera_enabled")
    two_d_lidar_enabled = LaunchConfiguration("two_d_lidar_enabled")
    odometry_source = LaunchConfiguration("odometry_source")

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(gz_sim_path, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": PythonExpression(["'", world_file, " -r'"])
        }.items(),
    )

    spawn_bcr_bot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(bcr_bot_path, "launch", "bcr_bot_gz_spawn.launch.py")
        ),
        launch_arguments={
            "camera_enabled": camera_enabled,
            "stereo_camera_enabled": stereo_camera_enabled,
            "two_d_lidar_enabled": two_d_lidar_enabled,
            "position_x": position_x,
            "position_y": position_y,
            "orientation_yaw": orientation_yaw,
            "odometry_source": odometry_source,
        }.items(),
    )

    return LaunchDescription(
        [
            AppendEnvironmentVariable(
                name="GZ_SIM_RESOURCE_PATH",
                value=join(sim_path, "worlds"),
            ),
            DeclareLaunchArgument("world_file", default_value=default_world),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("position_x", default_value="0.0"),
            DeclareLaunchArgument("position_y", default_value="0.0"),
            DeclareLaunchArgument(
                "orientation_yaw", default_value="0.0"
            ),
            DeclareLaunchArgument("camera_enabled", default_value="true"),
            DeclareLaunchArgument(
                "stereo_camera_enabled", default_value="false"
            ),
            DeclareLaunchArgument(
                "two_d_lidar_enabled", default_value="true"
            ),
            DeclareLaunchArgument(
                "odometry_source", default_value="world"
            ),
            gz_sim,
            spawn_bcr_bot,
        ]
    )
