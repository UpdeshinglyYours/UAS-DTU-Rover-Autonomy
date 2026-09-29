#!/usr/bin/env python3

from os.path import join


from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import SetEnvironmentVariable
from launch.actions import AppendEnvironmentVariable
from launch.conditions import IfCondition

def generate_launch_description():
    # Get bcr_bot package's share directory path
    bcr_bot_path = get_package_share_directory('bcr_bot')

    # Retrieve launch configuration arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    spawn_robot = LaunchConfiguration('spawn_robot', default='true')
    robot_namespace = LaunchConfiguration('robot_namespace', default='bcr_bot')
    position_x = LaunchConfiguration('position_x', default='0.0')
    position_y = LaunchConfiguration('position_y', default='0.0')


    world_file = LaunchConfiguration("world_file", default = join(bcr_bot_path, 'worlds', 'small_warehouse.sdf'))

    # Include the Gazebo launch file
    gazebo_share = get_package_share_directory("gazebo_ros")
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(gazebo_share, "launch", "gazebo.launch.py"))
    )

    # spawing bcr_bot - only when spawn_robot:=true. For multi-robot sim, set spawn_robot:=false
    # here so this just brings up an empty world, then spawn each robot separately via
    # bcr_bot_gazebo_spawn.launch.py with its own robot_namespace/position_x/position_y.
    spawn_bcr_bot_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(bcr_bot_path, "launch", "bcr_bot_gazebo_spawn.launch.py")),
        launch_arguments={
            'robot_namespace': robot_namespace,
            'position_x': position_x,
            'position_y': position_y,
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(spawn_robot),
    )

    return LaunchDescription([
        # Declare launch arguments

        AppendEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=join(bcr_bot_path, "models")),

        SetEnvironmentVariable(
        name='GAZEBO_RESOURCE_PATH',
        value="/usr/share/gazebo-11:" + join(bcr_bot_path, "worlds")),
        DeclareLaunchArgument('world', default_value = world_file),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('verbose', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value = use_sim_time),
        DeclareLaunchArgument('spawn_robot', default_value = spawn_robot,
            description='Whether gazebo.launch.py should also spawn a bcr_bot. Set false for multi-robot sim so this only brings up the empty world.'),
        DeclareLaunchArgument('robot_namespace', default_value = robot_namespace,
            description='robot_namespace for the single robot spawned when spawn_robot:=true'),
        DeclareLaunchArgument('position_x', default_value = position_x),
        DeclareLaunchArgument('position_y', default_value = position_y),
        gazebo,spawn_bcr_bot_node
    ])
