#!/usr/bin/env python3

from os.path import join
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch.substitutions import PythonExpression

from launch_ros.actions import Node

def get_xacro_to_doc(xacro_file_path, mappings):
    doc = xacro.parse(open(xacro_file_path))
    xacro.process_doc(doc, mappings=mappings)
    return doc

def generate_launch_description():
    # Get bcr_bot package's share directory path
    bcr_bot_path = get_package_share_directory('bcr_bot')
    
    # Retrieve launch configuration arguments
    position_x = LaunchConfiguration("position_x")
    position_y = LaunchConfiguration("position_y")
    orientation_yaw = LaunchConfiguration("orientation_yaw")
    camera_enabled = LaunchConfiguration("camera_enabled", default=True)
    stereo_camera_enabled = LaunchConfiguration("stereo_camera_enabled", default=False)
    two_d_lidar_enabled = LaunchConfiguration("two_d_lidar_enabled", default=True)
    # was previously hardcoded to mirror two_d_lidar_enabled's value instead of having its own
    # arg - meant there was no way to toggle the 3D lidar independently from the command line
    three_d_lidar_enabled = LaunchConfiguration("three_d_lidar_enabled", default=True)
    odometry_source = LaunchConfiguration("odometry_source", default="world")
    robot_namespace = LaunchConfiguration("robot_namespace", default='bcr_bot')
    # gazebo.launch.py declares its own use_sim_time arg but never actually threaded it down
    # into this file - robot_state_publisher ran with use_sim_time hardcoded False regardless.
    use_sim_time = LaunchConfiguration("use_sim_time", default='true')

    # Sensor mount offsets (relative to base_link) - same physical mount point for every
    # robot by default, exposed here so it's launch-configurable instead of hand-editing xacro.
    three_d_lidar_x = LaunchConfiguration("three_d_lidar_x", default="0.0")
    three_d_lidar_y = LaunchConfiguration("three_d_lidar_y", default="0")
    three_d_lidar_z = LaunchConfiguration("three_d_lidar_z", default="0.25")
    imu_x = LaunchConfiguration("imu_x", default="0.0")
    imu_y = LaunchConfiguration("imu_y", default="0.0")
    imu_z = LaunchConfiguration("imu_z", default="0.08")
    
    # Path to the Xacro file
    xacro_path = join(bcr_bot_path, 'urdf', 'bcr_bot.xacro')
    #doc = get_xacro_to_doc(xacro_path, {"wheel_odom_topic": "odom", "sim_gazebo": "true", "two_d_lidar_enabled": "true", "camera_enabled": "true"})

    # Launch the robot_state_publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
                    {'use_sim_time': use_sim_time},
                    {'robot_description': Command( \
                    ['xacro ', xacro_path,
                    ' camera_enabled:=', camera_enabled,
                    ' stereo_camera_enabled:=', stereo_camera_enabled,
                    ' two_d_lidar_enabled:=', two_d_lidar_enabled,
                    ' three_d_lidar_enabled:=', three_d_lidar_enabled,
                    ' sim_gazebo:=', "true",
                    ' odometry_source:=', odometry_source,
                    ' robot_namespace:=', robot_namespace,
                    ' three_d_lidar_x:=', three_d_lidar_x,
                    ' three_d_lidar_y:=', three_d_lidar_y,
                    ' three_d_lidar_z:=', three_d_lidar_z,
                    ' imu_x:=', imu_x,
                    ' imu_y:=', imu_y,
                    ' imu_z:=', imu_z,
                    ])}],
        remappings=[
            ('/joint_states', PythonExpression(['"', robot_namespace, '/joint_states"'])),
            # Without this, every spawned robot's robot_state_publisher publishes onto the
            # exact same global /robot_description topic - spawn_entity.py then races between
            # them and can grab the wrong robot's description, spawning a duplicate under the
            # wrong namespace and crashing Gazebo on a ROS node name collision.
            ('/robot_description', PythonExpression(['"', robot_namespace, '/robot_description"'])),
        ]
    )

    # Launch the spawn_entity node to spawn the robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-topic', PythonExpression(['"', robot_namespace, '/robot_description"']),
            '-entity', PythonExpression(['"', robot_namespace, '_robot"']), #default enitity name _bcr_bot
            '-z', "0.28",
            '-x', position_x,
            '-y', position_y,
            '-Y', orientation_yaw
        ]
    )



    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument("camera_enabled", default_value = camera_enabled),
        DeclareLaunchArgument("stereo_camera_enabled", default_value = stereo_camera_enabled),
        DeclareLaunchArgument("two_d_lidar_enabled", default_value = two_d_lidar_enabled),
        DeclareLaunchArgument("three_d_lidar_enabled", default_value = three_d_lidar_enabled),
        DeclareLaunchArgument("position_x", default_value="0.0"),
        DeclareLaunchArgument("position_y", default_value="0.0"),
        DeclareLaunchArgument("orientation_yaw", default_value="0.0"),
        DeclareLaunchArgument("odometry_source", default_value = odometry_source),
        DeclareLaunchArgument("robot_namespace", default_value = robot_namespace),
        DeclareLaunchArgument("use_sim_time", default_value = use_sim_time),
        DeclareLaunchArgument("three_d_lidar_x", default_value = three_d_lidar_x),
        DeclareLaunchArgument("three_d_lidar_y", default_value = three_d_lidar_y),
        DeclareLaunchArgument("three_d_lidar_z", default_value = three_d_lidar_z),
        DeclareLaunchArgument("imu_x", default_value = imu_x),
        DeclareLaunchArgument("imu_y", default_value = imu_y),
        DeclareLaunchArgument("imu_z", default_value = imu_z),
        # DeclareLaunchArgument('robot_description', default_value=doc.toxml()),
        robot_state_publisher,
        spawn_entity
    ])
