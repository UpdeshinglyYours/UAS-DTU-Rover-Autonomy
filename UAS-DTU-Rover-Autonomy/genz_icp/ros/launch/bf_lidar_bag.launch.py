"""GenZ-ICP + IMU on a recorded Blickfeld bag.

Thin wrapper over bf_lidar_genz_pipeline.launch.py with absolute topic names
(bags publish on /bf_lidar/... and /mavros/imu/data, with no namespace) and a
config whose imu_angular_velocity_scale is 1.0 - rover.yaml ships 0.01745
(deg->rad), which is right for a deg/s IMU but wrong for /mavros/imu/data,
which is already rad/s. rover.yaml's value would override the launch arg, so
the override has to live in the config file.

  ros2 launch genz_icp bf_lidar_bag.launch.py
  ros2 bag play <bag> --topics /bf_lidar/point_cloud_out /mavros/imu/data
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("genz_icp")
    return LaunchDescription([
        DeclareLaunchArgument("cloud_topic", default_value="/bf_lidar/point_cloud_out"),
        DeclareLaunchArgument("imu_topic",   default_value="/mavros/imu/data"),
        DeclareLaunchArgument("visualize",   default_value="true"),
        DeclareLaunchArgument(
            "config_file",
            default_value=PathJoinSubstitution([pkg, "config", "bf_lidar_bag.yaml"]),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg, "launch", "bf_lidar_genz_pipeline.launch.py"])
            ),
            launch_arguments={
                "namespace": "",
                "use_sim_time": "false",
                "config_file": LaunchConfiguration("config_file"),
                "raw_cloud_topic": LaunchConfiguration("cloud_topic"),
                "deskewed_cloud_topic": "/bf_lidar/point_cloud_deskewed",
                "deskew_imu_topic": LaunchConfiguration("imu_topic"),
                "odom_imu_topic": LaunchConfiguration("imu_topic"),
                # /mavros/imu/data is rad/s, not deg/s
                "deskew_imu_angular_velocity_scale": "1.0",
                "odom_imu_angular_velocity_scale": "1.0",
                # Blickfeld per-point timestamps
                "point_time_field": "point_time_offset",
                "point_time_unit": "nanoseconds",
                "cloud_stamp_location": "end",
                "deskew_reference": "end",
                # no URDF when replaying a bag, so publish base_link->lidar here
                "enable_static_tf": "true",
                "visualize": LaunchConfiguration("visualize"),
            }.items(),
        ),
    ])
