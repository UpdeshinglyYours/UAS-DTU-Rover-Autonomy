from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = {
        "input_topic": LaunchConfiguration("input_topic"),
        "output_topic": LaunchConfiguration("output_topic"),
        "fixed_frame": LaunchConfiguration("fixed_frame"),
        "lidar_frame": LaunchConfiguration("lidar_frame"),
        "buffer_size": LaunchConfiguration("buffer_size"),
        "dynamic_threshold_meters": LaunchConfiguration("dynamic_threshold_meters"),
        "min_history_matches": LaunchConfiguration("min_history_matches"),
        "transform_timeout_sec": LaunchConfiguration("transform_timeout_sec"),
        "publish_in_original_frame": LaunchConfiguration("publish_in_original_frame"),
        "warmup_publish_unfiltered": LaunchConfiguration("warmup_publish_unfiltered"),
        "use_filtered_cloud_for_history": LaunchConfiguration("use_filtered_cloud_for_history"),
        "debug_print": LaunchConfiguration("debug_print"),
        "use_sim_time": LaunchConfiguration("use_sim_time"),
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument("input_topic", default_value="/bf_lidar/point_cloud_deskewed"),
            DeclareLaunchArgument("output_topic", default_value="/bf_lidar/point_cloud_filtered"),
            DeclareLaunchArgument("fixed_frame", default_value="odom"),
            DeclareLaunchArgument("lidar_frame", default_value="lidar"),
            DeclareLaunchArgument("buffer_size", default_value="5"),
            DeclareLaunchArgument("dynamic_threshold_meters", default_value="0.35"),
            DeclareLaunchArgument("min_history_matches", default_value="2"),
            DeclareLaunchArgument("transform_timeout_sec", default_value="0.2"),
            DeclareLaunchArgument("publish_in_original_frame", default_value="true"),
            DeclareLaunchArgument("warmup_publish_unfiltered", default_value="true"),
            DeclareLaunchArgument("use_filtered_cloud_for_history", default_value="true"),
            DeclareLaunchArgument("debug_print", default_value="false"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            Node(
                package="genz_icp",
                executable="dynamic_pointcloud_filter_node",
                name="dynamic_pointcloud_filter",
                output="screen",
                parameters=[params],
            ),
        ]
    )
