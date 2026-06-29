from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import ast


def _parse_gyro_bias(value):
    text = value.strip()
    if text in ("", "unset", "none", "None", "null"):
        return None

    try:
        parsed = ast.literal_eval(text)
    except (SyntaxError, ValueError):
        parsed = [item.strip() for item in text.split(",")]

    if not isinstance(parsed, (list, tuple)) or len(parsed) != 3:
        raise ValueError("gyro_bias must be unset or a 3-value list like [0.0, 0.0, 0.0]")

    return [float(parsed[0]), float(parsed[1]), float(parsed[2])]


def _launch_setup(context, *args, **kwargs):
    params = {
        "cloud_topic": LaunchConfiguration("cloud_topic"),
        "imu_topic": LaunchConfiguration("imu_topic"),
        "output_topic": LaunchConfiguration("output_topic"),
        "use_sim_time": LaunchConfiguration("use_sim_time"),
        "point_time_field": LaunchConfiguration("point_time_field"),
        "point_time_unit": LaunchConfiguration("point_time_unit"),
        "cloud_stamp_location": LaunchConfiguration("cloud_stamp_location"),
        "deskew_reference": LaunchConfiguration("deskew_reference"),
        "imu_buffer_seconds": LaunchConfiguration("imu_buffer_seconds"),
        "max_imu_gap_seconds": LaunchConfiguration("max_imu_gap_seconds"),
        "invert_correction": LaunchConfiguration("invert_correction"),
        "debug_print": LaunchConfiguration("debug_print"),
        "imu_angular_velocity_scale": LaunchConfiguration("imu_angular_velocity_scale"),
        "enable_gyro_bias_calibration": LaunchConfiguration("enable_gyro_bias_calibration"),
        "gyro_bias_calibration_seconds": LaunchConfiguration("gyro_bias_calibration_seconds"),
        "gyro_bias_min_samples": LaunchConfiguration("gyro_bias_min_samples"),
        "print_gyro_stats": LaunchConfiguration("print_gyro_stats"),
        "publish_raw_during_gyro_bias_calibration": LaunchConfiguration(
            "publish_raw_during_gyro_bias_calibration"
        ),
        "enable_pending_cloud_queue": LaunchConfiguration("enable_pending_cloud_queue"),
        "max_pending_cloud_wait_seconds": LaunchConfiguration("max_pending_cloud_wait_seconds"),
        "publish_raw_on_missing_imu_after_wait": LaunchConfiguration(
            "publish_raw_on_missing_imu_after_wait"
        ),
        "drop_cloud_on_missing_imu_after_wait": LaunchConfiguration(
            "drop_cloud_on_missing_imu_after_wait"
        ),
        "max_pending_clouds": LaunchConfiguration("max_pending_clouds"),
    }

    gyro_bias = _parse_gyro_bias(LaunchConfiguration("gyro_bias").perform(context))
    if gyro_bias is not None:
        params["gyro_bias"] = gyro_bias

    return [
        Node(
            package="genz_icp",
            executable="imu_rotation_deskew_node",
            name="imu_rotation_deskew_node",
            output="screen",
            parameters=[params],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("cloud_topic", default_value="/bf_lidar/point_cloud_out"),
            DeclareLaunchArgument("imu_topic", default_value="/bf_lidar/imu_out"),
            DeclareLaunchArgument("output_topic", default_value="/bf_lidar/point_cloud_deskewed"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("point_time_field", default_value="auto"),
            DeclareLaunchArgument("point_time_unit", default_value="seconds"),
            DeclareLaunchArgument("cloud_stamp_location", default_value="start"),
            DeclareLaunchArgument("deskew_reference", default_value="middle"),
            DeclareLaunchArgument("imu_buffer_seconds", default_value="2.0"),
            DeclareLaunchArgument("max_imu_gap_seconds", default_value="0.03"),
            DeclareLaunchArgument("invert_correction", default_value="false"),
            DeclareLaunchArgument("debug_print", default_value="false"),
            DeclareLaunchArgument("imu_angular_velocity_scale", default_value="1.0"),
            DeclareLaunchArgument("enable_gyro_bias_calibration", default_value="false"),
            DeclareLaunchArgument("gyro_bias_calibration_seconds", default_value="2.0"),
            DeclareLaunchArgument("gyro_bias_min_samples", default_value="50"),
            DeclareLaunchArgument("gyro_bias", default_value="unset"),
            DeclareLaunchArgument("print_gyro_stats", default_value="true"),
            DeclareLaunchArgument("publish_raw_during_gyro_bias_calibration", default_value="true"),
            DeclareLaunchArgument("enable_pending_cloud_queue", default_value="true"),
            DeclareLaunchArgument("max_pending_cloud_wait_seconds", default_value="0.15"),
            DeclareLaunchArgument("publish_raw_on_missing_imu_after_wait", default_value="false"),
            DeclareLaunchArgument("drop_cloud_on_missing_imu_after_wait", default_value="true"),
            DeclareLaunchArgument("max_pending_clouds", default_value="20"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
