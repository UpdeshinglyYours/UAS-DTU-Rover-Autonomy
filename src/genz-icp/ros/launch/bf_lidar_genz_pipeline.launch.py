from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

import ast


def _parse_double_list(value, name):
    text = value.strip()
    if text in ("", "unset", "none", "None", "null"):
        return []

    try:
        parsed = ast.literal_eval(text)
    except (SyntaxError, ValueError):
        parsed = [item.strip() for item in text.split(",") if item.strip()]

    if not isinstance(parsed, (list, tuple)):
        raise ValueError(f"{name} must be a list like [0.30, -0.11, 0.50]")

    return [float(item) for item in parsed]


def _launch_setup(context, *args, **kwargs):
    pkg_share = FindPackageShare("genz_icp")

    lidar_lever_arm = _parse_double_list(
        LaunchConfiguration("lidar_lever_arm").perform(context),
        "lidar_lever_arm",
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_lidar_static_tf",
        output="screen",
        arguments=[
            "--x", LaunchConfiguration("lidar_tf_x"),
            "--y", LaunchConfiguration("lidar_tf_y"),
            "--z", LaunchConfiguration("lidar_tf_z"),
            "--yaw", LaunchConfiguration("lidar_tf_yaw"),
            "--pitch", LaunchConfiguration("lidar_tf_pitch"),
            "--roll", LaunchConfiguration("lidar_tf_roll"),
            "--frame-id", LaunchConfiguration("base_frame"),
            "--child-frame-id", LaunchConfiguration("lidar_frame"),
        ],
    )

    deskewer_params = {
        "use_sim_time": LaunchConfiguration("use_sim_time"),
        "cloud_topic": LaunchConfiguration("raw_cloud_topic"),
        "imu_topic": LaunchConfiguration("deskew_imu_topic"),
        "output_topic": LaunchConfiguration("deskewed_cloud_topic"),
        "cloud_stamp_location": LaunchConfiguration("cloud_stamp_location"),
        "deskew_reference": LaunchConfiguration("deskew_reference"),
        "point_time_field": LaunchConfiguration("point_time_field"),
        "point_time_unit": LaunchConfiguration("point_time_unit"),
        "max_imu_gap_seconds": LaunchConfiguration("max_imu_gap_seconds"),
        "imu_angular_velocity_scale": LaunchConfiguration("deskew_imu_angular_velocity_scale"),
        "enable_gyro_bias_calibration": LaunchConfiguration("enable_gyro_bias_calibration"),
        "gyro_bias_calibration_seconds": LaunchConfiguration("gyro_bias_calibration_seconds"),
        "gyro_bias_min_samples": LaunchConfiguration("gyro_bias_min_samples"),
        "enable_pending_cloud_queue": LaunchConfiguration("enable_pending_cloud_queue"),
        "max_pending_cloud_wait_seconds": LaunchConfiguration("max_pending_cloud_wait_seconds"),
        "publish_raw_on_missing_imu_after_wait": LaunchConfiguration(
            "publish_raw_on_missing_imu_after_wait"
        ),
        "drop_cloud_on_missing_imu_after_wait": LaunchConfiguration(
            "drop_cloud_on_missing_imu_after_wait"
        ),
        "max_pending_clouds": LaunchConfiguration("max_pending_clouds"),
        "enable_lidar_lever_arm_correction": LaunchConfiguration(
            "enable_lidar_lever_arm_correction"
        ),
        "lidar_lever_arm": lidar_lever_arm,
        "debug_print": LaunchConfiguration("deskew_debug_print"),
    }

    deskewer = Node(
        package="genz_icp",
        executable="imu_rotation_deskew_node",
        name="imu_rotation_deskew_node",
        output="screen",
        parameters=[deskewer_params],
    )

    odometry = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, "launch", "odometry.launch.py"])
        ),
        launch_arguments={
            "topic": LaunchConfiguration("deskewed_cloud_topic"),
            "config_file": LaunchConfiguration("config_file"),
            "base_frame": LaunchConfiguration("base_frame"),
            "odom_frame": LaunchConfiguration("odom_frame"),
            # robot_localization is the only odom -> base_link broadcaster.
            "publish_odom_tf": "false",
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "imu_topic": LaunchConfiguration("odom_imu_topic"),
            "imu_angular_velocity_scale": LaunchConfiguration("odom_imu_angular_velocity_scale"),
            "imu_prediction_max_gap_seconds": LaunchConfiguration(
                "imu_prediction_max_gap_seconds"
            ),
            "enable_map_update_quality_gate": LaunchConfiguration(
                "enable_map_update_quality_gate"
            ),
            "motion_prior_weight": LaunchConfiguration("motion_prior_weight"),
            "motion_prior_translation_sigma": LaunchConfiguration(
                "motion_prior_translation_sigma"
            ),
            "motion_prior_z_sigma": LaunchConfiguration("motion_prior_z_sigma"),
            "motion_prior_roll_pitch_sigma_deg": LaunchConfiguration(
                "motion_prior_roll_pitch_sigma_deg"
            ),
            "motion_prior_yaw_sigma_deg": LaunchConfiguration(
                "motion_prior_yaw_sigma_deg"
            ),
            "imu_prediction_max_rejected_frame_age_seconds": LaunchConfiguration(
                "imu_prediction_max_rejected_frame_age_seconds"
            ),
            "imu_prediction_debug": LaunchConfiguration("imu_prediction_debug"),
            "yaw_search_debug": LaunchConfiguration("yaw_search_debug"),
            "motion_prior_debug": LaunchConfiguration("motion_prior_debug"),
            "map_update_debug": LaunchConfiguration("map_update_debug"),
            "publish_twist": LaunchConfiguration("publish_twist"),
            "twist_in_child_frame": LaunchConfiguration("twist_in_child_frame"),
            "twist_debug": LaunchConfiguration("twist_debug"),
            "visualize": LaunchConfiguration("visualize"),
        }.items(),
    )

    return [static_tf, deskewer, odometry]


def generate_launch_description():
    return LaunchDescription(
        [
            # Shared
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("base_frame", default_value="base_link"),
            DeclareLaunchArgument("odom_frame", default_value="odom"),
            DeclareLaunchArgument("lidar_frame", default_value="lidar"), #kdareep this lidar for blickfeld
            DeclareLaunchArgument(
                "config_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("genz_icp"), "config", "rover.yaml"]
                ),
            ),

            # Static TF: Y-forward base_link -> Y-forward lidar.
            # Make these match your physical lidar mounting.
            DeclareLaunchArgument("lidar_tf_x", default_value="-0.11"),
            DeclareLaunchArgument("lidar_tf_y", default_value="0.30"),
            DeclareLaunchArgument("lidar_tf_z", default_value="0.54"),
            DeclareLaunchArgument("lidar_tf_yaw", default_value="0.0"),
            DeclareLaunchArgument("lidar_tf_pitch", default_value="0.0"),
            DeclareLaunchArgument("lidar_tf_roll", default_value="0.0"),

            # Deskewer
            DeclareLaunchArgument("raw_cloud_topic", default_value="/bf_lidar/point_cloud_out"),
            DeclareLaunchArgument(
                "deskewed_cloud_topic",
                default_value="/bf_lidar/point_cloud_deskewed",
            ),
            DeclareLaunchArgument("deskew_imu_topic", default_value="/mavros/imu/data"),
            DeclareLaunchArgument("cloud_stamp_location", default_value="end"),
            DeclareLaunchArgument("deskew_reference", default_value="end"),
            DeclareLaunchArgument("point_time_field", default_value="point_time_offset"),
            DeclareLaunchArgument("point_time_unit", default_value="nanoseconds"),
            DeclareLaunchArgument("max_imu_gap_seconds", default_value="0.06"),
            DeclareLaunchArgument("deskew_imu_angular_velocity_scale", default_value="1.0"),
            DeclareLaunchArgument("enable_gyro_bias_calibration", default_value="true"),
            DeclareLaunchArgument("gyro_bias_calibration_seconds", default_value="2.0"),
            DeclareLaunchArgument("gyro_bias_min_samples", default_value="50"),
            DeclareLaunchArgument("enable_pending_cloud_queue", default_value="true"),
            DeclareLaunchArgument("max_pending_cloud_wait_seconds", default_value="0.15"),
            DeclareLaunchArgument("publish_raw_on_missing_imu_after_wait", default_value="false"),
            DeclareLaunchArgument("drop_cloud_on_missing_imu_after_wait", default_value="true"),
            DeclareLaunchArgument("max_pending_clouds", default_value="20"),
            DeclareLaunchArgument("enable_lidar_lever_arm_correction", default_value="true"),
            DeclareLaunchArgument("lidar_lever_arm", default_value="[0.11, 0.30, 0.00]"),
            DeclareLaunchArgument("deskew_debug_print", default_value="true"),

            # GenZ odometry
            DeclareLaunchArgument("odom_imu_topic", default_value="/mavros/imu/data"),
            DeclareLaunchArgument("odom_imu_angular_velocity_scale", default_value="1.0"),
            DeclareLaunchArgument("imu_prediction_max_gap_seconds", default_value="0.06"),
            DeclareLaunchArgument("enable_map_update_quality_gate", default_value="false"),
            DeclareLaunchArgument("motion_prior_weight", default_value="0.25"),
            DeclareLaunchArgument("motion_prior_translation_sigma", default_value="0.80"),
            DeclareLaunchArgument("motion_prior_z_sigma", default_value="0.25"),
            DeclareLaunchArgument("motion_prior_roll_pitch_sigma_deg", default_value="15.0"),
            DeclareLaunchArgument("motion_prior_yaw_sigma_deg", default_value="60.0"),
            DeclareLaunchArgument(
                "imu_prediction_max_rejected_frame_age_seconds",
                default_value="2.0",
            ),
            DeclareLaunchArgument("imu_prediction_debug", default_value="false"),
            DeclareLaunchArgument("yaw_search_debug", default_value="false"),
            DeclareLaunchArgument("motion_prior_debug", default_value="false"),
            DeclareLaunchArgument("map_update_debug", default_value="false"),

            # Twist output you added
            DeclareLaunchArgument("publish_twist", default_value="true"),
            DeclareLaunchArgument("twist_in_child_frame", default_value="true"),
            DeclareLaunchArgument("twist_debug", default_value="false"),

            # RViz from odometry.launch.py
            DeclareLaunchArgument("visualize", default_value="true"),

            OpaqueFunction(function=_launch_setup),
        ]
    )
