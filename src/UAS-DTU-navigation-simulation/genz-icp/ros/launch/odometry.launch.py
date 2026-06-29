# MIT License
#
# Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill Stachniss.
# Modified by Daehan Lee, Hyungtae Lim, and Soohee Han, 2024
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import (LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import ast


def _parse_imu_prediction_gyro_bias(value):
    text = value.strip()
    if text in ("", "unset", "none", "None", "null"):
        return None

    try:
        parsed = ast.literal_eval(text)
    except (SyntaxError, ValueError):
        parsed = [item.strip() for item in text.split(",") if item.strip()]

    if isinstance(parsed, (list, tuple)) and len(parsed) == 0:
        return None

    if not isinstance(parsed, (list, tuple)) or len(parsed) != 3:
        raise ValueError(
            "imu_prediction_gyro_bias must be unset or a 3-value list like [0.0, 0.0, 0.0]"
        )

    return [float(parsed[0]), float(parsed[1]), float(parsed[2])]


def _parse_double_list(value, name):
    text = value.strip()
    if text in ("", "unset", "none", "None", "null"):
        return []

    try:
        parsed = ast.literal_eval(text)
    except (SyntaxError, ValueError):
        parsed = [item.strip() for item in text.split(",") if item.strip()]

    if not isinstance(parsed, (list, tuple)):
        raise ValueError(f"{name} must be a list like [-4.0, 0.0, 4.0]")

    return [float(item) for item in parsed]


def _launch_setup(context, *args, **kwargs):
    params = {
        "odom_frame": LaunchConfiguration("odom_frame"),
        "base_frame": LaunchConfiguration("base_frame"),
        "use_sim_time": LaunchConfiguration("use_sim_time"),
        "deskew": LaunchConfiguration("deskew"),
        "max_range": LaunchConfiguration("max_range"),
        "min_range": LaunchConfiguration("min_range"),
        "voxel_size": LaunchConfiguration("voxel_size"),
        "map_cleanup_radius": LaunchConfiguration("map_cleanup_radius"),
        "desired_num_voxelized_points": LaunchConfiguration("desired_num_voxelized_points"),
        "planarity_threshold": LaunchConfiguration("planarity_threshold"),
        "max_points_per_voxel": LaunchConfiguration("max_points_per_voxel"),
        "max_num_iterations": LaunchConfiguration("max_num_iterations"),
        "convergence_criterion": LaunchConfiguration("convergence_criterion"),
        "initial_threshold": LaunchConfiguration("initial_threshold"),
        "min_motion_th": LaunchConfiguration("min_motion_th"),
        "enable_registration_quality_gate": LaunchConfiguration("enable_registration_quality_gate"),
        "min_registration_correspondences": LaunchConfiguration("min_registration_correspondences"),
        "registration_rmse_reject_ratio": LaunchConfiguration("registration_rmse_reject_ratio"),
        "registration_rmse_ema_alpha": LaunchConfiguration("registration_rmse_ema_alpha"),
        "max_registration_translation_per_frame": LaunchConfiguration("max_registration_translation_per_frame"),
        "max_registration_rotation_per_frame_deg": LaunchConfiguration("max_registration_rotation_per_frame_deg"),
        "max_consecutive_registration_rejections": LaunchConfiguration("max_consecutive_registration_rejections"),
        "absolute_registration_rmse_limit": LaunchConfiguration("absolute_registration_rmse_limit"),
        "enable_imu_motion_prediction": LaunchConfiguration("enable_imu_motion_prediction"),
        "imu_topic": LaunchConfiguration("imu_topic"),
        "imu_prediction_point_time_field": LaunchConfiguration("imu_prediction_point_time_field"),
        "imu_prediction_point_time_unit": LaunchConfiguration("imu_prediction_point_time_unit"),
        "imu_prediction_cloud_stamp_location": LaunchConfiguration("imu_prediction_cloud_stamp_location"),
        "imu_prediction_deskew_reference": LaunchConfiguration("imu_prediction_deskew_reference"),
        "imu_angular_velocity_scale": LaunchConfiguration("imu_angular_velocity_scale"),
        "enable_imu_prediction_gyro_bias_calibration": LaunchConfiguration(
            "enable_imu_prediction_gyro_bias_calibration"
        ),
        "imu_prediction_gyro_bias_calibration_seconds": LaunchConfiguration(
            "imu_prediction_gyro_bias_calibration_seconds"
        ),
        "imu_prediction_gyro_bias_min_samples": LaunchConfiguration(
            "imu_prediction_gyro_bias_min_samples"
        ),
        "imu_prediction_max_gap_seconds": LaunchConfiguration("imu_prediction_max_gap_seconds"),
        "imu_prediction_max_age_seconds": LaunchConfiguration("imu_prediction_max_age_seconds"),
        "imu_prediction_max_rejected_frame_age_seconds": LaunchConfiguration(
            "imu_prediction_max_rejected_frame_age_seconds"
        ),
        "imu_prediction_rotation_only": LaunchConfiguration("imu_prediction_rotation_only"),
        "imu_prediction_debug": LaunchConfiguration("imu_prediction_debug"),
        "enable_yaw_search_initializer": LaunchConfiguration("enable_yaw_search_initializer"),
        "yaw_search_score_max_correspondence_distance": LaunchConfiguration(
            "yaw_search_score_max_correspondence_distance"
        ),
        "yaw_search_min_correspondences": LaunchConfiguration("yaw_search_min_correspondences"),
        "yaw_search_use_weighted_rmse": LaunchConfiguration("yaw_search_use_weighted_rmse"),
        "yaw_search_vertical_axis": LaunchConfiguration("yaw_search_vertical_axis"),
        "yaw_search_debug": LaunchConfiguration("yaw_search_debug"),
        "enable_motion_prior": LaunchConfiguration("enable_motion_prior"),
        "motion_prior_translation_sigma": LaunchConfiguration("motion_prior_translation_sigma"),
        "motion_prior_z_sigma": LaunchConfiguration("motion_prior_z_sigma"),
        "motion_prior_roll_pitch_sigma_deg": LaunchConfiguration(
            "motion_prior_roll_pitch_sigma_deg"
        ),
        "motion_prior_yaw_sigma_deg": LaunchConfiguration("motion_prior_yaw_sigma_deg"),
        "motion_prior_weight": LaunchConfiguration("motion_prior_weight"),
        "motion_prior_apply_during_recovery": LaunchConfiguration(
            "motion_prior_apply_during_recovery"
        ),
        "motion_prior_debug": LaunchConfiguration("motion_prior_debug"),
        "enable_map_update_quality_gate": LaunchConfiguration("enable_map_update_quality_gate"),
        "map_update_max_weighted_rmse_ratio": LaunchConfiguration(
            "map_update_max_weighted_rmse_ratio"
        ),
        "map_update_max_rmse": LaunchConfiguration("map_update_max_rmse"),
        "map_update_max_translation_delta": LaunchConfiguration("map_update_max_translation_delta"),
        "map_update_max_rotation_delta_deg": LaunchConfiguration(
            "map_update_max_rotation_delta_deg"
        ),
        "map_update_min_correspondences": LaunchConfiguration("map_update_min_correspondences"),
        "map_update_debug": LaunchConfiguration("map_update_debug"),
        "publish_odom_tf": LaunchConfiguration("publish_odom_tf"),
        "visualize": LaunchConfiguration("visualize"),
        "terminal_status": LaunchConfiguration("terminal_status"),
        "config_file": LaunchConfiguration("config_file"),
    }

    gyro_bias = _parse_imu_prediction_gyro_bias(
        LaunchConfiguration("imu_prediction_gyro_bias").perform(context)
    )
    if gyro_bias is not None:
        params["imu_prediction_gyro_bias"] = gyro_bias
    params["yaw_search_degrees"] = _parse_double_list(
        LaunchConfiguration("yaw_search_degrees").perform(context),
        "yaw_search_degrees",
    )

    return [
        Node(
            package="genz_icp",
            executable="odometry_node",
            name="odometry_node",
            output="screen",
            remappings=[("pointcloud_topic", LaunchConfiguration("topic"))],
            parameters=[params],
        )
    ]


def generate_launch_description():
    current_pkg = FindPackageShare("genz_icp")
    return LaunchDescription(
        [
            # ROS 2 parameters
            DeclareLaunchArgument("topic", description="sensor_msg/PointCloud2 topic to process"),
            DeclareLaunchArgument("bagfile", default_value=""),
            DeclareLaunchArgument("visualize", default_value="true"),
            DeclareLaunchArgument("terminal_status", default_value="false"),
            DeclareLaunchArgument("odom_frame", default_value="odom"),
            DeclareLaunchArgument("base_frame", default_value=""),
            DeclareLaunchArgument("publish_odom_tf", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            # GenZ-ICP parameters
            DeclareLaunchArgument("deskew", default_value="false"),
            DeclareLaunchArgument("max_range", default_value="75.0"),
            DeclareLaunchArgument("min_range", default_value="0.75"),
            # This thing is still not suported: https://github.com/ros2/launch/issues/290#issuecomment-1438476902
            #  DeclareLaunchArgument("voxel_size", default_value=None),
            DeclareLaunchArgument("voxel_size", default_value="0.6"),
            DeclareLaunchArgument("map_cleanup_radius", default_value="75.0"),
            DeclareLaunchArgument("desired_num_voxelized_points", default_value="5000"),
            DeclareLaunchArgument("planarity_threshold", default_value="0.1"),
            DeclareLaunchArgument("max_points_per_voxel", default_value="4"),
            DeclareLaunchArgument("max_num_iterations", default_value="100"),
            DeclareLaunchArgument("convergence_criterion", default_value="0.00005"),
            DeclareLaunchArgument("initial_threshold", default_value="2.0"),
            DeclareLaunchArgument("min_motion_th", default_value="0.075"),
            DeclareLaunchArgument("enable_registration_quality_gate", default_value="true"),
            DeclareLaunchArgument("min_registration_correspondences", default_value="300"),
            DeclareLaunchArgument("registration_rmse_reject_ratio", default_value="2.5"),
            DeclareLaunchArgument("registration_rmse_ema_alpha", default_value="0.05"),
            DeclareLaunchArgument("max_registration_translation_per_frame", default_value="1.0"),
            DeclareLaunchArgument("max_registration_rotation_per_frame_deg", default_value="45.0"),
            DeclareLaunchArgument("max_consecutive_registration_rejections", default_value="5"),
            DeclareLaunchArgument("absolute_registration_rmse_limit", default_value="1.0"),
            DeclareLaunchArgument("enable_imu_motion_prediction", default_value="false"),
            DeclareLaunchArgument("imu_topic", default_value="/bf_lidar/imu_out"),
            DeclareLaunchArgument("imu_prediction_point_time_field", default_value="auto"),
            DeclareLaunchArgument("imu_prediction_point_time_unit", default_value="nanoseconds"),
            DeclareLaunchArgument("imu_prediction_cloud_stamp_location", default_value="end"),
            DeclareLaunchArgument("imu_prediction_deskew_reference", default_value="middle"),
            DeclareLaunchArgument("imu_angular_velocity_scale", default_value="0.017453292519943295"),
            DeclareLaunchArgument("enable_imu_prediction_gyro_bias_calibration", default_value="true"),
            DeclareLaunchArgument("imu_prediction_gyro_bias_calibration_seconds", default_value="2.0"),
            DeclareLaunchArgument("imu_prediction_gyro_bias_min_samples", default_value="50"),
            DeclareLaunchArgument("imu_prediction_gyro_bias", default_value="unset"),
            DeclareLaunchArgument("imu_prediction_max_gap_seconds", default_value="0.06"),
            DeclareLaunchArgument("imu_prediction_max_age_seconds", default_value="0.25"),
            DeclareLaunchArgument("imu_prediction_max_rejected_frame_age_seconds", default_value="1.0"),
            DeclareLaunchArgument("imu_prediction_rotation_only", default_value="true"),
            DeclareLaunchArgument("imu_prediction_debug", default_value="true"),
            DeclareLaunchArgument("enable_yaw_search_initializer", default_value="false"),
            DeclareLaunchArgument(
                "yaw_search_degrees",
                default_value="[-12.0, -8.0, -4.0, 0.0, 4.0, 8.0, 12.0]",
            ),
            DeclareLaunchArgument("yaw_search_score_max_correspondence_distance", default_value="1.5"),
            DeclareLaunchArgument("yaw_search_min_correspondences", default_value="500"),
            DeclareLaunchArgument("yaw_search_use_weighted_rmse", default_value="true"),
            DeclareLaunchArgument("yaw_search_vertical_axis", default_value="z"),
            DeclareLaunchArgument("yaw_search_debug", default_value="true"),
            DeclareLaunchArgument("enable_motion_prior", default_value="false"),
            DeclareLaunchArgument("motion_prior_translation_sigma", default_value="0.35"),
            DeclareLaunchArgument("motion_prior_z_sigma", default_value="0.12"),
            DeclareLaunchArgument("motion_prior_roll_pitch_sigma_deg", default_value="6.0"),
            DeclareLaunchArgument("motion_prior_yaw_sigma_deg", default_value="30.0"),
            DeclareLaunchArgument("motion_prior_weight", default_value="1.0"),
            DeclareLaunchArgument("motion_prior_apply_during_recovery", default_value="true"),
            DeclareLaunchArgument("motion_prior_debug", default_value="true"),
            DeclareLaunchArgument("enable_map_update_quality_gate", default_value="false"),
            DeclareLaunchArgument("map_update_max_weighted_rmse_ratio", default_value="1.25"),
            DeclareLaunchArgument("map_update_max_rmse", default_value="0.35"),
            DeclareLaunchArgument("map_update_max_translation_delta", default_value="0.75"),
            DeclareLaunchArgument("map_update_max_rotation_delta_deg", default_value="10.0"),
            DeclareLaunchArgument("map_update_min_correspondences", default_value="2500"),
            DeclareLaunchArgument("map_update_debug", default_value="true"),
            DeclareLaunchArgument("config_file", default_value=""),
            OpaqueFunction(function=_launch_setup),
            Node(
                package="rviz2",
                executable="rviz2",
                output={"both": "log"},
                arguments=["-d", PathJoinSubstitution([current_pkg, "rviz", "genz_icp_ros2.rviz"])],
                condition=IfCondition(LaunchConfiguration("visualize")),
            ),
            ExecuteProcess(
                cmd=["ros2", "bag", "play", LaunchConfiguration("bagfile")],
                output="screen",
                condition=IfCondition(
                    PythonExpression(["'", LaunchConfiguration("bagfile"), "' != ''"])
                ),
            ),
        ]
    )
