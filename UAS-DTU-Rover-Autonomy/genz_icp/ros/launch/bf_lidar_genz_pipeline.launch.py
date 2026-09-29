from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

import ast
import os
import tempfile


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


def _qualified_frame(namespace, frame):
    """Prefix `frame` with `<namespace>/` if a namespace is set and the
    frame isn't already qualified (contains '/'). Empty namespace is a no-op."""
    frame = frame.strip()
    ns = namespace.strip().strip("/")
    if not ns or "/" in frame:
        return frame
    return f"{ns}/{frame}"


def _launch_setup(context, *args, **kwargs):
    pkg_share = FindPackageShare("genz_icp")

    # One namespace value drives everything: the two Nodes below and the
    # odometry.launch.py include, whose own `namespace` arg we forward.
    namespace_str = LaunchConfiguration("namespace").perform(context)
    namespace_lc = LaunchConfiguration("namespace")

    base_frame = _qualified_frame(namespace_str, LaunchConfiguration("base_frame").perform(context))
    odom_frame = _qualified_frame(namespace_str, LaunchConfiguration("odom_frame").perform(context))
    lidar_frame = _qualified_frame(namespace_str, LaunchConfiguration("lidar_frame").perform(context))

    lidar_lever_arm = _parse_double_list(
        LaunchConfiguration("lidar_lever_arm").perform(context),
        "lidar_lever_arm",
    )

    log_level = LaunchConfiguration("log_level")

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_lidar_static_tf",
        namespace=namespace_lc,     # per-Node namespace: publishes to /<ns>/tf_static
        output="screen",
        arguments=[
            "--x", LaunchConfiguration("lidar_tf_x"),
            "--y", LaunchConfiguration("lidar_tf_y"),
            "--z", LaunchConfiguration("lidar_tf_z"),
            "--yaw", LaunchConfiguration("lidar_tf_yaw"),
            "--pitch", LaunchConfiguration("lidar_tf_pitch"),
            "--roll", LaunchConfiguration("lidar_tf_roll"),
            "--frame-id", base_frame,
            "--child-frame-id", lidar_frame,
        ],
        # Disable when URDF/robot_state_publisher already publishes base->lidar.
        condition=IfCondition(LaunchConfiguration("enable_static_tf")),
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
        namespace=namespace_lc,     # per-Node namespace: relative topics get prefixed
        output="screen",
        emulate_tty=True,
        parameters=[deskewer_params],
        arguments=["--ros-args", "--log-level", log_level],
    )

    odometry = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, "launch", "odometry.launch.py"])
        ),
        launch_arguments={
            # Forward the same namespace value to odometry.launch.py, which
            # applies it to its own odometry_node via Node(namespace=...).
            "namespace": namespace_lc,
            "topic": LaunchConfiguration("deskewed_cloud_topic"),
            "config_file": LaunchConfiguration("config_file"),
            "base_frame": base_frame,
            "odom_frame": odom_frame,
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
            # Always false here - odometry.launch.py's own rviz Node() is NOT namespaced
            # (genz_icp_ros2.rviz has hardcoded absolute "/genz/..." topics and a plain
            # "Fixed Frame: odom", no <ns> placeholder anywhere), so under a real
            # robot_namespace it would silently show nothing. A properly namespaced rviz is
            # built below instead, when visualize is actually requested.
            "visualize": "false",
        }.items(),
    )

    actions = [static_tf, deskewer, odometry]

    # 2026-08-30: namespaced rviz, built here instead of relying on odometry.launch.py's own
    # (unnamespaced) one. genz_icp_ros2.rviz has no placeholder token convention of its own -
    # do a direct text substitution on the two literal strings that need to change:
    # "/genz/..." (absolute topics, bypass namespace push entirely) and the bare
    # "Fixed Frame: odom". Empty namespace (single-robot default) is a no-op - the file is
    # used as-is, matching pre-namespacing behavior exactly.
    if LaunchConfiguration("visualize").perform(context).lower() in ("true", "1"):
        src_rviz = os.path.join(pkg_share.perform(context), "rviz", "genz_icp_ros2.rviz")
        if namespace_str:
            with open(src_rviz) as f:
                content = f.read()
            content = content.replace("/genz/", f"/{namespace_str}/genz/")
            content = content.replace("Fixed Frame: odom", f"Fixed Frame: {namespace_str}/odom")
            fd, out_rviz = tempfile.mkstemp(
                prefix=f"genz_icp_ros2_{namespace_str}_", suffix=".rviz")
            with os.fdopen(fd, "w") as f:
                f.write(content)
        else:
            out_rviz = src_rviz

        actions.append(Node(
            package="rviz2",
            executable="rviz2",
            output={"both": "log"},
            arguments=["-d", out_rviz],
        ))

    # Plain flat list. Namespacing is per-Node (namespace=...) plus a forwarded
    # `namespace` arg to the odometry include. No GroupAction / PushRosNamespace.
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            # Shared
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("base_frame", default_value="base_link"),
            DeclareLaunchArgument("odom_frame", default_value="odom"),
            DeclareLaunchArgument("lidar_frame", default_value="lidar"),
            DeclareLaunchArgument("log_level", default_value="info"),
            DeclareLaunchArgument("enable_static_tf", default_value="true"),
            DeclareLaunchArgument(
                "config_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("genz_icp"), "config", "rover.yaml"]
                ),
            ),

            # Static TF: Y-forward base_link -> Y-forward lidar.
            # Make these match your physical lidar mounting.
            DeclareLaunchArgument("lidar_tf_x", default_value="0.25"),
            DeclareLaunchArgument("lidar_tf_y", default_value="-0.13"),
            DeclareLaunchArgument("lidar_tf_z", default_value="0.55"),
            DeclareLaunchArgument("lidar_tf_yaw", default_value="0.0"),
            DeclareLaunchArgument("lidar_tf_pitch", default_value="0.0"),
            DeclareLaunchArgument("lidar_tf_roll", default_value="0.0"),

            # Deskewer — relative topic defaults so the node's namespace prefixes them.
            # Absolute paths (leading '/') from the caller are respected as-is.
            DeclareLaunchArgument("raw_cloud_topic", default_value="bf_lidar/point_cloud_out"),
            DeclareLaunchArgument(
                "deskewed_cloud_topic",
                default_value="bf_lidar/point_cloud_deskewed",
            ),
            # 2026-08-30: was "mavros/imu/data" - wrong for this project's actual mavros
            # topic shape. mavros plugins are separate real nodes sitting directly under
            # <ns> (see mavros_hardware.launch.py / plugin.hpp:86, and
            # lirovo_hardware.launch.py's docstring, verified live) - no "mavros/" segment,
            # so the real topic is /<ns>/imu/data, not /<ns>/mavros/imu/data. With the old
            # default this node's namespace push would have resolved to a topic mavros
            # never publishes, silently starving the deskewer/odometry of real IMU data.
            DeclareLaunchArgument("deskew_imu_topic", default_value="imu/data"),
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
            # 2026-08-30: same fix as deskew_imu_topic above - "mavros/imu/data" doesn't
            # match this project's real mavros topic shape (/<ns>/imu/data, no "mavros/"
            # segment).
            DeclareLaunchArgument("odom_imu_topic", default_value="imu/data"),
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
