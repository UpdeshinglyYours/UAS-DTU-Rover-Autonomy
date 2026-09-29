import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import launch.actions


def generate_launch_description():

    """generate combined blickfeld scanner and rviz launch"""
    rviz_config = os.path.join(get_package_share_directory("blickfeld_driver"), "config", "blickfeld_scanner.rviz")
    bf_config = os.path.join(get_package_share_directory("blickfeld_driver"), "config", "driver_config.yaml")

    bf_node = Node(
        package="blickfeld_driver",
        executable="blickfeld_driver_node",
        name="bf_lidar",
        parameters=[bf_config],
        remappings=[
            ("~/ambient_image_out", "~/ambient_image"),
            ("~/diagnostic_out", "~/diagnostic"),
            ("~/imu_out", "~/imu"),
            ("~/intensity_image_out", "~/intensity_image"),
            ("~/point_id_image_out", "~/point_id_image"),
            ("~/point_cloud_out", "~/points_raw"),
            ("~/range_image_out", "~/range_image"),
            ("~/set_scan_pattern_service", "~/set_scan_pattern"),
            ("~/publish_imu_static_tf_service", "~/publish_imu_static_tf"),
        ],
        output="screen",
        on_exit=launch.actions.Shutdown(),
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
        on_exit=launch.actions.Shutdown(),
    )

    return LaunchDescription([bf_node, rviz2_node])
