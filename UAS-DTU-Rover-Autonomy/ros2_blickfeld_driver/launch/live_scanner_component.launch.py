import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    """launch file for blickfeld scanner as a component"""
    rviz_config = os.path.join(get_package_share_directory("blickfeld_driver"), "config", "blickfeld_scanner.rviz")

    driver_config_file = os.path.join(get_package_share_directory("blickfeld_driver"), "config", "driver_config.yaml")
    driver_config = None
    # Load the parameters specific to ComposableNode
    with open(driver_config_file, "r") as yaml_file:
        driver_config = yaml.safe_load(yaml_file)["bf_lidar"]["ros__parameters"]

    container = ComposableNodeContainer(
        name="custom_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="blickfeld_driver",
                plugin="blickfeld::ros_interop::BlickfeldDriverComponent",
                name="blickfeld_cube1_front",
                parameters=[driver_config],
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
            ),
        ],
        output="screen",
    )
    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
    )
    
    # Add a static transform broadcaster for "lidar" frame
    static_tf_broadcaster_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.376', '0', '0.249', '-1.570796327', '0', '0', 'blickfeld_cube1_front', 'lidar'],
        name='static_broadcaster_lidar'
    )

    return LaunchDescription([container,static_tf_broadcaster_node])
