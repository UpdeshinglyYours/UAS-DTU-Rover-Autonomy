from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    share = get_package_share_directory("dtu_prior_map")
    static_yaml = os.path.join(share, "maps", "dtu_static.yaml")
    road_mask_yaml = os.path.join(share, "maps", "dtu_road_mask.yaml")
    centerline_cost_yaml = os.path.join(share, "maps", "dtu_centerline_cost.yaml")
    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_centerline_cost = LaunchConfiguration("enable_centerline_cost")

    managed_nodes = [
        "dtu_static_map_server",
        "dtu_road_mask_server",
        "dtu_road_filter_info_server",
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use the simulation clock for all map lifecycle nodes.",
            ),
            DeclareLaunchArgument(
                "enable_centerline_cost",
                default_value="false",
                description="Launch the centreline-cost map and filter metadata.",
            ),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="dtu_static_map_server",
                output="screen",
                parameters=[
                    {
                        "yaml_filename": static_yaml,
                        "topic_name": "dtu_static_map",
                        "frame_id": "map",
                        "use_sim_time": use_sim_time,
                    }
                ],
            ),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="dtu_road_mask_server",
                output="screen",
                parameters=[
                    {
                        "yaml_filename": road_mask_yaml,
                        "topic_name": "dtu_road_mask",
                        "frame_id": "map",
                        "use_sim_time": use_sim_time,
                    }
                ],
            ),
            Node(
                package="nav2_map_server",
                executable="costmap_filter_info_server",
                name="dtu_road_filter_info_server",
                output="screen",
                parameters=[
                    {
                        "type": 0,
                        "filter_info_topic": "dtu_road_filter_info",
                        # Consumers such as global_costmap are namespaced. An
                        # absolute name keeps the metadata pointed at the map
                        # server's root-level topic in every namespace.
                        "mask_topic": "/dtu_road_mask",
                        "base": 0.0,
                        "multiplier": 1.0,
                        "use_sim_time": use_sim_time,
                    }
                ],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_dtu_prior_map",
                output="screen",
                parameters=[
                    {
                        "autostart": True,
                        "node_names": managed_nodes,
                        "use_sim_time": use_sim_time,
                    }
                ],
            ),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="dtu_centerline_cost_server",
                output="screen",
                parameters=[
                    {
                        "yaml_filename": centerline_cost_yaml,
                        "topic_name": "dtu_centerline_cost",
                        "frame_id": "map",
                        "use_sim_time": use_sim_time,
                    }
                ],
                condition=IfCondition(enable_centerline_cost),
            ),
            Node(
                package="nav2_map_server",
                executable="costmap_filter_info_server",
                name="dtu_centerline_filter_info_server",
                output="screen",
                parameters=[
                    {
                        "type": 0,
                        "filter_info_topic": "dtu_centerline_filter_info",
                        "mask_topic": "/dtu_centerline_cost",
                        "base": 0.0,
                        "multiplier": 1.0,
                        "use_sim_time": use_sim_time,
                    }
                ],
                condition=IfCondition(enable_centerline_cost),
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_dtu_centerline",
                output="screen",
                parameters=[
                    {
                        "autostart": True,
                        "node_names": [
                            "dtu_centerline_cost_server",
                            "dtu_centerline_filter_info_server",
                        ],
                        "use_sim_time": use_sim_time,
                    }
                ],
                condition=IfCondition(enable_centerline_cost),
            ),
        ]
    )
