from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    input_cloud_topic = LaunchConfiguration("input_cloud_topic")
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "input_cloud_topic",
                default_value="/kinect_camera/points",
                description="PointCloud2 topic consumed by Patchwork++.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use the ROS simulation clock.",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("patchwork_plusplus"),
                        "config",
                        "patchwork_plusplus.param.yaml",
                    ]
                ),
                description="Path to the Patchwork++ parameter file.",
            ),
            Node(
                package="patchwork_plusplus",
                executable="patchwork_plusplus_exe",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "input_cloud_topic": input_cloud_topic,
                        "use_sim_time": ParameterValue(
                            use_sim_time, value_type=bool
                        ),
                    },
                ],
            ),
        ]
    )
