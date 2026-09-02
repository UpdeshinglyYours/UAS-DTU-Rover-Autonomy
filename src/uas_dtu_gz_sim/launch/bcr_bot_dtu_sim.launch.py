#!/usr/bin/python3

"""Run BCR Bot and Nav2 against the georeferenced DTU prior map."""

from os.path import join

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


# Map-frame poses derived from the documented road-goal reference route. The
# model's thin X axis follows the road tangent, so its 1.60 m Y axis lies across
# the road. Alternating +/-0.65 m lateral offsets leave at least 2.04 m of
# physical road corridor on either side of every barricade.
BARRICADE_COURSE_POSES = (
    ("dtu_barricade_1", 475.674465, 378.623476, -1.895832),
    ("dtu_barricade_2", 478.468571, 366.197206, -0.252160),
    ("dtu_barricade_3", 494.286929, 363.464159, -0.252160),
    ("dtu_barricade_4", 507.777965, 359.540692, -0.053098),
)


def generate_launch_description():
    sim_path = get_package_share_directory("uas_dtu_gz_sim")
    bcr_bot_path = get_package_share_directory("bcr_bot")
    dtu_map_path = get_package_share_directory("dtu_prior_map")
    nav2_path = get_package_share_directory("nav2_bringup")
    gz_sim_path = get_package_share_directory("ros_gz_sim")

    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    use_rviz = LaunchConfiguration("use_rviz")
    headless = LaunchConfiguration("headless")
    enable_road_preference = LaunchConfiguration("enable_road_preference")
    enable_centerline_tether = LaunchConfiguration("enable_centerline_tether")
    cost_travel_multiplier = LaunchConfiguration("cost_travel_multiplier")
    reference_route_spacing = LaunchConfiguration("reference_route_spacing")
    spawn_test_obstacle = LaunchConfiguration("spawn_test_obstacle")
    spawn_barricade_course = LaunchConfiguration("spawn_barricade_course")
    test_obstacle_map_x = LaunchConfiguration("test_obstacle_map_x")
    test_obstacle_map_y = LaunchConfiguration("test_obstacle_map_y")
    test_obstacle_yaw = LaunchConfiguration("test_obstacle_yaw")
    initial_map_x = LaunchConfiguration("initial_map_x")
    initial_map_y = LaunchConfiguration("initial_map_y")
    initial_yaw = LaunchConfiguration("initial_yaw")
    world_file = LaunchConfiguration("world_file")
    world_name = LaunchConfiguration("world_name")

    preference_enabled = PythonExpression(
        [
            "'",
            enable_centerline_tether,
            "'.lower() == 'true' or '",
            enable_road_preference,
            "'.lower() == 'true'",
        ]
    )
    preference_info_topic = PythonExpression(
        [
            "'/dtu_centerline_filter_info' if '",
            enable_centerline_tether,
            "'.lower() == 'true' else '/dtu_road_filter_info'",
        ]
    )

    nav2_params = join(sim_path, "config", "bcr_bot_dtu_nav2.yaml")
    configured_nav2_params = RewrittenYaml(
        source_file=nav2_params,
        param_rewrites={
            (
                "global_costmap.global_costmap.ros__parameters."
                "road_preference_filter.enabled"
            ): preference_enabled,
            (
                "global_costmap.global_costmap.ros__parameters."
                "road_preference_filter.filter_info_topic"
            ): preference_info_topic,
            (
                "planner_server.ros__parameters.GridBased."
                "cost_travel_multiplier"
            ): cost_travel_multiplier,
        },
        convert_types=True,
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(gz_sim_path, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": PythonExpression(
                [
                    "'",
                    world_file,
                    " -r -s' if '",
                    headless,
                    "'.lower() == 'true' else '",
                    world_file,
                    " -r'",
                ]
            )
        }.items(),
    )

    robot_description = Command(
        [
            "xacro ",
            join(bcr_bot_path, "urdf", "bcr_bot.xacro"),
            " camera_enabled:=false",
            " stereo_camera_enabled:=false",
            " three_d_lidar_enabled:=true",
            " odometry_source:=world",
            " sim_gz:=true",
        ]
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": use_sim_time,
            }
        ],
        remappings=[("/joint_states", "/bcr_bot/joint_states")],
    )

    # Gazebo stays local around the origin. The map placement is supplied only
    # by map->odom below, so world odometry starts close to zero.
    spawn_bcr_bot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_bcr_bot",
        output="screen",
        arguments=[
            "-topic",
            "/robot_description",
            "-name",
            "bcr_bot",
            "-allow_renaming",
            "false",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.28",
            "-Y",
            initial_yaw,
        ],
    )

    # Optional, reproducible validation obstacle. Its default map pose lies on
    # the road-goal reference route about 15 m from the start. Since map->odom
    # has no rotation, subtracting the map translation gives Gazebo world XY.
    spawn_partial_road_box = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_dtu_partial_road_box",
        output="screen",
        arguments=[
            "-file",
            join(sim_path, "models", "dtu_partial_road_box", "model.sdf"),
            "-name",
            "dtu_partial_road_box",
            "-allow_renaming",
            "false",
            "-x",
            PythonExpression([test_obstacle_map_x, " - ", initial_map_x]),
            "-y",
            PythonExpression([test_obstacle_map_y, " - ", initial_map_y]),
            "-z",
            "0.6",
            "-Y",
            test_obstacle_yaw,
        ],
        condition=IfCondition(spawn_test_obstacle),
    )

    spawn_barricades = [
        Node(
            package="ros_gz_sim",
            executable="create",
            name=f"spawn_{entity_name}",
            output="screen",
            arguments=[
                "-file",
                join(sim_path, "models", "dtu_barricade", "model.sdf"),
                "-name",
                entity_name,
                "-allow_renaming",
                "false",
                "-x",
                PythonExpression([str(map_x), " - ", initial_map_x]),
                "-y",
                PythonExpression([str(map_y), " - ", initial_map_y]),
                "-z",
                "1.0",
                "-Y",
                str(yaw),
            ],
            condition=IfCondition(spawn_barricade_course),
        )
        for entity_name, map_x, map_y, yaw in BARRICADE_COURSE_POSES
    ]

    gazebo_joint_state_topic = [
        "/world/",
        world_name,
        "/model/bcr_bot/joint_state",
    ]
    gazebo_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="bcr_bot_gz_bridge",
        output="screen",
        arguments=[
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            # The current BCR GPU LiDAR publishes its LaserScan on the Gazebo
            # topic /points and its point cloud on /points/points.
            "/points@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/points/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            [
                *gazebo_joint_state_topic,
                "@sensor_msgs/msg/JointState[gz.msgs.Model",
            ],
        ],
        remappings=[
            ("/odom", "/bcr_bot/odom"),
            ("/points", "/bcr_bot/scan"),
            ("/points/points", "/bcr_bot/points"),
            ("/imu", "/bcr_bot/imu"),
            (gazebo_joint_state_topic, "/bcr_bot/joint_states"),
        ],
    )

    map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="dtu_map_to_odom",
        output="screen",
        arguments=[
            "--x",
            initial_map_x,
            "--y",
            initial_map_y,
            "--z",
            "0.0",
            "--yaw",
            "0.0",
            "--pitch",
            "0.0",
            "--roll",
            "0.0",
            "--frame-id",
            "map",
            "--child-frame-id",
            "odom",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    dtu_maps = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(dtu_map_path, "launch", "dtu_prior_map.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "enable_centerline_cost": enable_centerline_tether,
        }.items(),
    )

    reference_route = Node(
        package="dtu_prior_map",
        executable="dtu_reference_route_publisher.py",
        name="dtu_reference_route_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "densify_spacing": reference_route_spacing,
            }
        ],
        condition=IfCondition(enable_centerline_tether),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(nav2_path, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "autostart": autostart,
            "params_file": configured_nav2_params,
            # Humble's navigation_launch.py evaluates this in a PythonExpression
            # ("not False"), so preserve Python boolean capitalization.
            "use_composition": "False",
            "use_respawn": "False",
        }.items(),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", join(sim_path, "rviz", "bcr_bot_dtu_sim.rviz")],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            AppendEnvironmentVariable(
                name="GZ_SIM_RESOURCE_PATH",
                value=join(bcr_bot_path, "worlds"),
            ),
            AppendEnvironmentVariable(
                name="GZ_SIM_RESOURCE_PATH",
                value=join(bcr_bot_path, "models"),
            ),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="Run only the Gazebo server (no simulator GUI).",
            ),
            DeclareLaunchArgument(
                "enable_road_preference",
                default_value="false",
                description="Apply the DTU road mask as a soft global cost filter.",
            ),
            DeclareLaunchArgument(
                "enable_centerline_tether",
                default_value="true",
                description=(
                    "Use the normalized centreline mask and publish the nominal "
                    "road-graph route. This takes precedence over the legacy road mask."
                ),
            ),
            DeclareLaunchArgument(
                "cost_travel_multiplier",
                default_value="3.0",
                description="SmacPlanner2D traversal-cost weight.",
            ),
            DeclareLaunchArgument(
                "reference_route_spacing",
                default_value="0.5",
                description="Maximum spacing between reference-route poses (metres).",
            ),
            DeclareLaunchArgument(
                "spawn_test_obstacle",
                default_value="false",
                description="Spawn the partial road-blocking box used by Test B.",
            ),
            DeclareLaunchArgument(
                "spawn_barricade_course",
                default_value="false",
                description=(
                    "Spawn four alternating physical barricades along the "
                    "documented DTU road-goal route."
                ),
            ),
            DeclareLaunchArgument(
                "test_obstacle_map_x",
                default_value="477.294",
                description="Test-box centre X in the map frame.",
            ),
            DeclareLaunchArgument(
                "test_obstacle_map_y",
                default_value="385.465",
                description="Test-box centre Y in the map frame.",
            ),
            DeclareLaunchArgument(
                "test_obstacle_yaw",
                default_value="-0.325",
                description="Test-box yaw; its 2 m side lies across the road.",
            ),
            DeclareLaunchArgument(
                "initial_map_x",
                default_value="487.055800",
                description="Initial base position in the DTU map frame (metres).",
            ),
            DeclareLaunchArgument(
                "initial_map_y",
                default_value="393.518522",
                description="Initial base position in the DTU map frame (metres).",
            ),
            DeclareLaunchArgument(
                "initial_yaw",
                default_value="1.902790",
                description=(
                    "Initial map/world yaw in radians; zero faces map +X/east."
                ),
            ),
            DeclareLaunchArgument(
                "world_file",
                default_value=join(bcr_bot_path, "worlds", "empty.sdf"),
            ),
            DeclareLaunchArgument(
                "world_name",
                default_value="empty_world",
                description="SDF world name used by the joint-state bridge.",
            ),
            gz_sim,
            robot_state_publisher,
            spawn_bcr_bot,
            spawn_partial_road_box,
            *spawn_barricades,
            gazebo_bridge,
            map_to_odom,
            dtu_maps,
            reference_route,
            nav2,
            rviz,
        ]
    )
