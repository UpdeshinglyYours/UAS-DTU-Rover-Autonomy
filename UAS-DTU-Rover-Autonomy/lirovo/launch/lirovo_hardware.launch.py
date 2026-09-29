
"""
Real-hardware counterpart to lirovo.launch.py - same multi-robot architecture (namespacing,
peer_obstacle_relay, the shared-/tf nav2 fork), but:
  - EKF re-enabled (fusing genz_icp odometry + real MAVROS GPS/IMU 
Assumes a `mavros` node is already running separately (mavros_hardware.launch.py, same
robot_namespace), so that <robot_namespace>/imu/data and
<robot_namespace>/local_position/odom actually exist, matching ekf_localization.yaml's
imu0/odom1. Not something this launch file starts itself.

NOTE the topic shape (corrected 2026-08-30, verified live): there is NO "mavros/" segment.
mavros plugins are separate real nodes (plugin.hpp:86) and the global "-r __ns:=<ns>" remap
overrides their constructor namespace, so they sit directly under <ns> as /<ns>/imu,
/<ns>/local_position, etc. Do NOT re-add name='mavros' to mavros_hardware.launch.py - that
becomes a global "-r __node:=mavros" remap which collapses every plugin onto one FQN, breaks
apm_config.yaml's per-plugin keys (leaving frame_ids bare and un-namespaced), and flattens
these topics to <ns>/mavros/data and <ns>/mavros/odom.

Heads up, seen live this session (2026-08-27) testing against SITL with this exact
mavros/mavros_msgs build: several stock mavros plugins collide with each other on shared
topic names and CRASH THE WHOLE NODE on startup - specifically cellular_status vs sys_status
fighting over "~/status", and companion_process_status/debug_value both wanting "~/send".
The default plugin set is not safe on this build - use an explicit plugin_allowlist, e.g.
    -p plugin_allowlist:="[sys_status,imu,local_position,global_position,command]"
(add "setpoint_velocity" too for MAVROS's ENU->NED cmd_vel bridge if needed). That testing
used a manual `ros2 run mavros mavros_node -r __ns:=...` invocation rather than a proper
namespaced launch, though, so always verify the actual topic names live
(`ros2 topic list | grep <namespace>`) against however you actually launch it - don't take
either this comment or ekf_localization.yaml's topic names on faith.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import SetParameter
from launch_ros.actions import SetRemap
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import TimerAction
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch.conditions import IfCondition
from nav2_common.launch import ReplaceString
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    namePackage = 'lirovo'
    nav2_params_path = os.path.join(get_package_share_directory(namePackage), 'config', 'nav2_params.yaml')
    ekf_params_path = os.path.join(get_package_share_directory(namePackage), 'config', 'ekf_localization.yaml')

    # This rover's own namespace, e.g. "rover1". Empty default keeps single-robot behavior
    robot_namespace = LaunchConfiguration('robot_namespace')

    # The OTHER rover's namespace, for peer_obstacle_relay. Empty disables the relay entirely.
    peer_robot_namespace = LaunchConfiguration('peer_robot_namespace')

    # True only for rosbag playback testing - see DeclareLaunchArgument below for details.
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Real 3D lidar's native pointcloud topic (relative to this rover's namespace) - was
    # "/bf_lidar/point_cloud_out" in the original unnamespaced lirovo.launch.py, before it got
    # (correctly, for sim) redirected to bcr_bot's simulated topic. Updated 2026-08-30 to match
    # the real blickfeld_driver_node invocation actually used on hardware, which - via its own
    # node-name-based topic naming, not a ROS __ns:= namespace push - publishes to
    # "/<robot_namespace>/point_cloud_out" (no "bf_lidar" segment). If the driver invocation
    # changes, verify live with `ros2 topic list | grep point_cloud` before trusting this.
    lidar_topic = PythonExpression(['"/', robot_namespace, '/point_cloud_out"'])

    # Same two-pass substitution as lirovo.launch.py: <robot_ns_noslash> (no leading slash,
    # for frame_id fields tf2 would otherwise reject) applied first, by us; <robot_namespace>
    # (with leading slash, for absolute topics) applied second, by nav2_bringup's own
    # ReplaceString inside the fork below.
    nav2_params_path_namespaced = ReplaceString(
        source_file=nav2_params_path,
        replacements={'<robot_ns_noslash>': robot_namespace},
    )
    # ekf_localization.yaml isn't loaded through nav2_bringup at all, so both passes have to be
    # done here directly - it never goes through the fork's own <robot_namespace> substitution.
    # rviz2 is OPT-IN (use_rviz:=True).
    # the same two placeholders as every yaml here - <robot_ns_noslash> for frame names
    # (Fixed Frame: <ns>/odom) and <robot_namespace> for topic names (/<ns>/scan) - so ONE file
    # serves both rovers. Same trick nav2 uses for its own rviz config
    # (nav2_bringup/launch/rviz_launch.py:63-65). Pick which view with rviz_config:=.
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    rviz_config_path = PathJoinSubstitution(
        [get_package_share_directory(namePackage), 'rviz',
         PythonExpression(['"', rviz_config, '.rviz"'])])
    rviz_config_namespaced = ReplaceString(
        source_file=ReplaceString(
            source_file=rviz_config_path,
            replacements={'<robot_ns_noslash>': robot_namespace},
        ),
        replacements={'<robot_namespace>': ('/', robot_namespace)},
    )

    ekf_params_path_namespaced = ReplaceString(
        source_file=ReplaceString(
            source_file=ekf_params_path,
            replacements={'<robot_ns_noslash>': robot_namespace},
        ),
        replacements={'<robot_namespace>': ('/', robot_namespace)},
    )

    pkg_nav2_dir = get_package_share_directory('nav2_bringup')
    pkg_lirovo_dir = get_package_share_directory('lirovo')
    pkg_genz_icp_dir = get_package_share_directory('genz_icp')

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            #nav2_bringup remaps /tf -> tf, isolating
            # each rover onto its own empty <namespace>/tf instead of the shared global /tf
            os.path.join(pkg_lirovo_dir, 'launch', 'bringup_launch_shared_tf.py')
        ),
        launch_arguments={
            'params_file': nav2_params_path_namespaced,
            'autostart': 'True',
            'map': 'map',
            # Same fix as lirovo.launch.py: no real map yaml exists yet, so map_server was
            # throwing on this placeholder and aborting the whole localization lifecycle group
            # (amcl included). map->odom comes from static_tf_odom below regardless. Flip back
            # to 'True' and pass a real 'map' path once an actual map yaml exists for this rover.
            'use_localization': 'False',
            'namespace': robot_namespace,
            'use_namespace': 'True',
        }.items()
    )
    delayed_nav2_launch = TimerAction(period=3.0, actions=[nav2_launch])

    # 2026-09-01: switched from including odometry.launch.py directly to the fuller
    # bf_lidar_genz_pipeline.launch.py, which adds a real IMU-based deskewer node
    # (imu_rotation_deskew_node) ahead of odometry.launch.py instead of relying on
    # odometry.launch.py's own internal deskew:=true. publish_odom_tf is hardcoded 'false'
    # inside the pipeline itself (robot_localization stays the only odom -> base_link
    # broadcaster), so it's not passed here.
    #
    # enable_static_tf:='false' - the pipeline has its OWN base_link -> lidar static TF
    # publisher; this file already publishes one below (static_tf_lidar). Leaving the
    # pipeline's enabled would recreate the exact duplicate-static-TF race found and fixed
    # earlier: two transient-local publishers for the same edge, non-deterministic winner.
    # visualize:='false' - the pipeline can also spawn its own rviz2; this file already has
    # its own use_rviz/rviz_config mechanism, don't want a second rviz instance.
    # raw_cloud_topic - the pipeline's own default ('bf_lidar/point_cloud_out') assumes a
    # "bf_lidar" topic segment this project's real driver naming doesn't have (see
    # lidar_topic above: "/<robot_namespace>/point_cloud_out", no "bf_lidar" segment) -
    # override with the same absolute lidar_topic used elsewhere in this file.
    # base_frame/odom_frame are left at the pipeline's own defaults ("base_link"/"odom") -
    # its _qualified_frame() helper auto-prefixes them with `namespace`, producing the same
    # "<ns>/base_link" / "<ns>/odom" this file used to build manually via PythonExpression.
    genz_icp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_genz_icp_dir, 'launch', 'bf_lidar_genz_pipeline.launch.py')
        ),
        launch_arguments={
            'namespace': robot_namespace,
            'raw_cloud_topic': lidar_topic,
            'enable_static_tf': 'false',
            'visualize': 'false',
        }.items(),
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'robot_namespace',
            default_value='',
            description='This rover\'s own namespace, e.g. "rover1". Empty = unnamespaced single-robot use.'
        ),
        DeclareLaunchArgument(
            'peer_robot_namespace',
            default_value='',
            description='The OTHER rover\'s namespace, e.g. rover2 when this instance is rover1. Empty (default) disables the peer obstacle relay.'
        ),

        DeclareLaunchArgument(
            'use_rviz',
            default_value='False',
            description='Start rviz2 for THIS rover. Off by default - two rovers means two '
                        'rviz2 instances, which is heavy; each is namespaced so they do not '
                        'collide on the node name.'
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value='nav2_costmap_view',
            description='Which config in lirovo/rviz to load, WITHOUT the .rviz suffix. '
                        'One of: nav2_costmap_view, genz_odometry_view.'
        ),

        # 2026-09-08: default flipped to True per explicit request. The SetRemap below makes
        # every node in this file (and rviz2, via the global SetParameter) follow
        # /<robot_namespace>/clock automatically - but with NO bag actually publishing that
        # topic, every one of these nodes now sits FROZEN at time zero the moment you launch
        # this file plain (no /clock publisher = sim clock never advances = TF/timers/mavros
        # time-sync never fire). Pass use_sim_time:=false explicitly for real-FCU/no-bag runs,
        # or start `ros2 bag play ... --clock --ros-args -r /clock:=/<robot_namespace>/clock`
        # (same robot_namespace value) BEFORE/alongside this launch.
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Requires a bag publishing /<robot_namespace>/clock or every node '
                        'freezes at time zero. Pass use_sim_time:=false for real-FCU runs '
                        'with no bag.'
        ),
        # Absolute topic (leading slash) - namespace push (namespace=robot_namespace on the
        # Nodes below) does NOT rewrite this, same reason /clock isn't touched by mavros'
        # __ns:= remap for /uas<N>. Must be remapped explicitly so each rover's nodes pick up
        # ITS OWN bag's clock, not fight over one shared plain /clock.
        SetRemap(src='/clock', dst=PythonExpression(['"/', robot_namespace, '/clock"'])),

        SetParameter(name='use_sim_time', value=use_sim_time),

        # map -> odom - nothing else provides this without AMCL/SLAM running. EKF (below)
        # publishes odom -> base_link itself (world_frame == odom_frame in
        # ekf_localization.yaml), so this and EKF's own broadcast compose into a complete
        # chain without conflicting - they publish different, non-overlapping links.
        # namespace=robot_namespace on every Node() in this file (missing before, found live
        # this session running two-rover node lists side by side) - remappings/parameters
        # already sent their TOPICS to the right place, but without this the NODE ITSELF still
        # registered as the bare "/static_tf_odom" etc - fine for one rover, but two rovers
        # launched together would collide on that exact node name (ros2 refuses two nodes
        # sharing one full name). ekf_filter_node's namespace push also required rewriting
        # ekf_localization.yaml's top-level key from "ekf_filter_node" to "/**" - a bare
        # unqualified key only ever matches a node in the ROOT namespace, so once this node's
        # FQN became "/<ns>/ekf_filter_node" the whole params file silently stopped applying
        # (confirmed live: odom1/imu0 never loaded, EKF ended up subscribed to nothing).
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            namespace=robot_namespace,
            arguments=['0', '0', '0', '0', '0', '0',
                       PythonExpression(['"', robot_namespace, '/map"']),
                       PythonExpression(['"', robot_namespace, '/odom"'])],
            name='static_tf_odom',
        ),

        # base_link -> lidar: real measured sensor mount offset (2026-08-30, this rover's actual
        # hardware). x/y/z in meters, yaw/pitch/roll in radians - old positional
        # static_transform_publisher order was [x y z yaw pitch roll], now made explicit here as
        # named flags so it's unambiguous which axis is which. yaw-only rotation (no roll) -
        # corrected 2026-08-30 after roll=1.57 was found to put real lidar points outside
        # pointcloud_to_laserscan's angle filter window.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            namespace=robot_namespace,
            name='static_tf_lidar',
            arguments=['--x', '0', '--y', '0', '--z', '0',
                       '--yaw', '0', '--pitch', '0', '--roll', '0',
                       '--frame-id', PythonExpression(['"', robot_namespace, '/base_link"']),
                       '--child-frame-id', PythonExpression(['"', robot_namespace, '/lidar"'])],
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            namespace=robot_namespace,
            name='pointcloud_to_laserscan',
            parameters=[{
                'target_frame': PythonExpression(['"', robot_namespace, '/base_link"']),
                'transform_tolerance': 0.05,
                # 'min_height': 0.2,
                # 'max_height': 1.0,
                # 'angle_min': -0.6293,
                # 'angle_max': 0.6293,
                'min_height': -0.8,
                'max_height': 0.65,
                'angle_min': -0.55,
                'angle_max': 0.7,
                'angle_increment': 0.00872665,
                'scan_time': 0.1,
                'range_min': 1.5,
                'range_max': 41.0,
                'use_inf': True,
                'inf_epsilon': 1.0,
                'queue_size': 50,
                'use_sim_time': use_sim_time,
            }],
            remappings=[
                ('cloud_in', lidar_topic),
                ('scan', PythonExpression(['"/', robot_namespace, '/scan"'])),
            ],
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            namespace=robot_namespace,
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_params_path_namespaced],
        ),

        genz_icp_launch,

        # Relays this rover's own /scan UNCHANGED (frame_id stays this rover's own sensor
        # frame) into the peer rover's nav2 input topic - see peer_obstacle_relay.py's
        # docstring for why raw (not pre-transformed) is required for correct TF-based
        # marking/clearing. Same node as lirovo.launch.py, only active when
        # peer_robot_namespace is set.
        Node(
            package='lirovo',
            executable='peer_obstacle_relay',
            namespace=robot_namespace,
            name='peer_obstacle_relay',
            output='screen',
            parameters=[{
                'publish_rate_hz': 5.0,
            }],
            remappings=[
                ('scan_in', PythonExpression(['"/', robot_namespace, '/scan"'])),
                ('scan_out', PythonExpression(['"/', peer_robot_namespace, '/obstacles_in"'])),
            ],
            condition=IfCondition(PythonExpression(['"', peer_robot_namespace, '" != ""'])),
        ),

        # namespace=robot_namespace matters for more than the node name: nav2_costmap_view's
        # goal tool is nav2_rviz_plugins/GoalTool, which has no Topic field and publishes to a
        # RELATIVE "goal_pose" - under this namespace that resolves to /<ns>/goal_pose, which
        # is what this rover's bt_navigator listens on. Unnamespaced it would publish a bare
        # /goal_pose that nothing subscribes to.
        Node(
            condition=IfCondition(use_rviz),
            package='rviz2',
            executable='rviz2',
            namespace=robot_namespace,
            name='rviz2',
            arguments=['-d', rviz_config_namespaced],
            output='screen',
        ),

        delayed_nav2_launch,
    ])
