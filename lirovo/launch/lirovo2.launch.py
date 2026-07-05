
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import TimerAction
import os
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import SetParameter
from launch.conditions import IfCondition

def generate_launch_description():



    SLAM_DELAY = 2.0
    NAV2_DELAY = 5.0


    namePackage = 'lirovo'

    pkg_share = get_package_share_directory(namePackage)

    slam_params_path = os.path.join(
        pkg_share,
        'config',
        'slam_params2.yaml'
    )

    nav2_params_path = os.path.join(
        pkg_share,
        'config',
        'nav2_params.yaml'
    )    
    pkg_nav2_dir = get_package_share_directory('nav2_bringup')

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'params_file': nav2_params_path,
            'autostart': 'True',
            'map': 'map',  
            
        }.items() 
    )
    delayed_nav2_launch = TimerAction(
    period=NAV2_DELAY,
    actions=[nav2_launch]
)

    return LaunchDescription([
        # GLOBAL PARAMETERS
        SetParameter(name='use_sim_time', value=True),


        # STATIC TFs
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0.5', '0', '0', '0', 'base_link', 'lidar'],
            name='static_tf_lidar'
        ),
    
        # Rotate frame because LiDAR / localization stack
        # uses Y-forward convention while Nav2 assumes X-forward.

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '1.5708', '0', '0', 'base_link', 'base_footprint'],
            name = 'static_tf_basefootprint'
        ),


        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            parameters=[{
                'target_frame': 'lidar',
                'transform_tolerance': 0.5, #0.5,
                'angle_min': -3.14159,
                'angle_max': +3.14159,
                'angle_increment': 0.00872665,
                'scan_time': 0.1, #0.8,
                'use_inf': True,
                'inf_epsilon': 1.0,
                'queue_size': 50,
            }],
            remappings=[
                ('cloud_in', '/bf_lidar/point_cloud_out'),
                ('scan', '/scan'),
            ],
        ),
        # LOCALIZATION (GENZ ICP, VINS-Fusion, EKF)

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('genz_icp'), 'launch', 'odometry.launch.py') 
            ),
            launch_arguments={
                'topic': '/bf_lidar/point_cloud_out', 
                'publish_odom_tf': 'True',            # Let EKF handle the TF
            }.items()
        ),

        #vins
        # IncludeLaunchDescription(
        # PythonLaunchDescriptionSource(
        #     os.path.join(get_package_share_directory('vins'), 'launch', 'euroc.launch.py')
        # ),
        # launch_arguments={
        #     # If you want to use a different config than the default in that file:
        #     'config_path': os.path.join(get_package_share_directory('vins'), 'config', 'realsense_d435i', 'realsense_stereo_imu_config.yaml')
        # }.items()
        # ), 
        
        # Node(
        # package='robot_localization',
        # executable='ekf_node',
        # name='ekf_filter_node',
        # output='screen',
        # parameters=[os.path.join(
        #     get_package_share_directory(namePackage),
        #     'config', 'localization.yaml')],
        # ),

    
        TimerAction(
        period=SLAM_DELAY,
        actions=[
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='both',
            respawn=True,
            respawn_delay=2.0,
            # We separate the YAML and the explicit dictionary
            parameters=[
                slam_params_path, 
            ],
                            ),
    ]
    ),
       delayed_nav2_launch, 


    TimerAction(
    period=6.0,
    actions=[
        Node(
            package='lirovo',
            executable='startup_report',
            output='screen'
        )
    ]
),
    Node(
        package='lirovo',
        executable='stack_watchdog',
        name='stack_watchdog',
        output='screen',

        parameters=[{
            'odom_topic': '/genz/odometry',
            'scan_topic': '/scan',
            'pc_topic': '/bf_lidar/point_cloud_out',

            'odom_timeout': 2.0,
            'scan_timeout': 2.0,
            'pc_timeout': 2.0,

            'check_frequency': 1.0,
            'check_tf': True,
        }],

        respawn=True,
        respawn_delay=2.0,
    ),


    ])
