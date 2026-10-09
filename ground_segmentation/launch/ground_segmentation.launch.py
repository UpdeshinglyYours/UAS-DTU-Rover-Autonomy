from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='target_frame',
            default_value='base_link',
            description='Target coordinate frame for pointcloud and laserscan'
        ),
        DeclareLaunchArgument(
            name='max_slope_angle_deg',
            default_value='30.0',
            description='Maximum traversable slope angle in degrees'
        ),
        DeclareLaunchArgument(
            name='max_step_height',
            default_value='0.12',
            description='Maximum step jump in meters before flagging an obstacle'
        ),
        DeclareLaunchArgument(
            name='max_height',
            default_value='1.1',
            description='Maximum ceiling cutoff in meters (filters overhead branches)'
        ),
        # Direct Odometry Subscription Arguments (Bypasses TF lookup delays)
        DeclareLaunchArgument(
            name='use_odom_topic',
            default_value='True',
            description='Whether to subscribe directly to odometry topic to extract pitch/roll without TF lag'
        ),
        DeclareLaunchArgument(
            name='odom_topic',
            default_value='/mavros/local_position/odom',
            description='Odometry topic publishing chassis attitude and position'
        ),
        # 2D LiDAR Slope-Filtering Arguments (SF45/B Close-Range Blind Zone Assist)
        DeclareLaunchArgument(
            name='enable_2d_lidar_filtering',
            default_value='True',
            description='Whether to filter fake ramp obstacles out of 2D lidar scan'
        ),
        DeclareLaunchArgument(
            name='scan_2d_in_topic',
            default_value='/lightwarelidar/scan',
            description='Input 2D LaserScan topic from SF45/B'
        ),
        DeclareLaunchArgument(
            name='scan_2d_out_topic',
            default_value='/scan_2d_filtered',
            description='Filtered 2D LaserScan topic for dedicated Nav2 layer'
        ),
        DeclareLaunchArgument(
            name='scan_2d_max_range',
            default_value='50.0',
            description='Max range (inf) assigned to cleared fake ramp obstacles for Nav2 free-space raytracing'
        ),
        DeclareLaunchArgument(
            name='enable_drop_detection',
            default_value='True',
            description='Whether to detect and flag negative obstacles / cliff drop-offs'
        ),
        DeclareLaunchArgument(
            name='max_drop_range',
            default_value='4.0',
            description='Maximum range in meters to mark drop-off ledges across unobserved gaps'
        ),

        Node(
            package='ground_segmentation',
            executable='ground_segmentation_node',
            name='ground_segmentation',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'target_frame': LaunchConfiguration('target_frame'),
                'transform_tolerance': 0.05,
                'min_height': -0.5,
                'max_height': LaunchConfiguration('max_height'),
                'angle_min': -0.6293,  # -35 degrees (matches Blickfeld FOV)
                'angle_max': 0.6293,   # +35 degrees
                'angle_increment': 0.0055,
                'scan_time': 0.1,
                'range_min': 1.0,
                'range_max': 41.0,
                'use_inf': True,
                'inf_epsilon': 1.0,
                'queue_size': 50,
                # Dynamic Ground & Slope Filtering Parameters
                'enable_ground_filtering': True,
                'gravity_frame': 'map',
                'max_slope_angle_deg': LaunchConfiguration('max_slope_angle_deg'),
                'max_step_height': LaunchConfiguration('max_step_height'),
                'max_drop_height': 0.18,
                'enable_drop_detection': LaunchConfiguration('enable_drop_detection'),
                'max_drop_range': LaunchConfiguration('max_drop_range'),
                'ground_start_clearance': 0.15,
                'radial_bin_size': 0.10,
                # Direct Odometry Subscription (Bypasses TF lookup delay)
                'use_odom_topic': LaunchConfiguration('use_odom_topic'),
                'odom_topic': LaunchConfiguration('odom_topic'),
                # 3D Voxel Outlier Filter
                'enable_3d_point_filter': True,
                'filter_voxel_size': 0.10,
                'min_points_per_voxel': 2,
                # 2D LiDAR Slope-Filtering Parameters (SF45/B Close-Range Blind Zone Assist)
                'enable_2d_lidar_filtering': LaunchConfiguration('enable_2d_lidar_filtering'),
                'scan_2d_in_topic': LaunchConfiguration('scan_2d_in_topic'),
                'scan_2d_out_topic': LaunchConfiguration('scan_2d_out_topic'),
                'scan_2d_max_range': LaunchConfiguration('scan_2d_max_range'),
                'ramp_hit_tolerance': 0.15,
            }],
            remappings=[
                ('cloud_in', '/bf_lidar/point_cloud_out'),
                ('scan', '/scan'),
            ],
        )
    ])
