
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import TimerAction
import os
from ament_index_python.packages import get_package_share_directory

#extra addition, meine kiya
from launch_ros.actions import SetParameter

def generate_launch_description():
    namePackage = 'lirovo'
    #slam_params_path = os.path.join(get_package_share_directory(namePackage),'config','slam_params.yaml')
    nav2_params_path = os.path.join(get_package_share_directory(namePackage),'config','nav2_params.yaml')
    #print(f"SLAM params path: {slam_params_path}")

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
    period=3.0,
    actions=[nav2_launch]
)

    return LaunchDescription([
    
        SetParameter(name='use_sim_time', value=False), #extra addition, meine kiya 
        
         Node(
             package='tf2_ros',
             executable='static_transform_publisher',
             arguments=['0.25', '-0.13', '0.52', '0', '0', '0', 'base_link', 'lidar'],
             parameters=[{'use_sim_time': False}], #False
             name='static_tf_lidar'
         ),
         
         #Node(
         #    package='tf2_ros',
         #    executable='static_transform_publisher',
         #    arguments=['0', '0', '0', '1.5708', '0', '0', 'base_link', 'base_footprint'],
         #    parameters=[{'use_sim_time':False}], #False
         #    name='static_tf_base_footprint'
         #),
      
         Node(
             package='pointcloud_to_laserscan',
             executable='pointcloud_to_laserscan_node',
             name='pointcloud_to_laserscan',
             parameters=[{
                 'target_frame': 'base_link',
                 'transform_tolerance': 0.05, #0.5,
                 'min_height': 0.15, #0.0 , #-0.3, # bcr bot 3d lidar is 40.5 cm above ground
                 'max_height': 1.0, #0.7 , #1.0,
                 'angle_min': -0.6293,  # -35 degrees
                 'angle_max': 0.6293,   # +35 degrees
                 #'angle_min': -3.14159,
                 #'angle_max': +3.14159,
                 'angle_increment': 0.0055, #0.00872665,
                 'scan_time': 0.1, #0.8, #0.07, #0.8,
                 'range_min': 1.5, #0.2,
                 'range_max': 41.0, #100.0,
                 'use_inf': True,
                 'inf_epsilon': 1.0,
                 'queue_size': 50,
                 'enable_3d_point_filter': True,    # <--- Enable 3D point filter!
                 'filter_voxel_size': 0.10, #0.15,         # 15cm 3D voxel box
                 'min_points_per_voxel': 2, #2,         # Must have >= 2 points in the 3D voxel 
                 'use_sim_time':False, #False,
                 
             }],
             remappings=[
                 ('cloud_in', '/bf_lidar/point_cloud_out'),
                 ('scan', '/scan'),
             ],
             ),
         
        Node(
            package='lirovo',
            executable='odom_to_tf',
            name='odom_to_tf',
            output='screen',
            parameters=[{'use_sim_time':False}] 
        ),
        
        
        # Node(
        #     package='tf2_ros',
        #      executable='static_transform_publisher',
        #      arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        #      name='static_tf_odom'
        #  ),
        
         
	
        
    
     
     
        delayed_nav2_launch
    ])
