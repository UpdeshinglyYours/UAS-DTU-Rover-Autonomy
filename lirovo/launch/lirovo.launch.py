
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
    slam_params_path = os.path.join(get_package_share_directory(namePackage),'config','slam_params.yaml')
    nav2_params_path = os.path.join(get_package_share_directory(namePackage),'config','nav2_params.yaml')
    print(f"SLAM params path: {slam_params_path}")

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
    
        SetParameter(name='use_sim_time', value=True), #extra addition, meine kiya 
        
         #Node(
         #    package='tf2_ros',
         #    executable='static_transform_publisher',
         #    arguments=['0.25', '-0.15', '0.0', '0', '0', '0', 'base_link', 'livox_frame'],
         #    parameters=[{'use_sim_time': False}], #False
         #    name='static_tf_lidar'
         #),
         #Node(
         #    package='tf2_ros',
         #    executable='static_transform_publisher',
         #    arguments=['0', '0', '0', '1.5708', '0', '0', 'base_link', 'base_footprint'],
         #    parameters=[{'use_sim_time':False}], #False
         #    name='static_tf_base_footprint'
         #),
        # Node(
        #    package='lirovo',
        #     executable='pointcloud_processor',
        #     name='pointcloud_processor',
        #     output='screen',
        #     parameters=[{'use_sim_time':False}]
        # ),
         Node(
             package='pointcloud_to_laserscan',
             executable='pointcloud_to_laserscan_node',
             name='pointcloud_to_laserscan',
             parameters=[{
                 'target_frame': 'base_footprint',
                 'transform_tolerance': 0.05, #0.5,
                 'min_height': 0.2, #0.0 , #-0.3, # bcr bot 3d lidar is 40.5 cm above ground
                 'max_height': 1.0, #0.7 , #1.0,
                 'angle_min': -0.6293,  # -35 degrees
                 'angle_max': 0.6293,   # +35 degrees
                 #'angle_min': -3.14159,
                 #'angle_max': +3.14159,
                 'angle_increment': 0.00872665,
                 'scan_time': 0.125, #0.8, #0.07, #0.8,
                 'range_min': 1.5, #0.2,
                 'range_max': 41.0, #100.0,
                 'use_inf': True,
                 'inf_epsilon': 1.0,
                 'queue_size': 50,
                 'use_sim_time':True, #False,
             }],
             remappings=[
                 ('cloud_in', '/livox/lidar'),
                 ('scan', '/scan'),
             ],
             ),
        
        #Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        #     name='static_tf_odom'
        # ),
        # Node(
        #     package='lirovo',
        #     executable='mavros_bridge',
        #     name='mavros_bridge',
        #     output='screen',
        #     parameters=[{'use_sim_time':False}] #False
        # ),
        #THE MISSING ENGINE: Start GenZ-ICP to generate odometry from the bag
        #IncludeLaunchDescription(
        #    PythonLaunchDescriptionSource(
        #        os.path.join(get_package_share_directory('genz_icp'), 'launch', 'odometry.launch.py') # Make sure this filename is correct!
        #    ),
        #    launch_arguments={
        #        'topic': '/bf_lidar/point_cloud_out', # <--- REPLACE WITH YOUR BAG'S PC2 TOPIC
        #        'publish_odom_tf': 'false',            # Let EKF handle the TF
        #        'use_sim_time': 'false'
        #    }.items()
        #),
      #  vins
        # IncludeLaunchDescription(
        # PythonLaunchDescriptionSource(
        #     os.path.join(get_package_share_directory('vins'), 'launch', 'euroc.launch.py')
        # ),
        # launch_arguments={
        #     # If you want to use a different config than the default in that file:
        #     'config_path': os.path.join(get_package_share_directory('vins'), 'config', 'realsense_d435i', 'realsense_stereo_imu_config.yaml')
        # }.items()
        # ), 
        
         #Node(
         #package='robot_localization',
         #executable='ekf_node',
         #name='ekf_filter_node',
         #output='screen',
         #parameters=[os.path.join(
         #    get_package_share_directory(namePackage),
         #    'config', 'ekf_localization.yaml')],
         #),
         
         #..............................
         #TimerAction(
         #   period=3.0,
         #   actions=[
         #       IncludeLaunchDescription(
         #           PythonLaunchDescriptionSource(
         #               os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
         #           ),
         #           launch_arguments={
         #               'slam_params_file': slam_params_path,
         #               'use_sim_time': 'False'
         #           }.items()
         #       )
         #   ]
        #),
        # Node(
        #     package='lirovo',
        #     executable='navigator',
        #     name='navigator',
        #     output='screen',
        #     parameters=[{'use_sim_time':True}] #False
        # ),
        # Node(
        #     package='lirovo',
        #     executable='converter',
        #     name='converter',
        #     output='screen',
        #     parameters=[{'use_sim_time':False}] #False
        # ),
        # TimerAction(
        #     period=1.0,
        #     actions=[
        #         Node(
        #             package='slam_toolbox',
        #             executable='async_slam_toolbox_node',
        #             name='slam_toolbox',
        #             output='screen',
        #             parameters=[slam_params_path],
        #         ),
        #     ]
        # ),
	
        Node( #bcr bot ke liye add kiya
        package='bcr_bot',
        executable='remapper.py',
        name='remapper',
        output='screen',
       ),        
             TimerAction(
        period=3.0, #10.0,
         actions=[
        Node(
             package='slam_toolbox',
             executable='async_slam_toolbox_node',
             name='slam_toolbox',
             output='screen',
             # We separate the YAML and the explicit dictionary
             parameters=[
                 slam_params_path, 
                {'use_sim_time':True} # This MUST be a separate dictionary entry
             ],
             # HARD OVERRIDE: Force it at the command line level
             arguments=['--ros-args', '-p', 'use_sim_time:=true'] 
        ),
     ]
     ),
     
     #Node(
	#    package='octomap_server',
	#    executable='octomap_server_node',
	#    name='octomap_server',
	#    output='screen',
	#    parameters=[{
	#        'use_sim_time': True,
	#	'resolution': 0.05,                   # 5cm voxel size (matches your costmap)
	#	'frame_id': 'map',                    # The global frame to attach the map to
	#	'base_frame_id': 'base_',    # The robot's base frame
	#	
	#	# Sensor characteristics
	#	'sensor_model.max_range': 50.0,       # Only integrate points within 20m to save CPU
	#	'sensor_model.hit': 0.7,              # Probability increase on a hit
	#	'sensor_model.miss': 0.4,             # Probability decrease on a miss (clearing ghosts)
	#	
	#	# Slicing the map
	#	'pointcloud_min_z': 0.5,              # Ignore the floor so it doesn't become an obstacle
	#	'pointcloud_max_z': 1.0,              # Only map up to 1 meter high
	#	'occupancy_min_z': 0.5,
	#	'occupancy_max_z': 1.0,
	#	
	#	# Ground filtering (highly recommended for 3D LiDARs)
	#	'filter_ground_plane': False, #True,
	#	'ground_filter.distance': 0.20, #0.05,       
	#	'ground_filter.angle': 0.15,          
	 #   }],
	  #  remappings=[
	#	('cloud_in', '/bf_lidar/point_cloud_out'),         # Input: Your simulated 3D LiDAR
	 #   ]
	#),
     
     
        delayed_nav2_launch
    ])
