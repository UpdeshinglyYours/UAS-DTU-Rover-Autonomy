from setuptools import find_packages, setup

package_name = 'lirovo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/nav2_params.yaml']),
        ('share/' + package_name + '/config', ['config/slam_params.yaml']),
        ('share/' + package_name + '/config', ['config/mapper_params_online_async.yaml']),
        ('share/' + package_name + '/config', ['config/point_cloud_to_laser_scan.yaml']),
        ('share/' + package_name + '/config', ['config/localization.yaml']),
        ('share/' + package_name + '/config', ['config/ekf_localization.yaml']),
        ('share/' + package_name + '/config', ['config/nav2_params_namespaced_default.yaml']),
        ('share/' + package_name + '/config', ['config/apm_config_rover1.yaml']),
        ('share/' + package_name + '/config', ['config/apm_config_rover2.yaml']),
        ('share/' + package_name + '/launch', ['launch/lirovo.launch.py', 'launch/bringup_launch_shared_tf.py', 'launch/navigation_launch_shared_tf.py', 'launch/lirovo_hardware.launch.py', 'launch/lirovo_hardware_plain_odometry.launch.py', 'launch/mavros_hardware.launch.py', 'launch/genz_and_laserscan_standalone.launch.py', 'launch/play_both_bags.launch.py']),
        ('share/' + package_name + '/rviz', ['rviz/nav2_costmap_view.rviz', 'rviz/genz_odometry_view.rviz', 'rviz/combined_two_rover_view.rviz', 'rviz/rover1_genz_odometry_view.rviz', 'rviz/rover2_genz_odometry_view.rviz', 'rviz/rover1_nav2_costmap_view.rviz', 'rviz/rover2_nav2_costmap_view.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abhimanyu',
    maintainer_email='abhimanyu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mavros_bridge = lirovo.mavros_bridge:main',
            'pointcloud_processor = lirovo.pointcloud_processor:main',
            'navigator = lirovo.navigator:main',
            'converter = lirovo.converter:main',
            'livox_deskew_converter = lirovo.livox_deskew_converter:main',
            'pointcloud_deskew_field = lirovo.pointcloud_deskew_field:main',
            'odom_relay = lirovo.odom_relay:main',
            'peer_obstacle_relay = lirovo.peer_obstacle_relay:main',
            'bag_cloud_frame_relay = lirovo.bag_cloud_frame_relay:main',
        ],
    },
)
