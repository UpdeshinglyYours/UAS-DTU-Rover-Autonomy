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
        #('share/' + package_name + '/config', ['config/slam_params.yaml']),
        #('share/' + package_name + '/config', ['config/mapper_params_online_async.yaml']),
        ('share/' + package_name + '/config', ['config/point_cloud_to_laser_scan.yaml']),
        #('share/' + package_name + '/config', ['config/localization.yaml']),
        #('share/' + package_name + '/config', ['config/ekf_localization.yaml']),
        ('share/' + package_name + '/launch', ['launch/lirovo.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vortex',
    maintainer_email='vortex@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_to_tf = lirovo.odom_to_tf:main',
            #'mavros_bridge = lirovo.mavros_bridge:main',
            #'pointcloud_processor = lirovo.pointcloud_processor:main',
            #'navigator = lirovo.navigator:main',
            #'converter = lirovo.converter:main',
            # Added: Entry point for depth image to PointCloud2 projective geometry node
            #'depth_to_pointcloud = lirovo.depth_to_pointcloud:main',
            'node_monitor = lirovo.node_monitor:main',
        ],
    },
)
