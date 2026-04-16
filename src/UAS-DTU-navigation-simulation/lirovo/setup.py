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
        ('share/' + package_name + '/config', ['config/localization.yaml']),
        ('share/' + package_name + '/launch', ['launch/lirovo.launch.py']),
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
        ],
    },
)
