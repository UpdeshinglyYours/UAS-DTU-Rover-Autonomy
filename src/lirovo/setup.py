from glob import glob

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
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='UASDTU',
    maintainer_email='UASDTUtodo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'mavros_bridge = lirovo.mavros_bridge:main',
            'pointcloud_processor = lirovo.pointcloud_processor:main',
            'navigator = lirovo.navigator:main',
            'converter = lirovo.converter:main',
            'stack_watchdog = lirovo.stack_watchdog:main',
            'startup_report = lirovo.startup_report:main',
            'adaptive_controller = '
            'lirovo.adaptive_velocity_controller_both2:main',
        ],
    },
)
