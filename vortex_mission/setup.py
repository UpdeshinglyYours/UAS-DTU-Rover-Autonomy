from glob import glob

from setuptools import find_packages, setup

package_name = 'vortex_mission'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='UAS-DTU',
    maintainer_email='uasavionics@gmail.com',
    description='DARPA Triage casualty mission executive',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'casualty_manager = vortex_mission.casualty_manager:main',
            'casualty_manager_simple = vortex_mission.casualty_manager_simple:main',
            'casualty_models_bridge = vortex_mission.casualty_models_bridge:main',
            'rc_orbit_trigger = vortex_mission.rc_orbit_trigger:main',
            'keepout_monitor = vortex_mission.keepout_monitor:main',
            'add_casualty = vortex_mission.add_casualty:main',
            'kinematic_sim = vortex_mission.kinematic_sim:main',
            'rover_console = vortex_mission.rover_console:main',
            'estop = vortex_mission.rover_console:main_estop',
        ],
    },
)
