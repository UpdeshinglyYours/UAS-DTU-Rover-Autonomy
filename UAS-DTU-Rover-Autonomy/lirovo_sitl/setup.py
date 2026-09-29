import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'lirovo_sitl'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='reet',
    maintainer_email='reetkaur2687@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mavros_odom_relay = lirovo_sitl.mavros_odom_relay:main',
            'cmd_vel_to_mavros = lirovo_sitl.cmd_vel_to_mavros:main',
        ],
    },
)
