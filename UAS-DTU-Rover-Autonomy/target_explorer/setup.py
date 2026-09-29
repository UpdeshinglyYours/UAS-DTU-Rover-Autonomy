import os
from glob import glob

from setuptools import setup


package_name = 'target_explorer'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aryaman',
    maintainer_email='aryaman@todo.todo',
    description='Goal-directed exploration helper for Nav2.',
    license='TODO',
    entry_points={
        'console_scripts': [
            'target_explorer_node = target_explorer.target_explorer_node:main',
        ],
    },
)
