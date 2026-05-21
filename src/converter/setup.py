from setuptools import setup

package_name = 'converter'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/converter.launch.py']),
        ('share/' + package_name + '/config', ['config/params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Akshit',
    maintainer_email='abc@xyz.com',
    description='/cmd_vel to /rc/override',
    license='MIT',
    entry_points={
        'console_scripts': [
            'converter = converter.converter:main',
        ],
    },
)