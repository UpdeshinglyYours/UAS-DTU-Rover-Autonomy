import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_dir = get_package_share_directory('lightwarelidar')
    default_config_path = os.path.join(pkg_dir, 'config', 'sf45b_params.yaml')

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_path,
        description='Path to YAML configuration file'
    )

    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyUSB0',
        description='Serial port for LightWare SF45/B (e.g. /dev/ttyUSB0)'
    )

    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='Serial communication baudrate'
    )

    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='laser',
        description='TF Frame ID for sensor data'
    )

    publish_scan_arg = DeclareLaunchArgument(
        'publish_laser_scan',
        default_value='true',
        description='Whether to publish 2D sensor_msgs/LaserScan on /scan'
    )

    low_angle_arg = DeclareLaunchArgument(
        'low_angle_limit',
        default_value='-45.0',
        description='Scanning low angle limit in degrees (-160 to -10)'
    )

    high_angle_arg = DeclareLaunchArgument(
        'high_angle_limit',
        default_value='45.0',
        description='Scanning high angle limit in degrees (10 to 160)'
    )

    update_rate_arg = DeclareLaunchArgument(
        'update_rate',
        default_value='12',
        description='Readings per second update rate index (1 to 12: 12=5000Hz)'
    )

    max_points_arg = DeclareLaunchArgument(
        'max_points',
        default_value='100',
        description='Number of distance points gathered per published message'
    )

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='lightwarelidar',
        description='Namespace for the sf45b node and published topics'
    )

    sf45b_node = Node(
        package='lightwarelidar',
        executable='sf45b',
        name='sf45b',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        emulate_tty=True,
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'port': LaunchConfiguration('port'),
                'baudrate': LaunchConfiguration('baudrate'),
                'frame_id': LaunchConfiguration('frame_id'),
                'publish_laser_scan': LaunchConfiguration('publish_laser_scan'),
                'lowAngleLimit': LaunchConfiguration('low_angle_limit'),
                'highAngleLimit': LaunchConfiguration('high_angle_limit'),
                'updateRate': LaunchConfiguration('update_rate'),
                'maxPoints': LaunchConfiguration('max_points'),
            }
        ]
    )

    return LaunchDescription([
        config_file_arg,
        namespace_arg,
        port_arg,
        baudrate_arg,
        frame_id_arg,
        publish_scan_arg,
        low_angle_arg,
        high_angle_arg,
        update_rate_arg,
        max_points_arg,
        sf45b_node
    ])
