from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        # MAVROS
        ExecuteProcess(
            cmd=[
                'bash', '-c',
                'source ~/scp/mavros_ws/install/setup.bash && '
                'ros2 launch mavros apm.launch fcu_url:=udp://127.0.0.1:14550@'
            ],
            output='screen'
        ),

        # Gazebo
        ExecuteProcess(
            cmd=[
                'bash', '-c',
                'source ~/DARPA_rover_ws/install/setup.bash && '
                'ros2 launch bcr_bot gazebo.launch.py'
            ],
            output='screen'
        ),

        # Pointcloud -> Laser
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            parameters=[{
                'target_frame': 'base_link',
                'transform_tolerance': 0.05,
                'min_height': 0.4,
                'max_height': 2.0,
                'angle_min': -3.14159,
                'angle_max': 3.14159,
                'angle_increment': 0.00872665,
                'scan_time': 0.3,
                'range_min': 0.1,
                'range_max': 200.0,
                'use_inf': True,
                'inf_epsilon': 1.0,
                'queue_size': 50,
                'use_sim_time': True,
            }],
            remappings=[
                ('cloud_in', '/points')
            ]
        ),

        # SLAM
        ExecuteProcess(
            cmd=[
                'bash', '-c',
                'source ~/DARPA_rover_ws/install/setup.bash && '
                'ros2 launch bcr_bot mapping.launch.py'
            ],
            output='screen'
        ),

        # Nav2
        ExecuteProcess(
            cmd=[
                'bash', '-c',
                'source ~/DARPA_rover_ws/install/setup.bash && '
                'ros2 launch bcr_bot nav2.launch.py'
            ],
            output='screen'
        ),

        # cmd_vel relay
        Node(
            package='topic_tools',
            executable='relay',
            arguments=[
                '/bcr_bot/cmd_vel',
                '/mavros/setpoint_velocity/cmd_vel_unstamped'
            ],
            output='screen'
        ),

        # MAVROS TF Publisher
        ExecuteProcess(
            cmd=[
                'bash', '-c',
                'python3 ~/tf_mavros_publisher.py --ros-args -p use_sim_time:=true'
            ],
            output='screen'
        ),
    ])