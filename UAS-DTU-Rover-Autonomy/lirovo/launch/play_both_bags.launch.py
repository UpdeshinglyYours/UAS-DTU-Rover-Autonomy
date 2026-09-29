"""
Plays both rovers' bags at once, each on its own /clock, so rover1 and rover2 stop
freezing independently of each other (see lirovo_hardware.launch.py's use_sim_time
comment - a rover's whole stack, bond heartbeats included, stalls without its own
/<robot_namespace>/clock ticking).

Both commands below are EXACTLY what was being run by hand in two separate terminals -
this file just starts them together under one `ros2 launch` so they can't drift out of
sync (or get forgotten) again. --start-paused is preserved on purpose - unpause each
with SPACE in this launch's own terminal once both have loaded, same as running them
manually.

rover1_bag_dir/rover2_bag_dir default to where these two actually live on disk right
now - rover1's bag is NOT under the same parent as rover2's (confirmed live 2026-09-08:
rover1 -> /home/reet/7sept/7sept1.bag/, rover2 -> /home/reet/v_ws/7sept_2/7sept2_0_1.bag/
- two unrelated top-level directories). Each bag's actual .db3 file also sits ONE level
deeper than its own directory name suggests (e.g. 7sept2_0_1.bag/7sept2_0_1.bag_0.db3,
not 7sept2_0_1.bag_0.db3 directly) - both paths below account for that nesting. Override
either arg if the bags move.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    rover1_bag_dir = LaunchConfiguration('rover1_bag_dir')
    rover2_bag_dir = LaunchConfiguration('rover2_bag_dir')

    # Same --topics filter on both, copied verbatim from the manual commands - note this
    # does NOT include /rover2/point_cloud_out or /rover2/scan, so rover2's own lidar
    # pipeline gets no real sensor data from either bag as currently written. Add those
    # topic names here if that's meant to be included.
    topics = [
        '/rover1/imu/data',
        '/rover1/point_cloud_out',
        '/rover2/local_position/odom',
        '/tf_static',
    ]

    # bag_path here is the bag DIRECTORY (containing metadata.yaml) - ros2 bag play resolves
    # the actual .db3 storage file inside it itself, no need to name the .db3 file directly.
    rover1_bag_play = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play',
            PathJoinSubstitution([rover1_bag_dir, '7sept1.bag']),
            '--clock', '--start-paused',
            '--remap', '/clock:=/rover1/clock',
            '--topics', *topics,
        ],
        output='screen',
        name='bag_play_rover1',
    )

    rover2_bag_play = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play',
            PathJoinSubstitution([rover2_bag_dir, '7sept2_0_1.bag']),
            '--clock', '--start-paused',
            '--remap', '/clock:=/rover2/clock',
            '--topics', *topics,
        ],
        output='screen',
        name='bag_play_rover2',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'rover1_bag_dir',
            default_value='/home/reet/7sept',
            description='Parent directory of 7sept1.bag/ (rover1\'s bag).'
        ),
        DeclareLaunchArgument(
            'rover2_bag_dir',
            default_value='/home/reet/v_ws/7sept_2',
            description='Parent directory of 7sept2_0_1.bag/ (rover2\'s bag).'
        ),
        rover1_bag_play,
        rover2_bag_play,
    ])
