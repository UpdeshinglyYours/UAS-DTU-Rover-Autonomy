"""Bring up the FCB driver, respawning it if it dies.

    ros2 launch fcb_orin_driver fcb_driver.launch.py
    ros2 launch fcb_orin_driver fcb_driver.launch.py fc_source:=mavros
    ros2 launch fcb_orin_driver fcb_driver.launch.py fc_source:=none autostart:=true

Respawn stands in for start_recorder.sh's supervision loop: a driver that
exits non-zero comes back after a few seconds instead of leaving the drone
recording nothing. Do not stop this launch between flights -- on the
Oppila board, closing the stream needs a 12 V power cycle to recover.
Start and stop recordings with RC ch8 or the services instead.

Keys go to a separate process, so they can be closed freely:
    ros2 run fcb_orin_driver fcb_keys
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

#: Launch arguments passed straight through as parameter overrides when set.
OVERRIDES = ("fc_source", "mavros_namespace", "autostart", "icr", "record_dir",
             "encoder", "video", "port", "rc_url", "require_mavlink",
             "publish_image", "publish_rate_hz", "publish_compressed",
             "compressed_rate_hz", "compressed_width", "board_power_cycle_cmd")


def _typed(value):
    """A launch argument as the type its parameter is declared with."""
    if value.lower() in ("true", "false"):
        return value.lower() == "true"
    for kind in (int, float):
        try:
            return kind(value)
        except ValueError:
            pass
    return value


def _driver(context):
    params = [LaunchConfiguration("params_file").perform(context)]
    overrides = {}
    for name in OVERRIDES:
        value = LaunchConfiguration(name).perform(context)
        if value != "":
            overrides[name] = _typed(value)
    if overrides:
        params.append(overrides)
    return [Node(
        package="fcb_orin_driver",
        executable="fcb_driver",
        name="fcb_driver",
        namespace=LaunchConfiguration("namespace").perform(context),
        parameters=params,
        output="screen",
        emulate_tty=True,
        respawn=True,
        respawn_delay=3.0,
        # Time to finalise the video file and CSV before escalating.
        sigterm_timeout="15",
        sigkill_timeout="15",
    )]


def generate_launch_description():
    share = get_package_share_directory("fcb_orin_driver")
    args = [
        DeclareLaunchArgument(
            "params_file", default_value=os.path.join(share, "config", "fcb_driver.yaml")),
        DeclareLaunchArgument("namespace", default_value=""),
    ]
    args += [DeclareLaunchArgument(name, default_value="",
                                   description="overrides the parameter file")
             for name in OVERRIDES]
    return LaunchDescription(args + [OpaqueFunction(function=_driver)])
