"""
Two ArduRover SITL instances, side by side, purely so their MAVLink output can be picked up
by mavros.launch.py in this same package. This is NOT Gazebo - it's ArduPilot's own physics
sim (SITL), talking MAVLink only. It knows nothing about ROS2, bcr_bot, or nav2.

sim_vehicle.py's -I <instance> flag shifts every port it opens by 10*instance. This checkout
only opens ONE default MAVLink "--out" port per instance (verified directly in
Tools/autotest/sim_vehicle.py - "ports = [14550 + 10 * i]"; there's no separate
companion-computer port unless extra --out flags are passed):
  instance 0 (rover1): udp:14550
  instance 1 (rover2): udp:14560
mavros.launch.py in this package connects directly to that port.

Each instance also needs a distinct SYSID_THISMAV (--sysid) - otherwise both vehicles claim
MAVLink system ID 1 and every MAVROS/GCS client downstream can't tell them apart.

Each instance also needs a DISTINCT home location (-l lat,lon,alt,heading), not the same
one ("-L RATBeach" for both, as this used to do) - confirmed live: with identical homes,
both rovers' GPS fixes read as co-located, while the ROS-side rover1/map<->rover2/map static
transform (5m offset, matching bcr_bot's Gazebo spawn) has zero connection to that and was
just an assumption nothing actually measured. rover2's home here is rover1's home
(Tools/autotest/locations.txt RATBeach: 33.810313,-118.393867,0,270) shifted +5m north
(+0.000045 deg latitude, using 111320m/degree) - matching bcr_bot_gazebo_spawn's
position_y=5.0 for rover2 in gazebo_sitl_demo.launch.py, so the GPS-derived separation
between the two vehicles now actually agrees with what the static TF claims, instead of the
two being unrelated fictions.

--mavproxy-args=--daemon is required here: MAVProxy's normal foreground mode expects a real
controlling TTY for its interactive console loop, which launch's ExecuteProcess doesn't give
it - without --daemon, the whole process comes up looking healthy (SITL boots, "ArduPilot
Ready" prints fine) but MAVProxy's own --out UDP forwarding silently never actually sends a
single packet (confirmed live: a raw socket bound to the --out port received nothing, no
matter how long you wait). --daemon is MAVProxy's own documented flag for exactly this
headless/scripted use case and fixes it. (Note: --daemon isn't a sim_vehicle.py option
itself - it has to be passed through via --mavproxy-args, not tacked on directly.)
"""

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction


def generate_launch_description():
    sim_vehicle = os.path.expanduser('~/ardupilot/Tools/autotest/sim_vehicle.py')

    rover1_sitl = ExecuteProcess(
        cmd=[sim_vehicle, '-v', 'Rover', '-I', '0', '--sysid', '1',
             '-l', '33.810313,-118.393867,0,270',
             '--no-rebuild', '--mavproxy-args=--daemon', '--map'],
        name='sitl_rover1',
        output='screen',
    )

    # Staggered by a few seconds - sim_vehicle.py's first-run build step (if the SITL binary
    # isn't already compiled for this vehicle type) can briefly hog CPU/disk; launching both
    # at once the very first time has caused one of them to time out waiting on the other's
    # build lock in testing.
    rover2_sitl = TimerAction(
        period=5.0,
        actions=[ExecuteProcess(
            cmd=[sim_vehicle, '-v', 'Rover', '-I', '1', '--sysid', '2',
                 '-l', '33.810358,-118.393867,0,270',
                 '--no-rebuild', '--mavproxy-args=--daemon', '--map'],
            name='sitl_rover2',
            output='screen',
        )],
    )

    return LaunchDescription([rover1_sitl, rover2_sitl])
