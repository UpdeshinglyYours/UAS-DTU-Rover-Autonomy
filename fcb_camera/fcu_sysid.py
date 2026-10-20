#!/usr/bin/env python3
"""Print the MAVLink system ID of the autopilot on a port, from its heartbeat.

MAVROS only talks to tgt_system (default 1). An autopilot set to another
SYSID_THISMAV -- this drone's Cube is 2 -- is otherwise ignored: MAVROS
connects to the port and reports nothing. Prints 1 if nothing answers.

    fcu_sysid.py /dev/serial/by-id/usb-CubePilot...-if00:115200
"""
import sys

url = sys.argv[1] if len(sys.argv) > 1 else ""
port, _, baud = url.partition(":")
try:
    from pymavlink import mavutil
    link = mavutil.mavlink_connection(port, baud=int(baud or 115200))
    for _ in range(10):
        hb = link.recv_match(type="HEARTBEAT", blocking=True, timeout=8)
        # Skip GCSes and companions repeating their own heartbeats.
        if hb is not None and hb.type not in (mavutil.mavlink.MAV_TYPE_GCS,
                                              mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER):
            print(hb.get_srcSystem())
            sys.exit(0)
except Exception as exc:
    print(f"fcu_sysid: {exc}", file=sys.stderr)
print(1)
