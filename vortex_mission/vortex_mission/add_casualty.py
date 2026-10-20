#!/usr/bin/env python3
"""Add a casualty from the command line.

  ros2 run vortex_mission add_casualty --lat 28.7501234 --lon 77.1176543
  ros2 run vortex_mission add_casualty --x 12.5 --y -3.0          # map frame, metres
  ros2 run vortex_mission add_casualty --file casualties.yaml     # list of {lat, lon} or {x, y}
"""

import argparse
import sys

import rclpy
import yaml

from vortex_interfaces.srv import AddCasualty


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--lat', type=float)
    parser.add_argument('--lon', type=float)
    parser.add_argument('--x', type=float)
    parser.add_argument('--y', type=float)
    parser.add_argument('--file', help='YAML list of {lat, lon} or {x, y} entries')
    parser.add_argument('--source', default='cli')
    args = parser.parse_args(rclpy.utilities.remove_ros_args(argv or sys.argv)[1:])

    entries = []
    if args.file:
        with open(args.file) as f:
            entries = yaml.safe_load(f) or []
    elif args.lat is not None and args.lon is not None:
        entries = [{'lat': args.lat, 'lon': args.lon}]
    elif args.x is not None and args.y is not None:
        entries = [{'x': args.x, 'y': args.y}]
    else:
        parser.error('give --lat/--lon, --x/--y, or --file')

    rclpy.init()
    node = rclpy.create_node('add_casualty_cli')
    client = node.create_client(AddCasualty, '/casualty_manager/add_casualty')
    if not client.wait_for_service(timeout_sec=5.0):
        print('casualty_manager is not running (/casualty_manager/add_casualty)')
        return 1
    rc = 0
    for e in entries:
        req = AddCasualty.Request()
        if 'lat' in e:
            req.frame, req.x, req.y = AddCasualty.Request.FRAME_LATLON, float(e['lat']), float(e['lon'])
        else:
            req.frame, req.x, req.y = AddCasualty.Request.FRAME_MAP, float(e['x']), float(e['y'])
        req.source = args.source
        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        res = future.result()
        if res is None:
            print('no reply')
            rc = 1
            continue
        print(('OK  ' if res.ok else 'FAIL') + f' id={res.id}: {res.message}')
        rc |= 0 if res.ok else 1
    node.destroy_node()
    rclpy.shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
