#!/usr/bin/env python3
"""Desk comparison: orbit with Nav2 MPPI (FollowPath, controller_id=OrbitController) vs the
custom OrbitCasualty tracker, on the same planned path, in kinematic_sim (drivetrain model on).

Run inside a sim session (ROS_DOMAIN_ID=77) where:
  - kinematic_sim starts at (cx - 3.3, cy) heading +90 deg (clockwise tangent),
  - the casualty is registered (keepout disc in the costmaps) and the mission is paused.

  python3 compare_mppi_vs_custom.py <cx> <cy>
"""
import math
import sys
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowPath
from nav2_msgs.msg import Costmap
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from vortex_interfaces.action import OrbitCasualty
from vortex_mission.orbit_geometry import (OrbitParams, blended_target, body_distance,
                                           grid_from_costmap_msg, path_heading, profile_at,
                                           radius_profile, spiral_angle, wrap_angle)

CX, CY = float(sys.argv[1]), float(sys.argv[2])
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)


class Harness:
    def __init__(self):
        self.node = rclpy.create_node('orbit_compare')
        self.odom = []
        self.costmap = None
        self.node.create_subscription(Odometry, '/odometry/filtered', self._odom, 50)
        self.node.create_subscription(Costmap, '/global_costmap/costmap_raw', self._cm, LATCHED)
        self.follow = ActionClient(self.node, FollowPath, 'follow_path')
        self.orbit = ActionClient(self.node, OrbitCasualty, 'orbit_casualty')
        self.path_pub = self.node.create_publisher(Path, '/orbit_compare/path', LATCHED)
        ex = MultiThreadedExecutor()
        ex.add_node(self.node)
        threading.Thread(target=ex.spin, daemon=True).start()

    def _odom(self, m):
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z)
        self.odom.append((time.monotonic(), m.pose.pose.position.x, m.pose.pose.position.y, yaw))

    def _cm(self, m):
        self.costmap = m

    def pose(self):
        while not self.odom:
            time.sleep(0.05)
        return self.odom[-1][1:]

    def wait_result(self, client, goal, timeout):
        client.wait_for_server(timeout_sec=10)
        f = client.send_goal_async(goal)
        while not f.done():
            time.sleep(0.05)
        h = f.result()
        if not h.accepted:
            return 'REJECTED'
        r = h.get_result_async()
        t0 = time.monotonic()
        while not r.done() and time.monotonic() - t0 < timeout:
            time.sleep(0.1)
        if not r.done():
            h.cancel_goal_async()
            return 'TIMEOUT'
        return {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}.get(r.result().status, r.result().status)


def leg_path(thetas, prof, th0, length, start_r, end_r, sp_in, sp_out, frame_stamp):
    path = Path()
    path.header = frame_stamp
    n = max(2, int(math.degrees(length) / 1.0))
    for k in range(n + 1):
        s = length * k / n
        th = th0 - s                                           # clockwise
        r_p, dr_p = profile_at(thetas, prof, th)
        r, dr = blended_target(r_p, dr_p, start_r, s, length - s, sp_in, True,
                               end_radius=end_r, spiral_out=sp_out)
        p = PoseStamped()
        p.header = frame_stamp
        p.pose.position.x = CX + r * math.cos(th)
        p.pose.position.y = CY + r * math.sin(th)
        yaw = path_heading(th, r, dr, True)
        p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        path.poses.append(p)
    return path


def metrics(samples, thetas, prof):
    """min real-body distance, mean/max |radial error| vs profile (spirals excluded by caller)."""
    mb, errs = float('inf'), []
    for _, x, y, yaw in samples:
        mb = min(mb, body_distance(CX, CY, x, y, yaw, 0.8, 0.7))
        th = math.atan2(y - CY, x - CX)
        r_p, _ = profile_at(thetas, prof, th)
        if not math.isnan(r_p):
            errs.append(math.hypot(x - CX, y - CY) - r_p)
    errs = np.array(errs) if errs else np.zeros(1)
    return mb, float(np.mean(errs)), float(np.max(np.abs(errs))), float(np.min(errs))


def swept_deg(samples):
    tot, prev = 0.0, None
    for _, x, y, _ in samples:
        th = math.atan2(y - CY, x - CX)
        if prev is not None:
            tot += -wrap_angle(th - prev)       # clockwise positive
        prev = th
    return math.degrees(tot)


def main():
    rclpy.init()
    h = Harness()
    while h.costmap is None:
        time.sleep(0.1)
    time.sleep(2.0)                              # let the keepout disc land in the costmap
    grid = grid_from_costmap_msg(h.costmap)
    p = OrbitParams()
    thetas, prof = radius_profile(grid, CX, CY, p)
    x, y, _ = h.pose()
    th0 = math.atan2(y - CY, x - CX)
    start_r = math.hypot(x - CX, y - CY)
    sp_in, sp_out = spiral_angle(start_r - p.radius), spiral_angle(3.3 - p.radius)

    # ---------------- A) Nav2 MPPI OrbitController on the planned path (two halves) -------------
    stamp = PoseStamped().header
    stamp.frame_id = 'map'
    legs = [(th0, math.pi, start_r, p.radius, sp_in, 0.0),
            (th0 - math.pi, math.pi, p.radius, 3.3, 0.0, sp_out)]
    h.odom.clear()
    t0 = time.monotonic()
    results = []
    for (a0, L, r0, r1, si, so) in legs:
        stamp.stamp = h.node.get_clock().now().to_msg()
        path = leg_path(thetas, prof, a0, L, r0, r1, si, so, stamp)
        h.path_pub.publish(path)
        g = FollowPath.Goal()
        g.path = path
        g.controller_id = 'OrbitController'
        g.goal_checker_id = 'general_goal_checker'
        results.append(h.wait_result(h.follow, g, 180.0))
    t_mppi = time.monotonic() - t0
    s_mppi = list(h.odom)
    cut = len(s_mppi) // 8                       # drop spiral-in / spiral-out parts for errors
    mb, me, mx, mn = metrics(s_mppi[cut:-cut] if len(s_mppi) > 16 else s_mppi, thetas, prof)
    mb_all = metrics(s_mppi, thetas, prof)[0]
    print(f'MPPI   : legs {results} | {t_mppi:5.1f} s | swept {swept_deg(s_mppi):6.1f} deg | '
          f'min body {mb_all:.2f} m | radial err mean {me:+.3f} max|e| {mx:.3f} most-inward {mn:+.3f}')

    # ---------------- B) custom OrbitCasualty from wherever MPPI left the rover -----------------
    time.sleep(2.0)
    h.odom.clear()
    goal = OrbitCasualty.Goal()
    goal.casualty_id = 1
    goal.center.header.frame_id = 'map'
    goal.center.pose.position.x, goal.center.pose.position.y = CX, CY
    goal.center.pose.orientation.w = 1.0
    goal.radius, goal.standby_radius, goal.keepout_radius = 2.9, 3.3, 2.2
    goal.max_detour_radius, goal.clockwise, goal.max_sweep_deg = 4.0, True, 360.0
    t0 = time.monotonic()
    res = h.wait_result(h.orbit, goal, 300.0)
    t_cust = time.monotonic() - t0
    s_c = list(h.odom)
    cut = len(s_c) // 8
    mb, me, mx, mn = metrics(s_c[cut:-cut] if len(s_c) > 16 else s_c, thetas, prof)
    mb_all = metrics(s_c, thetas, prof)[0]
    print(f'CUSTOM : {res} | {t_cust:5.1f} s | swept {swept_deg(s_c):6.1f} deg | '
          f'min body {mb_all:.2f} m | radial err mean {me:+.3f} max|e| {mx:.3f} most-inward {mn:+.3f}')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
