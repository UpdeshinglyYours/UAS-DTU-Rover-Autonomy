#!/usr/bin/env python3
"""casualty_manager_simple: casualty list + mission, built only from stock Nav2 actions.

Per casualty:
  1. keepout   a filled disc of points (costmap_keepout_radius) on /casualty_keepout; the
               costmaps' casualty_layer marks it lethal, so nothing Nav2 plans or drives
               can put the rover inside it.
  2. waypoints orbit_waypoints points on a circle of orbit_radius around the casualty.
               Points that sit on a lethal / inscribed cell of the global costmap are dropped.
  3. approach  NavigateToPose to the nearest waypoint, then Spin to face the casualty.
  4. STANDBY   wait for ~/start_orbit(id) (RC flick, rover_console 'o', or the model script).
  5. orbit     NavigateThroughPoses through the waypoints (clockwise, back to the start).
               If Nav2 gives up part way, the next waypoint is dropped and the rest resent.
  6. coverage  degrees of bearing swept around the casualty while orbiting -> DONE.

Planning, obstacle avoidance and velocity control are Nav2's (planner + FollowPath MPPI).
Same services / topics as casualty_manager.py, so add_casualty, mission_planner_live2,
rover_console and rc_orbit_trigger work unchanged.
"""

import csv
import math
import os
import threading
import time
from dataclasses import dataclass
from typing import Dict, List, Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from builtin_interfaces.msg import Duration as DurationMsg
from geographic_msgs.msg import GeoPointStamped
from geometry_msgs.msg import PointStamped, PoseStamped
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose, Spin
from nav2_msgs.msg import Costmap
from sensor_msgs.msg import NavSatFix, PointCloud2, PointField
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from vortex_interfaces.msg import Casualty, CasualtyArray, RoverState
from vortex_interfaces.srv import AddCasualty, StartOrbit

from vortex_mission.approach import face_yaw_delta, keepout_disc_points
from vortex_mission.geo import latlon_to_meter_offset, meter_offset_to_latlon

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST)
SUCCEEDED = 4                 # action_msgs/GoalStatus.STATUS_SUCCEEDED
INSCRIBED = 253               # nav2 costmap: the footprint would touch an obstacle here
STATUS_NAMES = {Casualty.PENDING: 'PENDING', Casualty.ACTIVE: 'ACTIVE',
                Casualty.STANDBY: 'STANDBY', Casualty.ORBITING: 'ORBITING',
                Casualty.DONE: 'DONE', Casualty.SKIPPED: 'SKIPPED'}


class Aborted(Exception):
    """Raised inside the mission thread when the operator skips / aborts."""


class NotReady(Exception):
    """Nav2 isn't up yet (no costmap / pose / action server): retry later, don't skip."""


@dataclass
class Waypoint:
    idx: int          # position on the circle, 0 .. orbit_waypoints-1 (counter-clockwise)
    theta: float      # angle around the casualty (rad, map frame)
    x: float
    y: float


class CasualtyManagerSimple(Node):

    def __init__(self):
        super().__init__('casualty_manager')
        dp = self.declare_parameter
        self.keepout_radius = dp('keepout_radius', 2.2).value
        self.costmap_keepout_radius = dp('costmap_keepout_radius', 2.3).value
        self.orbit_radius = dp('orbit_radius', 3.3).value
        self.n_waypoints = int(dp('orbit_waypoints', 12).value)
        # NavigateThroughPoses only checks the LAST pose: a goal ending where the rover already
        # is (a closed loop) "succeeds" at once. So the loop is sent in pieces of this many.
        self.chunk = max(1, int(dp('orbit_chunk_waypoints', 6).value))
        self.clockwise = dp('orbit_clockwise', True).value
        self.reached_radius = dp('waypoint_reached_radius', 0.7).value
        self.wait_for_trigger = dp('wait_for_trigger', True).value
        self.standby_timeout = dp('standby_timeout', 0.0).value
        self.face_casualty = dp('face_casualty', True).value
        self.auto_start = dp('auto_start', True).value
        self.approach_timeout_base = dp('approach_timeout_base', 60.0).value
        self.approach_timeout_per_m = dp('approach_timeout_per_m', 4.0).value
        self.orbit_timeout = dp('orbit_timeout', 300.0).value
        self.duplicate_radius = dp('duplicate_radius', 0.5).value
        self.global_frame = dp('global_frame', 'map').value
        self.robot_frame = dp('robot_frame', 'base_link').value
        self.global_costmap_topic = dp('global_costmap_topic', '/global_costmap/costmap_raw').value
        self.origin_lat_param = dp('origin_lat', float('nan')).value
        self.origin_lon_param = dp('origin_lon', float('nan')).value
        self.log_dir = os.path.expanduser(dp('log_dir', '~/casualty_logs').value)

        cb = ReentrantCallbackGroup()
        self.lock = threading.RLock()
        self.casualties: Dict[int, Casualty] = {}
        self.order: List[int] = []
        self.waypoints: Dict[int, List[Waypoint]] = {}
        self.next_id = 1
        self.current_id: Optional[int] = None
        self.state = RoverState.IDLE
        self.running = self.auto_start
        self.paused = False
        self.skip_event = threading.Event()
        self.abort_event = threading.Event()
        self.orbit_request = threading.Event()
        self.keepout_msg: Optional[PointCloud2] = None
        self.gp_origin = None
        self.gui_origin = None
        self._active_goal = None
        self._costmap_msg = None
        self._costmap_event = threading.Event()
        # orbit progress, filled by the 10 Hz tracker while orbiting
        self.track_centre = None          # (x, y) of the casualty being orbited
        self.covered = [False] * 360
        self.prev_bearing = None
        self.wp_min_dist: Dict[int, float] = {}
        self._track_wps: List[Waypoint] = []

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub_casualties = self.create_publisher(CasualtyArray, '/casualties', LATCHED)
        self.pub_state = self.create_publisher(RoverState, '/rover_state', LATCHED)
        self.pub_keepout = self.create_publisher(PointCloud2, '/casualty_keepout', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '~/markers', LATCHED)

        self.create_subscription(PointStamped, '/clicked_point', self.on_clicked_point, 10,
                                 callback_group=cb)
        self.create_subscription(GeoPointStamped, '/mavros/global_position/gp_origin',
                                 self.on_gp_origin, qos_profile_sensor_data, callback_group=cb)
        self.create_subscription(NavSatFix, '/mission_planner/origin', self.on_gui_origin,
                                 LATCHED, callback_group=cb)

        self.create_service(AddCasualty, '~/add_casualty', self.srv_add, callback_group=cb)
        self.create_service(StartOrbit, '~/start_orbit', self.srv_start_orbit, callback_group=cb)
        self.create_service(Trigger, '~/start', self.srv_start, callback_group=cb)
        self.create_service(Trigger, '~/pause', self.srv_pause, callback_group=cb)
        self.create_service(Trigger, '~/skip_current', self.srv_skip, callback_group=cb)
        self.create_service(Trigger, '~/abort', self.srv_abort, callback_group=cb)

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose', callback_group=cb)
        self.through_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses',
                                           callback_group=cb)
        self.spin_client = ActionClient(self, Spin, 'spin', callback_group=cb)

        self.create_timer(1.0, self.publish_keepout, callback_group=cb)
        self.create_timer(0.1, self.track_orbit, callback_group=cb)

        os.makedirs(self.log_dir, exist_ok=True)
        self.log_path = os.path.join(self.log_dir, time.strftime('casualties_%Y%m%d_%H%M%S.csv'))
        with open(self.log_path, 'w', newline='') as f:
            csv.writer(f).writerow(['time', 'id', 'x', 'y', 'lat', 'lon', 'source', 'status',
                                    'covered_deg', 'reason'])

        self.publish_state(RoverState.IDLE)
        self.publish_casualties()
        threading.Thread(target=self.mission_loop, daemon=True).start()
        self.get_logger().info(
            f'casualty_manager (simple, stock Nav2) ready: keepout disc {self.costmap_keepout_radius} m, '
            f'orbit {self.n_waypoints} waypoints at {self.orbit_radius} m, '
            f'wait_for_trigger={self.wait_for_trigger}. Log: {self.log_path}')

    # ------------------------------------------------------------------ origin
    def on_gp_origin(self, msg: GeoPointStamped):
        self.gp_origin = (msg.position.latitude, msg.position.longitude)

    def on_gui_origin(self, msg: NavSatFix):
        self.gui_origin = (msg.latitude, msg.longitude)

    def origin(self):
        if not math.isnan(self.origin_lat_param) and not math.isnan(self.origin_lon_param):
            return (self.origin_lat_param, self.origin_lon_param), 'parameter'
        if self.gp_origin is not None:
            return self.gp_origin, 'mavros gp_origin'
        if self.gui_origin is not None:
            return self.gui_origin, 'mission_planner_live2'
        return None, ''

    # ------------------------------------------------------------------ adding
    def on_clicked_point(self, msg: PointStamped):
        if msg.header.frame_id and msg.header.frame_id != self.global_frame:
            self.get_logger().warn(f'/clicked_point in frame {msg.header.frame_id}, '
                                   f'expected {self.global_frame}; ignored')
            return
        _, _, text = self.add_casualty(msg.point.x, msg.point.y, float('nan'), float('nan'),
                                       'rviz')
        self.get_logger().info(text)

    def srv_add(self, req: AddCasualty.Request, res: AddCasualty.Response):
        if req.frame == AddCasualty.Request.FRAME_LATLON:
            org, src = self.origin()
            if org is None:
                res.ok, res.id = False, 0
                res.message = ('no EKF origin yet (need /mavros/global_position/gp_origin, '
                               'mission_planner_live2, or origin_lat/origin_lon params)')
                return res
            x, y = latlon_to_meter_offset(org[0], org[1], req.x, req.y)
            res.ok, res.id, res.message = self.add_casualty(x, y, req.x, req.y,
                                                            req.source or 'latlon')
            res.message += f' (origin from {src})'
        else:
            res.ok, res.id, res.message = self.add_casualty(req.x, req.y, float('nan'),
                                                            float('nan'), req.source or 'map')
        return res

    def add_casualty(self, x: float, y: float, lat: float, lon: float, source: str):
        with self.lock:
            for c in self.casualties.values():
                if math.hypot(c.pose.pose.position.x - x, c.pose.pose.position.y - y) \
                        < self.duplicate_radius:
                    return True, c.id, f'casualty {c.id} already at ({x:.2f}, {y:.2f})'
            if math.isnan(lat):
                org, _ = self.origin()
                if org is not None:
                    lat, lon = meter_offset_to_latlon(org[0], org[1], x, y)
            c = Casualty()
            c.id = self.next_id
            self.next_id += 1
            c.pose.header.frame_id = self.global_frame
            c.pose.header.stamp = self.get_clock().now().to_msg()
            c.pose.pose.position.x, c.pose.pose.position.y = float(x), float(y)
            c.pose.pose.orientation.w = 1.0
            c.lat, c.lon = float(lat), float(lon)
            c.status = Casualty.PENDING
            c.source = source
            self.casualties[c.id] = c
            self.order.append(c.id)
            self.rebuild_keepout()
        self.publish_casualties()
        self.log_row(c)
        return True, c.id, f'added casualty {c.id} at ({x:.2f}, {y:.2f}) from {source}'

    # ------------------------------------------------------------------ mission control
    def srv_start(self, req, res):
        self.running, self.paused = True, False
        self.abort_event.clear()
        res.success, res.message = True, 'mission running'
        return res

    def srv_pause(self, req, res):
        self.paused = True
        res.success, res.message = True, 'will stop after the current casualty'
        return res

    def srv_skip(self, req, res):
        if self.current_id is None:
            res.success, res.message = False, 'no casualty in progress'
            return res
        self.skip_event.set()
        self.cancel_active_goal()
        res.success, res.message = True, f'skipping casualty {self.current_id}'
        return res

    def srv_abort(self, req, res):
        self.running = False
        self.abort_event.set()
        self.cancel_active_goal()
        res.success, res.message = True, 'mission aborted; call ~/start to resume'
        return res

    def srv_start_orbit(self, req: StartOrbit.Request, res: StartOrbit.Response):
        if self.state != RoverState.STANDBY or self.current_id is None:
            res.accepted, res.message = False, 'rover is not in STANDBY'
        elif req.casualty_id != self.current_id:
            res.accepted = False
            res.message = (f'rover is in STANDBY at casualty {self.current_id}, '
                           f'not {req.casualty_id}')
        else:
            self.orbit_request.set()
            res.accepted, res.message = True, f'orbit of casualty {req.casualty_id} starting'
        self.get_logger().info(f'start_orbit({req.casualty_id}): {res.message}')
        return res

    # ------------------------------------------------------------------ mission thread
    def mission_loop(self):
        while rclpy.ok():
            time.sleep(0.2)
            if not self.running or self.paused:
                continue
            with self.lock:
                pending = [self.casualties[i] for i in self.order
                           if self.casualties[i].status == Casualty.PENDING]
            if not pending:
                if self.state != RoverState.IDLE:
                    self.publish_state(RoverState.IDLE)
                continue
            pose = self.robot_pose()
            if pose is None:
                self.get_logger().warn('no robot pose (map -> base_link); waiting',
                                       throttle_duration_sec=5.0)
                continue
            c = min(pending, key=lambda k: math.hypot(k.pose.pose.position.x - pose[0],
                                                      k.pose.pose.position.y - pose[1]))
            self.skip_event.clear()
            try:
                self.process(c)
            except NotReady as e:
                with self.lock:
                    c.status, c.reason = Casualty.PENDING, ''
                self.current_id = None
                self.publish_casualties()
                self.publish_state(RoverState.IDLE)
                self.get_logger().warn(f'{e}; retrying in 5 s', throttle_duration_sec=10.0)
                time.sleep(5.0)
            except Aborted as e:
                self.stop_tracking()
                with self.lock:
                    if self.abort_event.is_set():
                        c.status, c.reason = Casualty.PENDING, ''
                    else:
                        c.status, c.reason = Casualty.SKIPPED, str(e) or 'SKIPPED_BY_OPERATOR'
                self.finish(c)
            except Exception as e:  # never let one casualty kill the mission
                self.stop_tracking()
                self.get_logger().error(f'casualty {c.id}: {e!r}')
                with self.lock:
                    c.status, c.reason = Casualty.SKIPPED, f'ERROR: {e}'
                self.finish(c)

    def process(self, c: Casualty):
        cx, cy = c.pose.pose.position.x, c.pose.pose.position.y
        self.current_id = c.id
        with self.lock:
            c.status = Casualty.ACTIVE
        self.publish_casualties()
        self.publish_state(RoverState.TRAVEL, c)
        self.get_logger().info(f'--- casualty {c.id} at ({cx:.2f}, {cy:.2f}) ---')

        # 1. Waypoints on the circle, minus the ones on an obstacle.
        costmap = self.snapshot_costmap()
        pose = self.robot_pose()
        if costmap is None or pose is None:
            raise NotReady('no global costmap or robot pose yet (is Nav2 running?)')
        wps = self.make_waypoints(cx, cy, costmap)
        self.waypoints[c.id] = wps
        self.publish_markers()
        if not wps:
            with self.lock:
                c.status, c.reason = Casualty.SKIPPED, 'NO_FREE_WAYPOINT'
            self.finish(c)
            return

        # 2. Approach the nearest waypoint (straight-line nearest; the 2nd nearest if that fails).
        start = None
        for w in sorted(wps, key=lambda k: math.hypot(k.x - pose[0], k.y - pose[1]))[:2]:
            self.check_abort()
            dist = math.hypot(w.x - pose[0], w.y - pose[1])
            self.get_logger().info(f'approach: waypoint at {math.degrees(w.theta) % 360:.0f} deg '
                                   f'({w.x:.2f}, {w.y:.2f}), {dist:.1f} m away')
            goal = NavigateToPose.Goal()
            goal.pose = self.pose_stamped(w.x, w.y, math.atan2(cy - w.y, cx - w.x))
            result = self.run_action(self.nav_client, goal,
                                     self.approach_timeout_base + self.approach_timeout_per_m * dist)
            if result is not None and result[0] == SUCCEEDED:
                start = w
                break
            self.get_logger().warn('approach attempt failed')
        if start is None:
            with self.lock:
                c.status, c.reason = Casualty.SKIPPED, 'APPROACH_FAILED'
            self.finish(c)
            return

        # 3. Face the casualty (Nav2's goal checker ignores the final heading).
        if self.face_casualty:
            self.face(cx, cy)

        # 4. STANDBY until start_orbit.
        if self.wait_for_trigger:
            with self.lock:
                c.status = Casualty.STANDBY
            self.publish_casualties()
            self.orbit_request.clear()
            self.publish_state(RoverState.STANDBY, c)
            self.get_logger().info(f'STANDBY at casualty {c.id}: waiting for '
                                   f'/casualty_manager/start_orbit {{casualty_id: {c.id}}}')
            t0 = time.monotonic()
            while not self.orbit_request.is_set():
                self.check_abort()
                if self.standby_timeout > 0 and time.monotonic() - t0 > self.standby_timeout:
                    self.get_logger().warn('standby timeout: starting the orbit anyway')
                    break
                time.sleep(0.05)

        # 5. Orbit through the waypoints.
        with self.lock:
            c.status = Casualty.ORBITING
        self.publish_casualties()
        self.publish_state(RoverState.ORBIT, c)
        covered, skipped = self.orbit(cx, cy, wps, start)
        with self.lock:
            c.status = Casualty.DONE
            c.covered_deg = float(covered)
            c.reason = 'FULL' if covered >= 355 else 'PARTIAL'
            if skipped:
                c.reason += f' ({skipped} waypoint(s) unreachable)'
        self.finish(c)

    def orbit(self, cx: float, cy: float, wps: List[Waypoint], start: Waypoint):
        """NavigateThroughPoses around the circle; returns (covered_deg, skipped waypoints)."""
        k0 = wps.index(start)
        step = -1 if self.clockwise else 1
        # all free waypoints in driving order, ending back at the start (closes the loop)
        seq = [wps[(k0 + step * i) % len(wps)] for i in range(1, len(wps) + 1)]
        self.start_tracking(cx, cy, seq)
        remaining, skipped = seq, 0
        while remaining:
            self.check_abort()
            chunk = remaining[:self.chunk]
            goal = NavigateThroughPoses.Goal()
            goal.poses = [self.pose_stamped(w.x, w.y, self.tangent(w.theta)) for w in chunk]
            self.get_logger().info(
                f'orbit: NavigateThroughPoses through {len(chunk)} waypoint(s) '
                f'({len(remaining)} left), {"clockwise" if self.clockwise else "ccw"}')
            result = self.run_action(self.through_client, goal, self.orbit_timeout)
            if result is not None and result[0] == SUCCEEDED:
                remaining = remaining[len(chunk):]
                continue
            # Nav2 gave up part way: drop everything reached plus the first one not reached.
            missing = next((i for i, w in enumerate(chunk)
                            if self.wp_min_dist.get(w.idx, math.inf) > self.reached_radius), None)
            if missing is None:
                remaining = remaining[len(chunk):]
                continue
            w = remaining[missing]
            self.get_logger().warn(f'orbit: Nav2 could not reach the waypoint at '
                                   f'{math.degrees(w.theta) % 360:.0f} deg; skipping it')
            skipped += 1
            remaining = remaining[missing + 1:]
        covered = self.stop_tracking()
        self.get_logger().info(f'orbit done: covered {covered} deg, {skipped} waypoint(s) skipped')
        return covered, skipped

    def finish(self, c: Casualty):
        self.current_id = None
        self.publish_casualties()
        self.log_row(c)
        self.publish_state(RoverState.TRAVEL)
        self.get_logger().info(f'casualty {c.id}: {STATUS_NAMES[c.status]} '
                               f'(covered {c.covered_deg:.0f} deg) {c.reason}')

    # ------------------------------------------------------------------ geometry
    def make_waypoints(self, cx: float, cy: float, costmap: Costmap) -> List[Waypoint]:
        out = []
        for i in range(self.n_waypoints):
            th = 2.0 * math.pi * i / self.n_waypoints
            x, y = cx + self.orbit_radius * math.cos(th), cy + self.orbit_radius * math.sin(th)
            cost = self.cost_at(costmap, x, y)
            if cost is not None and INSCRIBED <= cost <= 254:
                self.get_logger().info(f'waypoint at {math.degrees(th):.0f} deg is on an '
                                       f'obstacle (cost {cost}); dropped')
                continue
            out.append(Waypoint(i, th, x, y))
        return out

    @staticmethod
    def cost_at(costmap: Costmap, x: float, y: float) -> Optional[int]:
        md = costmap.metadata
        i = int(math.floor((x - md.origin.position.x) / md.resolution))
        j = int(math.floor((y - md.origin.position.y) / md.resolution))
        if i < 0 or j < 0 or i >= md.size_x or j >= md.size_y:
            return None
        return costmap.data[j * md.size_x + i]

    def tangent(self, theta: float) -> float:
        return theta - math.pi / 2 if self.clockwise else theta + math.pi / 2

    def face(self, cx: float, cy: float):
        pose = self.robot_pose()
        if pose is None:
            return
        delta = face_yaw_delta(pose[0], pose[1], pose[2], cx, cy)
        if abs(delta) < math.radians(5.0):
            return
        goal = Spin.Goal()
        goal.target_yaw = float(delta)
        goal.time_allowance = DurationMsg(sec=20)
        result = self.run_action(self.spin_client, goal, 30.0)
        if result is None or result[0] != SUCCEEDED:
            self.get_logger().warn('could not turn to face the casualty; continuing')

    # ------------------------------------------------------------------ coverage tracking
    def start_tracking(self, cx: float, cy: float, seq: List[Waypoint]):
        self.covered = [False] * 360
        self.prev_bearing = None
        self.wp_min_dist = {w.idx: math.inf for w in seq}
        self._track_wps = seq
        self.track_centre = (cx, cy)

    def stop_tracking(self) -> int:
        self.track_centre = None
        return sum(self.covered)

    def track_orbit(self):
        centre = self.track_centre
        if centre is None:
            return
        pose = self.robot_pose()
        if pose is None:
            return
        x, y = pose[0], pose[1]
        for w in self._track_wps:
            d = math.hypot(x - w.x, y - w.y)
            if d < self.wp_min_dist.get(w.idx, math.inf):
                self.wp_min_dist[w.idx] = d
        b = math.atan2(y - centre[1], x - centre[0])
        prev = self.prev_bearing
        self.prev_bearing = b
        if prev is None:
            self.covered[int(math.degrees(b) % 360)] = True
            return
        delta = math.atan2(math.sin(b - prev), math.cos(b - prev))
        if abs(delta) > math.radians(30):        # a jump (pose glitch): don't fill the gap
            return
        steps = max(1, int(math.ceil(abs(math.degrees(delta)) * 2)))
        for k in range(steps + 1):
            a = prev + delta * k / steps
            self.covered[int(math.degrees(a) % 360)] = True

    # ------------------------------------------------------------------ action plumbing
    def check_abort(self):
        if self.abort_event.is_set():
            raise Aborted('ABORTED')
        if self.skip_event.is_set():
            raise Aborted('SKIPPED_BY_OPERATOR')

    def run_action(self, client: ActionClient, goal, timeout: float):
        """Send a goal and wait. Returns (status, result) or None. Raises Aborted on skip/abort."""
        if not client.wait_for_server(timeout_sec=5.0):
            raise NotReady(f'action server {client._action_name} not available')
        send = client.send_goal_async(goal)
        if not self.wait_future(send, 10.0):
            return None
        handle = send.result()
        if handle is None or not handle.accepted:
            self.get_logger().error(f'{client._action_name} goal rejected')
            return None
        self._active_goal = handle
        res_future = handle.get_result_async()
        t0 = time.monotonic()
        try:
            while not res_future.done():
                if time.monotonic() - t0 > timeout:
                    self.get_logger().warn(f'{client._action_name} timed out; cancelling')
                    handle.cancel_goal_async()
                    self.wait_future(res_future, 5.0)
                    return None
                if self.abort_event.is_set() or self.skip_event.is_set():
                    handle.cancel_goal_async()
                    self.wait_future(res_future, 5.0)
                    self.check_abort()
                time.sleep(0.05)
        finally:
            self._active_goal = None
        r = res_future.result()
        return (r.status, r.result)

    def cancel_active_goal(self):
        h = self._active_goal
        if h is not None:
            h.cancel_goal_async()

    @staticmethod
    def wait_future(future, timeout: float) -> bool:
        t0 = time.monotonic()
        while not future.done():
            if time.monotonic() - t0 > timeout:
                return False
            time.sleep(0.02)
        return True

    # ------------------------------------------------------------------ costmap / tf
    def snapshot_costmap(self) -> Optional[Costmap]:
        """One global costmap message (large, and only sent while someone subscribes)."""
        self._costmap_msg = None
        self._costmap_event.clear()

        def cb(msg):
            self._costmap_msg = msg
            self._costmap_event.set()
        sub = self.create_subscription(Costmap, self.global_costmap_topic, cb, LATCHED)
        got = self._costmap_event.wait(timeout=5.0)
        self.destroy_subscription(sub)
        msg, self._costmap_msg = self._costmap_msg, None
        if not got or msg is None:
            self.get_logger().error(f'no costmap on {self.global_costmap_topic}')
            return None
        return msg

    def robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(self.global_frame, self.robot_frame, Time())
        except TransformException:
            return None
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return t.transform.translation.x, t.transform.translation.y, yaw

    def pose_stamped(self, x, y, yaw) -> PoseStamped:
        p = PoseStamped()
        p.header.frame_id = self.global_frame
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x, p.pose.position.y = float(x), float(y)
        p.pose.orientation.z = math.sin(yaw / 2.0)
        p.pose.orientation.w = math.cos(yaw / 2.0)
        return p

    # ------------------------------------------------------------------ publishing
    def publish_state(self, state: int, c: Optional[Casualty] = None):
        self.state = state
        m = RoverState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.global_frame
        m.state = state
        if c is not None:
            m.casualty_id = c.id
            m.casualty_pose = c.pose
        self.pub_state.publish(m)

    def publish_casualties(self):
        m = CasualtyArray()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.global_frame
        with self.lock:
            m.casualties = [self.casualties[i] for i in self.order]
        self.pub_casualties.publish(m)
        self.publish_markers()

    def rebuild_keepout(self):
        centres = [(c.pose.pose.position.x, c.pose.pose.position.y)
                   for c in self.casualties.values()]
        pts = keepout_disc_points(centres, self.costmap_keepout_radius)
        msg = PointCloud2()
        msg.header.frame_id = self.global_frame
        msg.height, msg.width = 1, len(pts)
        msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                      for i, n in enumerate('xyz')]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * len(pts)
        msg.is_dense = True
        msg.data = pts.tobytes()
        self.keepout_msg = msg
        self.publish_keepout()

    def publish_keepout(self):
        msg = self.keepout_msg
        if msg is not None:
            msg.header.stamp = self.get_clock().now().to_msg()
            self.pub_keepout.publish(msg)

    def publish_markers(self):
        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        colors = {Casualty.PENDING: (1.0, 0.6, 0.0), Casualty.ACTIVE: (1.0, 1.0, 0.0),
                  Casualty.STANDBY: (0.2, 0.6, 1.0), Casualty.ORBITING: (0.2, 0.6, 1.0),
                  Casualty.DONE: (0.0, 0.9, 0.3), Casualty.SKIPPED: (0.6, 0.6, 0.6)}
        stamp = self.get_clock().now().to_msg()
        with self.lock:
            items = [self.casualties[i] for i in self.order]

        def marker(ns, mid, mtype):
            m = Marker()
            m.header.frame_id, m.header.stamp = self.global_frame, stamp
            m.ns, m.id, m.type = ns, mid, mtype
            m.pose.orientation.w = 1.0
            return m
        for c in items:
            x, y = c.pose.pose.position.x, c.pose.pose.position.y
            disc = marker('keepout', c.id, Marker.CYLINDER)
            disc.pose.position.x, disc.pose.position.y, disc.pose.position.z = x, y, 0.02
            disc.scale.x = disc.scale.y = 2 * self.keepout_radius
            disc.scale.z = 0.04
            disc.color.r, disc.color.g, disc.color.b, disc.color.a = 1.0, 0.1, 0.1, 0.35
            arr.markers.append(disc)
            ring = marker('orbit', c.id, Marker.LINE_STRIP)
            ring.scale.x = 0.05
            r, g, b = colors.get(c.status, (1.0, 1.0, 1.0))
            ring.color.r, ring.color.g, ring.color.b, ring.color.a = r, g, b, 0.9
            for k in range(73):
                a = 2 * math.pi * k / 72
                p = PointStamped().point
                p.x = x + self.orbit_radius * math.cos(a)
                p.y = y + self.orbit_radius * math.sin(a)
                p.z = 0.05
                ring.points.append(p)
            arr.markers.append(ring)
            wpm = marker('orbit_waypoints', c.id, Marker.SPHERE_LIST)
            wpm.scale.x = wpm.scale.y = wpm.scale.z = 0.25
            wpm.color.g, wpm.color.b, wpm.color.a = 0.9, 0.3, 1.0
            for w in self.waypoints.get(c.id, []):
                p = PointStamped().point
                p.x, p.y, p.z = w.x, w.y, 0.15
                wpm.points.append(p)
            if wpm.points:
                arr.markers.append(wpm)
            label = marker('label', c.id, Marker.TEXT_VIEW_FACING)
            label.pose.position.x, label.pose.position.y, label.pose.position.z = x, y, 1.2
            label.scale.z = 0.45
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            label.text = f'{c.id} {STATUS_NAMES[c.status]}'
            if c.status == Casualty.DONE:
                label.text += f' {c.covered_deg:.0f}deg'
            arr.markers.append(label)
        self.pub_markers.publish(arr)

    def log_row(self, c: Casualty):
        with open(self.log_path, 'a', newline='') as f:
            csv.writer(f).writerow([
                time.strftime('%H:%M:%S'), c.id, f'{c.pose.pose.position.x:.3f}',
                f'{c.pose.pose.position.y:.3f}', f'{c.lat:.8f}', f'{c.lon:.8f}', c.source,
                STATUS_NAMES[c.status], f'{c.covered_deg:.0f}', c.reason])


def main(args=None):
    rclpy.init(args=args)
    node = CasualtyManagerSimple()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
