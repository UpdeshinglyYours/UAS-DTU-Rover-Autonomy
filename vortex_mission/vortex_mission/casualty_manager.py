#!/usr/bin/env python3
"""casualty_manager: casualty registry + mission executive.

Per casualty:  TRAVEL -> FACE -> STANDBY -(start_orbit)-> ORBIT -> DONE/SKIPPED -> next

Inputs   ~/add_casualty (srv), /clicked_point (RViz "Publish Point"),
         ~/start_orbit (srv, called by rc_orbit_trigger now, the model script later),
         ~/start ~/pause ~/skip_current ~/abort (std_srvs/Trigger)
Outputs  /casualties (latched), /rover_state (latched), /casualty_keepout (costmap),
         ~/markers (RViz), a CSV log per run.
"""

import csv
import math
import os
import threading
import time
from typing import Dict, List, Optional

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from builtin_interfaces.msg import Duration as DurationMsg
from geographic_msgs.msg import GeoPointStamped
from geometry_msgs.msg import PointStamped, PoseStamped
from nav2_msgs.action import ComputePathToPose, NavigateToPose, Spin
from nav2_msgs.msg import Costmap
from sensor_msgs.msg import NavSatFix, PointCloud2, PointField
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from vortex_interfaces.action import OrbitCasualty
from vortex_interfaces.msg import Casualty, CasualtyArray, OrbitReport, RoverState
from vortex_interfaces.srv import AddCasualty, StartOrbit

from vortex_mission.approach import (ApproachPlan, face_yaw_delta, keepout_disc_points,
                                     plan_approach)
from vortex_mission.geo import latlon_to_meter_offset, meter_offset_to_latlon
from vortex_mission.orbit_geometry import Grid, OrbitParams, grid_from_costmap_msg

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST)

STATUS_NAMES = {Casualty.PENDING: 'PENDING', Casualty.ACTIVE: 'ACTIVE',
                Casualty.STANDBY: 'STANDBY', Casualty.ORBITING: 'ORBITING',
                Casualty.DONE: 'DONE', Casualty.SKIPPED: 'SKIPPED'}
OUTCOME_NAMES = {OrbitReport.FULL: 'FULL', OrbitReport.PARTIAL: 'PARTIAL',
                 OrbitReport.BLOCKED: 'ORBIT_BLOCKED', OrbitReport.KEEPOUT_GUARD: 'KEEPOUT_GUARD',
                 OrbitReport.TIMEOUT: 'TIMEOUT', OrbitReport.CANCELED: 'CANCELED',
                 OrbitReport.ERROR: 'ERROR'}


class Aborted(Exception):
    """Raised inside the mission thread when the operator skips / aborts."""


class NotReady(Exception):
    """Nav2 isn't up yet (no costmap / pose / action server): retry later, don't skip."""


class CasualtyManager(Node):

    def __init__(self):
        super().__init__('casualty_manager')
        dp = self.declare_parameter
        self.keepout_radius = dp('keepout_radius', 2.2).value
        self.costmap_keepout_radius = dp('costmap_keepout_radius', 2.3).value
        self.approach_radius = dp('approach_radius', 3.3).value
        self.approach_candidates = dp('approach_candidates', 16).value
        self.planner_candidates = dp('planner_candidates', 4).value
        self.orbit_radius = dp('orbit_radius', 2.9).value
        self.orbit_max_detour_radius = dp('orbit_max_detour_radius', 4.0).value
        self.orbit_max_sweep_deg = dp('orbit_max_sweep_deg', 360.0).value
        self.orbit_clockwise = dp('orbit_clockwise', True).value
        self.geom = OrbitParams(
            radius=self.orbit_radius,
            max_detour_radius=self.orbit_max_detour_radius,
            detour_step=dp('orbit_detour_step', 0.1).value,
            sample_deg=dp('orbit_sample_deg', 3.0).value,
            max_radius_slope=dp('orbit_max_radius_slope', 0.05).value,
            keepout_radius=self.keepout_radius,
            keepout_guard=dp('planning_guard', 0.1).value,
            footprint_length=dp('footprint_length', 1.0).value,
            footprint_width=dp('footprint_width', 0.9).value,
            edge_margin_deg=dp('orbit_edge_margin_deg', 6.0).value)
        self.standby_timeout = dp('standby_timeout', 0.0).value
        self.auto_start = dp('auto_start', True).value
        self.face_casualty = dp('face_casualty', True).value
        # Spin 360 deg in place right before the orbit so its plan sees (almost) the whole
        # orbit band: ~46% of it is visible when parked, ~96% after a spin. Done after the
        # trigger, not on arrival: the global costmap forgets after ~30 s and the audio
        # model wants the rover still during STANDBY.
        self.look_around = dp('look_around_before_orbit', True).value
        # Who drives the orbit: 'custom' (plugin tracker) or 'mppi' (Nav2 FollowPath with
        # OrbitController). Read at every orbit -> switch live with `ros2 param set`.
        dp('orbit_controller', 'custom')
        self.look_around_settle = dp('look_around_settle', 1.0).value
        self.approach_timeout_base = dp('approach_timeout_base', 60.0).value
        self.approach_timeout_per_m = dp('approach_timeout_per_m', 4.0).value
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
        self.next_id = 1
        self.current_id: Optional[int] = None
        self.state = RoverState.IDLE
        self.running = self.auto_start
        self.paused = False
        self.skip_event = threading.Event()
        self.abort_event = threading.Event()
        self.orbit_request = threading.Event()
        self.keepout_msg: Optional[PointCloud2] = None
        self.reports: Dict[int, OrbitReport] = {}
        self.gp_origin = None        # (lat, lon) from MAVROS
        self.gui_origin = None       # (lat, lon) from mission_planner_live2
        self._active_goal = None     # goal handle we may need to cancel
        self._costmap_msg = None
        self._costmap_event = threading.Event()
        self._last_plan = None       # (ApproachPlan, Casualty) shown in RViz

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
        self.create_subscription(OrbitReport, '/orbit_casualty/report', self.on_orbit_report,
                                 LATCHED, callback_group=cb)

        self.create_service(AddCasualty, '~/add_casualty', self.srv_add, callback_group=cb)
        self.create_service(StartOrbit, '~/start_orbit', self.srv_start_orbit, callback_group=cb)
        self.create_service(Trigger, '~/start', self.srv_start, callback_group=cb)
        self.create_service(Trigger, '~/pause', self.srv_pause, callback_group=cb)
        self.create_service(Trigger, '~/skip_current', self.srv_skip, callback_group=cb)
        self.create_service(Trigger, '~/abort', self.srv_abort, callback_group=cb)

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose', callback_group=cb)
        self.plan_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose',
                                        callback_group=cb)
        self.spin_client = ActionClient(self, Spin, 'spin', callback_group=cb)
        self.orbit_client = ActionClient(self, OrbitCasualty, 'orbit_casualty', callback_group=cb)

        self.create_timer(1.0, self.publish_keepout, callback_group=cb)

        os.makedirs(self.log_dir, exist_ok=True)
        self.log_path = os.path.join(self.log_dir, time.strftime('casualties_%Y%m%d_%H%M%S.csv'))
        with open(self.log_path, 'w', newline='') as f:
            csv.writer(f).writerow(['time', 'id', 'x', 'y', 'lat', 'lon', 'source', 'status',
                                    'covered_deg', 'reason', 'min_body_distance'])

        self.publish_state(RoverState.IDLE)
        self.publish_casualties()
        self.mission_thread = threading.Thread(target=self.mission_loop, daemon=True)
        self.mission_thread.start()
        self.get_logger().info(
            f'casualty_manager ready: keepout {self.keepout_radius} m, approach '
            f'{self.approach_radius} m, orbit {self.orbit_radius} m, auto_start={self.auto_start}. '
            f'Log: {self.log_path}')

    # ------------------------------------------------------------------ origin
    def on_gp_origin(self, msg: GeoPointStamped):
        if self.gp_origin is None:
            self.get_logger().info(
                f'EKF origin from MAVROS: {msg.position.latitude:.7f}, {msg.position.longitude:.7f}')
        self.gp_origin = (msg.position.latitude, msg.position.longitude)

    def on_gui_origin(self, msg: NavSatFix):
        self.gui_origin = (msg.latitude, msg.longitude)

    def origin(self):
        """EKF origin (lat, lon) used to place lat/lon casualties in `map`."""
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
        ok, cid, text = self.add_casualty(msg.point.x, msg.point.y, float('nan'),
                                          float('nan'), 'rviz')
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
                d = math.hypot(c.pose.pose.position.x - x, c.pose.pose.position.y - y)
                if d < self.duplicate_radius:
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
            c.pose.pose.position.x = float(x)
            c.pose.pose.position.y = float(y)
            c.pose.pose.orientation.w = 1.0
            c.lat, c.lon = float(lat), float(lon)
            c.status = Casualty.PENDING
            c.source = source
            self.casualties[c.id] = c
            self.order.append(c.id)
            # Orbit circle (+ padded half-width) of one casualty vs keepout of another.
            reach = self.orbit_radius + self.geom.footprint_width / 2 + self.keepout_radius
            warn = [o.id for o in self.casualties.values() if o.id != c.id and math.hypot(
                o.pose.pose.position.x - x, o.pose.pose.position.y - y) < reach]
            self.rebuild_keepout()
        self.publish_casualties()
        self.log_row(c)
        text = f'added casualty {c.id} at ({x:.2f}, {y:.2f}) from {source}'
        if warn:
            text += (f'; WARNING: within {reach:.1f} m of casualty {warn} -- their keepouts '
                     f'will block part of each other\'s orbit')
        return True, c.id, text

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

    def on_orbit_report(self, msg: OrbitReport):
        self.reports[msg.casualty_id] = msg

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
            rx, ry, _ = pose
            c = min(pending, key=lambda k: math.hypot(k.pose.pose.position.x - rx,
                                                      k.pose.pose.position.y - ry))
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
                with self.lock:
                    if self.abort_event.is_set():
                        c.status, c.reason = Casualty.PENDING, ''
                    else:
                        c.status, c.reason = Casualty.SKIPPED, str(e) or 'SKIPPED_BY_OPERATOR'
                self.finish(c)
            except Exception as e:  # never let one casualty kill the mission
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

        # 1. Plan where to park.
        grid = self.snapshot_costmap()
        pose = self.robot_pose()
        if grid is None or pose is None:
            raise NotReady('no global costmap or robot pose yet (is Nav2 running?)')
        plan = plan_approach(grid, cx, cy, pose[0], pose[1], self.geom, self.approach_radius,
                             self.approach_candidates)
        if plan.circle_free:
            self.get_logger().info('orbit circle fully free')
        elif plan.arcs:
            self.get_logger().info(f'orbit circle partly blocked: largest free arc '
                                   f'{plan.largest_arc_deg:.0f} deg; parking at its end')
        else:
            self.get_logger().warn('no free orbit arc: will park for STANDBY only')
        self._last_plan = (plan, c)
        self.publish_markers()

        # 2. Drive there.
        if not self.approach(c, plan):
            with self.lock:
                c.status, c.reason = Casualty.SKIPPED, 'APPROACH_FAILED'
            self.finish(c)
            return

        # 3. Face the casualty.
        if self.face_casualty:
            self.face(c)

        # 4. STANDBY until start_orbit.
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

        # 5. ORBIT (look around first, so the orbit plans on a full, fresh view).
        with self.lock:
            c.status = Casualty.ORBITING
        self.publish_casualties()
        self.publish_state(RoverState.ORBIT, c)
        # read live, so it can be toggled with `ros2 param set` during testing
        if self.get_parameter('look_around_before_orbit').value:
            self.look_around_spin()
        report = self.orbit(c)
        with self.lock:
            if report is None:
                c.status, c.reason = Casualty.DONE, 'NO_REPORT'
            else:
                c.covered_deg = report.covered_deg
                c.reason = OUTCOME_NAMES.get(report.outcome, str(report.outcome))
                if report.outcome == OrbitReport.CANCELED and self.skip_event.is_set():
                    c.status = Casualty.SKIPPED
                    c.reason = 'SKIPPED_BY_OPERATOR'
                else:
                    c.status = Casualty.DONE
        self.finish(c, report)

    def finish(self, c: Casualty, report: Optional[OrbitReport] = None):
        self.current_id = None
        self.publish_casualties()
        self.log_row(c, report)
        self.publish_state(RoverState.TRAVEL)
        self.get_logger().info(f'casualty {c.id}: {STATUS_NAMES[c.status]} '
                               f'(covered {c.covered_deg:.0f} deg) {c.reason}')

    # ------------------------------------------------------------------ steps
    def approach(self, c: Casualty, plan: ApproachPlan) -> bool:
        for client in (self.plan_client, self.nav_client):
            if not client.wait_for_server(timeout_sec=5.0):
                raise NotReady(f'action server {client._action_name} not available')
        if not plan.candidates:
            self.get_logger().error('no parking spot around the casualty is free')
            return False
        # Ask the planner for each candidate (stop early: a few is enough), then
        # full circle -> shortest path first; partial circle -> keep the arc order.
        scored = []
        for k in plan.candidates:
            self.check_abort()
            length = self.path_length(k)
            if length is not None:
                scored.append((length, k))
            if len(scored) >= self.planner_candidates:
                break
        if not any(k.on_arc for _, k in scored):
            scored.sort(key=lambda s: s[0])
        for attempt, (length, k) in enumerate(scored[:2]):
            self.check_abort()
            timeout = self.approach_timeout_base + self.approach_timeout_per_m * length
            self.get_logger().info(f'approach: parking at {math.degrees(k.theta) % 360:.0f} deg '
                                   f'({k.x:.2f}, {k.y:.2f}), path {length:.1f} m')
            goal = NavigateToPose.Goal()
            goal.pose = self.pose_stamped(k.x, k.y, k.yaw)
            result = self.run_action(self.nav_client, goal, timeout)
            if result is not None and result[0] == 4:   # STATUS_SUCCEEDED
                return True
            self.get_logger().warn(f'approach attempt {attempt + 1} failed')
        return False

    def face(self, c: Casualty):
        pose = self.robot_pose()
        if pose is None:
            return
        delta = face_yaw_delta(pose[0], pose[1], pose[2], c.pose.pose.position.x,
                               c.pose.pose.position.y)
        if abs(delta) < math.radians(5.0):
            return
        goal = Spin.Goal()
        goal.target_yaw = float(delta)
        goal.time_allowance = DurationMsg(sec=20)
        result = self.run_action(self.spin_client, goal, 30.0)
        if result is None or result[0] != 4:
            self.get_logger().warn('could not turn to face the casualty (ALIGN_FAILED); '
                                   'continuing to STANDBY')

    def look_around_spin(self):
        self.get_logger().info('look-around: spinning 360 deg so the orbit sees the whole circle')
        goal = Spin.Goal()
        goal.target_yaw = float(-2.0 * math.pi)   # clockwise, same sense as the orbit
        goal.time_allowance = DurationMsg(sec=40)
        result = self.run_action(self.spin_client, goal, 50.0)
        if result is None or result[0] != 4:
            self.get_logger().warn('look-around spin did not complete; orbiting with what was seen')
        time.sleep(self.look_around_settle)       # let the costmaps take in the last scans

    def orbit(self, c: Casualty) -> Optional[OrbitReport]:
        self.reports.pop(c.id, None)
        goal = OrbitCasualty.Goal()
        goal.casualty_id = c.id
        goal.center = c.pose
        goal.radius = float(self.orbit_radius)
        goal.standby_radius = float(self.approach_radius)
        goal.keepout_radius = float(self.keepout_radius)
        goal.max_detour_radius = float(self.orbit_max_detour_radius)
        goal.clockwise = bool(self.orbit_clockwise)
        goal.max_sweep_deg = float(self.orbit_max_sweep_deg)
        goal.controller = str(self.get_parameter('orbit_controller').value)
        self.get_logger().info(f'orbit controller: {goal.controller}')
        self.run_action(self.orbit_client, goal, 600.0, allow_skip=True)
        t0 = time.monotonic()
        while time.monotonic() - t0 < 3.0 and c.id not in self.reports:
            time.sleep(0.05)
        return self.reports.get(c.id)

    # ------------------------------------------------------------------ action plumbing
    def check_abort(self):
        if self.abort_event.is_set():
            raise Aborted('ABORTED')
        if self.skip_event.is_set():
            raise Aborted('SKIPPED_BY_OPERATOR')

    def run_action(self, client: ActionClient, goal, timeout: float, allow_skip: bool = False):
        """Send a goal and wait. Returns (status, result) or None. Raises Aborted on skip."""
        if not client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f'action server {client._action_name} not available')
            return None
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
                if self.abort_event.is_set() or (self.skip_event.is_set() and not allow_skip):
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

    def path_length(self, k) -> Optional[float]:
        if not self.plan_client.wait_for_server(timeout_sec=5.0):
            return None
        goal = ComputePathToPose.Goal()
        goal.goal = self.pose_stamped(k.x, k.y, k.yaw)
        goal.use_start = False
        send = self.plan_client.send_goal_async(goal)
        if not self.wait_future(send, 5.0) or not send.result().accepted:
            return None
        res = send.result().get_result_async()
        if not self.wait_future(res, 10.0):
            return None
        poses = res.result().result.path.poses
        if res.result().status != 4 or len(poses) < 2:
            return None
        xy = np.array([[p.pose.position.x, p.pose.position.y] for p in poses])
        return float(np.sum(np.hypot(*np.diff(xy, axis=0).T)))

    # ------------------------------------------------------------------ costmap / tf
    def snapshot_costmap(self) -> Optional[Grid]:
        """One global costmap message (it is ~36 MB and only sent while subscribed)."""
        self._costmap_msg = None
        self._costmap_event.clear()

        def cb(msg):
            self._costmap_msg = msg
            self._costmap_event.set()
        sub = self.create_subscription(Costmap, self.global_costmap_topic, cb, LATCHED)
        got = self._costmap_event.wait(timeout=5.0)
        self.destroy_subscription(sub)
        if not got or self._costmap_msg is None:
            self.get_logger().error(f'no costmap on {self.global_costmap_topic}')
            return None
        grid = grid_from_costmap_msg(self._costmap_msg)
        self._costmap_msg = None
        return grid

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
        plan, planned = self._last_plan if self._last_plan else (None, None)
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
        mid = 0
        for c in items:
            x, y = c.pose.pose.position.x, c.pose.pose.position.y
            disc = Marker()
            disc.header.frame_id, disc.header.stamp = self.global_frame, stamp
            disc.ns, disc.id, disc.type = 'keepout', c.id, Marker.CYLINDER
            disc.pose.position.x, disc.pose.position.y, disc.pose.position.z = x, y, 0.02
            disc.pose.orientation.w = 1.0
            disc.scale.x = disc.scale.y = 2 * self.keepout_radius
            disc.scale.z = 0.04
            disc.color.r, disc.color.g, disc.color.b, disc.color.a = 1.0, 0.1, 0.1, 0.35
            arr.markers.append(disc)
            ring = Marker()
            ring.header.frame_id, ring.header.stamp = self.global_frame, stamp
            ring.ns, ring.id, ring.type = 'orbit', c.id, Marker.LINE_STRIP
            ring.pose.orientation.w = 1.0
            ring.scale.x = 0.05
            r, g, b = colors.get(c.status, (1.0, 1.0, 1.0))
            ring.color.r, ring.color.g, ring.color.b, ring.color.a = r, g, b, 0.9
            for k in range(73):
                a = 2 * math.pi * k / 72
                p = PointStamped().point
                p.x, p.y, p.z = x + self.orbit_radius * math.cos(a), \
                    y + self.orbit_radius * math.sin(a), 0.05
                ring.points.append(p)
            arr.markers.append(ring)
            label = Marker()
            label.header.frame_id, label.header.stamp = self.global_frame, stamp
            label.ns, label.id, label.type = 'label', c.id, Marker.TEXT_VIEW_FACING
            label.pose.position.x, label.pose.position.y, label.pose.position.z = x, y, 1.2
            label.pose.orientation.w = 1.0
            label.scale.z = 0.45
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            label.text = f'{c.id} {STATUS_NAMES[c.status]}'
            if c.status == Casualty.DONE:
                label.text += f' {c.covered_deg:.0f}deg'
            arr.markers.append(label)
            mid = max(mid, c.id)
        if plan is not None and planned is not None:
            # Planned orbit path: profile r(theta), blocked parts omitted.
            path = Marker()
            path.header.frame_id, path.header.stamp = self.global_frame, stamp
            path.ns, path.id, path.type = 'planned_orbit', planned.id, Marker.POINTS
            path.pose.orientation.w = 1.0
            path.scale.x = path.scale.y = 0.08
            path.color.g, path.color.a = 1.0, 1.0
            px, py = planned.pose.pose.position.x, planned.pose.pose.position.y
            for th, r in zip(plan.thetas, plan.profile):
                if not math.isnan(r):
                    p = PointStamped().point
                    p.x, p.y, p.z = px + r * math.cos(th), py + r * math.sin(th), 0.1
                    path.points.append(p)
            arr.markers.append(path)
        self.pub_markers.publish(arr)

    def log_row(self, c: Casualty, report: Optional[OrbitReport] = None):
        with open(self.log_path, 'a', newline='') as f:
            csv.writer(f).writerow([
                time.strftime('%H:%M:%S'), c.id, f'{c.pose.pose.position.x:.3f}',
                f'{c.pose.pose.position.y:.3f}', f'{c.lat:.8f}', f'{c.lon:.8f}', c.source,
                STATUS_NAMES[c.status], f'{c.covered_deg:.0f}', c.reason,
                f'{report.min_body_distance:.3f}' if report else ''])


def main(args=None):
    rclpy.init(args=args)
    node = CasualtyManager()
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
