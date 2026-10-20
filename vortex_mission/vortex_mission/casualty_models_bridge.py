#!/usr/bin/env python3
"""casualty_models_bridge: two-way link between the Nav2 mission and the casualty models.

Runs next to casualty_manager (inside the rover container). The models run on the Orin host
and are reached over HTTP on localhost (the container uses --net=host):

  frame extractor  ~/frame_extraction_final/frame_extractor.py --trigger http   (extractor_url)
  audio service    vortex_mission/host/audio_service.py -> audioPipe.py          (audio_url)

Per casualty, driven by /rover_state:
  STANDBY(id)  arrived + facing the casualty, rover still
               extractor: global start, region start   (segment 1 = parked front view)
               audio:     POST /run -> question wav, 10 s recording, triage   (poll /status)
               audio done (or failed / timed out) -> extractor region stop
               -> /casualty_manager/start_orbit(id)                      (models -> autonomy)
  ORBIT(id)    extractor: region start, then a new region every orbit_segment_deg swept
  leaves ORBIT extractor: region stop, global stop    (selection finishes on the Orin)
Results (triage, transcript, extractor run id) go to /casualty_models/result (latched JSON)
and a CSV log. A model that is down, fails or hangs never stops the mission: the orbit
starts anyway (orbit_if_audio_failed) and the extractor calls are best-effort.
"""

import csv
import json
import math
import os
import queue
import threading
import time
import urllib.error
import urllib.request
from typing import Optional

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from vortex_interfaces.msg import CasualtyArray, RoverState
from vortex_interfaces.srv import StartOrbit

from vortex_mission.orbit_segments import SweepTracker


def latched(depth=1):
    return QoSProfile(depth=depth, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL,
                      history=HistoryPolicy.KEEP_LAST)


def http_json(url: str, body: Optional[dict] = None, timeout: float = 2.0):
    """GET (body None) or POST JSON; returns (ok, parsed_json_or_error_text)."""
    data = None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(url, data=data, method='GET' if body is None else 'POST',
                                 headers={'Content-Type': 'application/json'})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return True, json.loads(r.read() or b'{}')
    except urllib.error.HTTPError as e:
        try:
            return False, json.loads(e.read() or b'{}')
        except ValueError:
            return False, f'HTTP {e.code}'
    except (urllib.error.URLError, OSError, ValueError) as e:
        return False, str(e)


class CasualtyModelsBridge(Node):

    def __init__(self):
        super().__init__('casualty_models_bridge')
        dp = self.declare_parameter
        self.extractor_url = dp('extractor_url', 'http://127.0.0.1:8090').value.rstrip('/')
        self.audio_url = dp('audio_url', 'http://127.0.0.1:8091').value.rstrip('/')
        self.enable_extractor = dp('enable_extractor', True).value
        self.enable_audio = dp('enable_audio', True).value
        self.audio_timeout = dp('audio_timeout', 120.0).value
        self.standby_min_sec = dp('standby_min_sec', 3.0).value
        self.orbit_if_audio_failed = dp('orbit_if_audio_failed', True).value
        self.orbit_segment_deg = dp('orbit_segment_deg', 90.0).value
        self.http_timeout = dp('http_timeout', 2.0).value
        self.global_frame = dp('global_frame', 'map').value
        self.robot_frame = dp('robot_frame', 'base_link').value
        self.log_dir = os.path.expanduser(dp('log_dir', '~/casualty_logs').value)

        cb = ReentrantCallbackGroup()
        self.lock = threading.Lock()
        self.phase = None            # None | 'standby' | 'orbit'
        self.cid = None              # casualty being handled
        self.cpose = None            # (x, y) of that casualty
        self.leave_standby = threading.Event()
        self.audio_running = False
        self.tracker: Optional[SweepTracker] = None
        self.run_ids = {}            # casualty id -> extractor run id
        self.latlon = {}             # casualty id -> (lat, lon)
        self.ext_q: 'queue.Queue' = queue.Queue()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(RoverState, '/rover_state', self.on_state, latched(),
                                 callback_group=cb)
        self.create_subscription(CasualtyArray, '/casualties', self.on_casualties, latched(),
                                 callback_group=cb)
        self.start_orbit = self.create_client(StartOrbit, '/casualty_manager/start_orbit',
                                              callback_group=cb)
        self.pub_result = self.create_publisher(String, '/casualty_models/result', latched(50))
        self.create_timer(0.2, self.track_orbit, callback_group=cb)
        threading.Thread(target=self.extractor_worker, daemon=True).start()

        os.makedirs(self.log_dir, exist_ok=True)
        self.log_path = os.path.join(self.log_dir, time.strftime('models_%Y%m%d_%H%M%S.csv'))
        with open(self.log_path, 'w', newline='') as f:
            csv.writer(f).writerow(['time', 'casualty_id', 'lat', 'lon', 'extractor_run',
                                    'audio_status', 'verbal_alertness', 'injury_regions',
                                    'injury_types', 'transcript', 'audio_out_dir'])
        self.check_services()
        self.get_logger().info(
            f'casualty_models_bridge ready: extractor {self.extractor_url} '
            f'({"on" if self.enable_extractor else "off"}), audio {self.audio_url} '
            f'({"on" if self.enable_audio else "off"}), orbit segments every '
            f'{self.orbit_segment_deg:g} deg. Log: {self.log_path}')

    # ------------------------------------------------------------------ inputs
    def check_services(self):
        for name, url, on in (('frame extractor', self.extractor_url + '/status',
                               self.enable_extractor),
                              ('audio service', self.audio_url + '/status', self.enable_audio)):
            if on:
                ok, res = http_json(url, timeout=self.http_timeout)
                if ok:
                    self.get_logger().info(f'{name} reachable at {url}')
                else:
                    self.get_logger().warn(f'{name} NOT reachable at {url} ({res}); the '
                                           f'mission still runs without it')

    def on_casualties(self, msg: CasualtyArray):
        for c in msg.casualties:
            self.latlon[c.id] = (c.lat, c.lon)

    def on_state(self, msg: RoverState):
        cid = msg.casualty_id
        pose = (msg.casualty_pose.pose.position.x, msg.casualty_pose.pose.position.y)
        with self.lock:
            phase, cur = self.phase, self.cid
        if msg.state == RoverState.STANDBY:
            if phase is None or cur != cid:
                if phase is not None:
                    self.close_casualty('a new STANDBY started')
                with self.lock:
                    self.phase, self.cid, self.cpose = 'standby', cid, pose
                    self.leave_standby.clear()
                threading.Thread(target=self.standby_sequence, args=(cid,), daemon=True).start()
        elif msg.state == RoverState.ORBIT:
            if phase != 'orbit' or cur != cid:
                self.orbit_started(cid, pose)
        elif phase is not None:                       # TRAVEL / IDLE: done with this casualty
            self.close_casualty('rover left the casualty')

    # ------------------------------------------------------------------ STANDBY: audio
    def standby_sequence(self, cid: int):
        t0 = time.monotonic()
        self.get_logger().info(f'casualty {cid}: arrived -> starting frame extractor + audio')
        self.extractor('global', 'start', cid)
        self.extractor('region', 'start', cid)
        result = {'casualty_id': cid, 'audio_status': 'disabled'}
        if self.enable_audio:
            result = self.run_audio(cid)
        # keep the parked segment open a minimum time so it has frames even if audio was quick
        while time.monotonic() - t0 < self.standby_min_sec and not self.leave_standby.is_set():
            time.sleep(0.1)
        if self.leave_standby.is_set():
            self.get_logger().warn(f'casualty {cid}: rover left STANDBY before the audio step '
                                   f'finished (skip / manual trigger / abort)')
            self.publish_result(cid, result)
            return
        self.extractor('region', 'stop', cid)
        self.publish_result(cid, result)
        if result['audio_status'] != 'done' and not self.orbit_if_audio_failed:
            self.get_logger().warn(f'casualty {cid}: audio {result["audio_status"]}; waiting '
                                   f'for a manual orbit trigger (orbit_if_audio_failed=false)')
            return
        self.request_orbit(cid)

    def run_audio(self, cid: int) -> dict:
        res = {'casualty_id': cid, 'audio_status': 'failed'}
        ok, r = http_json(self.audio_url + '/run', {'casualty_id': cid}, self.http_timeout)
        if not ok:
            self.get_logger().error(f'casualty {cid}: audio service did not start a job: {r}')
            res['error'] = f'start failed: {r}'
            return res
        self.audio_running = True
        self.get_logger().info(f'casualty {cid}: audio job started (question, 10 s recording, '
                               f'triage)')
        t0, last = time.monotonic(), None
        try:
            while True:
                if self.leave_standby.is_set():
                    http_json(self.audio_url + '/cancel', {}, self.http_timeout)
                    res['audio_status'] = 'cancelled'
                    return res
                if time.monotonic() - t0 > self.audio_timeout:
                    http_json(self.audio_url + '/cancel', {}, self.http_timeout)
                    self.get_logger().error(f'casualty {cid}: audio took more than '
                                            f'{self.audio_timeout:.0f} s; cancelled')
                    res['audio_status'] = 'timeout'
                    return res
                ok, st = http_json(self.audio_url + '/status', None, self.http_timeout)
                if ok:
                    state = st.get('state')
                    if state != last:
                        self.get_logger().info(f'casualty {cid}: audio {state}')
                        last = state
                    if state in ('done', 'failed', 'cancelled'):
                        res.update({k: st.get(k) for k in ('triage', 'transcript', 'out_dir',
                                                           'error', 'question_played')})
                        res['audio_status'] = state
                        res['audio_seconds'] = round(time.monotonic() - t0, 1)
                        if state == 'done':
                            self.get_logger().info(f'casualty {cid}: audio triage '
                                                   f'{json.dumps(st.get("triage"))}')
                        else:
                            self.get_logger().error(f'casualty {cid}: audio {state}: '
                                                    f'{st.get("error")}')
                        return res
                time.sleep(1.0)
        finally:
            self.audio_running = False

    def request_orbit(self, cid: int):
        if not self.start_orbit.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/casualty_manager/start_orbit not available')
            return
        req = StartOrbit.Request()
        req.casualty_id = cid
        fut = self.start_orbit.call_async(req)
        t0 = time.monotonic()
        while not fut.done() and time.monotonic() - t0 < 5.0:
            time.sleep(0.05)
        r = fut.result()
        if r is None:
            self.get_logger().error(f'casualty {cid}: start_orbit gave no answer')
        else:
            log = self.get_logger().info if r.accepted else self.get_logger().warn
            log(f'casualty {cid}: start_orbit -> {r.message}')

    # ------------------------------------------------------------------ ORBIT: segments
    def orbit_started(self, cid: int, pose):
        with self.lock:
            was_standby = self.phase == 'standby' and self.cid == cid
            self.phase, self.cid, self.cpose = 'orbit', cid, pose
            self.tracker = SweepTracker(pose[0], pose[1], self.orbit_segment_deg)
        self.leave_standby.set()            # a manual trigger during audio cancels the audio
        if not was_standby:                 # bridge started mid-orbit, or no STANDBY seen
            self.extractor('global', 'start', cid)
        self.extractor('region', 'start', cid)
        self.get_logger().info(f'casualty {cid}: orbit started -> extractor segment every '
                               f'{self.orbit_segment_deg:g} deg')

    def track_orbit(self):
        with self.lock:
            tracker, cid = (self.tracker, self.cid) if self.phase == 'orbit' else (None, None)
        if tracker is None:
            return
        pose = self.robot_xy()
        if pose is not None and tracker.update(*pose):
            self.get_logger().info(f'casualty {cid}: {tracker.swept_deg:.0f} deg swept -> '
                                   f'next extractor segment')
            self.extractor('region', 'stop', cid)
            self.extractor('region', 'start', cid)

    def close_casualty(self, why: str):
        with self.lock:
            phase, cid, tracker = self.phase, self.cid, self.tracker
            self.phase, self.tracker = None, None
        self.leave_standby.set()
        if phase is None:
            return
        swept = f', {tracker.swept_deg:.0f} deg swept' if tracker else ''
        self.get_logger().info(f'casualty {cid}: {why}{swept} -> closing extractor run')
        self.extractor('region', 'stop', cid)
        self.extractor('global', 'stop', cid)

    def robot_xy(self):
        try:
            t = self.tf_buffer.lookup_transform(self.global_frame, self.robot_frame, Time())
        except TransformException:
            return None
        return t.transform.translation.x, t.transform.translation.y

    # ------------------------------------------------------------------ extractor (ordered)
    def extractor(self, kind: str, state: str, cid: int):
        if self.enable_extractor:
            self.ext_q.put((kind, state, cid))

    def extractor_worker(self):
        """Sends extractor triggers one by one, in order, off the ROS callback threads."""
        warned = 0.0
        while rclpy.ok():
            kind, state, cid = self.ext_q.get()
            ok, r = http_json(self.extractor_url + '/trigger', {'kind': kind, 'state': state},
                              self.http_timeout)
            if not ok:
                if time.monotonic() - warned > 10.0:
                    self.get_logger().warn(f'frame extractor: {kind} {state} failed ({r})')
                    warned = time.monotonic()
                continue
            self.get_logger().info(f'frame extractor: {kind} {state} (casualty {cid})')
            if kind == 'global' and state == 'start':
                time.sleep(0.3)                 # the extractor handles triggers on its own thread
                ok, st = http_json(self.extractor_url + '/status', None, self.http_timeout)
                if ok and st.get('run_id'):
                    self.run_ids[cid] = st['run_id']
                    self.get_logger().info(f'casualty {cid} = extractor run {st["run_id"]}')

    # ------------------------------------------------------------------ results
    def publish_result(self, cid: int, res: dict):
        lat, lon = self.latlon.get(cid, (math.nan, math.nan))
        res = dict(res, casualty_id=cid, lat=lat, lon=lon, extractor_run=self.run_ids.get(cid),
                   stamp=time.strftime('%Y-%m-%d %H:%M:%S'))
        self.pub_result.publish(String(data=json.dumps(res)))
        tri = res.get('triage') or {}
        with open(self.log_path, 'a', newline='') as f:
            csv.writer(f).writerow([
                res['stamp'], cid, f'{lat:.8f}', f'{lon:.8f}', res['extractor_run'] or '',
                res.get('audio_status'), tri.get('verbal_alertness', ''),
                json.dumps(tri.get('injury_regions', '')), json.dumps(tri.get('injury_types', '')),
                (res.get('transcript') or '').strip(), res.get('out_dir') or ''])


def main(args=None):
    rclpy.init(args=args)
    node = CasualtyModelsBridge()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.close_casualty('bridge shutting down')
        time.sleep(0.5)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
