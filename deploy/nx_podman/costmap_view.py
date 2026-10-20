#!/usr/bin/env python3
"""Cheap 2-D viewer for the Nav2 costmaps: an OpenCV window and/or an MJPEG page.

RViz on the NX renders in software (no GPU on the NoMachine display), costs ~1.2 cores and
cannot draw Map/Costmap displays there at all (GLSL link error in indexed_8bit_image). This
draws the same grids as a plain image, centred on the robot, north up:

    global costmap   grey -> blue      (free -> inflated -> lethal)
    local costmap    yellow -> red     (drawn on top), lethal black, inscribed magenta
    footprint        green polygon     (/local_costmap/published_footprint)
    global plan      cyan line         (/plan)
    status line      age of map->base_link TF and of each costmap

Run inside rover_humble with the same DDS config as the stack:

    sudo podman exec -it -e CYCLONEDDS_URI=file:///workspace/rover_ws/src/UAS-DTU-navigation-simulation/lirovo/stack2/cyclonedds_wifi.xml \
        rover_humble bash -c "source /opt/ros/humble/setup.bash && \
        DISPLAY=:1001 XAUTHORITY=/home/rover1/.Xauthority python3 /home/rover1/podman_rover/costmap_view.py"

Options: --http 8095 also serves http://<rover-ip>:8095/ (use --no-window for the page only),
--span 16 (metres shown), --size 700 (pixels), --hz 4.
Window keys: + / - zoom, q quit.
"""
import argparse
import math
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PolygonStamped
from map_msgs.msg import OccupancyGridUpdate
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


def make_lut(free, low, high, lethal, inscribed, unknown):
    """BGR colour for each cost byte (0-100, and 255 = unknown / -1)."""
    lut = np.zeros((256, 3), np.uint8)
    lut[:] = unknown
    lut[0] = free
    t = np.linspace(0.0, 1.0, 98)[:, None]
    lut[1:99] = (np.array(low) * (1 - t) + np.array(high) * t).astype(np.uint8)
    lut[99] = inscribed
    lut[100] = lethal
    return lut


GLOBAL_LUT = make_lut(free=(235, 235, 235), low=(235, 215, 190), high=(170, 90, 20),
                      lethal=(90, 30, 0), inscribed=(200, 150, 60), unknown=(205, 205, 205))
LOCAL_LUT = make_lut(free=(0, 0, 0), low=(80, 230, 255), high=(0, 0, 230),
                     lethal=(0, 0, 0), inscribed=(200, 0, 200), unknown=(0, 0, 0))
LOCAL_DRAW = np.zeros(256, bool)
LOCAL_DRAW[1:101] = True                      # free and unknown local cells stay transparent


class Grid:
    """An OccupancyGrid as a writable uint8 array plus its geometry."""

    def __init__(self, msg):
        info = msg.info
        self.frame = msg.header.frame_id
        self.res = info.resolution
        self.ox, self.oy = info.origin.position.x, info.origin.position.y
        self.w, self.h = info.width, info.height
        self.cells = np.frombuffer(msg.data, np.int8).view(np.uint8).reshape(self.h, self.w).copy()
        self.stamp = time.time()

    def apply(self, up):
        self.cells[up.y:up.y + up.height, up.x:up.x + up.width] = (
            np.frombuffer(up.data, np.int8).view(np.uint8).reshape(up.height, up.width))
        self.stamp = time.time()

    def sample(self, xs, ys):
        """Cost bytes at world points (nearest cell); 255 outside the grid."""
        i = np.floor((xs - self.ox) / self.res).astype(np.int64)
        j = np.floor((ys - self.oy) / self.res).astype(np.int64)
        inside = (i >= 0) & (i < self.w) & (j >= 0) & (j < self.h)
        out = np.full(xs.shape, 255, np.uint8)
        out[inside] = self.cells[j[inside], i[inside]]
        return out


class Viewer(Node):
    def __init__(self):
        super().__init__('costmap_viewer')
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.local = self.glob = None
        self.footprint = None
        self.plan = None
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap', self.on_local, latched)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.on_global, latched)
        self.create_subscription(OccupancyGridUpdate, '/global_costmap/costmap_updates',
                                 self.on_global_update, 10)
        self.create_subscription(PolygonStamped, '/local_costmap/published_footprint',
                                 self.on_footprint, 10)
        self.create_subscription(Path, '/plan', self.on_plan, 10)
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)

    def on_local(self, msg):
        self.local = Grid(msg)

    def on_global(self, msg):
        self.glob = Grid(msg)

    def on_global_update(self, up):
        if self.glob is not None:
            self.glob.apply(up)

    def on_footprint(self, msg):
        self.footprint = [(p.x, p.y) for p in msg.polygon.points]

    def on_plan(self, msg):
        self.plan = np.array([(p.pose.position.x, p.pose.position.y) for p in msg.poses])

    def robot_pose(self, frame):
        """(x, y, yaw, age_s) of base_link in the costmap frame, or None."""
        try:
            t = self.tf.lookup_transform(frame, 'base_link', Time())
        except Exception:
            return None
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        stamp = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9
        age = time.time() - stamp if stamp > 0 else float('nan')
        return t.transform.translation.x, t.transform.translation.y, yaw, age


def render(v, size, span):
    img = np.full((size, size, 3), 60, np.uint8)
    ref = v.local or v.glob
    if ref is None:
        cv2.putText(img, 'waiting for /local_costmap/costmap ...', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        return img
    pose = v.robot_pose(ref.frame)
    cx, cy = (pose[0], pose[1]) if pose else (ref.ox + ref.w * ref.res / 2, ref.oy + ref.h * ref.res / 2)
    ppm = size / span                                   # pixels per metre
    pix = (np.arange(size) - size / 2 + 0.5) / ppm
    xs, ys = np.meshgrid(cx + pix, cy - pix)            # north (+y) up, +x right

    def to_px(x, y):
        return int(round(size / 2 + (x - cx) * ppm)), int(round(size / 2 - (y - cy) * ppm))

    if v.glob is not None:
        img = GLOBAL_LUT[v.glob.sample(xs, ys)]
    if v.local is not None:
        cost = v.local.sample(xs, ys)
        mask = LOCAL_DRAW[cost]
        img[mask] = LOCAL_LUT[cost[mask]]
        # outline of the local window
        l = v.local
        cv2.rectangle(img, to_px(l.ox, l.oy + l.h * l.res), to_px(l.ox + l.w * l.res, l.oy),
                      (120, 120, 120), 1)
    # 1 m grid ticks
    for k in range(-int(span), int(span) + 1):
        gx, _ = to_px(math.floor(cx) + k, cy)
        _, gy = to_px(cx, math.floor(cy) + k)
        cv2.line(img, (gx, 0), (gx, 6), (90, 90, 90), 1)
        cv2.line(img, (0, gy), (6, gy), (90, 90, 90), 1)
    if v.plan is not None and len(v.plan) > 1:
        pts = np.array([to_px(x, y) for x, y in v.plan], np.int32)
        cv2.polylines(img, [pts], False, (255, 200, 0), 2)
    if v.footprint:
        pts = np.array([to_px(x, y) for x, y in v.footprint], np.int32)
        cv2.polylines(img, [pts], True, (0, 200, 0), 2)
    if pose:
        x0, y0 = to_px(pose[0], pose[1])
        x1, y1 = to_px(pose[0] + 0.6 * math.cos(pose[2]), pose[1] + 0.6 * math.sin(pose[2]))
        cv2.arrowedLine(img, (x0, y0), (x1, y1), (0, 140, 0), 2, tipLength=0.3)

    now = time.time()
    tf_txt = 'no map->base_link TF' if pose is None else 'TF age %.2fs' % pose[3]
    age = lambda g: '-' if g is None else '%.1fs' % (now - g.stamp)
    txt = '%s | local %s | global %s | %.0fm view' % (tf_txt, age(v.local), age(v.glob), span)
    stale = pose is None or not (pose[3] < 0.3)
    cv2.rectangle(img, (0, size - 24), (size, size), (30, 30, 30), -1)
    cv2.putText(img, txt, (6, size - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (80, 80, 255) if stale else (230, 230, 230), 1)
    return img


class Stream:
    """Latest JPEG, served as an MJPEG page."""

    def __init__(self, port):
        self.jpeg = None
        self.cond = threading.Condition()
        stream = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass

            def do_GET(self):
                if self.path != '/stream':
                    body = (b'<html><body style="margin:0;background:#222">'
                            b'<img src="/stream" style="height:100vh"></body></html>')
                    self.send_response(200)
                    self.send_header('Content-Type', 'text/html')
                    self.send_header('Content-Length', str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return
                self.send_response(200)
                self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
                self.end_headers()
                try:
                    while True:
                        with stream.cond:
                            stream.cond.wait(timeout=5)
                            jpeg = stream.jpeg
                        if jpeg is None:
                            continue
                        self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '
                                         + str(len(jpeg)).encode() + b'\r\n\r\n' + jpeg + b'\r\n')
                except (BrokenPipeError, ConnectionResetError):
                    pass

        server = ThreadingHTTPServer(('0.0.0.0', port), Handler)
        server.daemon_threads = True
        threading.Thread(target=server.serve_forever, daemon=True).start()

    def push(self, img):
        ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            with self.cond:
                self.jpeg = buf.tobytes()
                self.cond.notify_all()


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--span', type=float, default=16.0, help='metres across the view')
    ap.add_argument('--size', type=int, default=700, help='image size in pixels')
    ap.add_argument('--hz', type=float, default=4.0, help='redraw rate')
    ap.add_argument('--http', type=int, default=0, help='also serve an MJPEG page on this port')
    ap.add_argument('--no-window', action='store_true', help='no OpenCV window (page only)')
    args = ap.parse_args()
    window = not args.no_window and bool(os.environ.get('DISPLAY'))
    if not window and not args.http:
        ap.error('no DISPLAY and no --http: nothing to show')

    rclpy.init()
    v = Viewer()
    threading.Thread(target=rclpy.spin, args=(v,), daemon=True).start()
    stream = Stream(args.http) if args.http else None
    if stream:
        print('costmap page: http://<rover-ip>:%d/' % args.http, flush=True)
    span = args.span
    try:
        while rclpy.ok():
            t0 = time.time()
            img = render(v, args.size, span)
            if stream:
                stream.push(img)
            if window:
                cv2.imshow('costmaps', img)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord('q'), 27):
                    break
                if key in (ord('+'), ord('=')):
                    span = max(2.0, span / 1.5)
                if key in (ord('-'), ord('_')):
                    span = min(200.0, span * 1.5)
            time.sleep(max(0.0, 1.0 / args.hz - (time.time() - t0)))
    except KeyboardInterrupt:
        pass
    finally:
        if window:
            cv2.destroyAllWindows()
        v.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
