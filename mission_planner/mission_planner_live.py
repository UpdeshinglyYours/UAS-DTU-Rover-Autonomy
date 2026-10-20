#!/usr/bin/env python3
"""
================================================================================
ROS 2 Tool: mission_planner_live.py (PyQtGraph edition)
================================================================================
Same tool as before -- click on the map to queue mission waypoints, press
'g' to dispatch the whole queue live -- rebuilt on PyQtGraph instead of
matplotlib, because matplotlib's full-figure redraw every tick (no
blitting) caused visible lag on the live rover/GPS trail, and its CPU-only
rendering made zooming the satellite image feel sluggish. PyQtGraph's
ViewBox does GPU-accelerated pan/zoom by default and is built specifically
for this "high-frequency real-time update" use case.

Feature set is unchanged from the matplotlib version:
- Live GPS + odom subscription, same origin-lock as find_location.py /
  casualty_dispatcher.py: running average of the MAVROS-origin estimate,
  locked after 10 samples (~1s) or 3+ samples once the rover has moved
  >0.5m. No settling/quality gate -- only run this once you've judged the
  GPS fix/lat-lon to already be stable.
- Satellite background image, distance rings, rover trail (smooth,
  MAVROS-odom-based) vs raw GPS trail (dotted), rover position + heading.
- Click to queue a numbered waypoint; 'g' dispatches the whole queue via
  NavigateThroughPoses + publishes the keepout-ring PointCloud2, same
  mechanism as casualty_dispatcher.py.

Keyboard Shortcuts:
  - Left Click:  queue a mission waypoint (numbered marker, not yet sent)
  - 'g':         dispatch the entire queued mission right now
  - 'c':         clear queued (undispatched) waypoints
  - 'f':         full 1km map view
  - 'r':         zoom in on rover
  - 'o':         zoom in on MAVROS (0, 0) origin

Mouse: left-drag pans, scroll wheel zooms (PyQtGraph ViewBox defaults) --
a genuine click (press+release with minimal movement) queues a waypoint
without being swallowed by the pan gesture.
================================================================================
"""

import os
import sys
import math
import struct
import argparse
from typing import Optional, List, Tuple

import numpy as np
from PIL import Image

import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui, QtWidgets

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import NavSatFix, PointCloud2, PointField
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav2_msgs.action import NavigateThroughPoses
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import StaticTransformBroadcaster

# WGS-84 Ellipsoid Constants
WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = 2.0 * WGS84_F - WGS84_F * WGS84_F


def latlon_to_meter_offset(lat0: float, lon0: float, lat: float, lon: float) -> Tuple[float, float]:
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    lat0_rad = math.radians(lat0)
    lon0_rad = math.radians(lon0)
    sin_lat0 = math.sin(lat0_rad)
    denom = math.sqrt(1.0 - WGS84_E2 * sin_lat0 * sin_lat0)
    rn = WGS84_A / denom
    rm = WGS84_A * (1.0 - WGS84_E2) / (denom * denom * denom)
    east_m = (lon_rad - lon0_rad) * rn * math.cos(lat0_rad)
    north_m = (lat_rad - lat0_rad) * rm
    return east_m, north_m


def meter_offset_to_latlon(lat0: float, lon0: float, east_m: float, north_m: float) -> Tuple[float, float]:
    lat0_rad = math.radians(lat0)
    sin_lat0 = math.sin(lat0_rad)
    denom = math.sqrt(1.0 - WGS84_E2 * sin_lat0 * sin_lat0)
    rn = WGS84_A / denom
    rm = WGS84_A * (1.0 - WGS84_E2) / (denom * denom * denom)
    dlat = north_m / rm
    dlon = east_m / (rn * math.cos(lat0_rad))
    return math.degrees(lat0_rad + dlat), math.degrees(math.radians(lon0) + dlon)


def quaternion_to_yaw_deg(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.degrees(math.atan2(siny, cosy))


def compass_bearing_deg(yaw_enu_deg: float) -> float:
    return (90.0 - yaw_enu_deg) % 360.0


def circle_xy(cx: float, cy: float, r: float, n: int = 96):
    t = np.linspace(0, 2 * np.pi, n)
    return cx + r * np.cos(t), cy + r * np.sin(t)


class RosBridge(Node):
    """All ROS2 I/O: subscriptions, the Nav2 action client, and the
    keepout-ring publisher. Kept separate from the Qt widget so GUI code
    never has to think about QoS/messages directly."""

    def __init__(self):
        super().__init__('mission_planner_live')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                          history=HistoryPolicy.KEEP_LAST, depth=10)

        self.rover_lat = None
        self.rover_lon = None
        self.rover_gps_east = None
        self.rover_gps_north = None
        self.gps_count = 0

        self.rover_local_x = None
        self.rover_local_y = None
        self.rover_yaw_deg = 0.0
        self.odom_count = 0

        self.trail_local_x: List[float] = []
        self.trail_local_y: List[float] = []
        self.raw_gps_e: List[float] = []
        self.raw_gps_n: List[float] = []

        self.on_new_data = None  # set by the GUI to a no-arg callback

        self.create_subscription(NavSatFix, '/mavros/global_position/raw/fix', self._on_gps, qos)
        self.create_subscription(Odometry, '/mavros/local_position/odom', self._on_odom, qos)

        self._action_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        self._keepout_pub = self.create_publisher(PointCloud2, '/casualty_keepout', 10)
        self._keepout_msg = None
        self.create_timer(1.0, self._publish_keepout_rings)

        # Goal pins for RViz -- lets you see exactly where queued/dispatched
        # waypoints are, in the same 'map' frame as the AerialMapDisplay and
        # everything else in nav2_bringup's RViz, not just in this window.
        self._marker_pub = self.create_publisher(MarkerArray, '/mission_planner/goal_markers', 10)
        self._marker_msg = None
        self.create_timer(1.0, self._publish_goal_markers)

        # Satellite-image anchor for RViz's AerialMapDisplay -- published
        # once, the instant the origin lock fires, so the tile
        # imagery lands at the exact same map-frame point that
        # _map_to_mavros() uses for every click/goal in this window. A
        # live GPS topic would let AerialMapDisplay re-anchor itself using
        # its own internal projection math (not this file's WGS84
        # flat-earth formula), which would slowly drift apart from where
        # goals are actually being sent. A one-shot static TF + latched
        # fix removes that whole failure mode.
        self._satellite_anchor_broadcaster = StaticTransformBroadcaster(self)
        anchor_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 history=HistoryPolicy.KEEP_LAST, depth=1)
        self._satellite_anchor_pub = self.create_publisher(
            NavSatFix, '/mission_planner/satellite_anchor_fix', anchor_qos)
        self._satellite_anchor_published = False

    def publish_goal_markers_now(self, poses_mavros_xy: List[Tuple[float, float]], dispatched: bool):
        """poses_mavros_xy: list of (x, y) already in the MAVROS local
        frame -- the same values actually sent to Nav2's action, so the
        pin is guaranteed to match the real goal, not a re-derived copy."""
        arr = MarkerArray()
        # Clear first so removing a queued point actually removes its pin
        # instead of leaving a stale marker behind.
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)

        color = (0.0, 0.9, 0.46, 1.0) if dispatched else (1.0, 0.09, 0.27, 1.0)  # green vs red
        for i, (x, y) in enumerate(poses_mavros_xy):
            pin = Marker()
            pin.header.frame_id = 'map'
            pin.header.stamp = self.get_clock().now().to_msg()
            pin.ns = 'mission_planner_goals'
            pin.id = i + 1
            pin.type = Marker.CYLINDER
            pin.action = Marker.ADD
            pin.pose.position.x = float(x)
            pin.pose.position.y = float(y)
            pin.pose.position.z = 0.5
            pin.pose.orientation.w = 1.0
            pin.scale.x = 0.6
            pin.scale.y = 0.6
            pin.scale.z = 1.0
            pin.color.r, pin.color.g, pin.color.b, pin.color.a = color
            arr.markers.append(pin)

            label = Marker()
            label.header.frame_id = 'map'
            label.header.stamp = pin.header.stamp
            label.ns = 'mission_planner_goal_labels'
            label.id = i + 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = float(x)
            label.pose.position.y = float(y)
            label.pose.position.z = 1.6
            label.scale.z = 0.5
            label.color.r, label.color.g, label.color.b, label.color.a = (1.0, 1.0, 1.0, 1.0)
            label.text = str(i + 1)
            arr.markers.append(label)

        self._marker_msg = arr
        self._marker_pub.publish(arr)

    def _publish_goal_markers(self):
        # Re-publish at 1Hz too (not just on change) so a late-joining RViz
        # instance still sees the current pins without needing a fresh click.
        if self._marker_msg is not None:
            self._marker_pub.publish(self._marker_msg)

    def publish_satellite_anchor(self, origin_east: float, origin_north: float,
                                  map_center_lat: float, map_center_lon: float):
        """Called once, right when the origin lock fires. Publishes
        the exact map-frame offset of the satellite image's (0,0) point
        (map_center_lat/lon), using this file's own _map_to_mavros math
        (map_frame_xy = image_xy - origin), so AerialMapDisplay's tiles line
        up with the same frame goals are given in -- not a re-derived or
        independently-tracked estimate."""
        if self._satellite_anchor_published:
            return
        self._satellite_anchor_published = True

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'satellite_anchor'
        t.transform.translation.x = -origin_east
        t.transform.translation.y = -origin_north
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0
        self._satellite_anchor_broadcaster.sendTransform(t)

        fix = NavSatFix()
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.header.frame_id = 'satellite_anchor'
        fix.latitude = map_center_lat
        fix.longitude = map_center_lon
        fix.altitude = 0.0
        self._satellite_anchor_pub.publish(fix)

        self.get_logger().info(
            f"[SATELLITE ANCHOR PUBLISHED] map->satellite_anchor = "
            f"({-origin_east:+.2f}, {-origin_north:+.2f}) -- AerialMapDisplay "
            f"will now anchor to the same origin as goal dispatch.")

    def _on_gps(self, msg: NavSatFix):
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            return
        if msg.latitude == 0.0 and msg.longitude == 0.0:
            return
        self.rover_lat = msg.latitude
        self.rover_lon = msg.longitude
        self.gps_count += 1
        if self.on_new_data:
            self.on_new_data()

    def _on_odom(self, msg: Odometry):
        self.rover_local_x = msg.pose.pose.position.x
        self.rover_local_y = msg.pose.pose.position.y
        self.rover_yaw_deg = quaternion_to_yaw_deg(msg.pose.pose.orientation)
        self.odom_count += 1

        if not self.trail_local_x:
            self.trail_local_x.append(self.rover_local_x)
            self.trail_local_y.append(self.rover_local_y)
        else:
            last_x, last_y = self.trail_local_x[-1], self.trail_local_y[-1]
            if math.hypot(self.rover_local_x - last_x, self.rover_local_y - last_y) >= 0.05:
                self.trail_local_x.append(self.rover_local_x)
                self.trail_local_y.append(self.rover_local_y)

        if self.on_new_data:
            self.on_new_data()

    def _publish_keepout_rings(self):
        if self._keepout_msg is not None:
            self._keepout_msg.header.stamp = self.get_clock().now().to_msg()
            self._keepout_pub.publish(self._keepout_msg)

    def set_keepout_cloud(self, msg: PointCloud2):
        self._keepout_msg = msg

    def dispatch(self, poses_map_xy, done_callback, error_callback):
        self.get_logger().info('Waiting for /navigate_through_poses action server...')
        self._action_client.wait_for_server()
        goal_msg = NavigateThroughPoses.Goal()
        for (mx, my) in poses_map_xy:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = float(mx)
            pose.pose.position.y = float(my)
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            goal_msg.poses.append(pose)

        def on_goal_response(future):
            goal_handle = future.result()
            if not goal_handle.accepted:
                error_callback('Mission rejected by Nav2! Check if BT Navigator is running.')
                return
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(lambda f: done_callback(f.result().status))

        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(on_goal_response)


class MapWindow(QtWidgets.QMainWindow):
    def __init__(self, ros: RosBridge, satellite_bg: str,
                 map_center_lat: float, map_center_lon: float, radius_m: float):
        super().__init__()
        self.ros = ros
        self.map_center_lat = map_center_lat
        self.map_center_lon = map_center_lon
        self.radius_m = radius_m

        self.origin_east = None
        self.origin_north = None
        self.origin_lat = None
        self.origin_lon = None
        self.origin_locked = False
        self.origin_samples: List[Tuple[float, float]] = []
        self.rover_smooth_east = None
        self.rover_smooth_north = None

        self.queued_e: List[float] = []
        self.queued_n: List[float] = []
        self.mission_dispatched = False

        self.setWindowTitle("Mission Planner Live -- click to queue, 'g' to dispatch")
        self.setFocusPolicy(QtCore.Qt.StrongFocus)

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QVBoxLayout(central)
        layout.setContentsMargins(4, 4, 4, 4)

        self.hud_label = QtWidgets.QLabel("Waiting for GPS/odom...")
        self.hud_label.setStyleSheet(
            "background-color: #12161f; color: #ffffff; font-family: monospace; "
            "font-size: 11px; padding: 6px; border: 1px solid #00e5ff;")
        layout.addWidget(self.hud_label)

        self.plot_widget = pg.PlotWidget(background='#1a2130')
        layout.addWidget(self.plot_widget)

        self.cursor_label = QtWidgets.QLabel(
            "Hover to see MAVROS (X,Y) | Click to queue | 'g' dispatch | 'c' clear")
        self.cursor_label.setStyleSheet(
            "background-color: #12161f; color: #00e5ff; font-family: monospace; "
            "font-size: 11px; font-weight: bold; padding: 6px; border: 1px solid #ffd600;")
        self.cursor_label.setAlignment(QtCore.Qt.AlignCenter)
        layout.addWidget(self.cursor_label)

        vb = self.plot_widget.getPlotItem().getViewBox()
        vb.setAspectLocked(True)
        vb.setRange(xRange=(-radius_m * 1.05, radius_m * 1.05),
                    yRange=(-radius_m * 1.05, radius_m * 1.05))
        self.plot_widget.setLabel('bottom', 'East from Map Center (m)')
        self.plot_widget.setLabel('left', 'North from Map Center (m)')

        self._setup_static_map_items(satellite_bg)
        self._setup_dynamic_items()

        scene = self.plot_widget.getPlotItem().scene()
        scene.sigMouseClicked.connect(self._on_scene_clicked)
        scene.sigMouseMoved.connect(self._on_scene_moved)

        self.ros.on_new_data = self._on_ros_data
        self.resize(1000, 1000)

    # -------- static map layer (drawn once) --------

    def _setup_static_map_items(self, satellite_bg):
        if os.path.exists(satellite_bg):
            try:
                img = np.array(Image.open(satellite_bg).convert('RGB'))
                img = np.flipud(img)  # image row 0 = top; plot y increases upward
                img_item = pg.ImageItem(image=np.transpose(img, (1, 0, 2)))
                r = self.radius_m
                img_item.setRect(QtCore.QRectF(-r, -r, 2 * r, 2 * r))
                img_item.setOpacity(0.92)
                img_item.setZValue(-10)
                self.plot_widget.addItem(img_item)
            except Exception as e:
                self.ros.get_logger().warn(f"Could not load satellite image: {e}")

        for r in [250, 500, 750, int(self.radius_m)]:
            x, y = circle_xy(0, 0, r)
            pen = pg.mkPen('#00e5ff', width=2.8 if r >= self.radius_m else 1.2,
                            style=(QtCore.Qt.SolidLine if r >= self.radius_m else QtCore.Qt.DashLine))
            ring = pg.PlotCurveItem(x, y, pen=pen)
            ring.setZValue(-5)
            self.plot_widget.addItem(ring)

        center = pg.ScatterPlotItem([0], [0], pen=pg.mkPen('#00e5ff', width=2),
                                     brush=None, symbol='+', size=18)
        self.plot_widget.addItem(center)

    # -------- dynamic items (updated live) --------

    def _setup_dynamic_items(self):
        self.origin_marker = pg.ScatterPlotItem([], [], pen=pg.mkPen('#00e5ff', width=3),
                                                 brush=None, symbol='+', size=22)
        self.plot_widget.addItem(self.origin_marker)

        self.trail_curve = pg.PlotCurveItem([], [], pen=pg.mkPen('#00e676', width=2.6))
        self.plot_widget.addItem(self.trail_curve)

        self.raw_gps_curve = pg.PlotCurveItem([], [], pen=pg.mkPen('#e040fb', width=1.8,
                                               style=QtCore.Qt.DotLine))
        self.plot_widget.addItem(self.raw_gps_curve)

        self.rover_dot = pg.ScatterPlotItem([], [], pen=pg.mkPen('#ffffff', width=1.8),
                                             brush=pg.mkBrush('#00e676'), size=14)
        self.plot_widget.addItem(self.rover_dot)

        self.rover_arrow = pg.ArrowItem(angle=0, brush='#00e676', pen=pg.mkPen('#ffffff'))
        self.plot_widget.addItem(self.rover_arrow)
        self.rover_arrow.setVisible(False)

        self.queue_dots = pg.ScatterPlotItem([], [], pen=pg.mkPen('#ffffff', width=2),
                                              brush=pg.mkBrush('#ff1744'), symbol='x', size=16)
        self.plot_widget.addItem(self.queue_dots)
        self.queue_labels: List[pg.TextItem] = []

    # -------- coordinate helpers --------

    def _map_to_mavros(self, e: float, n: float) -> Tuple[float, float]:
        if self.origin_east is None:
            return e, n
        return e - self.origin_east, n - self.origin_north

    # -------- ROS data callback (fires on every GPS/odom message) --------

    def _on_ros_data(self):
        # Cheap bookkeeping only -- actual widget redraw happens on the GUI
        # timer, not here, so ROS message rate doesn't dictate redraw rate.
        if self.ros.rover_lat is None or self.ros.rover_local_x is None:
            return
        e, n = latlon_to_meter_offset(self.map_center_lat, self.map_center_lon,
                                       self.ros.rover_lat, self.ros.rover_lon)
        self.ros.rover_gps_east, self.ros.rover_gps_north = e, n

        if not self.ros.raw_gps_e:
            self.ros.raw_gps_e.append(e)
            self.ros.raw_gps_n.append(n)
        elif math.hypot(e - self.ros.raw_gps_e[-1], n - self.ros.raw_gps_n[-1]) >= 0.3:
            self.ros.raw_gps_e.append(e)
            self.ros.raw_gps_n.append(n)

        if not self.origin_locked:
            # Same lock logic as find_location.py/casualty_dispatcher.py:
            # running average of the MAVROS-origin estimate, locked after
            # 10 samples (~1s) or after 3+ samples once the rover has
            # visibly moved >0.5m. No settling/dispersion gate -- run this
            # script only once you've judged the GPS fix/lat-lon to already
            # be stable.
            e_orig = e - self.ros.rover_local_x
            n_orig = n - self.ros.rover_local_y
            self.origin_samples.append((e_orig, n_orig))
            self.origin_east = float(np.mean([s[0] for s in self.origin_samples]))
            self.origin_north = float(np.mean([s[1] for s in self.origin_samples]))
            self.origin_lat, self.origin_lon = meter_offset_to_latlon(
                self.map_center_lat, self.map_center_lon, self.origin_east, self.origin_north)

            dist_moved = math.hypot(self.ros.rover_local_x, self.ros.rover_local_y)
            if len(self.origin_samples) >= 10 or (len(self.origin_samples) >= 3 and dist_moved > 0.5):
                self.origin_locked = True
                self.ros.get_logger().info(
                    f"[ORIGIN LOCKED] Lat={self.origin_lat:.7f} Lon={self.origin_lon:.7f} "
                    f"East={self.origin_east:+.2f}m North={self.origin_north:+.2f}m "
                    f"(averaged across {len(self.origin_samples)} samples)")
                self.ros.publish_satellite_anchor(
                    self.origin_east, self.origin_north,
                    self.map_center_lat, self.map_center_lon)

        if self.origin_east is not None:
            self.rover_smooth_east = self.origin_east + self.ros.rover_local_x
            self.rover_smooth_north = self.origin_north + self.ros.rover_local_y

    # -------- periodic GUI refresh (decoupled from ROS message rate) --------

    def refresh_gui(self):
        if self.rover_smooth_east is None:
            return

        re_, rn_ = self.rover_smooth_east, self.rover_smooth_north
        if self.ros.trail_local_x:
            trail_e = np.array(self.ros.trail_local_x) + self.origin_east
            trail_n = np.array(self.ros.trail_local_y) + self.origin_north
            self.trail_curve.setData(trail_e, trail_n)
        if self.ros.raw_gps_e:
            self.raw_gps_curve.setData(self.ros.raw_gps_e, self.ros.raw_gps_n)

        self.rover_dot.setData([re_], [rn_])
        self.rover_arrow.setVisible(True)
        yaw = self.ros.rover_yaw_deg
        # ArrowItem's angle=0 points left by convention; correct so it
        # points along ENU yaw (0=East, CCW positive).
        self.rover_arrow.setStyle(angle=180 - yaw)
        self.rover_arrow.setPos(re_, rn_)

        if self.origin_east is not None:
            self.origin_marker.setData([self.origin_east], [self.origin_north])

        origin_str = ("LOCKED" if self.origin_locked
                      else f"LOCKING ({len(self.origin_samples)}/10)...")
        orig_coords = f"Lat={self.origin_lat:.7f}, Lon={self.origin_lon:.7f}" if self.origin_lat else "n/a"
        queue_state = "DISPATCHED" if self.mission_dispatched else f"{len(self.queued_e)} queued, not sent"

        self.hud_label.setText(
            f"MISSION PLANNER LIVE   |   Origin: {origin_str}   |   Origin Geodetic: {orig_coords}\n"
            f"Rover: X={self.ros.rover_local_x:+7.2f}m Y={self.ros.rover_local_y:+7.2f}m   |   "
            f"Mission queue: {queue_state}   |   Messages: GPS={self.ros.gps_count} Odom={self.ros.odom_count}"
        )

    # -------- mouse handling --------

    def _on_scene_moved(self, scene_pos):
        vb = self.plot_widget.getPlotItem().getViewBox()
        pt = vb.mapSceneToView(scene_pos)
        e, n = pt.x(), pt.y()
        mx, my = self._map_to_mavros(e, n)
        lat, lon = meter_offset_to_latlon(self.map_center_lat, self.map_center_lon, e, n)
        self.cursor_label.setText(
            f"HOVER -> MAVROS: X={mx:+7.2f}m Y={my:+7.2f}m | GPS: Lat={lat:.7f} Lon={lon:.7f} | "
            f"Click to queue | 'g' dispatch ({len(self.queued_e)} queued) | 'c' clear")

    def _on_scene_clicked(self, event):
        if event.button() != QtCore.Qt.LeftButton:
            return
        vb = self.plot_widget.getPlotItem().getViewBox()
        pt = vb.mapSceneToView(event.scenePos())
        e, n = pt.x(), pt.y()
        self.queued_e.append(e)
        self.queued_n.append(n)
        self.mission_dispatched = False
        mx, my = self._map_to_mavros(e, n)
        self.ros.get_logger().info(f"Queued waypoint #{len(self.queued_e)}: "
                                    f"MAVROS X={mx:+.3f} Y={my:+.3f} (map E={e:+.2f} N={n:+.2f})")
        self._redraw_queue()

    def _redraw_queue(self):
        for lbl in self.queue_labels:
            self.plot_widget.removeItem(lbl)
        self.queue_labels = []
        if self.queued_e:
            self.queue_dots.setData(self.queued_e, self.queued_n)
            for i, (e, n) in enumerate(zip(self.queued_e, self.queued_n), start=1):
                lbl = pg.TextItem(str(i), color='#ffffff', anchor=(0.5, 0.5))
                lbl.setPos(e + 6.0, n + 6.0)
                self.plot_widget.addItem(lbl)
                self.queue_labels.append(lbl)
        else:
            self.queue_dots.setData([], [])
        # Mirror the same queue into RViz as pins, in the map frame, using
        # the exact MAVROS (x,y) values -- not re-derived, so the pin can
        # never drift from what would actually be sent to Nav2.
        poses_mavros = [self._map_to_mavros(e, n) for (e, n) in zip(self.queued_e, self.queued_n)]
        self.ros.publish_goal_markers_now(poses_mavros, dispatched=False)

    # -------- mission dispatch --------

    def dispatch_queued_mission(self):
        if not self.queued_e:
            self.ros.get_logger().warn("No queued waypoints -- click on the map first.")
            return
        if self.origin_east is None:
            self.ros.get_logger().error("Origin not locked yet -- cannot dispatch.")
            return

        coords_map = list(zip(self.queued_e, self.queued_n))
        # self.ros.set_keepout_cloud(self._generate_keepout_cloud(coords_map))

        poses_mavros = [self._map_to_mavros(e, n) for (e, n) in coords_map]
        self.mission_dispatched = True
        self.ros.publish_goal_markers_now(poses_mavros, dispatched=True)
        self.ros.get_logger().info(f'Dispatching mission with {len(poses_mavros)} queued waypoints...')

        def on_done(status):
            if status == 4:
                self.ros.get_logger().info('Mission complete!')
            else:
                self.ros.get_logger().warn(f'Mission ended with status code: {status}')

        def on_error(msg):
            self.ros.get_logger().error(msg)

        self.ros.dispatch(poses_mavros, on_done, on_error)

    def _generate_keepout_cloud(self, casualty_locations_map_xy):
        points = []
        radius, resolution = 2.0, 0.05
        for (e, n) in casualty_locations_map_xy:
            mx, my = self._map_to_mavros(e, n)
            angle = 0.0
            while angle < 2 * math.pi:
                points.append((mx + radius * math.cos(angle), my + radius * math.sin(angle), 0.5))
                angle += resolution

        msg = PointCloud2()
        msg.header.frame_id = 'map'
        msg.height = 1
        msg.width = len(points)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        buffer = bytearray()
        for p in points:
            buffer.extend(struct.pack('fff', p[0], p[1], p[2]))
        msg.data = bytes(buffer)
        return msg

    # -------- keyboard --------

    def keyPressEvent(self, event):
        key = event.text().lower()
        vb = self.plot_widget.getPlotItem().getViewBox()
        if key == 'c':
            self.queued_e.clear()
            self.queued_n.clear()
            self.mission_dispatched = False
            self._redraw_queue()
            self.ros.get_logger().info("Queue cleared.")
        elif key == 'g':
            self.dispatch_queued_mission()
        elif key == 'f':
            m = self.radius_m * 1.05
            vb.setRange(xRange=(-m, m), yRange=(-m, m))
        elif key == 'r':
            if self.rover_smooth_east is not None:
                span = 120.0
                vb.setRange(xRange=(self.rover_smooth_east - span, self.rover_smooth_east + span),
                            yRange=(self.rover_smooth_north - span, self.rover_smooth_north + span))
        elif key == 'o':
            if self.origin_east is not None:
                span = 120.0
                vb.setRange(xRange=(self.origin_east - span, self.origin_east + span),
                            yRange=(self.origin_north - span, self.origin_north + span))
        else:
            super().keyPressEvent(event)


def main():
    parser = argparse.ArgumentParser(description="Live click-to-queue, press-g-to-dispatch mission planner.")
    parser.add_argument('--map', default=os.path.expanduser('~/map_satellite_1km.png'))
    parser.add_argument('--map-lat', type=float, default=28.7484727)
    parser.add_argument('--map-lon', type=float, default=77.1171744)
    parser.add_argument('--radius', type=float, default=1000.0)
    cleaned_args = [a for a in sys.argv[1:] if not a.startswith('--ros-args')]
    args, _ = parser.parse_known_args(cleaned_args)

    rclpy.init()
    ros_node = RosBridge()

    app = QtWidgets.QApplication(sys.argv)
    win = MapWindow(ros_node, args.map, args.map_lat, args.map_lon, args.radius)
    win.show()

    # Two independent timers: one pumps rclpy (fast, cheap), the other
    # redraws the GUI at a fixed rate decoupled from ROS message rate --
    # this is the core fix for the matplotlib version's lag, which
    # re-rendered on every single incoming message instead of on a clock.
    def pump_ros():
        # Guard against firing after shutdown has already been requested
        # (e.g. a forced external kill mid-event-loop) -- PyQt5 doesn't
        # stop a QTimer just because a slot raised, so without this check
        # a single bad tick can repeat every 5ms and spam the same
        # "context is not valid" error indefinitely.
        if rclpy.ok():
            rclpy.spin_once(ros_node, timeout_sec=0)

    ros_timer = QtCore.QTimer()
    ros_timer.timeout.connect(pump_ros)
    ros_timer.start(5)  # ~200Hz pump, cheap since spin_once returns immediately if idle

    gui_timer = QtCore.QTimer()
    gui_timer.timeout.connect(win.refresh_gui)
    gui_timer.start(50)  # 20Hz redraw, matches the matplotlib version's target rate

    def shutdown():
        # Stop both timers FIRST, before touching rclpy at all, so nothing
        # can fire into a context that's about to become invalid.
        ros_timer.stop()
        gui_timer.stop()
        if rclpy.ok():
            ros_node.destroy_node()
            rclpy.shutdown()

    app.aboutToQuit.connect(shutdown)

    try:
        app.exec_()
    finally:
        # Belt-and-suspenders: aboutToQuit covers normal window-close exits;
        # this covers anything that skips that signal (e.g. an exception
        # propagating out of exec_ itself).
        shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
