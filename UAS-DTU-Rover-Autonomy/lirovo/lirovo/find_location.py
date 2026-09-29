#!/usr/bin/env python3
"""
================================================================================
ROS 2 Tool: find_location.py
================================================================================
Displays the 1km satellite map (centered at DTU campus with concentric distance rings),
subscribes to MAVROS raw GPS fix and local position odometry, and finds where MAVROS
local position was (0, 0).

When you hover your mouse anywhere on the map, it shows you the exact (X, Y) coordinates
in the MAVROS local frame, geodetic (Lat, Lon), and distance/bearing from the rover.
Clicking on any location drops a target pin and prints the exact (X, Y) coordinates
to the terminal in copy-paste format for Nav2 / autonomous navigation.

Keyboard Shortcuts:
  - Left Click: Place / move target waypoint
  - 'c': Clear target waypoint
  - 'f': Full 1km map view
  - 'r': Zoom in on rover
  - 'o': Zoom in on MAVROS (0, 0) origin
================================================================================
"""

import os
import sys
import math
import argparse
from typing import Optional, List, Tuple

import numpy as np
from PIL import Image
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import matplotlib.lines as mlines

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

# WGS-84 Ellipsoid Constants
WGS84_A = 6378137.0            # Semi-major axis in meters
WGS84_F = 1.0 / 298.257223563   # Flattening
WGS84_E2 = 2.0 * WGS84_F - WGS84_F * WGS84_F


def latlon_to_meter_offset(lat0: float, lon0: float, lat: float, lon: float) -> Tuple[float, float]:
    """Converts (lat, lon) to metric ENU offsets (East, North) from datum (lat0, lon0)."""
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
    """Converts metric ENU offsets (East, North) relative to (lat0, lon0) into (lat, lon)."""
    lat0_rad = math.radians(lat0)
    sin_lat0 = math.sin(lat0_rad)
    denom = math.sqrt(1.0 - WGS84_E2 * sin_lat0 * sin_lat0)
    rn = WGS84_A / denom
    rm = WGS84_A * (1.0 - WGS84_E2) / (denom * denom * denom)

    dlat = north_m / rm
    dlon = east_m / (rn * math.cos(lat0_rad))
    return math.degrees(lat0_rad + dlat), math.degrees(math.radians(lon0) + dlon)


def quaternion_to_yaw_deg(q) -> float:
    """Computes yaw angle in degrees from quaternion (ENU standard)."""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.degrees(math.atan2(siny, cosy))


def compass_bearing_deg(yaw_enu_deg: float) -> float:
    """Converts ENU yaw (0° = East, 90° = North) to standard compass bearing (0° = North, 90° = East)."""
    return (90.0 - yaw_enu_deg) % 360.0


def bearing_to_cardinal(deg: float) -> str:
    """Converts compass bearing degrees to cardinal string (e.g. N, NE, ESE)."""
    cardinals = ["N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
                 "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"]
    idx = int((deg + 11.25) / 22.5) % 16
    return cardinals[idx]


class FindLocationNode(Node):
    def __init__(self, satellite_bg=os.path.expanduser("~/map_satellite_1km.png"),
                 map_center_lat=28.7484727, map_center_lon=77.1171744, radius_m=1000.0):
        super().__init__('find_location_node')

        self.satellite_bg = satellite_bg
        self.map_center_lat = map_center_lat
        self.map_center_lon = map_center_lon
        self.radius_m = radius_m

        # Live GPS state (raw)
        self.rover_lat: Optional[float] = None
        self.rover_lon: Optional[float] = None
        self.rover_alt: Optional[float] = None
        self.rover_gps_east: Optional[float] = None
        self.rover_gps_north: Optional[float] = None
        self.gps_count = 0

        # Live MAVROS Local Odometry state (filtered, smooth)
        self.rover_local_x: Optional[float] = None
        self.rover_local_y: Optional[float] = None
        self.rover_local_z: Optional[float] = None
        self.rover_yaw_deg: float = 0.0
        self.odom_count = 0

        # Smooth rover position in map coordinates (East, North)
        self.rover_smooth_east: Optional[float] = None
        self.rover_smooth_north: Optional[float] = None

        # MAVROS Origin in map coordinates (East, North from Map Center where Local X=0, Y=0)
        self.origin_east: Optional[float] = None
        self.origin_north: Optional[float] = None
        self.origin_lat: Optional[float] = None
        self.origin_lon: Optional[float] = None
        self.origin_locked = False
        self.origin_samples: List[Tuple[float, float]] = []

        # Rover trail in local coordinates (MAVROS Local Odom: smooth, zero GPS jitter)
        self.trail_local_x: List[float] = []
        self.trail_local_y: List[float] = []

        # Raw GPS trail in map coordinates (for reference/debugging, dotted magenta)
        self.raw_gps_e: List[float] = []
        self.raw_gps_n: List[float] = []

        # Interactive Target Pin (in map coordinates)
        self.target_e: Optional[float] = None
        self.target_n: Optional[float] = None

        # Hover state
        self.hover_e: Optional[float] = None
        self.hover_n: Optional[float] = None

        # Load satellite image
        self.sat_img = None
        if os.path.exists(self.satellite_bg):
            try:
                self.sat_img = Image.open(self.satellite_bg)
            except Exception as e:
                self.get_logger().warn(f"Could not load satellite image: {e}")

        # QoS Profiles
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribers
        self.sub_gps = self.create_subscription(NavSatFix, '/mavros/global_position/raw/fix', self.gps_callback, qos)
        self.sub_gps_alt1 = self.create_subscription(NavSatFix, '/mavros/global_position/raw_fix', self.gps_callback, qos)
        self.sub_gps_alt2 = self.create_subscription(NavSatFix, '/raw_fix', self.gps_callback, qos)
        self.sub_gps_alt3 = self.create_subscription(NavSatFix, '/mavros/global_position/global', self.gps_callback, qos)

        self.sub_odom = self.create_subscription(Odometry, '/mavros/local_position/odom', self.odom_callback, qos)
        self.sub_pose = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pose_callback, qos)

        self.get_logger().info("=" * 70)
        self.get_logger().info("Find Location & Target Picker Node Online")
        self.get_logger().info(f"  Map Center:  Lat={self.map_center_lat:.7f}, Lon={self.map_center_lon:.7f}")
        self.get_logger().info(f"  Map Radius:  {self.radius_m:.0f} meters")
        self.get_logger().info("  Waiting for GPS fix and MAVROS local position to lock (0,0) origin...")
        self.get_logger().info("=" * 70)

        # Setup GUI
        self.setup_gui()

    def gps_callback(self, msg: NavSatFix):
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            return
        if msg.latitude == 0.0 and msg.longitude == 0.0:
            return

        self.rover_lat = msg.latitude
        self.rover_lon = msg.longitude
        self.rover_alt = msg.altitude

        # Metric offset from map center
        self.rover_gps_east, self.rover_gps_north = latlon_to_meter_offset(
            self.map_center_lat, self.map_center_lon, self.rover_lat, self.rover_lon
        )
        self.gps_count += 1

        # Buffer raw GPS points for optional dotted magenta line (spatial downsampling >= 0.3m)
        if not self.raw_gps_e:
            self.raw_gps_e.append(self.rover_gps_east)
            self.raw_gps_n.append(self.rover_gps_north)
        else:
            if math.hypot(self.rover_gps_east - self.raw_gps_e[-1],
                          self.rover_gps_north - self.raw_gps_n[-1]) >= 0.3:
                self.raw_gps_e.append(self.rover_gps_east)
                self.raw_gps_n.append(self.rover_gps_north)

        self.update_origin_estimate()

    def odom_callback(self, msg: Odometry):
        self.rover_local_x = msg.pose.pose.position.x
        self.rover_local_y = msg.pose.pose.position.y
        self.rover_local_z = msg.pose.pose.position.z
        self.rover_yaw_deg = quaternion_to_yaw_deg(msg.pose.pose.orientation)
        self.odom_count += 1

        # Record smooth local trail (downsampled to >= 0.05m = 5cm threshold)
        if not self.trail_local_x:
            self.trail_local_x.append(self.rover_local_x)
            self.trail_local_y.append(self.rover_local_y)
        else:
            last_x = self.trail_local_x[-1]
            last_y = self.trail_local_y[-1]
            if math.hypot(self.rover_local_x - last_x, self.rover_local_y - last_y) >= 0.05:
                self.trail_local_x.append(self.rover_local_x)
                self.trail_local_y.append(self.rover_local_y)

        # Update smooth rover position in map coordinates
        if self.origin_east is not None:
            self.rover_smooth_east = self.origin_east + self.rover_local_x
            self.rover_smooth_north = self.origin_north + self.rover_local_y
        else:
            self.update_origin_estimate()

    def pose_callback(self, msg: PoseStamped):
        # Fallback only if /mavros/local_position/odom has not received any messages
        if self.odom_count == 0:
            self.rover_local_x = msg.pose.position.x
            self.rover_local_y = msg.pose.position.y
            self.rover_local_z = msg.pose.position.z
            self.rover_yaw_deg = quaternion_to_yaw_deg(msg.pose.orientation)

            if not self.trail_local_x:
                self.trail_local_x.append(self.rover_local_x)
                self.trail_local_y.append(self.rover_local_y)
            else:
                last_x = self.trail_local_x[-1]
                last_y = self.trail_local_y[-1]
                if math.hypot(self.rover_local_x - last_x, self.rover_local_y - last_y) >= 0.05:
                    self.trail_local_x.append(self.rover_local_x)
                    self.trail_local_y.append(self.rover_local_y)

            if self.origin_east is not None:
                self.rover_smooth_east = self.origin_east + self.rover_local_x
                self.rover_smooth_north = self.origin_north + self.rover_local_y
            else:
                self.update_origin_estimate()

    def update_origin_estimate(self):
        """Calculates and locks the MAVROS local (0, 0) origin location relative to map center."""
        if self.rover_gps_east is None or self.rover_local_x is None:
            return

        if not self.origin_locked:
            # MAVROS (0, 0) origin estimate in map coordinates
            e_orig = self.rover_gps_east - self.rover_local_x
            n_orig = self.rover_gps_north - self.rover_local_y
            self.origin_samples.append((e_orig, n_orig))

            # Running average of origin to eliminate raw GPS multipath/jitter
            self.origin_east = float(np.mean([s[0] for s in self.origin_samples]))
            self.origin_north = float(np.mean([s[1] for s in self.origin_samples]))
            self.origin_lat, self.origin_lon = meter_offset_to_latlon(
                self.map_center_lat, self.map_center_lon, self.origin_east, self.origin_north
            )

            # Update origin marker on plot
            self.origin_marker.set_offsets([[self.origin_east, self.origin_north]])
            self.origin_circle.set_offsets([[self.origin_east, self.origin_north]])
            self.origin_text.set_position((self.origin_east + 4.0, self.origin_north + 4.0))
            self.origin_text.set_visible(True)

            # Lock after 10 samples (~1 second) or if rover moved > 0.5m
            dist_moved = math.hypot(self.rover_local_x, self.rover_local_y)
            if len(self.origin_samples) >= 10 or (len(self.origin_samples) >= 3 and dist_moved > 0.5):
                self.origin_locked = True
                self.get_logger().info("=" * 70)
                self.get_logger().info("[ORIGIN LOCKED] MAVROS Local Position (0, 0) Located on Map!")
                self.get_logger().info(f"  Geodetic Origin: Lat={self.origin_lat:.7f}, Lon={self.origin_lon:.7f}")
                self.get_logger().info(f"  Map Coordinates: East={self.origin_east:+7.2f}m, North={self.origin_north:+7.2f}m")
                self.get_logger().info(f"  Rover Pos Now:   X={self.rover_local_x:+7.2f}m, Y={self.rover_local_y:+7.2f}m")
                self.get_logger().info(f"  Averaged across {len(self.origin_samples)} GPS/Odom sync samples.")
                self.get_logger().info("=" * 70)

        # Update smooth rover position
        if self.origin_east is not None:
            self.rover_smooth_east = self.origin_east + self.rover_local_x
            self.rover_smooth_north = self.origin_north + self.rover_local_y

    def setup_gui(self):
        """Initializes the live Matplotlib GUI with the complete 1km satellite map."""
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(11, 11), facecolor='#12161f')
        if hasattr(self.fig.canvas, 'manager') and self.fig.canvas.manager is not None:
            self.fig.canvas.manager.set_window_title("MAVROS Local Coordinates & Target Finder (1km Satellite View)")

        self.ax.set_facecolor('#1a2130')

        center_x = 0.0
        center_y = 0.0

        # Render 1km satellite map immediately (fills entire plot area)
        if self.sat_img is not None:
            extent = [center_x - self.radius_m, center_x + self.radius_m,
                      center_y - self.radius_m, center_y + self.radius_m]
            self.sat_artist = self.ax.imshow(self.sat_img, extent=extent, origin='upper', alpha=0.92, zorder=1)

        # Concentric Distance Rings on the Map (identical to plot_trajectories_map.py)
        ring_radii = [250, 500, 750, int(self.radius_m)]
        for r in ring_radii:
            circle = Circle((center_x, center_y), r, color='#00e5ff', fill=False,
                            linestyle='--' if r < self.radius_m else '-',
                            linewidth=1.2 if r < self.radius_m else 2.8, alpha=0.75, zorder=2)
            self.ax.add_patch(circle)
            self.ax.text(center_x + r * 0.707 + 5, center_y + r * 0.707 + 5, f"{r}m",
                         color='#ffffff', fontsize=9, fontweight='bold',
                         bbox=dict(boxstyle='round,pad=0.2', facecolor='#000000', alpha=0.65), zorder=3)

        # Map Center crosshair
        self.ax.scatter([center_x], [center_y], color='#00e5ff', s=80, marker='+', linewidths=2.2, zorder=4,
                        label='Map Center (DTU)')

        # MAVROS Origin (0, 0) marker (placed dynamically on lock)
        self.origin_marker = self.ax.scatter([], [], color='#00e5ff', s=140, marker='+', linewidths=2.8, zorder=10,
                                             label='MAVROS Origin (0, 0)')
        self.origin_circle = self.ax.scatter([], [], color='#00e5ff', s=260, facecolors='none', edgecolors='#00e5ff',
                                             linewidths=1.8, zorder=9)
        self.origin_text = self.ax.text(0, 0, "MAVROS (0,0)", color='#00e5ff', fontsize=9.5, fontweight='bold',
                                        bbox=dict(boxstyle='round,pad=0.2', facecolor='#12161f', edgecolor='#00e5ff', alpha=0.85),
                                        visible=False, zorder=11)

        # Rover Trail (Smooth MAVROS Local Odometry - solid neon green like plot_trajectories_map.py)
        self.line_trail, = self.ax.plot([], [], color='#00e676', linewidth=2.6, linestyle='-',
                                        label='Rover Trajectory (MAVROS Odom)', zorder=7)

        # Raw GPS Fixes (Dotted Magenta - identical to plot_trajectories_map.py)
        self.line_raw_gps, = self.ax.plot([], [], color='#e040fb', linewidth=1.8, linestyle=':',
                                          label='Raw GPS Fixes (/mavros/.../raw/fix)', zorder=6, alpha=0.75)

        # Rover Position Dot & Heading Arrow
        self.rover_dot, = self.ax.plot([], [], marker='o', markersize=9, color='#00e676',
                                       markeredgecolor='#ffffff', markeredgewidth=1.8, zorder=12, label='Rover Current Pos')
        self.rover_arrow = self.ax.quiver([0], [0], [0], [0], color='#00e676', angles='xy', scale_units='xy',
                                          scale=1.0, width=0.008, zorder=13)

        # Target Waypoint Pin & Line
        self.target_dot, = self.ax.plot([], [], marker='X', markersize=14, color='#ff1744',
                                        markeredgecolor='#ffffff', markeredgewidth=2.0, zorder=15, label='Target Waypoint')
        self.target_line, = self.ax.plot([], [], color='#ff1744', linestyle=':', linewidth=2.0, zorder=14)
        self.target_callout = self.ax.text(0, 0, "", color='#ffffff', fontsize=10, fontweight='bold',
                                           bbox=dict(boxstyle='round,pad=0.3', facecolor='#b71c1c', edgecolor='#ffffff', alpha=0.9),
                                           visible=False, zorder=16)

        # Compass North Arrow in top-left
        self.ax.annotate('N', xy=(0.06, 0.95), xytext=(0.06, 0.88), xycoords='axes fraction',
                         arrowprops=dict(facecolor='#00e676', edgecolor='#ffffff', width=2.5, headwidth=9),
                         color='#00e676', fontweight='bold', ha='center', va='bottom', fontsize=13,
                         bbox=dict(boxstyle='circle,pad=0.25', facecolor='#12161f', edgecolor='#00e5ff', alpha=0.85), zorder=20)

        # Top-Right HUD Info Box
        self.hud_text = self.ax.text(
            0.98, 0.97, "Waiting for GPS Fix & MAVROS Local Position...",
            transform=self.ax.transAxes, verticalalignment='top', horizontalalignment='right',
            color='#ffffff', fontsize=9.5, family='monospace',
            bbox=dict(boxstyle='round,pad=0.5', facecolor='#12161f', edgecolor='#00e5ff', alpha=0.92),
            zorder=20
        )

        # Bottom Live Cursor Hover Box
        self.cursor_hud = self.ax.text(
            0.50, 0.03, "Hover over any location to see its MAVROS (X, Y) coordinates | Click to select Target",
            transform=self.ax.transAxes, verticalalignment='bottom', horizontalalignment='center',
            color='#00e5ff', fontsize=10.0, family='monospace', fontweight='bold',
            bbox=dict(boxstyle='round,pad=0.45', facecolor='#12161f', edgecolor='#ffd600', alpha=0.95),
            zorder=20
        )

        # Axes setup: Map fills entire window exactly like plot_trajectories_map.py
        margin = self.radius_m * 1.05
        self.ax.set_xlim(center_x - margin, center_x + margin)
        self.ax.set_ylim(center_y - margin, center_y + margin)
        self.ax.set_aspect('equal', adjustable='box')

        self.ax.set_xlabel('East from Map Center (meters)', color='#eceff1', fontsize=11, fontweight='bold')
        self.ax.set_ylabel('North from Map Center (meters)', color='#eceff1', fontsize=11, fontweight='bold')
        self.ax.tick_params(colors='#eceff1', which='both')

        self.ax.legend(loc='lower right', facecolor='#12161f', edgecolor='#00e5ff', labelcolor='#ffffff', fontsize=9.5)

        # Connect Matplotlib events
        self.fig.canvas.mpl_connect('motion_notify_event', self.on_mouse_move)
        self.fig.canvas.mpl_connect('button_press_event', self.on_mouse_click)
        self.fig.canvas.mpl_connect('key_press_event', self.on_key_press)

        # Override default toolbar coordinate display
        self.ax.format_coord = self.format_coordinate

        plt.tight_layout()

    def map_to_mavros(self, e: float, n: float) -> Tuple[float, float]:
        """Converts map coordinates (East, North) into MAVROS local coordinates (X, Y)."""
        if self.origin_east is None or self.origin_north is None:
            # If origin not locked yet, return offset from map center
            return e, n
        return e - self.origin_east, n - self.origin_north

    def format_coordinate(self, e: float, n: float) -> str:
        """Custom toolbar string displayed when cursor hovers over map."""
        mx, my = self.map_to_mavros(e, n)
        lat, lon = meter_offset_to_latlon(self.map_center_lat, self.map_center_lon, e, n)
        if self.rover_smooth_east is not None and self.rover_smooth_north is not None:
            dist = math.hypot(e - self.rover_smooth_east, n - self.rover_smooth_north)
            return f"MAVROS: (X={mx:+.2f}m, Y={my:+.2f}m) | GPS: (Lat={lat:.7f}, Lon={lon:.7f}) | Dist: {dist:.1f}m"
        return f"MAVROS: (X={mx:+.2f}m, Y={my:+.2f}m) | GPS: (Lat={lat:.7f}, Lon={lon:.7f})"

    def on_mouse_move(self, event):
        """Triggered on mouse move: updates live hover box with MAVROS (X, Y) coordinates."""
        if event.inaxes != self.ax or event.xdata is None or event.ydata is None:
            return

        he = float(event.xdata)
        hn = float(event.ydata)
        self.hover_e = he
        self.hover_n = hn

        # Convert to MAVROS Local (X, Y)
        mx, my = self.map_to_mavros(he, hn)
        lat, lon = meter_offset_to_latlon(self.map_center_lat, self.map_center_lon, he, hn)

        if self.rover_smooth_east is not None and self.rover_smooth_north is not None:
            de = he - self.rover_smooth_east
            dn = hn - self.rover_smooth_north
            dist = math.hypot(de, dn)
            enu_deg = math.degrees(math.atan2(dn, de))
            comp_deg = compass_bearing_deg(enu_deg)
            card = bearing_to_cardinal(comp_deg)

            cursor_str = (
                f"HOVER LOCATION -> MAVROS: X = {mx:+7.2f}m, Y = {my:+7.2f}m\n"
                f"GPS Geodetic:   Lat = {lat:.7f}°, Lon = {lon:.7f}°\n"
                f"From Rover:     Distance = {dist:6.1f}m | Heading = {comp_deg:5.1f}° ({card})"
            )
        else:
            cursor_str = (
                f"HOVER LOCATION -> MAVROS: X = {mx:+7.2f}m, Y = {my:+7.2f}m\n"
                f"GPS Geodetic:   Lat = {lat:.7f}°, Lon = {lon:.7f}°"
            )

        self.cursor_hud.set_text(cursor_str)
        self.fig.canvas.draw_idle()

    def on_mouse_click(self, event):
        """Triggered on click: drops target pin and prints MAVROS (X, Y) coordinates."""
        if event.inaxes != self.ax or event.xdata is None or event.ydata is None:
            return

        if event.button == 1:
            self.target_e = float(event.xdata)
            self.target_n = float(event.ydata)

            mx, my = self.map_to_mavros(self.target_e, self.target_n)
            lat, lon = meter_offset_to_latlon(self.map_center_lat, self.map_center_lon, self.target_e, self.target_n)

            dist_str = ""
            bearing_str = ""
            if self.rover_smooth_east is not None and self.rover_smooth_north is not None:
                de = self.target_e - self.rover_smooth_east
                dn = self.target_n - self.rover_smooth_north
                dist = math.hypot(de, dn)
                comp_deg = compass_bearing_deg(math.degrees(math.atan2(dn, de)))
                card = bearing_to_cardinal(comp_deg)
                dist_str = f"{dist:.2f} meters"
                bearing_str = f"{comp_deg:.1f}° ({card})"

            print("\n" + "=" * 70, flush=True)
            print(">>> TARGET WAYPOINT SELECTED <<<", flush=True)
            print(f"  MAVROS Local X:    {mx:+.3f} meters", flush=True)
            print(f"  MAVROS Local Y:    {my:+.3f} meters", flush=True)
            print(f"  MAVROS Local Z:     0.000 meters (ground plane)", flush=True)
            print(f"  GPS Latitude:      {lat:.8f}°", flush=True)
            print(f"  GPS Longitude:     {lon:.8f}°", flush=True)
            if dist_str:
                print(f"  Range from Rover:  {dist_str}", flush=True)
                print(f"  Heading from Rover: {bearing_str}", flush=True)
            print("  Copy-Paste for Nav2 / Target Goal:", flush=True)
            print(f"    x: {mx:.3f}, y: {my:.3f}, z: 0.0", flush=True)
            print("=" * 70 + "\n", flush=True)

            # Update target graphics
            self.target_dot.set_data([self.target_e], [self.target_n])
            if self.rover_smooth_east is not None and self.rover_smooth_north is not None:
                self.target_line.set_data([self.rover_smooth_east, self.target_e],
                                          [self.rover_smooth_north, self.target_n])
            self.target_callout.set_position((self.target_e + 8.0, self.target_n + 8.0))
            self.target_callout.set_text(f"TARGET\nX: {mx:+.2f}m\nY: {my:+.2f}m\n({dist_str})")
            self.target_callout.set_visible(True)
            self.fig.canvas.draw_idle()

    def on_key_press(self, event):
        """Keyboard shortcuts."""
        if event.key in ['c', 'escape']:
            # Clear target pin
            self.target_e = None
            self.target_n = None
            self.target_dot.set_data([], [])
            self.target_line.set_data([], [])
            self.target_callout.set_visible(False)
            self.get_logger().info("Target waypoint cleared.")
            self.fig.canvas.draw_idle()
        elif event.key == 'f':
            # Reset to full 1km map view
            margin = self.radius_m * 1.05
            self.ax.set_xlim(-margin, margin)
            self.ax.set_ylim(-margin, margin)
            self.fig.canvas.draw_idle()
        elif event.key == 'r':
            # Zoom to rover
            if self.rover_smooth_east is not None and self.rover_smooth_north is not None:
                span = 120.0
                self.ax.set_xlim(self.rover_smooth_east - span, self.rover_smooth_east + span)
                self.ax.set_ylim(self.rover_smooth_north - span, self.rover_smooth_north + span)
                self.fig.canvas.draw_idle()
        elif event.key == 'o':
            # Zoom to MAVROS (0,0) origin
            if self.origin_east is not None and self.origin_north is not None:
                span = 120.0
                self.ax.set_xlim(self.origin_east - span, self.origin_east + span)
                self.ax.set_ylim(self.origin_north - span, self.origin_north + span)
                self.fig.canvas.draw_idle()

    def update_gui(self):
        """Periodic dynamic updates for vehicle marker, smooth trail, and HUD."""
        if self.rover_smooth_east is None or self.rover_smooth_north is None:
            # If origin not locked yet, but we have raw GPS, show raw GPS dot temporarily
            if self.rover_gps_east is not None and self.rover_gps_north is not None:
                self.rover_dot.set_data([self.rover_gps_east], [self.rover_gps_north])
            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()
            return

        re = self.rover_smooth_east
        rn = self.rover_smooth_north

        # Update smooth rover trail (MAVROS Local Odom)
        if self.trail_local_x:
            trail_e = np.array(self.trail_local_x) + self.origin_east
            trail_n = np.array(self.trail_local_y) + self.origin_north
            self.line_trail.set_data(trail_e, trail_n)

        # Update raw GPS trail
        if self.raw_gps_e:
            self.line_raw_gps.set_data(self.raw_gps_e, self.raw_gps_n)

        # Update rover dot (smooth EKF position)
        self.rover_dot.set_data([re], [rn])

        # Update rover heading arrow
        arrow_len = 25.0  # Visible on 1km scale
        yaw_rad = math.radians(self.rover_yaw_deg)
        u = arrow_len * math.cos(yaw_rad)
        v = arrow_len * math.sin(yaw_rad)
        self.rover_arrow.set_offsets([[re, rn]])
        self.rover_arrow.set_UVC([u], [v])

        # Update target line
        if self.target_e is not None and self.target_n is not None:
            self.target_line.set_data([re, self.target_e], [rn, self.target_n])

        # Update HUD text
        comp_heading = compass_bearing_deg(self.rover_yaw_deg)
        card_heading = bearing_to_cardinal(comp_heading)

        origin_str = "LOCKED" if self.origin_locked else f"LOCKING ({len(self.origin_samples)}/10)..."
        orig_coords = f"Lat={self.origin_lat:.7f}, Lon={self.origin_lon:.7f}" if self.origin_lat else "Waiting for GPS"
        rx_str = f"X = {self.rover_local_x:+7.2f}m, Y = {self.rover_local_y:+7.2f}m" if self.rover_local_x is not None else "Waiting for Odom"

        hud = (
            f"MAVROS LOCAL LOCATION FINDER\n"
            f"Origin (0,0):   {origin_str}\n"
            f"Origin Geodetic:{orig_coords}\n"
            f"Origin Map Off: E={self.origin_east:+6.1f}m, N={self.origin_north:+6.1f}m\n"
            f"-----------------------------------------\n"
            f"Rover MAVROS:   {rx_str}\n"
            f"Rover Heading:  ENU {self.rover_yaw_deg:+6.1f}° | Compass {comp_heading:5.1f}° ({card_heading})\n"
            f"Trail Length:   {len(self.trail_local_x)} pts (Smooth EKF)\n"
            f"Messages:       GPS={self.gps_count} | Odom={self.odom_count}"
        )
        self.hud_text.set_text(hud)

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()


def main():
    parser = argparse.ArgumentParser(
        description="Finds MAVROS (0, 0) origin on 1km satellite map and interactively reads/picks (X, Y) target coordinates.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument('--map', default=os.path.expanduser('~/map_satellite_1km.png'),
                        help="Path to 1km satellite map image")
    parser.add_argument('--map-lat', type=float, default=28.7484727,
                        help="Map center origin latitude in degrees (DTU campus)")
    parser.add_argument('--map-lon', type=float, default=77.1171744,
                        help="Map center origin longitude in degrees (DTU campus)")
    parser.add_argument('--radius', type=float, default=1000.0,
                        help="Map radius in meters")

    cleaned_args = [arg for arg in sys.argv[1:] if not arg.startswith('--ros-args')]
    args, _ = parser.parse_known_args(cleaned_args)

    rclpy.init()
    node = FindLocationNode(
        satellite_bg=args.map,
        map_center_lat=args.map_lat,
        map_center_lon=args.map_lon,
        radius_m=args.radius
    )

    last_gui_update = 0.0
    gui_interval = 0.05  # 20 FPS redraw

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.005)
            if not plt.fignum_exists(node.fig.number):
                node.get_logger().info("Map display window closed by user.")
                break
            now = node.get_clock().now().nanoseconds / 1e9
            if (now - last_gui_update) >= gui_interval:
                node.update_gui()
                last_gui_update = now
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Find Location Node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
