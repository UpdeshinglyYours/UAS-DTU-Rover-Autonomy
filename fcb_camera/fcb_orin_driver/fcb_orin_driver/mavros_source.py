"""RC channels and vehicle telemetry from a running MAVROS.

fcb_record.py opened the autopilot's serial port itself, with pymavlink
(fcb_base_driver.mavlink_source). On a ROS drone MAVROS usually owns that
port already, and a second reader on it corrupts both streams. This reads
the same things from MAVROS's topics instead, and answers the same
questions MavlinkSource does -- rc_age(), channel(), telemetry(),
is_connected() -- so RcZoomSource, RcButton, RcSelector and the recorder
use it unchanged.

What MAVROS gives differs from the raw messages in two places:
  - attitude arrives as an ENU/FLU quaternion on imu/data, and is turned
    back into the NED roll/pitch/yaw degrees the ATTITUDE message carried;
  - there is no time_boot_ms, so fc_time_boot_ms stays empty in the CSV.
    Line the CSV up with the .bin log on t_wall_utc instead.
alt_msl_m is VFR_HUD's MSL altitude and alt_rel_m is global_position/rel_alt.
"""
import math
import threading
import time

from rclpy.qos import qos_profile_sensor_data

from fcb_base_driver.mavlink_source import (MAX_RC_CHANNELS, Telemetry,
                                            _pwm_or_absent)


def enu_quaternion_to_ned_rpy(x, y, z, w):
    """ROS ENU/FLU orientation -> aircraft NED/FRD roll, pitch, yaw in degrees.

    Roll is the same about the forward axis, pitch flips sign with the
    lateral axis, and yaw turns from "anticlockwise from east" into
    "clockwise from north".
    """
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sinp)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    yaw_ned = math.degrees(math.pi / 2.0 - yaw)
    yaw_ned = (yaw_ned + 180.0) % 360.0 - 180.0
    return math.degrees(roll), -math.degrees(pitch), yaw_ned


class MavrosSource:
    """MavlinkSource's interface, fed by MAVROS topics."""

    def __init__(self, node, namespace="/mavros", on_status=None):
        from mavros_msgs.msg import RCIn, State, VfrHud
        from sensor_msgs.msg import Imu, NavSatFix
        from std_msgs.msg import Float64

        self._status = on_status or (lambda msg: None)
        self._lock = threading.Lock()
        self._channels = [0] * MAX_RC_CHANNELS
        self._rc_rx = 0.0
        self._connected = False
        self._fix = None        # (lat, lon, t) -- lat/lon None without a fix
        self._rel_alt = None    # (alt, t)
        self._attitude = None   # (roll, pitch, yaw, t)
        self._hud = None        # (heading, groundspeed, alt_msl, t)
        self._node = node
        ns = namespace.rstrip("/")
        qos = qos_profile_sensor_data
        self._subs = [
            node.create_subscription(RCIn, f"{ns}/rc/in", self._on_rc, qos),
            node.create_subscription(State, f"{ns}/state", self._on_state, 10),
            node.create_subscription(NavSatFix, f"{ns}/global_position/global",
                                     self._on_fix, qos),
            node.create_subscription(Float64, f"{ns}/global_position/rel_alt",
                                     self._on_rel_alt, qos),
            node.create_subscription(Imu, f"{ns}/imu/data", self._on_imu, qos),
            node.create_subscription(VfrHud, f"{ns}/vfr_hud", self._on_hud, qos),
        ]
        self._status(f"reading RC and telemetry from {ns}/*")

    # -- callbacks -------------------------------------------------------

    def _on_rc(self, msg):
        values = [0] * MAX_RC_CHANNELS
        for i, pwm in enumerate(list(msg.channels)[:MAX_RC_CHANNELS]):
            values[i] = _pwm_or_absent(pwm)
        with self._lock:
            self._channels = values
            self._rc_rx = time.monotonic()

    def _on_state(self, msg):
        with self._lock:
            changed = self._connected != bool(msg.connected)
            self._connected = bool(msg.connected)
        if changed:
            self._status("MAVROS reports the flight controller %s"
                         % ("connected" if msg.connected else "DISCONNECTED"))

    def _on_fix(self, msg):
        # NavSatFix status -1 is STATUS_NO_FIX. ArduPilot reports 0,0
        # before a fix; both are "no position", not the Gulf of Guinea.
        unfixed = msg.status.status < 0 or (msg.latitude == 0 and msg.longitude == 0)
        with self._lock:
            self._fix = (None if unfixed else msg.latitude,
                         None if unfixed else msg.longitude,
                         time.monotonic())

    def _on_rel_alt(self, msg):
        with self._lock:
            self._rel_alt = (float(msg.data), time.monotonic())

    def _on_imu(self, msg):
        q = msg.orientation
        if q.x == 0 and q.y == 0 and q.z == 0 and q.w == 0:
            return
        roll, pitch, yaw = enu_quaternion_to_ned_rpy(q.x, q.y, q.z, q.w)
        with self._lock:
            self._attitude = (roll, pitch, yaw, time.monotonic())

    def _on_hud(self, msg):
        with self._lock:
            self._hud = (float(msg.heading), float(msg.groundspeed),
                         float(msg.altitude), time.monotonic())

    # -- MavlinkSource interface -----------------------------------------

    def channels(self):
        with self._lock:
            return list(self._channels)

    def channel(self, number):
        if not 1 <= number <= MAX_RC_CHANNELS:
            raise ValueError(f"channel must be 1-{MAX_RC_CHANNELS}, got {number}")
        with self._lock:
            return self._channels[number - 1]

    def rc_age(self):
        with self._lock:
            return time.monotonic() - self._rc_rx if self._rc_rx else None

    def telemetry(self):
        now = time.monotonic()
        with self._lock:
            fix, rel, attitude, hud = (self._fix, self._rel_alt,
                                       self._attitude, self._hud)
        sample = Telemetry()
        if fix is not None or rel is not None:
            if fix is not None:
                sample.lat_deg, sample.lon_deg = fix[0], fix[1]
            if rel is not None:
                sample.alt_rel_m = rel[0]
            if hud is not None:
                sample.alt_msl_m = hud[2]
            stamp = fix[2] if fix is not None else rel[1]
            sample.position_age_s = now - stamp
        if attitude is not None:
            sample.roll_deg, sample.pitch_deg, sample.yaw_deg = attitude[:3]
            sample.attitude_age_s = now - attitude[3]
        if hud is not None:
            sample.heading_deg = hud[0]
            sample.groundspeed_ms = hud[1]
            sample.hud_age_s = now - hud[3]
        return sample

    def is_connected(self):
        with self._lock:
            return self._connected

    def sysid(self):
        return None

    def stop(self):
        for sub in self._subs:
            try:
                self._node.destroy_subscription(sub)
            except Exception:
                pass
        self._subs = []
