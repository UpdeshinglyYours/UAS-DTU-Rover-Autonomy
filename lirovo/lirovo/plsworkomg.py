#!/usr/bin/env python3
"""
cmd_vel → PWM Mapper for ROS2 Humble
======================================
Subscribes to: /cmd_vel              (geometry_msgs/msg/Twist)
Publishes to:  /mavros/rc/override   (mavros_msgs/msg/OverrideRCIn)

Angular-Z (steering)
---------------------
PWM range: 1501 (zero/stop) | 1701 (min positive rotation) → 2101 (max positive angular-z)
Mapping is derived from empirical az_midpoint calibration data
using high-resolution piecewise linear interpolation over all 41 PWM steps.

The az_mean_window column is used for the velocity axis because it is
monotone-smoothed; raw az_midpoint is kept as reference only.

Linear-X (throttle)
--------------------
PWM = 1500 ± (slope_linear * |linear.x| + intercept_linear)
Clamped to [pwm_min, pwm_max].  Forward → above 1500, reverse → below 1500.

Usage
-----
ros2 run <your_package> az_vel_to_pwm

Parameters (ROS2)
-----------------
  Angular-Z / steering:
    pwm_channel      : RC channel index (0-based), default 0
    deadband_vel     : |vel| below which PWM = 1501 (stop), default 0.03 rad/s
    publish_rate     : Hz, default 50
    invert_direction : flip sign of incoming angular.z, default False

  Linear-X / throttle:
    ch_throttle      : RC channel index (0-based), default 1
    pwm_min          : lower PWM clamp, default 1051
    pwm_max          : upper PWM clamp, default 1951
    pwm_neutral      : stop PWM for throttle, default 1501
    slope_linear     : throttle gain slope, default 314
    intercept_linear : throttle gain intercept, default 140
    linear_vel_max   : m/s, used for reference only, default 1.0

  Timeout:
    cmd_timeout      : seconds of /cmd_vel silence before RC override stops,
                       default 0.2 s (~10 missed cycles at 50 Hz)
"""

import time

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import OverrideRCIn
from geometry_msgs.msg import Twist
import numpy as np


# ---------------------------------------------------------------------------
# Calibration data  (1701–2101 range, positive angular-z only)
#
# Columns: PWM  |  az_midpoint (rad/s)  |  az_mean_window (rad/s)
#
# az_mean_window is the smoothed velocity that corresponds to each PWM step.
# It is used as the velocity axis for the piecewise mapping because it is
# monotone after a running-max pass (raw az_midpoint has local dips).
# ---------------------------------------------------------------------------
_RAW_TABLE = [
    # PWM    az_midpoint    az_mean_window
    (1701, +0.0049694,  +0.0116746),
    (1711, +0.2274574,  +0.0979008),
    (1721, +0.1737052,  +0.1362236),
    (1731, +0.4751986,  +0.2596646),
    (1741, +0.1476481,  +0.0829669),  # local dip — resolved by monotone pass
    (1751, +0.3071875,  +0.2271002),
    (1761, +0.4833515,  +0.3657181),
    (1771, +0.6804709,  +0.2407416),  # local dip — resolved by monotone pass
    (1781, +0.5793779,  +0.4451320),
    (1791, +1.0248756,  +0.5320357),
    (1801, +1.1032788,  +0.5747312),
    (1811, +1.0711821,  +0.6207493),
    (1821, +1.2955393,  +0.6905345),
    (1831, +1.1781380,  +0.7251384),
    (1841, +1.0658966,  +0.7950879),
    (1851, +1.4582719,  +0.8642207),
    (1861, +1.4239148,  +0.8996943),
    (1871, +1.7182883,  +0.9620392),
    (1881, +1.5694072,  +1.0192515),
    (1891, +1.9333812,  +1.0672085),
    (1901, +1.9507320,  +1.1195627),
    (1911, +1.9800199,  +1.1534857),
    (1921, +2.0167842,  +1.1996787),
    (1931, +2.0566902,  +1.2736796),
    (1941, +2.2430718,  +1.3122753),
    (1951, +2.3966208,  +1.3747363),
    (1961, +2.4894938,  +1.4250761),
    (1971, +2.3577766,  +1.4557761),
    (1981, +2.5835660,  +1.4988349),
    (1991, +2.6570659,  +1.5663777),
    (2001, +2.7864964,  +1.6056126),
    (2011, +2.6721101,  +1.5999318),  # slight dip — resolved by monotone pass
    (2021, +2.7697933,  +1.6120573),
    (2031, +2.5703838,  +1.6245543),
    (2041, +2.7050591,  +1.5916198),  # slight dip — resolved by monotone pass
    (2051, +2.7829666,  +1.6222680),
    (2061, +2.6038892,  +1.6164773),  # slight dip — resolved by monotone pass
    (2071, +2.8462231,  +1.6217815),
    (2081, +2.7862916,  +1.6369924),
    (2091, +2.7955372,  +1.6103469),  # slight dip — resolved by monotone pass
    (2101, +2.5281668,  +1.6286706),
]

_PWM_NEUTRAL = 1501   # zero angular-z (full stop)
_PWM_MIN_POS = 1701   # threshold where positive rotation begins
_PWM_MAX     = 2101


# ---------------------------------------------------------------------------
# Build piecewise-linear segments: vel → PWM
#
# Strategy
# --------
# 1. Use az_mean_window as the velocity axis (smoother than az_midpoint).
# 2. Enforce strict monotonicity via a running-maximum pass so every
#    segment has vel_lo < vel_hi (required for a proper inverse mapping).
# 3. Every consecutive monotone pair becomes one piecewise segment.
#    Non-monotone points are skipped; the surrounding segments span them.
#
# Result: up to 41 piecewise linear segments, one per PWM step pair.
# ---------------------------------------------------------------------------

def _build_segments():
    pwms = [r[0] for r in _RAW_TABLE]
    vels = [r[2] for r in _RAW_TABLE]   # az_mean_window column

    mono_pwm = [pwms[0]]
    mono_vel = [vels[0]]
    running_max = vels[0]

    for p, v in zip(pwms[1:], vels[1:]):
        if v > running_max:
            running_max = v
            mono_pwm.append(p)
            mono_vel.append(v)
        # Non-monotone point: skip; the span is covered by the previous segment.

    segs = []
    for i in range(len(mono_pwm) - 1):
        segs.append((
            mono_vel[i],       # vel_lo
            mono_vel[i + 1],   # vel_hi
            mono_pwm[i],       # pwm_lo
            mono_pwm[i + 1],   # pwm_hi
        ))
    return segs


_SEGMENTS = _build_segments()
_VEL_MIN  = _SEGMENTS[0][0]    # velocity that corresponds to PWM 1701
_VEL_MAX  = _SEGMENTS[-1][1]   # maximum calibrated velocity


# ---------------------------------------------------------------------------
# Core mapping function
# ---------------------------------------------------------------------------

def vel_to_pwm(angular_z: float) -> int:
    """
    Convert a commanded angular-z velocity (rad/s) to a PWM value.

    Piecewise linear interpolation across all calibration segments.

    Parameters
    ----------
    angular_z : float
        Positive angular-z velocity in rad/s.
        Values at or below _VEL_MIN  → PWM 1701 (slowest positive step).
        Values at or above _VEL_MAX  → PWM 2101 (maximum).

    Returns
    -------
    int
        PWM in microseconds, clamped to [1701, 2101].
    """
    vel = float(angular_z)

    if vel <= _VEL_MIN:
        return _PWM_MIN_POS   # slowest positive rotation (1701)
    if vel >= _VEL_MAX:
        return _PWM_MAX

    # Binary search for the enclosing segment
    lo, hi = 0, len(_SEGMENTS) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if _SEGMENTS[mid][1] < vel:
            lo = mid + 1
        else:
            hi = mid

    vel_lo, vel_hi, pwm_lo, pwm_hi = _SEGMENTS[lo]

    if vel_hi == vel_lo:
        return int(round(pwm_lo))

    t   = (vel - vel_lo) / (vel_hi - vel_lo)
    pwm = pwm_lo + t * (pwm_hi - pwm_lo)
    return int(round(float(np.clip(pwm, _PWM_MIN_POS, _PWM_MAX))))


# ---------------------------------------------------------------------------
# Linear-X throttle helper  (ported from converter.py)
# ---------------------------------------------------------------------------

def _linear_vel_to_pwm_gain(vel: float,
                             slope: float,
                             intercept: float,
                             pwm_max: int,
                             pwm_min: int) -> int:
    """
    Compute the unsigned PWM gain for a given linear velocity magnitude.

    pwm_gain = slope * |vel| + intercept, clamped to [0, pwm_max - 1500].
    The caller adds/subtracts this from 1500 to get the final throttle PWM.
    """
    gain = slope * vel + intercept

    if gain > pwm_max - 1500:
        gain = pwm_max - 1500

    if gain < 0:
        gain = 0

    return int(round(gain))


# ---------------------------------------------------------------------------
# ROS2 Node
# ---------------------------------------------------------------------------

class CmdVelToPwmNode(Node):
    """
    Listens to /cmd_vel (geometry_msgs/Twist), converts angular.z to a
    PWM value via the piecewise calibration map, and publishes it on the
    configured channel of /mavros/rc/override.

    angular.z = 0 or within deadband → PWM 1501 (full stop)
    angular.z > 0  → PWM 1701–2101  (positive rotation, calibrated)
    Negative angular.z  → symmetric mirror below 1501 (magnitude mirrored)

    Publishing stops automatically when /cmd_vel has been silent for longer
    than cmd_timeout seconds, allowing MAVROS/FC failsafe to take over.
    Publishing resumes as soon as a new /cmd_vel message arrives.
    """

    def __init__(self):
        super().__init__('az_vel_to_pwm')

        # --- ROS2 parameters (angular-z / steering) ---
        self.declare_parameter('pwm_channel',      0)      # 0-based RC channel index
        self.declare_parameter('deadband_vel',     0.03)   # rad/s
        self.declare_parameter('publish_rate',     50.0)   # Hz
        self.declare_parameter('invert_direction', False)  # flip sign of angular.z

        self._channel  = self.get_parameter('pwm_channel').value
        self._deadband = self.get_parameter('deadband_vel').value
        self._rate_hz  = self.get_parameter('publish_rate').value
        self._invert   = self.get_parameter('invert_direction').value

        # --- ROS2 parameters (linear-x / throttle) ---
        self.declare_parameter('ch_throttle',        1)      # 0-based RC channel index
        self.declare_parameter('pwm_min',            1051)
        self.declare_parameter('pwm_max',            1951)
        self.declare_parameter('pwm_neutral',        1501)
        self.declare_parameter('slope_linear',       314)
        self.declare_parameter('intercept_linear',   140)
        self.declare_parameter('linear_vel_max',     1.0)   # m/s

        self._ch_throttle        = self.get_parameter('ch_throttle').value
        self._pwm_min            = self.get_parameter('pwm_min').value
        self._pwm_max            = self.get_parameter('pwm_max').value
        self._pwm_neutral        = self.get_parameter('pwm_neutral').value
        self._slope_linear       = self.get_parameter('slope_linear').value
        self._intercept_linear   = self.get_parameter('intercept_linear').value
        self._linear_vel_max     = self.get_parameter('linear_vel_max').value

        # --- ROS2 parameter (timeout) ---
        self.declare_parameter('cmd_timeout', 0.2)   # seconds of silence → stop publishing
        self._cmd_timeout = self.get_parameter('cmd_timeout').value

        # --- State ---
        self._latest_pwm: int          = _PWM_NEUTRAL        # steering (angular-z)
        self._latest_throttle_pwm: int = self._pwm_neutral   # throttle (linear-x)
        self._last_cmd_time: float     = 0.0                 # monotonic time of last /cmd_vel

        # --- Publisher ---
        self._rc_pub = self.create_publisher(
            OverrideRCIn,
            '/mavros/rc/override',
            10
        )

        # --- Subscriber ---
        self._cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel1',
            self._cmd_vel_callback,
            10
        )

        # --- Periodic publish timer ---
        self._timer = self.create_timer(
            1.0 / self._rate_hz,
            self._publish_rc
        )

        self.get_logger().info(
            f'[az_vel_to_pwm] ready | '
            f'steering CH{self._channel} deadband={self._deadband} rad/s | '
            f'throttle CH{self._ch_throttle} slope={self._slope_linear} intercept={self._intercept_linear} | '
            f'rate={self._rate_hz} Hz | '
            f'cmd_timeout={self._cmd_timeout} s | '
            f'PWM stop={_PWM_NEUTRAL} pos_start={_PWM_MIN_POS} max={_PWM_MAX} | '
            f'{len(_SEGMENTS)} piecewise segments'
        )

    # ------------------------------------------------------------------
    def _cmd_vel_callback(self, msg: Twist):
        # Stamp the arrival time so _publish_rc can detect silence
        self._last_cmd_time = time.monotonic()

        # ---- Throttle: linear.x → PWM (ported from converter.py) --------
        pwm_gain_x = _linear_vel_to_pwm_gain(
            abs(msg.linear.x),
            self._slope_linear,
            self._intercept_linear,
            self._pwm_max,
            self._pwm_min,
        )

        if msg.linear.x > 0.0:
            self._latest_throttle_pwm = 1500 + pwm_gain_x
        elif msg.linear.x < 0.0:
            self._latest_throttle_pwm = 1500 - pwm_gain_x
        elif msg.linear.x < 10 ** (-3):
            self._latest_throttle_pwm = 1500
        else:
            self._latest_throttle_pwm = 1500

        # ---- Steering: angular.z → PWM (piecewise calibration map) ------
        az = msg.angular.z

        if self._invert:
            az = -az

        if abs(az) < self._deadband:
            # Within deadband → full stop
            self._latest_pwm = _PWM_NEUTRAL   # 1501

        elif az >= 0.0:
            # Positive angular-z: direct calibration lookup (1701–2101)
            self._latest_pwm = vel_to_pwm(az)

        else:
            # Negative angular-z: mirror below 1501.
            # vel_to_pwm gives a PWM in [1701,2101]; mirror offset above 1701 below 1501.
            # NOTE: replace with proper negative-range cal (1251–1501) when available.
            pos_pwm = vel_to_pwm(-az)
            delta   = pos_pwm - _PWM_MIN_POS
            self._latest_pwm = max(1000, _PWM_NEUTRAL - delta)  # mirror below 1501

    # ------------------------------------------------------------------
    def _publish_rc(self):
        # Never received a command yet
        if self._last_cmd_time == 0.0:
            return

        # /cmd_vel has gone silent — stop publishing RC override so the
        # FC/MAVROS failsafe can take over cleanly
        if (time.monotonic() - self._last_cmd_time) > self._cmd_timeout:
            return

        msg      = OverrideRCIn()
        # channels: 18-element list; 65535 means "do not override this channel"
        channels = [65535] * 18
        channels[self._channel]     = self._latest_pwm           # steering (angular-z)
        channels[self._ch_throttle] = self._latest_throttle_pwm  # throttle (linear-x)
        msg.channels = channels
        self._rc_pub.publish(msg)


# ---------------------------------------------------------------------------
# Standalone diagnostics — run without ROS2
#   python3 az_vel_to_pwm.py --diag
# ---------------------------------------------------------------------------

def _print_diagnostic_table():
    print(f"\n{'Vel (rad/s)':>14}  {'PWM':>6}")
    print("-" * 25)
    for v in np.linspace(0.0, _VEL_MAX + 0.1, 40):
        print(f"{v:>14.6f}  {vel_to_pwm(v):>6}")

    print(f"\n{len(_SEGMENTS)} piecewise segments (vel_lo → vel_hi : pwm_lo → pwm_hi):")
    for i, (vl, vh, pl, ph) in enumerate(_SEGMENTS):
        print(f"  [{i:02d}]  {vl:.6f} → {vh:.6f}  :  {pl} → {ph}")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToPwmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    import sys
    if '--diag' in sys.argv:
        _print_diagnostic_table()
    else:
        main()
