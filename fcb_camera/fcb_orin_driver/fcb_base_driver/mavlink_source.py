"""One MAVLink connection serving both RC channels and vehicle telemetry.

Everything that needs the flight controller shares this: the zoom knob on
an RC channel, the recording switch on another, and the position/attitude
stream the flight recorder writes alongside each video frame. It is one
class because it must be one connection -- two processes (or two objects)
opening the same serial port would interleave reads and corrupt both.

ArduPilot sends almost nothing to a companion link unless asked, so the
streams are requested on connect and re-requested after a reconnect.

Every reading is stamped with the monotonic time it arrived. Telemetry is
sampled asynchronously and far slower than 60 fps video -- GPS lands at a
few hertz -- so a consumer pairing telemetry with a video frame needs to
know how stale each value is rather than assuming it is frame-accurate.
"""
import math
import threading
import time
from dataclasses import dataclass

try:
    from pymavlink import mavutil
except ImportError:  # pymavlink is an optional dependency of the caller
    mavutil = None

MAX_RC_CHANNELS = 18

#: A link this quiet is treated as gone, and reconnected. ArduPilot streams
#: several messages a second once asked; a port that stays open but goes
#: silent -- the autopilot rebooted without the USB dropping, or the link
#: froze -- otherwise read as "connected" for ever, with the stream request
#: that a reconnect would re-send never sent again.
SILENCE_TIMEOUT = 5.0

#: Plausible RC pulse widths. MAVLink marks an unused channel with
#: UINT16_MAX (65535), and anything outside this band is not a stick or a
#: switch -- read as a real value, 65535 is "switch high" and would start a
#: recording on a channel the transmitter does not even send.
PWM_VALID = (800, 2200)


def _pwm_or_absent(value):
    value = int(value or 0)
    return value if PWM_VALID[0] <= value <= PWM_VALID[1] else 0


@dataclass
class Telemetry:
    """A snapshot of the vehicle state, with the age of each reading.

    Ages are seconds since that message arrived, or None if no message of
    that kind has been seen at all. The three groups age independently
    because they come from three different MAVLink messages.
    """

    lat_deg: float = None
    lon_deg: float = None
    alt_msl_m: float = None
    alt_rel_m: float = None
    position_age_s: float = None
    position_boot_ms: int = None

    roll_deg: float = None
    pitch_deg: float = None
    yaw_deg: float = None
    attitude_age_s: float = None
    attitude_boot_ms: int = None

    heading_deg: float = None
    groundspeed_ms: float = None
    hud_age_s: float = None


class MavlinkSource:
    """Background reader for RC channels and vehicle telemetry."""

    #: Streams to ask ArduPilot for, and which messages each one carries.
    #: RC_CHANNELS is the switches, POSITION is GLOBAL_POSITION_INT,
    #: EXTRA1 is ATTITUDE, EXTRA2 is VFR_HUD.
    STREAMS = ("RC_CHANNELS", "POSITION", "EXTRA1", "EXTRA2")

    def __init__(self, url, baud=115200, stream_rate_hz=10, on_status=None):
        if mavutil is None:
            raise RuntimeError(
                "pymavlink is not installed -- pip install pymavlink"
            )
        self.url = url
        self.baud = baud
        self.stream_rate_hz = stream_rate_hz
        self._status = on_status or (lambda msg: None)

        self._lock = threading.Lock()
        self._channels = [0] * MAX_RC_CHANNELS
        self._rc_rx = 0.0
        self._connected = False
        self._sysid = None

        self._position = None      # (lat, lon, alt_msl, alt_rel, boot_ms, t)
        self._attitude = None      # (roll, pitch, yaw, boot_ms, t)
        self._hud = None           # (heading, groundspeed, t)

        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    # -- public API ------------------------------------------------------

    def channels(self):
        """Snapshot of every channel's raw PWM, for diagnostics."""
        with self._lock:
            return list(self._channels)

    def channel(self, number):
        """One channel's raw PWM (1-based), or 0 if absent/not yet seen."""
        if not 1 <= number <= MAX_RC_CHANNELS:
            raise ValueError(f"channel must be 1-{MAX_RC_CHANNELS}, got {number}")
        with self._lock:
            return self._channels[number - 1]

    def rc_age(self):
        """Seconds since the last RC_CHANNELS message, or None if never."""
        with self._lock:
            return time.monotonic() - self._rc_rx if self._rc_rx else None

    def telemetry(self):
        """Current vehicle state with per-reading ages, taken atomically."""
        now = time.monotonic()
        with self._lock:
            position, attitude, hud = self._position, self._attitude, self._hud

        sample = Telemetry()
        if position is not None:
            lat, lon, alt_msl, alt_rel, boot_ms, stamp = position
            sample.lat_deg = lat
            sample.lon_deg = lon
            sample.alt_msl_m = alt_msl
            sample.alt_rel_m = alt_rel
            sample.position_boot_ms = boot_ms
            sample.position_age_s = now - stamp
        if attitude is not None:
            roll, pitch, yaw, boot_ms, stamp = attitude
            sample.roll_deg = roll
            sample.pitch_deg = pitch
            sample.yaw_deg = yaw
            sample.attitude_boot_ms = boot_ms
            sample.attitude_age_s = now - stamp
        if hud is not None:
            heading, groundspeed, stamp = hud
            sample.heading_deg = heading
            sample.groundspeed_ms = groundspeed
            sample.hud_age_s = now - stamp
        return sample

    def is_connected(self):
        with self._lock:
            return self._connected

    def sysid(self):
        with self._lock:
            return self._sysid

    def stop(self):
        self._running = False
        self._thread.join(timeout=1.0)

    # -- background reader -----------------------------------------------

    def _run(self):
        connection = None
        last_rx = 0.0
        while self._running:
            try:
                if connection is None:
                    connection = self._connect()
                    last_rx = time.monotonic()
                msg = connection.recv_match(blocking=True, timeout=1.0)
                if msg is not None:
                    last_rx = time.monotonic()
                    self._on_message(msg)
                elif time.monotonic() - last_rx > SILENCE_TIMEOUT:
                    raise RuntimeError(
                        f"no MAVLink traffic for {SILENCE_TIMEOUT:.0f}s")
            except Exception as exc:
                with self._lock:
                    self._connected = False
                self._status(f"MAVLink link error, reconnecting: {exc}")
                try:
                    if connection is not None:
                        connection.close()
                except Exception:
                    pass
                connection = None
                time.sleep(1.0)

    def _connect(self):
        self._status(f"connecting to {self.url}")
        connection = mavutil.mavlink_connection(self.url, baud=self.baud)
        heartbeat = connection.wait_heartbeat(timeout=10)
        if heartbeat is None:
            raise RuntimeError(f"no MAVLink heartbeat from {self.url} within 10s")
        # Read the ids off the heartbeat itself: connection.target_system is
        # not always populated by the time this returns, and logging "system
        # 0" makes a healthy link look broken.
        sysid = heartbeat.get_srcSystem()
        compid = heartbeat.get_srcComponent()
        with self._lock:
            self._connected = True
            self._sysid = sysid
        self._status(f"heartbeat from system {sysid} component {compid}")
        self._request_streams(connection)
        return connection

    def _request_streams(self, connection):
        for name in self.STREAMS:
            stream_id = getattr(mavutil.mavlink, f"MAV_DATA_STREAM_{name}")
            connection.mav.request_data_stream_send(
                connection.target_system,
                connection.target_component,
                stream_id,
                self.stream_rate_hz,
                1,  # start sending
            )
        self._status(
            f"requested {', '.join(self.STREAMS)} at {self.stream_rate_hz} Hz"
        )

    def _on_message(self, msg):
        kind = msg.get_type()
        now = time.monotonic()

        if kind == "RC_CHANNELS":
            # Channels past chancount are not being sent by the transmitter.
            count = int(getattr(msg, "chancount", MAX_RC_CHANNELS) or 0) \
                or MAX_RC_CHANNELS
            values = [_pwm_or_absent(getattr(msg, f"chan{i}_raw", 0))
                      if i <= count else 0
                      for i in range(1, MAX_RC_CHANNELS + 1)]
            with self._lock:
                self._channels = values
                self._rc_rx = now

        elif kind == "RC_CHANNELS_RAW":
            # Only channels 1-8 exist in this (MAVLink 1) message. Writing
            # all 18 from it zeroed 9-18 on every one, and on a link that
            # carries both messages the snapshot (9) and imaging-mode (10)
            # channels flickered between their real value and "absent".
            with self._lock:
                for i in range(1, 9):
                    self._channels[i - 1] = _pwm_or_absent(
                        getattr(msg, f"chan{i}_raw", 0))
                self._rc_rx = now

        elif kind == "GLOBAL_POSITION_INT":
            # ArduPilot publishes this before it has a GPS fix, with lat and
            # lon at exactly zero. Recorded as-is that reads as a real
            # position in the Gulf of Guinea, so it is reported as no
            # position at all. The altitudes are kept either way: they come
            # from the barometer and EKF and are meaningful without a fix.
            unfixed = msg.lat == 0 and msg.lon == 0
            with self._lock:
                self._position = (
                    None if unfixed else msg.lat / 1e7,
                    None if unfixed else msg.lon / 1e7,
                    msg.alt / 1000.0,
                    msg.relative_alt / 1000.0,
                    int(msg.time_boot_ms),
                    now,
                )

        elif kind == "ATTITUDE":
            with self._lock:
                self._attitude = (
                    math.degrees(msg.roll),
                    math.degrees(msg.pitch),
                    math.degrees(msg.yaw),
                    int(msg.time_boot_ms),
                    now,
                )

        elif kind == "VFR_HUD":
            with self._lock:
                self._hud = (float(msg.heading), float(msg.groundspeed), now)
