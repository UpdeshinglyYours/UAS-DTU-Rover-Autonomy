"""Maps one RC channel's PWM to a normalized 0.0-1.0 zoom target.

The MAVLink plumbing lives in mavlink_source.MavlinkSource; this is just
the zoom-shaping layer on top of it, shared by the ROS 2 rc_zoom node, the
standalone teleop, and the flight recorder.

Pass an existing `source` to share one connection with other consumers --
the flight recorder does this, since it also wants position and attitude
off the same link. Left unset, one is opened and owned here.
"""
from fcb_base_driver import zoom_map
from fcb_base_driver.mavlink_source import MAX_RC_CHANNELS, MavlinkSource

__all__ = ["MAX_RC_CHANNELS", "RcZoomSource"]


class RcZoomSource:
    """Tracks one RC channel's PWM as a normalized 0.0-1.0 zoom target."""

    def __init__(self, url=None, baud=115200, channel=7, pwm_min=1000,
                 pwm_max=2000, pwm_deadband=8, reverse=False,
                 stream_rate_hz=10, rc_timeout=2.0, on_status=None,
                 source=None):
        if not 1 <= channel <= MAX_RC_CHANNELS:
            raise ValueError(f"channel must be 1-{MAX_RC_CHANNELS}, got {channel}")

        self.channel = channel
        self.pwm_min = pwm_min
        self.pwm_max = pwm_max
        self.pwm_deadband = pwm_deadband
        self.reverse = reverse
        self.rc_timeout = rc_timeout

        self._owns_source = source is None
        self.source = source or MavlinkSource(
            url=url, baud=baud, stream_rate_hz=stream_rate_hz,
            on_status=on_status,
        )
        self._last_pwm = None

    # -- public API ------------------------------------------------------

    def unit(self):
        """Latest normalized 0.0-1.0 target, or None if RC is stale/unseen."""
        age = self.source.rc_age()
        if age is None or age > self.rc_timeout:
            return None
        pwm = self.source.channel(self.channel)
        if pwm <= 0:
            return None
        # Ignore sub-deadband jitter so a noisy channel does not keep the
        # servo loop nudging the lens back and forth.
        if (self._last_pwm is not None
                and abs(pwm - self._last_pwm) < self.pwm_deadband):
            pwm = self._last_pwm
        self._last_pwm = pwm
        return zoom_map.pwm_to_unit(pwm, self.pwm_min, self.pwm_max, self.reverse)

    def pwm(self):
        """Latest raw PWM for the configured channel, or 0 if never seen."""
        return self.source.channel(self.channel)

    def channels(self):
        """A snapshot of all channels' raw PWM, for diagnostics."""
        return self.source.channels()

    def is_connected(self):
        return self.source.is_connected()

    def stop(self):
        if self._owns_source:
            self.source.stop()
