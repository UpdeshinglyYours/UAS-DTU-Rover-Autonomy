"""RC channels that are switches rather than knobs.

The zoom knob is continuously variable and belongs to rc_source; these are
the channels that pick something instead of setting a level -- a momentary
button, and a rotary or toggle with a few detented positions.

Both are edge-triggered on purpose. RC channels report a *level*, tens of
times a second, for as long as the pilot holds the switch where it is.
Acting on that level directly would fire a momentary button hundreds of
times per press, and would make a mode switch re-assert its position
continuously -- which means it would win every arbitration against any
other commander, forever, and no keyboard or script could ever set the
mode while the switch sat where it was. Reporting only transitions makes
the switch behave the way the pilot expects: it acts when they move it,
and otherwise leaves the mode alone.

Stale RC is treated as "no information", never as a release or a change.
A transmitter that browns out mid-flight must not be able to fire a button
press or a mode change on the way back up.
"""
from fcb_base_driver.mavlink_source import MAX_RC_CHANNELS

__all__ = ["RcButton", "RcSelector"]


class _RcChannel:
    """Shared plumbing: one channel's PWM, or None when it cannot be trusted."""

    def __init__(self, source, channel, rc_timeout=2.0):
        if not 1 <= channel <= MAX_RC_CHANNELS:
            raise ValueError(
                f"channel must be 1-{MAX_RC_CHANNELS}, got {channel}"
            )
        self.source = source
        self.channel = channel
        self.rc_timeout = rc_timeout

    def pwm(self):
        """Raw microseconds, or None if RC is stale or the channel is absent."""
        age = self.source.rc_age()
        if age is None or age > self.rc_timeout:
            return None
        pwm = self.source.channel(self.channel)
        # MAVLink reports an unmapped channel as 0, which is not a low
        # reading -- it means the transmitter is not sending that channel.
        return pwm if pwm > 0 else None


class RcButton(_RcChannel):
    """A momentary channel that fires once per press.

    `pressed()` is True on exactly the sample where the channel goes from
    low to high, and False on every other sample including the whole time
    it is held. Poll it as often as you like.
    """

    def __init__(self, source, channel, threshold=1500, reverse=False,
                 rc_timeout=2.0):
        super().__init__(source, channel, rc_timeout)
        self.threshold = threshold
        self.reverse = reverse
        self.presses = 0
        # None, not False: the first sample only establishes where the
        # switch already is. A switch found sitting high at startup has not
        # just been pressed, and firing on it would take a snapshot nobody
        # asked for every time the recorder restarts.
        self._high = None

    def pressed(self):
        pwm = self.pwm()
        if pwm is None:
            # Forget where it was. The first sample after RC comes back only
            # re-establishes the position, as at startup -- otherwise a switch
            # read low before a dropout and high after it fired a press nobody
            # made, exactly what the module docstring promises cannot happen.
            self._high = None
            return False
        high = pwm >= self.threshold
        if self.reverse:
            high = not high
        fired = high and self._high is False
        self._high = high
        if fired:
            self.presses += 1
        return fired


class RcSelector(_RcChannel):
    """An N-position switch: the channel's band picks one of `positions`.

    The travel between `pwm_min` and `pwm_max` is divided into equal bands,
    one per position. `changed()` returns the newly selected position on
    the sample where the band changes, and None otherwise -- including the
    long stretches where the pilot is not touching it.

    The first reading does report, so a recorder starting up adopts
    whatever the switch is already set to rather than leaving the camera
    contradicting the switch in the pilot's hand.
    """

    def __init__(self, source, channel, positions, pwm_min=1000, pwm_max=2000,
                 reverse=False, hysteresis=0.15, rc_timeout=2.0):
        super().__init__(source, channel, rc_timeout)
        if len(positions) < 2:
            raise ValueError("a selector needs at least two positions")
        self.positions = list(positions)
        self.pwm_min = pwm_min
        self.pwm_max = pwm_max
        self.reverse = reverse
        # Fraction of a band the switch must clear before the neighbouring
        # position is accepted. A three-position switch resting a hair off
        # centre would otherwise flip between two modes at the poll rate.
        self.hysteresis = hysteresis
        self._index = None
        self._reported = None

    def index(self):
        """Which position the switch is in now, or None if RC is unusable."""
        pwm = self.pwm()
        if pwm is None:
            return None
        span = self.pwm_max - self.pwm_min
        if span == 0:
            return None
        frac = (pwm - self.pwm_min) / span
        if self.reverse:
            frac = 1.0 - frac
        count = len(self.positions)
        # Continuous band coordinate: 0.0 at the bottom of band 0, count at
        # the top of the last one. Comparing against it, rather than against
        # a re-derived band number, is what lets the hysteresis be stated
        # once and still work across a switch flicked past two bands at once.
        where = max(0.0, min(float(count), frac * count))

        index = self._index
        if index is None:
            index = int(min(count - 1, where))
        else:
            while index < count - 1 and where >= index + 1 + self.hysteresis:
                index += 1
            while index > 0 and where < index - self.hysteresis:
                index -= 1
        self._index = index
        return index

    def position(self):
        """The value the switch currently selects, or None if RC is unusable."""
        index = self.index()
        return None if index is None else self.positions[index]

    def changed(self):
        """The newly selected position, or None if it has not moved."""
        index = self.index()
        if index is None or index == self._reported:
            return None
        self._reported = index
        return self.positions[index]
