"""RC switch flick detection, kept separate from ROS so it can be unit-tested."""


class RcEdgeDetector:
    """Turns a stream of PWM readings into single 'flick' events.

    A flick is the switch going from LOW (< low_us) to HIGH (> high_us). The gap between
    the two thresholds is hysteresis, so a noisy reading near the middle can't fire twice.
    Holding the switch high never repeats, and a new flick is ignored for `debounce_s`.
    """

    def __init__(self, high_us: int = 1700, low_us: int = 1300, debounce_s: float = 0.3):
        self.high_us = high_us
        self.low_us = low_us
        self.debounce_s = debounce_s
        self._state = None          # None until the first clear reading, then 'low' / 'high'
        self._last_fire = -1e9

    def update(self, pwm: int, now_s: float) -> bool:
        """Feed one reading; returns True exactly once per flick."""
        if pwm <= 0:                # channel missing / receiver lost
            return False
        if pwm >= self.high_us:
            new = 'high'
        elif pwm <= self.low_us:
            new = 'low'
        else:
            return False            # in the hysteresis band: keep the previous state
        fired = (self._state == 'low' and new == 'high'
                 and now_s - self._last_fire >= self.debounce_s)
        self._state = new
        if fired:
            self._last_fire = now_s
        return fired
