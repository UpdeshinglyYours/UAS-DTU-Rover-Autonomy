"""Orbit progress around a casualty, cut into fixed-size segments (no ROS, unit-tested).

casualty_models_bridge uses this to tell the frame extractor "next segment" every
`segment_deg` of bearing the rover sweeps around the casualty, so each quarter of the
circle becomes its own extractor segment (and its own K crops).
"""

import math


class SweepTracker:
    """Accumulates the bearing swept around (cx, cy) in either direction.

    update(x, y) returns True each time another `segment_deg` has been completed.
    A jump larger than `max_step_deg` between two updates (pose glitch, relocalisation)
    is ignored rather than counted.
    """

    def __init__(self, cx: float, cy: float, segment_deg: float = 90.0,
                 max_step_deg: float = 30.0):
        self.cx, self.cy = cx, cy
        self.segment = math.radians(max(segment_deg, 1.0))
        self.max_step = math.radians(max_step_deg)
        self.swept = 0.0              # rad, always >= 0
        self.segments_done = 0
        self._prev = None

    def update(self, x: float, y: float) -> bool:
        b = math.atan2(y - self.cy, x - self.cx)
        prev, self._prev = self._prev, b
        if prev is None:
            return False
        step = abs(math.atan2(math.sin(b - prev), math.cos(b - prev)))
        if step > self.max_step:
            return False
        self.swept += step
        if self.swept >= (self.segments_done + 1) * self.segment:
            self.segments_done += 1
            return True
        return False

    @property
    def swept_deg(self) -> float:
        return math.degrees(self.swept)
