"""Mapping between RC channel PWM, magnification, and VISCA zoom position.

The FCB-EV9520L's zoom position is not proportional to magnification. From
the technical manual's ratio table, position 0x2063 -- just over a third of
full-scale -- is already 3x, and the remaining two thirds of the range cover
3x through 30x. Driving position linearly from a linear input (an RC knob)
therefore spends most of the stick travel in the first few multiples and
crams 3x-30x into the top.

Three curves are offered:

    raw     position is linear in stick travel. Fast at the wide end,
            almost motionless near tele. Included mostly for comparison.
    ratio   magnification is linear in stick travel: 1x at bottom, 30x at
            top, 15.5x at centre. Even in absolute magnification.
    log     magnification is exponential in stick travel, so equal stick
            movement always multiplies the field of view by the same
            factor (~1.035x per 1% of travel). This is how a camera zoom
            rocker behaves and is the most natural to fly with, so it is
            the default.
"""
import math
from bisect import bisect_left

from fcb_base_driver.visca import ZOOM_OPTICAL_TELE_END, ZOOM_RATIO_TABLE

CURVES = ("log", "ratio", "raw")

MIN_RATIO = ZOOM_RATIO_TABLE[0][0]   # 1x
MAX_RATIO = ZOOM_RATIO_TABLE[-1][0]  # 30x

_RATIOS = [r for r, _ in ZOOM_RATIO_TABLE]
_POSITIONS = [p for _, p in ZOOM_RATIO_TABLE]


def ratio_to_position(ratio: float) -> int:
    """Interpolate the manual's ratio table: magnification -> VISCA position."""
    ratio = max(MIN_RATIO, min(MAX_RATIO, float(ratio)))
    i = bisect_left(_RATIOS, ratio)
    if i == 0:
        return _POSITIONS[0]
    if i >= len(_RATIOS):
        return _POSITIONS[-1]
    lo_r, hi_r = _RATIOS[i - 1], _RATIOS[i]
    lo_p, hi_p = _POSITIONS[i - 1], _POSITIONS[i]
    span = hi_r - lo_r
    frac = 0.0 if span == 0 else (ratio - lo_r) / span
    return int(round(lo_p + frac * (hi_p - lo_p)))


def position_to_ratio(position: int) -> float:
    """Inverse of ratio_to_position: VISCA position -> magnification."""
    position = max(0, min(ZOOM_OPTICAL_TELE_END, int(position)))
    i = bisect_left(_POSITIONS, position)
    if i == 0:
        return float(_RATIOS[0])
    if i >= len(_POSITIONS):
        return float(_RATIOS[-1])
    lo_p, hi_p = _POSITIONS[i - 1], _POSITIONS[i]
    lo_r, hi_r = _RATIOS[i - 1], _RATIOS[i]
    span = hi_p - lo_p
    frac = 0.0 if span == 0 else (position - lo_p) / span
    return lo_r + frac * (hi_r - lo_r)


def unit_to_position(unit: float, curve: str = "log") -> int:
    """Map a normalized 0.0-1.0 stick position onto a VISCA zoom position."""
    unit = max(0.0, min(1.0, float(unit)))
    if curve == "raw":
        return int(round(unit * ZOOM_OPTICAL_TELE_END))
    if curve == "ratio":
        return ratio_to_position(MIN_RATIO + unit * (MAX_RATIO - MIN_RATIO))
    if curve == "log":
        # Constant multiplicative rate: ratio = MAX_RATIO ** unit, which runs
        # 1x -> 30x as unit runs 0 -> 1 (MIN_RATIO is 1x, so no offset needed).
        return ratio_to_position(MAX_RATIO ** unit)
    raise ValueError(f"unknown zoom curve {curve!r}, expected one of {CURVES}")


def unit_to_ratio(unit: float, curve: str = "log") -> float:
    """The magnification a given stick position corresponds to."""
    return position_to_ratio(unit_to_position(unit, curve))


def position_to_unit(position: int, curve: str = "log") -> float:
    """Inverse of unit_to_position: which stick position holds this zoom.

    Handing control between commanders needs this. A teleop taking over from
    the RC knob has to start from a setpoint matching where the lens already
    is, or the first keypress would snap the lens across the range.
    """
    position = max(0, min(ZOOM_OPTICAL_TELE_END, int(position)))
    if curve == "raw":
        return position / ZOOM_OPTICAL_TELE_END
    ratio = position_to_ratio(position)
    if curve == "ratio":
        return (ratio - MIN_RATIO) / (MAX_RATIO - MIN_RATIO)
    if curve == "log":
        return math.log(ratio) / math.log(MAX_RATIO)
    raise ValueError(f"unknown zoom curve {curve!r}, expected one of {CURVES}")


#: Magnification step for a detented zoom knob. 0.5x divides 1x-30x into
#: 59 detents, which is fine enough to frame a target and coarse enough
#: that the same knob position gives the same shot twice.
DEFAULT_RATIO_STEP = 0.5


def snap_ratio(ratio: float, step: float) -> float:
    """Round magnification to the nearest multiple of `step`, within range.

    The grid is anchored at MIN_RATIO (1x), so a 0.5 step gives 1.0, 1.5,
    2.0 ... 30.0 -- round numbers, which is the point of asking for it.
    """
    if not step or step <= 0:
        return max(MIN_RATIO, min(MAX_RATIO, float(ratio)))
    ratio = max(MIN_RATIO, min(MAX_RATIO, float(ratio)))
    snapped = MIN_RATIO + round((ratio - MIN_RATIO) / step) * step
    return max(MIN_RATIO, min(MAX_RATIO, snapped))


def ratio_steps(step: float = DEFAULT_RATIO_STEP):
    """Every detent a given step size produces across 1x-30x."""
    if not step or step <= 0:
        return []
    out, ratio = [], MIN_RATIO
    while ratio <= MAX_RATIO + 1e-9:
        out.append(round(ratio, 6))
        ratio += step
    return out


class RatioQuantizer:
    """Turns a continuously variable zoom request into detents.

    Snapping alone is not enough. A knob parked on a boundary sits at, say,
    2.2499x one sample and 2.2501x the next, and plain rounding would flip
    the target between 2.0x and 2.5x for the rest of the flight -- the lens
    hunting back and forth, audibly, on video. So a detent is only left
    once the request is `hysteresis` of a step *past* the halfway point,
    which is what makes a physical detent feel sticky.

    Stateful, and deliberately so: the current detent is part of the
    answer. One instance per commander.
    """

    def __init__(self, step=DEFAULT_RATIO_STEP, hysteresis=0.15):
        self.step = float(step or 0.0)
        self.hysteresis = float(hysteresis)
        self.current = None

    @property
    def enabled(self) -> bool:
        return self.step > 0

    def reset(self, ratio=None):
        """Adopt a detent without moving through the ones in between.

        Used when something other than the knob has just placed the lens,
        so the next knob sample is judged against where the lens actually
        is rather than against a detent chosen a while ago.
        """
        self.current = None if ratio is None else snap_ratio(ratio, self.step)
        return self.current

    def snap(self, ratio: float) -> float:
        if not self.enabled:
            self.current = max(MIN_RATIO, min(MAX_RATIO, float(ratio)))
            return self.current
        ratio = max(MIN_RATIO, min(MAX_RATIO, float(ratio)))
        if self.current is None:
            self.current = snap_ratio(ratio, self.step)
        elif abs(ratio - self.current) >= self.step * (0.5 + self.hysteresis):
            self.current = snap_ratio(ratio, self.step)
        return self.current


def pwm_to_unit(pwm: int, pwm_min: int = 1000, pwm_max: int = 2000,
                reverse: bool = False) -> float:
    """Normalize an RC channel's PWM microseconds to 0.0-1.0.

    Returns 0.0 for a channel reading 0, which is how MAVLink reports a
    channel that is not present rather than one parked at its low end.
    """
    if pwm <= 0:
        return 0.0
    span = pwm_max - pwm_min
    if span == 0:
        return 0.0
    unit = (pwm - pwm_min) / span
    unit = max(0.0, min(1.0, unit))
    return 1.0 - unit if reverse else unit
