"""lat/lon <-> metres, the same WGS-84 flat-earth math as ~/mission_planner_live.py.

The rover's `map` frame is the MAVROS local ENU frame, whose (0, 0) is the EKF
origin. So a casualty at (lat, lon) is at
    x, y = latlon_to_meter_offset(origin_lat, origin_lon, lat, lon)
in `map`, where (origin_lat, origin_lon) is that EKF origin.
"""

import math
from typing import Tuple

WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = 2.0 * WGS84_F - WGS84_F * WGS84_F


def _radii(lat0_rad: float) -> Tuple[float, float]:
    sin_lat0 = math.sin(lat0_rad)
    denom = math.sqrt(1.0 - WGS84_E2 * sin_lat0 * sin_lat0)
    rn = WGS84_A / denom                                   # prime vertical radius
    rm = WGS84_A * (1.0 - WGS84_E2) / (denom * denom * denom)  # meridian radius
    return rn, rm


def latlon_to_meter_offset(lat0: float, lon0: float, lat: float, lon: float) -> Tuple[float, float]:
    """(east, north) in metres of (lat, lon) relative to (lat0, lon0)."""
    lat0_rad = math.radians(lat0)
    rn, rm = _radii(lat0_rad)
    east_m = (math.radians(lon) - math.radians(lon0)) * rn * math.cos(lat0_rad)
    north_m = (math.radians(lat) - lat0_rad) * rm
    return east_m, north_m


def meter_offset_to_latlon(lat0: float, lon0: float, east_m: float, north_m: float) -> Tuple[float, float]:
    """Inverse of latlon_to_meter_offset."""
    lat0_rad = math.radians(lat0)
    rn, rm = _radii(lat0_rad)
    dlat = north_m / rm
    dlon = east_m / (rn * math.cos(lat0_rad))
    return math.degrees(lat0_rad + dlat), math.degrees(math.radians(lon0) + dlon)
