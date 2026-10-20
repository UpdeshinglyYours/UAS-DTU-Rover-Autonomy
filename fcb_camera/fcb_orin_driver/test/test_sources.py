"""The MAVROS source, and the terminal key reader's EOF handling."""
import math
import os
import types

import pytest

from fcb_base_driver.rc_switch import RcButton, RcSelector
from fcb_orin_driver.mavros_source import MavrosSource, enu_quaternion_to_ned_rpy


def quat_from_enu_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


@pytest.mark.parametrize("enu_yaw,ned_yaw", [(90, 0), (0, 90), (180, -90), (-90, 180)])
def test_heading_east_north_conversion(enu_yaw, ned_yaw):
    _, _, yaw = enu_quaternion_to_ned_rpy(*quat_from_enu_rpy(0, 0, math.radians(enu_yaw)))
    assert abs(((yaw - ned_yaw) + 180) % 360 - 180) < 1e-6


def test_nose_up_is_positive_pitch_and_right_wing_down_positive_roll():
    # ENU/FLU: nose up is negative rotation about +Y (left); NED/FRD flips it.
    _, pitch, _ = enu_quaternion_to_ned_rpy(*quat_from_enu_rpy(0, math.radians(-10), 0))
    assert abs(pitch - 10) < 1e-6
    roll, _, _ = enu_quaternion_to_ned_rpy(*quat_from_enu_rpy(math.radians(15), 0, 0))
    assert abs(roll - 15) < 1e-6


class FakeNode:
    def __init__(self):
        self.subs = {}

    def create_subscription(self, kind, topic, callback, qos):
        self.subs[topic] = callback
        return topic

    def destroy_subscription(self, sub):
        self.subs.pop(sub, None)


def test_mavros_source_answers_like_mavlink_source():
    pytest.importorskip("mavros_msgs")
    node = FakeNode()
    src = MavrosSource(node, "/mavros")
    assert src.rc_age() is None and src.channel(8) == 0

    rc = types.SimpleNamespace(channels=[1500] * 7 + [1900, 65535, 1000])
    node.subs["/mavros/rc/in"](rc)
    assert src.rc_age() is not None and src.rc_age() < 1.0
    assert src.channel(8) == 1900
    assert src.channel(9) == 0, "UINT16_MAX must read as absent, not switch high"
    assert src.channel(18) == 0

    fix = types.SimpleNamespace(status=types.SimpleNamespace(status=-1),
                                latitude=0.0, longitude=0.0)
    node.subs["/mavros/global_position/global"](fix)
    node.subs["/mavros/global_position/rel_alt"](types.SimpleNamespace(data=12.5))
    node.subs["/mavros/vfr_hud"](types.SimpleNamespace(heading=270, groundspeed=4.0,
                                                       altitude=230.0))
    t = src.telemetry()
    assert t.lat_deg is None and t.alt_rel_m == 12.5 and t.alt_msl_m == 230.0
    assert t.heading_deg == 270 and t.position_age_s is not None

    node.subs["/mavros/state"](types.SimpleNamespace(connected=True))
    assert src.is_connected()

    # The existing switch logic runs on it unchanged.
    button = RcButton(src, 10)
    assert not button.pressed()
    node.subs["/mavros/rc/in"](types.SimpleNamespace(channels=[1500] * 9 + [2000]))
    assert button.pressed()
    selector = RcSelector(src, 1, ["a", "b", "c"])
    assert selector.changed() == "b"
    src.stop()
    assert not node.subs


def test_keys_retire_on_eof():
    from fcb_orin_driver.keys import TerminalKeys
    read_fd, write_fd = os.pipe()
    os.close(write_fd)                       # the pane closed: EOF for ever
    retired = []
    keys = TerminalKeys(on_retire=retired.append)
    keys.enabled, keys._fd = True, read_fd
    for _ in range(1000):
        if not keys.pending():
            break
        keys.get()
    assert retired == ["the terminal closed"] and not keys.enabled
    os.close(read_fd)


def test_live_line_survives_a_dead_terminal():
    import io
    from fcb_orin_driver.keys import LiveLine

    class Dead(io.StringIO):
        def isatty(self):
            return True

        def write(self, s):
            raise OSError(5, "Input/output error")
    line = LiveLine(Dead())
    line.show("status")
    line.clear()
    line.show("again")
    assert not line.enabled
