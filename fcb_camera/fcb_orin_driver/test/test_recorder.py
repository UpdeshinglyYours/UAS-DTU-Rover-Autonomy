"""The fcb_orin regression tests, against the ROS port of the recorder.

Each one encodes a fault that happened in flight (see fcb_orin/test): a
recorder that could never arm and never said so, a camera outage that took
the telemetry track with it, a stop that chopped a flight in two, a full
disk that killed the process, filled frames that spiralled.
"""
import csv
import io
import logging
import os
import threading
import time
import types
from datetime import datetime, timezone

import numpy as np
import pytest

from fcb_base_driver import visca
from fcb_orin_driver import recorder as fr

FRAME = np.zeros((64, 64, 3), "uint8")


class Grab:
    """A camera that can be made to stop delivering."""

    def __init__(self):
        self.seq = 0
        self.alive = True

    def wait_for_frame(self, after, timeout=1.0):
        time.sleep(0.005)
        if not self.alive:
            return None, after, None
        self.seq += 1
        return FRAME, self.seq, time.monotonic()

    def fps(self):
        return 30.0

    def stop(self):
        pass


class Mav:
    def __init__(self, pwm=1900):
        self.pwm = pwm
        self.alive = True

    def rc_age(self):
        return 0.1 if self.alive else None

    def channel(self, n):
        return self.pwm

    def telemetry(self):
        return fr._NO_TELEMETRY

    def is_connected(self):
        return self.alive


def make(tmp_path, mavlink=None, **overrides):
    args = dict(
        rec_channel=8, rec_channel_threshold=1500, rec_channel_reverse=False,
        rc_timeout=2.0, rec_debounce=0.3, camera_timeout=0.4,
        record_dir=str(tmp_path), container="avi", encoder="cpu", bitrate=5.0,
        fps=30.0, telemetry_interval=0.05, snap_channel=0, icr_channel=0,
        status_interval=0.25, log_status_interval=99.0, fc_source="pymavlink",
        no_mavlink=False, state_file="", board_power_cycle_cmd="",
        rc_channel=7, stabilizer="on")
    args.update(overrides)
    r = fr.Recorder(types.SimpleNamespace(**args))
    r.mavlink = mavlink
    r.declared_fps = 30.0
    r.poll_rc_switches = lambda: None
    r.update_display = lambda now: None
    r.open_camera = lambda quiet=False: False   # a lost camera stays lost
    return r


def run_in_background(r):
    th = threading.Thread(target=r.capture_loop, daemon=True)
    th.start()
    return th


def stop_on_loop(r, **kwargs):
    """Stop from the capture thread, the way a service call does."""
    return r.submit(lambda: r.request_stop_recording(**kwargs), timeout=5)


# -- resilience --------------------------------------------------------

def test_camera_dies_mid_recording_telemetry_keeps_going(tmp_path):
    mav = Mav()
    r = make(tmp_path, mav)
    r.grabber = Grab()
    th = run_in_background(r)
    time.sleep(1.0)
    assert r.recording, "ch8 high did not start a recording"
    rows_before = r.csv.rows
    r.grabber.alive = False
    time.sleep(1.6)
    assert r.recording, "the flight ended with the camera"
    assert r.video is None, "the video segment was not closed at the break"
    assert r.csv.rows > rows_before, "the CSV stopped when video did"
    r.running = False
    th.join(5)
    r.stop_recording()


def test_ch8_low_stops_and_high_restarts(tmp_path):
    mav = Mav()
    r = make(tmp_path, mav)
    r.grabber = Grab()
    th = run_in_background(r)
    time.sleep(0.8)
    assert r.recording
    mav.pwm = 1000
    time.sleep(1.0)
    assert not r.recording, "the RC stop did not take effect"
    mav.pwm = 1900
    time.sleep(0.8)
    assert r.recording

    ok, _, base = stop_on_loop(r)
    assert ok and base
    time.sleep(0.8)
    assert not r.recording, "a stop with ch8 still high chopped the flight in two"
    r.running = False
    th.join(5)


def test_mavlink_dies_video_keeps_going(tmp_path):
    mav = Mav()
    r = make(tmp_path, mav)
    r.grabber = Grab()
    th = run_in_background(r)
    time.sleep(0.8)
    frames_before = r.video.frames
    mav.alive = False
    time.sleep(1.2)
    assert r.recording
    assert r.video is not None and r.video.frames > frames_before
    stop_on_loop(r)
    r.running = False
    th.join(5)


def test_csv_marks_gap_rows(tmp_path):
    mav = Mav()
    r = make(tmp_path, mav)
    r.grabber = Grab()
    th = run_in_background(r)
    time.sleep(0.8)
    r.grabber.alive = False
    time.sleep(0.8)
    stop_on_loop(r)
    r.running = False
    th.join(5)
    gap = frame = 0
    for name in os.listdir(tmp_path):
        if name.endswith(".csv"):
            with open(tmp_path / name) as fh:
                for row in csv.DictReader(fh):
                    if row["frame"] == "":
                        gap += 1
                    else:
                        frame += 1
    assert frame > 0 and gap > 0


def test_camera_returns_as_a_new_segment_of_the_same_session(tmp_path):
    r = make(tmp_path, Mav())
    r.grabber = Grab()
    attempts = {"n": 0}

    def reopen(quiet=False):
        attempts["n"] += 1
        if attempts["n"] < 3:
            return False
        r.grabber = Grab()
        return True
    r.open_camera = reopen
    th = run_in_background(r)
    time.sleep(0.8)
    assert r.segment == 1 and r.video is not None
    base = r.base
    r.grabber.alive = False
    time.sleep(0.9)
    assert r.video is None and r.recording
    time.sleep(2.5)
    assert r.segment >= 2 and r.video is not None and r.base == base
    stop_on_loop(r)
    time.sleep(0.8)
    assert not r.recording
    r.running = False
    th.join(5)
    files = sorted(os.listdir(tmp_path))
    assert len([f for f in files if f.endswith(".avi")]) == 2
    assert len([f for f in files if f.endswith(".csv")]) == 1


def test_sighup_is_ignored():
    import inspect
    from fcb_orin_driver import node
    assert "signal.signal(signal.SIGHUP, signal.SIG_IGN)" in inspect.getsource(node)


# -- the switch --------------------------------------------------------

def test_debounce(tmp_path):
    r = make(tmp_path, Mav(1000))
    t = 100.0
    r.apply_record_switch(t, None, None)
    r.apply_record_switch(t + 0.5, None, None)
    r.mavlink.pwm = 1900
    r.apply_record_switch(t + 0.6, None, None)
    r.mavlink.pwm = 1000
    r.apply_record_switch(t + 0.65, None, None)
    r.apply_record_switch(t + 1.2, None, None)
    assert not r.recording, "a 50 ms glitch high started a recording"

    r.mavlink.pwm = 1900
    r.apply_record_switch(t + 2.0, None, None)
    assert not r.recording
    r.apply_record_switch(t + 2.35, None, None)
    assert r.recording

    r.mavlink.pwm = 1000
    r.apply_record_switch(t + 3.0, None, None)
    r.mavlink.pwm = 1900
    r.apply_record_switch(t + 3.05, None, None)
    r.apply_record_switch(t + 3.5, None, None)
    assert r.recording, "a glitch low stopped it"

    r.mavlink.pwm = 1000
    r.apply_record_switch(t + 4.0, None, None)
    r.apply_record_switch(t + 4.4, None, None)
    assert not r.recording
    assert any(n.endswith(".csv") for n in os.listdir(tmp_path))


def test_stale_rc_holds(tmp_path):
    r = make(tmp_path, Mav(1900))
    r.apply_record_switch(10.0, None, None)
    r.apply_record_switch(10.4, None, None)
    r.mavlink.rc_age = lambda: 9.0
    r.mavlink.pwm = 1000
    r.apply_record_switch(11.0, None, None)
    r.apply_record_switch(12.0, None, None)
    assert r.recording, "RC lost mid-flight ended the recording"
    r.stop_recording()


def test_write_failure_stops_cleanly_and_blocks_restarts(tmp_path):
    r = make(tmp_path, Mav(1900))
    r.apply_record_switch(20.0, None, None)
    r.apply_record_switch(20.4, None, None)
    r.on_write_failure(OSError(28, "No space left on device"))
    assert not r.recording
    r.apply_record_switch(21.0, None, None)
    r.apply_record_switch(21.5, None, None)
    assert not r.recording
    ok, _ = r.request_start_recording()
    assert not ok and not r.recording


def test_by_hand_with_no_flight_controller(tmp_path):
    r = make(tmp_path, None)
    ok, _ = r.request_start_recording()
    assert ok and r.recording
    for i in range(10):
        r.apply_record_switch(30.0 + i, None, None)
    assert r.recording, "no flight controller stopped a recording started by hand"
    ok, _, base = r.request_stop_recording()
    assert ok and not r.recording and base


def test_by_hand_is_not_stopped_by_ch8_sitting_low(tmp_path):
    r = make(tmp_path, Mav(1000))
    r.apply_record_switch(40.0, None, None)
    r.apply_record_switch(40.5, None, None)
    r.request_start_recording()
    r.apply_record_switch(41.0, None, None)
    r.apply_record_switch(42.0, None, None)
    assert r.recording
    r.mavlink.pwm = 1900
    r.apply_record_switch(43.0, None, None)
    r.apply_record_switch(43.5, None, None)
    r.mavlink.pwm = 1000
    r.apply_record_switch(44.0, None, None)
    r.apply_record_switch(44.5, None, None)
    assert not r.recording, "ch8 flicked up and down did not stop it"
    r.apply_record_switch(45.0, None, None)
    assert not r.recording


def test_late_autopilot_gets_the_zoom_knob(tmp_path, monkeypatch):
    r = make(tmp_path, None, stream_rate=10, rc_pwm_min=1000, rc_pwm_max=2000,
             rc_deadband=8, rc_reverse=False)
    handed = []
    r.servo = types.SimpleNamespace(rc=None, manual_ratio=None,
                                    hand_to_rc=lambda: handed.append(True))
    made = []

    class FakeSource:
        def __init__(self, **kw):
            made.append(kw)
    monkeypatch.setattr(fr, "MavlinkSource", FakeSource)
    r._attach_mavlink("/dev/serial/by-id/usb-CubePilot", 115200)
    assert r.mavlink is not None and made
    assert r.servo.rc is r.rc and r.rc is not None
    assert handed, "the knob was attached but never handed control"


# -- no flight controller ----------------------------------------------

def test_no_fc_is_unknown_not_no(tmp_path):
    r = make(tmp_path, None)
    assert r.wants_recording() is None
    text = r.status_text(time.monotonic())
    assert text.startswith("NO FC -- r TO RECORD"), text
    assert "NO FC" not in make(tmp_path, Mav()).status_text(time.monotonic())
    assert "NO FC" not in make(tmp_path, None, fc_source="none",
                               no_mavlink=True).status_text(time.monotonic())


def test_no_fc_keeps_shouting(tmp_path):
    buf = io.StringIO()
    handler = logging.StreamHandler(buf)
    log = logging.getLogger("fcb")
    log.addHandler(handler)
    log.setLevel(logging.DEBUG)
    try:
        r = make(tmp_path, None)
        r.grabber = Grab()

        def stopper():
            time.sleep(11.5)
            r.running = False
        threading.Thread(target=stopper, daemon=True).start()
        r.capture_loop()
    finally:
        log.removeHandler(handler)
    out = buf.getvalue()
    assert out.count("NO FLIGHT CONTROLLER") >= 2
    assert "channel 8" in out and "nothing is being recorded" in out


# -- frames ------------------------------------------------------------

class FakeVideo:
    def __init__(self, fps):
        self.fps, self.frames, self.written = fps, 0, []

    def write(self, f):
        self.written.append(f)
        self.frames += 1


class FakeSheet:
    def __init__(self):
        self.rows = []

    def write(self, n, wall, t, tel, servo, now, filled=0):
        self.rows.append((n, filled))


def _frame_recorder(tmp_path):
    r = make(tmp_path, None)
    r.record_started_mono = 0.0
    r.record_started_wall = datetime.now(timezone.utc)
    return r


def test_lost_frames_are_filled(tmp_path):
    r = _frame_recorder(tmp_path)
    video, sheet = FakeVideo(60.0), FakeSheet()
    for i, t in enumerate([0.0, 1 / 60, 2 / 60, 6 / 60, 7 / 60]):
        r.write_frame(video, sheet, f"frame{i}", t, t)
    assert video.frames == 8
    assert video.written[3:6] == ["frame2"] * 3
    assert [f for _, f in sheet.rows] == [0, 0, 0, 1, 1, 1, 0, 0]
    assert [n for n, _ in sheet.rows] == list(range(8))


def test_long_outage_and_slow_loop_are_not_filled(tmp_path):
    r = _frame_recorder(tmp_path)
    video, sheet = FakeVideo(60.0), FakeSheet()
    r.write_frame(video, sheet, "a", 10.0, 10.0)
    r.write_frame(video, sheet, "b", 20.0, 20.0)
    assert video.frames <= 5

    r._last_written = r._last_written_at = None
    video, sheet = FakeVideo(60.0), FakeSheet()
    r.write_frame(video, sheet, "a", 0.0, 0.0)
    r.write_frame(video, sheet, "b", 5 / 60, 5 / 60, seq_gap=5)
    assert video.frames == 2, "the fill spiral is back"

    r._last_written = r._last_written_at = None
    video, sheet = FakeVideo(60.0), FakeSheet()
    t = 0.0
    for dt in [0.0104, 0.0104, 0.0125, 0.0271, 0.0104, 0.0208, 0.0167] * 20:
        t += dt
        r.write_frame(video, sheet, "f", t, t)
    assert sum(1 for _, f in sheet.rows if f) == 0


def test_stuck_board_power_cycled_at_most_every_90s(tmp_path):
    marker = tmp_path / "cycled"
    r = make(tmp_path, None, board_power_cycle_cmd=f"echo x >> {marker}")
    r.board_is_stuck("test")
    r.board_is_stuck("test again")
    assert marker.read_text().count("x") == 1
    r = make(tmp_path, None)
    r.board_is_stuck("no command")
    r.board_is_stuck("no command")
    assert r._board_advice_given


# -- naming ------------------------------------------------------------

def _session(tmp_path, segments=1, extra=()):
    base = str(tmp_path / "fcb-20260909-120000")
    open(base + ".csv", "w").write("frame\n")
    open(base + ".avi", "w").write("v")
    for i in range(2, segments + 1):
        open(f"{base}-{i}.avi", "w").write("v")
    for name in extra:
        (tmp_path / name).write_text("x")
    return make(tmp_path, None), base


def test_rename_every_file(tmp_path):
    r, base = _session(tmp_path, segments=2)
    ok, _, new_base, files = r.rename_recording(base, "morning pass 3")
    assert ok and new_base.endswith("morning_pass_3") and len(files) == 3
    assert sorted(os.listdir(tmp_path)) == [
        "morning_pass_3-2.avi", "morning_pass_3.avi", "morning_pass_3.csv"]


def test_rename_refuses_junk_and_traversal(tmp_path):
    r, base = _session(tmp_path)
    ok, *_ = r.rename_recording(base, "!!!")
    assert not ok and "fcb-20260909-120000.avi" in os.listdir(tmp_path)
    ok, *_ = r.rename_recording(base, "../../etc/passwd")
    assert ok and "passwd.avi" in os.listdir(tmp_path)


def test_rename_never_overwrites(tmp_path):
    r, base = _session(tmp_path, segments=2, extra=("sortie.avi", "sortie.csv"))
    r.rename_recording(base, "sortie")
    names = os.listdir(tmp_path)
    assert (tmp_path / "sortie.avi").read_text() == "x"
    assert {"sortie-2.avi", "sortie-2.csv", "sortie-2-2.avi"} <= set(names)


def test_rename_leaves_unrelated_files(tmp_path):
    r, base = _session(tmp_path, extra=("fcb-20260909-120000-notes.txt", "other.avi"))
    r.rename_recording(base, "flight")
    left = set(os.listdir(tmp_path))
    assert {"fcb-20260909-120000-notes.txt", "other.avi",
            "flight.avi", "flight.csv"} <= left


# -- stabilizer --------------------------------------------------------

class FakeLink:
    address = 1

    def __init__(self, ignore_commands=False, dead=False):
        self.state = 0x03
        self.ignore_commands = ignore_commands
        self.dead = dead
        self.sent = []

    def command(self, packet, **kwargs):
        if self.dead:
            raise fr.ViscaTimeout("no complete frame")
        self.sent.append(packet)
        if packet[:4] == bytes([0x81, 0x01, 0x04, 0x34]) and not self.ignore_commands:
            self.state = packet[4]

    def inquiry(self, packet, **kwargs):
        if self.dead:
            raise fr.ViscaTimeout("no complete frame")
        return bytes([0x50, self.state])


@pytest.mark.parametrize("wanted,start,end,sends", [
    ("on", 0x03, 0x02, True), ("off", 0x02, 0x03, True), ("keep", 0x03, 0x03, False)])
def test_stabilizer(tmp_path, wanted, start, end, sends):
    link = FakeLink()
    link.state = start
    make(tmp_path, None, stabilizer=wanted).apply_stabilizer(link)
    assert link.state == end
    assert bool(link.sent) == sends


def test_stabilizer_failures_are_not_fatal(tmp_path):
    make(tmp_path, None).apply_stabilizer(FakeLink(ignore_commands=True))
    make(tmp_path, None).apply_stabilizer(FakeLink(dead=True))
    assert visca.image_stabilizer(True)[4] == 0x02


# -- the ROS side's plumbing -------------------------------------------

def test_commands_run_on_the_capture_thread(tmp_path):
    r = make(tmp_path, None)
    r.grabber = Grab()
    th = run_in_background(r)
    seen = r.submit(lambda: threading.current_thread() is th, timeout=5)
    r.running = False
    th.join(5)
    assert seen


def test_commands_time_out_when_the_loop_is_not_running(tmp_path):
    r = make(tmp_path, None)
    with pytest.raises(TimeoutError):
        r.submit(lambda: True, timeout=0.2)


def test_icr_names_and_aliases():
    assert fr.resolve_icr("RGB") == "day"
    assert fr.resolve_icr("rgb+ir") == "night_color"
    assert fr.resolve_icr("ir") == "night"
    assert fr.resolve_icr("auto") == "auto"
    assert fr.resolve_icr("colour") == "day"
    with pytest.raises(ValueError):
        fr.resolve_icr("thermal")


def test_two_channel_frames_are_decoded():
    yuyv = np.zeros((4, 4, 2), "uint8")
    assert fr.to_bgr(yuyv).shape == (4, 4, 3)
    assert fr.to_bgr(np.zeros((4, 4), "uint8")).shape == (4, 4, 3)
