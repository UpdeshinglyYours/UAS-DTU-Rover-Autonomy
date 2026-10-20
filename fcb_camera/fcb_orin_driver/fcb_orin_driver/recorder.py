"""The flight recorder from fcb_orin/fcb_record.py, without the terminal.

Everything fcb_record.py did to keep a flight on disk is here unchanged:
camera autodetection by USB identity, the Oppila board's repeated frames
and misreported rate, YUYV/UYVY decoding, reopening a dead camera with
back-off, the stuck-board power-cycle hook, the blank-picture watch, the
VISCA link found by protocol and rebuilt when it dies, the image
stabilizer re-applied on every link, the detented zoom servo, the RC
switches, NVENC with a CPU fallback, AVI segments inside one session with
one CSV, filled frames, gap rows, write failures and the disk watch.

What is gone is the tmux pane. The keyboard became ROS services (see
node.py; fcb_keys gives the same keys back in a terminal), the live status
line became a topic, and the snapshot preview is published as a JPEG.

Services arrive on executor threads, but the recorder's state was written
to be touched from one thread -- the capture loop -- and the keyboard was
read inside that loop for exactly that reason. So a service does not act
on the recorder directly: it queues a function, and the capture loop runs
it between frames, where the keys used to be read (submit / poll_commands).
"""
import csv
import glob
import logging
import os
import queue
import re
import shutil
import subprocess
import sys
import threading
import time
from concurrent.futures import Future
from concurrent.futures import TimeoutError as FuturesTimeoutError
from datetime import datetime, timedelta, timezone

from fcb_orin_driver import prefer_cv2  # noqa: F401  -- before cv2
import cv2

from fcb_base_driver import devices, visca, zoom_map
from fcb_base_driver.frame_grabber import FrameGrabber
from fcb_base_driver.mavlink_source import MavlinkSource
from fcb_base_driver.rc_source import RcZoomSource
from fcb_base_driver.rc_switch import RcButton, RcSelector
from fcb_base_driver.visca_link import ViscaError, ViscaLink, ViscaTimeout
from fcb_base_driver.zoom_servo import ZoomServo
from fcb_orin_driver import snapshot

log = logging.getLogger("fcb")

CSV_COLUMNS = [
    "frame",              # index into the video, 0-based
    "t_wall_utc",         # wall clock at capture, ISO 8601
    "t_mono_s",           # seconds since recording started
    "lat_deg", "lon_deg",
    "alt_msl_m",          # above mean sea level
    "alt_rel_m",          # above the home/arming point
    "gps_age_ms",
    "roll_deg", "pitch_deg", "yaw_deg",
    "att_age_ms",
    "heading_deg", "groundspeed_ms",
    "hud_age_ms",
    "zoom_ratio",         # e.g. 5.7 for 5.7x
    "zoom_position",      # raw VISCA counts, lossless
    "zoom_age_ms",
    "fc_time_boot_ms",    # the Pixhawk's clock, to align with its .bin log
    "filled",             # 1: a repeat of the previous frame, written to
                          # cover frames the camera link lost, keeping the
                          # video in real time; 0: a captured frame
]

#: Imaging modes, in the operator's vocabulary rather than Sony's. The
#: FCB-EV9520L has one sensor behind a removable IR cut filter; "IR" means
#: the filter is out. visca.ICR_MODES does the translation to ICR On/Off.
#:
#: Each entry is (key, name shown, ICR mode, what it does).
ICR_MODES = (
    ("1", "RGB",    "day",         "daylight colour, IR cut filter in"),
    ("2", "IR",     "night",       "filter out, infrared-sensitive mono"),
    ("3", "RGB+IR", "night_color", "filter out, colour retained"),
    ("4", "AUTO",   "auto",        "camera switches on scene brightness"),
)

ICR_BY_KEY = {key: mode for key, _, mode, _ in ICR_MODES}
ICR_BY_NAME = {name.lower(): mode for _, name, mode, _ in ICR_MODES}
ICR_LABELS = {mode: f"{name} ({what})" for _, name, mode, what in ICR_MODES}
ICR_SHORT = {mode: name for _, name, mode, _ in ICR_MODES}
ICR_ORDER = [mode for _, _, mode, _ in ICR_MODES]


def resolve_icr(text):
    """rgb / ir / rgb+ir / auto, or any visca alias -> ICR mode name."""
    text = str(text).strip().lower()
    if text in ICR_BY_NAME:
        return ICR_BY_NAME[text]
    return visca.normalize_icr_mode(text)   # raises ValueError if unknown


#: Container defaults. AVI is the default because of how the two behave
#: when a write is cut short -- a power cut, a kill -9, a card pulled. An
#: mp4 keeps its index (the moov atom) at the end, so a truncated one is
#: not a short video, it is no video at all. AVI's frames are recoverable
#: without the index. Measured on the drone by truncating a 60-frame clip
#: to 75%: 43 frames came back out of the AVI and 0 out of the mp4.
CONTAINERS = {
    #             GStreamer muxer, OpenCV fourcc for the CPU fallback
    "avi": ("avimux", "XVID"),
    "mp4": ("qtmux", "mp4v"),
}

#: Seconds between looks for a control port or autopilot that did not
#: answer at launch.
CONTROL_RETRY = 10.0


def to_bgr(frame, yuv_code=cv2.COLOR_YUV2BGR_YUY2):
    """Whatever the camera handed over, as three-channel BGR.

    With CAP_PROP_CONVERT_RGB off OpenCV passes 4:2:2 through as a
    *two*-channel image, which the encoder would accept and write as
    nonsense. `yuv_code` says which packing those pairs are: YUYV on the
    NeoHD board, UYVY on the Oppila one.
    """
    if frame.ndim == 2:
        return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    channels = frame.shape[2]
    if channels == 3:
        return frame
    if channels == 2:
        return cv2.cvtColor(frame, yuv_code)
    if channels == 4:
        return cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
    return frame


def unique_base(base):
    """A base path no existing recording is already using.

    The timestamp only resolves to the second, so a stop and an immediate
    restart produces the same name twice. Letters for the suffix, because
    digits are what the video segments of one session use.
    """
    if not glob.glob(glob.escape(base) + ".*"):
        return base
    for letter in "bcdefghijklmnopqrstuvwxyz":
        candidate = f"{base}{letter}"
        if not glob.glob(glob.escape(candidate) + ".*"):
            return candidate
    return f"{base}-{int(time.time() * 1000) % 100000}"


def _delta(seconds):
    return timedelta(seconds=seconds)


class VideoRecorder:
    """Writes frames to a video file, on the GPU where one is available.

    Software encoding 1080p60 is roughly 250 MB/s of raw input, which an
    Orin's CPU will not keep up with. nvv4l2h264enc hands that to the
    hardware encoder. The CPU path stays available, and is fallen back to
    loudly, since the difference shows up as dropped frames, not an error.
    """

    def __init__(self, path, width, height, fps, encoder="nvenc", bitrate_mbps=25):
        self.path = path
        extension = os.path.splitext(path)[1].lstrip(".").lower()
        self.muxer, self.fourcc = CONTAINERS.get(extension, CONTAINERS["avi"])
        self.width = width
        self.height = height
        self.fps = fps
        self.frames = 0
        self.started = time.monotonic()

        self.encoder, self.writer = self._open(encoder, bitrate_mbps)
        if self.writer is None:
            raise RuntimeError(f"could not open any video writer for {path}")

    def _open(self, requested, bitrate_mbps):
        if requested in ("nvenc", "auto"):
            writer = self._open_nvenc(bitrate_mbps)
            if writer is not None:
                return "nvenc", writer
            log.warning(
                "NVENC/GStreamer writer would not open -- falling back to "
                "CPU %s. Expect dropped frames at 1080p60. Check that this "
                "OpenCV was built with GStreamer "
                "(cv2.getBuildInformation()).", self.fourcc
            )
        # Whole frames per second here too: a measured rate like 96.67
        # reaches FFmpeg's mpeg4 encoder as a time base it rejects, so the
        # CPU fallback opened or failed at random.
        writer = cv2.VideoWriter(
            self.path, cv2.VideoWriter_fourcc(*self.fourcc),
            float(max(1, int(round(self.fps)))), (self.width, self.height),
        )
        return ("cpu", writer) if writer.isOpened() else ("cpu", None)

    def _open_nvenc(self, bitrate_mbps):
        # The framerate has to be a whole number: a measured 56.39 arrives
        # in the appsrc caps as 5639/100, which nvv4l2h264enc will not
        # negotiate -- the pipeline opens, then refuses every buffer and
        # leaves an empty file while the counters carry on.
        fps = max(1, int(round(self.fps)))
        pipeline = (
            f"appsrc ! video/x-raw,format=BGR,width={self.width},"
            f"height={self.height},framerate={fps}/1 "
            f"! queue ! videoconvert ! video/x-raw,format=NV12 "
            f"! nvvidconv ! video/x-raw(memory:NVMM),format=NV12 "
            f"! nvv4l2h264enc bitrate={int(bitrate_mbps * 1_000_000)} "
            f"insert-sps-pps=1 maxperf-enable=1 "
            f"! h264parse ! {self.muxer} ! filesink location={self.path}"
        )
        log.debug("trying GStreamer pipeline: %s", pipeline)
        try:
            writer = cv2.VideoWriter(
                pipeline, cv2.CAP_GSTREAMER, 0, float(fps),
                (self.width, self.height),
            )
        except Exception as exc:
            log.debug("GStreamer writer raised: %s", exc)
            return None
        return writer if writer.isOpened() else None

    def write(self, frame):
        self.writer.write(frame)
        self.frames += 1

    def close(self):
        self.writer.release()
        return time.monotonic() - self.started


class TelemetryCsv:
    """One row per recorded frame, flushed as it goes, so a power cut
    mid-flight leaves a readable file up to the last moment."""

    def __init__(self, path):
        self.path = path
        self.rows = 0
        self._file = open(path, "w", newline="")
        self._writer = csv.writer(self._file)
        self._writer.writerow(CSV_COLUMNS)
        self._file.flush()

    def write(self, frame_number, wall_utc, t_mono, telemetry, servo, now,
              filled=0):
        def ms(age):
            return None if age is None else round(age * 1000, 1)

        def num(value, digits):
            return None if value is None else round(value, digits)

        zoom_age = None if servo.updated_at is None else now - servo.updated_at
        self._writer.writerow([
            frame_number,
            wall_utc.isoformat(timespec="milliseconds"),
            round(t_mono, 4),
            num(telemetry.lat_deg, 7), num(telemetry.lon_deg, 7),
            num(telemetry.alt_msl_m, 2), num(telemetry.alt_rel_m, 2),
            ms(telemetry.position_age_s),
            num(telemetry.roll_deg, 2), num(telemetry.pitch_deg, 2),
            num(telemetry.yaw_deg, 2),
            ms(telemetry.attitude_age_s),
            num(telemetry.heading_deg, 1), num(telemetry.groundspeed_ms, 2),
            ms(telemetry.hud_age_s),
            num(servo.ratio, 2), servo.position, ms(zoom_age),
            telemetry.position_boot_ms if telemetry.position_boot_ms is not None
            else telemetry.attitude_boot_ms,
            filled if frame_number is not None else None,
        ])
        self.rows += 1
        self._file.flush()

    def close(self):
        self._file.close()


def draw_snapshot(args, frame, why, number, on_snapshot=None):
    """Write the JPEG, hand it on, and optionally draw it. Logs its outcome.

    The terminal preview of fcb_record.py is still drawn when this process
    has a terminal (`preview`), but under ros2 launch it usually does not:
    the JPEG published on ~/snapshot/compressed is what fcb_keys draws in
    the operator's pane instead.
    """
    path = size = None
    try:
        path, size = snapshot.save(
            frame, args.snap_dir or os.path.join(args.record_dir, "snapshots"),
            max_width=args.snap_width, quality=args.snap_quality,
        )
    except Exception as exc:
        log.error("snapshot: could not write the JPEG: %s", exc)

    if on_snapshot is not None:
        try:
            on_snapshot(path, frame)
        except Exception:
            log.exception("snapshot: publishing failed; the JPEG is still on "
                          "the drone")

    drawn = ""
    if getattr(args, "preview", False) and sys.stdout.isatty():
        try:
            text, cols, rows = snapshot.render(frame, max_cols=args.preview_cols)
            sys.stdout.write(text + "\n")
            sys.stdout.flush()
            drawn = ", %dx%d preview (%.0f KB) to this terminal" % (
                cols, rows, len(text.encode("utf-8")) / 1024.0
            )
        except Exception:
            log.exception("snapshot: preview failed; the JPEG is still on "
                          "the drone")

    if path is not None:
        log.info("snapshot %d [%s]: %s (%.0f KB)%s",
                 number, why, path, size / 1024.0, drawn)
    else:
        log.info("snapshot %d [%s]: not saved%s", number, why, drawn)


class SnapshotWorker:
    """Files snapshots off the capture thread.

    One press costs ~21 ms on the Orin against a 16.7 ms frame budget, so
    the capture loop hands the frame over and goes back to grabbing. One
    deep, newest wins: a pilot leaning on the button wants the frame they
    are looking at now, not a backlog.
    """

    def __init__(self, args, on_snapshot=None):
        self.args = args
        self.on_snapshot = on_snapshot
        self.superseded = 0
        self._pending = None
        self._cond = threading.Condition()
        self._running = True
        self._thread = threading.Thread(
            target=self._run, name="snapshot", daemon=True
        )
        self._thread.start()

    def submit(self, frame, why, number):
        """Queue a frame, displacing any not yet drawn. Never blocks."""
        with self._cond:
            displaced = self._pending is not None
            self._pending = (frame, why, number)
            self._cond.notify()
        if displaced:
            self.superseded += 1
        return not displaced

    def stop(self, timeout=3.0):
        with self._cond:
            self._running = False
            self._cond.notify()
        self._thread.join(timeout)

    def _run(self):
        while True:
            with self._cond:
                while self._running and self._pending is None:
                    self._cond.wait()
                if self._pending is None:
                    return
                job, self._pending = self._pending, None
            try:
                draw_snapshot(self.args, *job, on_snapshot=self.on_snapshot)
            except Exception:
                log.exception("snapshot: failed; continuing")


class _NoTelemetry:
    lat_deg = lon_deg = alt_msl_m = alt_rel_m = position_age_s = None
    roll_deg = pitch_deg = yaw_deg = attitude_age_s = None
    heading_deg = groundspeed_ms = hud_age_s = None
    position_boot_ms = attitude_boot_ms = None


class _NoZoom:
    ratio = position = updated_at = None


_NO_TELEMETRY = _NoTelemetry()
_NO_ZOOM = _NoZoom()


class Recorder:

    #: How raw two-channel frames are decoded; replaced by open_camera()
    #: with whatever packing the camera actually negotiated.
    yuv_code = cv2.COLOR_YUV2BGR_YUY2
    #: Whether the control port was found on a known USB camera board.
    control_on_board = False
    control_baud = None
    # Defaults for state set in __init__, so a Recorder built without it
    # (the tests do) still has every field the loop reads.
    _reopens = 0
    _blank_checked = 0.0
    picture_blank = False
    _switch_raw = None
    _switch_since = 0.0
    _switch_settled = None
    _disk_checked = 0.0
    disk_free_gb = None
    _write_failed = False
    _last_rc_warning = 0.0
    _segment_failed_at = -1e9
    repeating_board = False
    _last_written = None
    #: Most frames filled in one go. A longer hole is a camera outage, which
    #: the reopen path handles -- not something to paper over with a still.
    MAX_FILL_PER_GAP = 3
    _last_written_at = None
    _board_cycled_at = -1e9
    _board_advice_given = False
    _blank_since = None
    #: The current recording was started by hand (service or autostart), not
    #: by ch8. A switch merely sitting low -- or no flight controller at
    #: all -- does not end it; a stop does, or ch8 flicked up and back down.
    _by_hand = False
    _last_stopped_base = None
    _commands = None
    snap_worker = None
    video_device = None
    control_port = None
    link = None
    servo = None
    mavlink = None
    rc = None
    grabber = None
    snap_button = None
    icr_switch = None
    icr_mode = None
    snapshots = 0
    last_frame = None
    _state_broken = False
    _status_broken = False
    _last_status = 0.0
    _last_live = 0.0
    _start_suppressed = False
    _last_gap_row = 0.0

    # Hooks for the ROS node. Each is optional and must not raise; the
    # recorder guards the calls anyway.
    on_frame = None          # (bgr_frame, captured_at_monotonic)
    on_status = None         # (status_text, now_monotonic)
    on_snapshot = None       # (jpeg_path_or_None, bgr_frame)
    make_mavros = None       # () -> a MavlinkSource-shaped object

    def __init__(self, args):
        self.args = args
        self.declared_fps = None

        # A recording session and a video file are not the same thing.
        # `recording` is the flight: it owns the CSV and lasts from the ch8
        # flick to a deliberate stop. `video` is one file inside it, and can
        # come and go if the camera does.
        self.recording = False
        self.segment = 0
        self.video = None
        self.video_path = None
        self.csv = None
        self.base = None
        self.record_started_mono = None
        self.record_started_wall = None
        self._last_gap_row = 0.0
        # Set when a recording is ended by hand while ch8 is still high.
        # Without it the next loop iteration sees "switch high, not
        # recording" and starts a new one, so the stop would just chop the
        # flight in two. Cleared when the switch is next seen low.
        self._start_suppressed = False

        self.frames_seen = 0
        self.frames_dropped = 0
        self._commands = queue.Queue()
        self.running = True

    # -- commands from other threads -------------------------------------

    def submit(self, fn, timeout=10.0):
        """Run `fn()` on the capture thread and return its result.

        Raises TimeoutError if the loop does not get to it in time -- it is
        blocked in a camera reopen, say, or not running yet.
        """
        future = Future()
        self._commands.put((fn, future))
        try:
            return future.result(timeout=timeout)
        except FuturesTimeoutError:
            # Python 3.10 (Humble): not yet the builtin TimeoutError.
            raise TimeoutError(f"not run within {timeout} s") from None

    def poll_commands(self):
        """Run whatever the services queued since the last frame.

        Bounded, as the key drain was: draining is meant to clear a burst,
        and an unbounded loop here could hold the capture loop for ever.
        Nothing here is worth losing a recording over.
        """
        if self._commands is None:
            return
        for _ in range(64):
            try:
                fn, future = self._commands.get_nowait()
            except queue.Empty:
                return
            if not future.set_running_or_notify_cancel():
                continue
            try:
                future.set_result(fn())
            except Exception as exc:
                log.exception("command failed; continuing")
                future.set_exception(exc)

    def _fail_pending_commands(self):
        """Answer anything still queued at shutdown, so no caller hangs."""
        if self._commands is None:
            return
        while True:
            try:
                _, future = self._commands.get_nowait()
            except queue.Empty:
                return
            if future.set_running_or_notify_cancel():
                future.set_exception(RuntimeError("the recorder is shutting down"))

    # -- setup -----------------------------------------------------------

    def open_control(self):
        self.link = self.make_visca_link()
        if self.link is None:
            log.error("no VISCA port answered -- zoom and imaging mode are "
                      "unavailable until it does; still looking every %gs",
                      CONTROL_RETRY)
            threading.Thread(target=self._find_control_later, daemon=True,
                             name="visca-retry").start()

    def _find_control_later(self):
        """Keep looking for a control port that did not answer at startup.

        A camera slow to answer at launch -- a board just powered up, or
        one holding replies from before -- would otherwise leave the whole
        run with no zoom and no imaging mode.
        """
        attempts = 0
        while self.running and self.link is None:
            time.sleep(CONTROL_RETRY)
            attempts += 1
            link = self.make_visca_link()
            if link is not None:
                self.link = link
                log.warning("VISCA: camera control found after %d retries", attempts)
                self.start_servo()
                return
            if attempts % 6 == 0:
                log.warning("VISCA: still no camera control (%d retries); if "
                            "the camera board was unplugged or reset it may "
                            "need its 12 V power cycled", attempts)

    def make_visca_link(self):
        """Find and open the camera's control port. None if it is not there.

        Doubles as the servo's reconnect factory, so a link lost in flight
        is rebuilt by rediscovering the port rather than assuming it came
        back with the same name -- a re-enumerated USB device usually does
        not.
        """
        ports = [self.args.port] if self.args.port else None
        bauds = [self.args.baud] if self.args.baud else None
        if ports is None and self.control_on_board:
            # Coming back to a board found by USB ID: only its own ports.
            # The full sweep would also probe the autopilot's, writing VISCA
            # into the MAVLink stream for as long as the camera is gone.
            ports = devices.camera_serial_ports()
            if not ports:
                return None
            # And the baud it answered at: a camera register, not something
            # a replug changes, and every extra rate is another slow open.
            if bauds is None and self.control_baud:
                bauds = [self.control_baud]
        port, baud = devices.autodetect_visca(ports, bauds)
        if port is None:
            return None
        link = None
        try:
            link = ViscaLink(port, baud, self.args.address)
            link.if_clear()
        except Exception as exc:
            log.warning("VISCA: %s at %d baud would not open: %s",
                        port, baud, exc)
            # Closed here, or its exclusive lock outlives it and every later
            # reconnect finds the port "locked by another process".
            if link is not None:
                try:
                    link.close()
                except Exception:
                    pass
            return None
        self.control_port = port
        self.control_on_board = (
            devices.usb_id(port) in devices.CAMERA_BOARD_USB_IDS)
        self.control_baud = baud
        log.info("VISCA: %s at %d baud", port, baud)
        self.apply_stabilizer(link)
        return link

    def apply_stabilizer(self, link):
        """Put the camera's image stabilizer where `stabilizer` asks.

        Done for every new link -- startup, the late retry and the servo's
        reconnects -- because the setting lives in the camera and is lost
        with its power. Read back rather than trusted, and never fatal.
        """
        wanted = getattr(self.args, "stabilizer", "on")
        try:
            if wanted != "keep":
                link.command(visca.image_stabilizer(wanted == "on", link.address))
            state = visca.parse_stabilizer(
                link.inquiry(visca.stabilizer_inq(link.address)))
        except Exception as exc:
            log.warning("image stabilizer: could not %s: %s",
                        "read it" if wanted == "keep" else "turn it %s" % wanted,
                        exc)
            return
        if wanted == "keep":
            log.info("image stabilizer: %s (as found -- stabilizer: keep)", state)
        elif state == wanted:
            log.info("image stabilizer: %s (confirmed by the camera)", state)
        else:
            log.warning("image stabilizer: asked for %s, camera reports %s",
                        wanted, state)

    def open_mavlink(self):
        """Attach the flight controller: RC switches and telemetry.

        `fc_source` picks where it comes from:
          pymavlink  probe the serial ports for a heartbeat, as fcb_record.py
                     did (or use rc_url). Do not use this while MAVROS owns
                     the autopilot's port -- two readers corrupt both.
          mavros     read RC and telemetry from a running MAVROS instead.
          none       no flight controller (fcb_record.py --no-mavlink).
        """
        source = getattr(self.args, "fc_source", "pymavlink")
        if source == "none" or getattr(self.args, "no_mavlink", False):
            log.info("fc_source none: not looking for a flight controller -- "
                     "start and stop recordings by hand")
            return
        if source == "mavros":
            if self.make_mavros is None:
                log.critical("fc_source mavros, but no ROS node to subscribe "
                             "with -- no flight controller")
                return
            self._attach_source(self.make_mavros(), "MAVROS")
            return

        url, baud = self.args.rc_url, self.args.rc_baud
        if not _have_pymavlink():
            log.critical("pymavlink is not installed, so fc_source pymavlink "
                         "cannot reach the flight controller. Install it "
                         "(pip install pymavlink) or set fc_source:=mavros "
                         "with MAVROS running.")
            self._no_fc_banner(retrying=False)
            return
        if not url:
            log.info("probing for the flight controller...")
            url, baud = devices.autodetect_mavlink(exclude=self.control_port)
            if url is None:
                self._no_fc_banner(retrying=True)
                threading.Thread(target=self._find_mavlink_later, daemon=True,
                                 name="mavlink-retry").start()
                return
        self._attach_mavlink(url, baud)

    def _no_fc_banner(self, retrying):
        log.critical("=" * 68)
        log.critical("NO FLIGHT CONTROLLER FOUND -- ch%d CANNOT START "
                     "A RECORDING.", self.args.rec_channel)
        log.critical("Recording is armed by RC channel %d, which arrives "
                     "over MAVLink. Without", self.args.rec_channel)
        log.critical("it, call ~/start_recording (r in fcb_keys) to record, "
                     "or set autostart.")
        log.critical("Zoom, imaging mode and snapshots still work. "
                     "fc_source:=none skips all this.")
        log.critical("Fix: check the autopilot's USB lead and that it is "
                     "powered, or set rc_url")
        log.critical("to pin the port. Set require_mavlink to make this fatal.")
        if retrying:
            log.critical("Still looking every %gs -- plug it in and it is "
                         "picked up without a restart.", CONTROL_RETRY)
        log.critical("=" * 68)

    def _find_mavlink_later(self):
        """Keep looking for an autopilot that was not there at startup."""
        while self.running and self.mavlink is None:
            time.sleep(CONTROL_RETRY)
            url, baud = devices.autodetect_mavlink(exclude=self.control_port)
            if url is not None:
                self._attach_mavlink(url, baud)
                log.warning("MAVLink: flight controller found -- ch%d now "
                            "arms recording", self.args.rec_channel)
                return

    def _attach_mavlink(self, url, baud):
        try:
            mavlink = MavlinkSource(
                url=url, baud=baud, stream_rate_hz=self.args.stream_rate,
                on_status=lambda msg: log.info("MAVLink: %s", msg),
            )
        except Exception as exc:
            log.critical("MAVLink: could not start on %s: %s", url, exc)
            return
        self._attach_source(mavlink, f"MAVLink {url} at {baud} baud")

    def _attach_source(self, mavlink, label):
        """Bring up everything read through the flight controller.

        The switches are built before self.mavlink is set, because the
        capture loop takes self.mavlink being there to mean all of it is.
        """
        log.info("%s, zoom on ch%d, record on ch%d",
                 label, self.args.rc_channel, self.args.rec_channel)
        rc = RcZoomSource(
            channel=self.args.rc_channel,
            pwm_min=self.args.rc_pwm_min, pwm_max=self.args.rc_pwm_max,
            pwm_deadband=self.args.rc_deadband, reverse=self.args.rc_reverse,
            rc_timeout=self.args.rc_timeout,
            source=mavlink,
        )

        if self.args.snap_channel:
            self.snap_button = RcButton(
                mavlink, self.args.snap_channel,
                threshold=self.args.snap_threshold,
                reverse=self.args.snap_reverse,
                rc_timeout=self.args.rc_timeout,
            )
            log.info("snapshot: ch%d, one press publishes a frame",
                     self.args.snap_channel)

        if self.args.icr_channel:
            # Low, centre, high -- in the order the switch travels.
            self.icr_switch = RcSelector(
                mavlink, self.args.icr_channel,
                ["day", "night_color", "night"],
                pwm_min=self.args.rc_pwm_min, pwm_max=self.args.rc_pwm_max,
                reverse=self.args.icr_channel_reverse,
                rc_timeout=self.args.rc_timeout,
            )
            log.info("imaging mode: ch%d, low RGB / centre RGB+IR / high IR",
                     self.args.icr_channel)
        self.rc = rc
        if self.servo is not None and self.servo.rc is None:
            self.servo.rc = rc      # the zoom knob, for an autopilot found late
            # Started without RC, the servo came up under manual control and
            # would ignore the knob for good; hand it over unless a zoom was
            # set by hand meanwhile.
            if getattr(self.servo, "manual_ratio", None) is None:
                getattr(self.servo, "hand_to_rc", lambda: None)()
        self.mavlink = mavlink

    def start_servo(self):
        if self.link is None:
            return
        self.servo = ZoomServo(
            self.link, curve=self.args.curve, rc=self.rc,
            on_status=lambda msg: log.warning("%s", msg),
            link_factory=self.make_visca_link,
            ratio_step=self.args.zoom_step,
        )
        if self.args.zoom_step > 0:
            steps = zoom_map.ratio_steps(self.args.zoom_step)
            log.info("zoom: detented in %.2gx steps -- %d positions from "
                     "%.1fx to %.1fx", self.args.zoom_step, len(steps),
                     steps[0], steps[-1])
        try:
            ratio = self.servo.sync_from_camera()
            log.info("zoom: currently %.1fx, following %s",
                     ratio, "RC" if self.rc else "nothing (no RC link)")
        except Exception as exc:
            log.warning("could not read the starting zoom position: %s", exc)

        self.start_icr()

    def start_icr(self):
        """Set the imaging mode if one was asked for, otherwise adopt it.

        With no `icr` the camera is left as it was found -- it keeps the
        mode across a power cycle -- and the mode is only read back.
        """
        if self.args.icr:
            self.set_icr(resolve_icr(self.args.icr), announce="imaging mode")
            return
        try:
            payload = self.link.inquiry(visca.icr_mode_inq(self.link.address))
            self.icr_mode = visca.parse_icr_mode(payload)
        except (ViscaError, ViscaTimeout, ValueError) as exc:
            log.warning("could not read the current imaging mode: %s", exc)
            return
        # The inquiry reports where the filter is, not how it got there, so
        # a camera left in automatic reads back as whichever mode it picked.
        log.info("imaging mode: %s (as found -- set icr, or call "
                 "~/set_imaging_mode)", ICR_LABELS[self.icr_mode])

    def open_camera(self, quiet=False):
        device = self.args.video or devices.autodetect_video(
            log=(lambda msg: None) if quiet else
                (lambda msg: log.info("video: %s", msg))
        )
        if device is None:
            if not quiet:
                log.critical("no FCB camera found")
            return False

        capture = devices.open_capture(
            device, self.args.width, self.args.height, self.args.fourcc or None
        )
        if capture is None:
            if not quiet:
                log.critical("could not open %s", device)
            return False

        width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        # YUYV and UYVY arrive as the same two-channel shape when OpenCV
        # does not convert, so the negotiated format tells them apart.
        pixel_format = devices.fourcc_of(capture)
        self.yuv_code = devices.yuv422_to_bgr_code(pixel_format)
        declared = capture.get(cv2.CAP_PROP_FPS)
        self.declared_fps = declared if 1.0 < declared < 1000.0 else None
        repeats = devices.usb_id(device) in devices.REPEATING_FRAME_USB_IDS
        log.info("video: %s at %dx%d %s, %s", device, width, height,
                 pixel_format or "format not reported",
                 f"{self.declared_fps:.2f} fps" if self.declared_fps
                 else "rate not reported")
        self.repeating_board = repeats
        if repeats:
            # Its figure is not the camera's; tagging a recording with it
            # would play it back at the wrong speed.
            self.declared_fps = None
            log.info("video: this board repeats frames and misreports its "
                     "rate -- dropping the repeats and measuring the rate")
        try:
            capture.set(cv2.CAP_PROP_CONVERT_RGB, 1.0)
        except Exception:
            pass
        self.video_device = device
        self.grabber = FrameGrabber(capture, self.yuv_code,
                                    drop_repeats=repeats)
        return True

    def reopen_camera(self):
        """Tear the capture down and build it again after the stream died.

        A USB camera that browns out, is knocked loose, or wedges comes
        back as a device that opens but never delivers, and nothing short
        of reopening it recovers. Nobody can replug anything mid-flight.
        """
        log.warning("video: reopening the camera")
        if self.grabber is not None:
            try:
                self.grabber.stop()
            except Exception:
                pass
            self.grabber = None

        # A video file cannot span a gap in its own frame stream, so this
        # one is finalised now. The flight is not over: the CSV stays open,
        # and frames returning open a new numbered segment.
        if self.video is not None:
            log.warning("video: closing segment %d at the break -- telemetry "
                        "keeps recording", self.segment)
            self.close_segment()

        if self.open_camera(quiet=True):
            # Opened is not recovered: a wedged board opens fine and then
            # sends nothing. "frames flowing again" is the real recovery.
            log.info("video: camera reopened -- waiting for frames")
            return True
        log.error("video: camera did not come back -- retrying every %gs",
                  self.args.camera_timeout)
        return False

    # -- imaging mode ----------------------------------------------------

    def set_icr(self, mode, announce="imaging mode"):
        """Switch the IR cut filter. Logged either way, including failures."""
        if self.servo is None:
            log.warning("no VISCA control -- cannot change the imaging mode")
            return False
        try:
            self.servo.set_icr(mode)
        except (ViscaError, ViscaTimeout, ValueError) as exc:
            log.error("imaging mode: %s would not take: %s",
                      ICR_LABELS.get(mode, mode), exc)
            return False
        except Exception as exc:
            # The link itself is gone -- the servo is already reconnecting.
            log.error("imaging mode: %s not set -- no camera control right "
                      "now (%s); it reconnects by itself",
                      ICR_LABELS.get(mode, mode), type(exc).__name__)
            return False
        self.icr_mode = mode
        log.info("%s: %s", announce, ICR_LABELS[mode])
        return True

    def cycle_icr(self):
        """Step to the next mode, starting from whatever is current."""
        try:
            index = ICR_ORDER.index(self.icr_mode)
        except ValueError:
            index = -1
        mode = ICR_ORDER[(index + 1) % len(ICR_ORDER)]
        return self.set_icr(mode), mode

    # -- zoom by hand ----------------------------------------------------

    def apply_zoom(self, wanted):
        """Drive to an exact magnification. Returns (ok, ratio, message)."""
        if self.servo is None:
            log.warning("no VISCA control -- cannot set the zoom")
            return False, float("nan"), "no VISCA control"
        try:
            got = self.servo.set_ratio(wanted)
        except Exception as exc:
            log.error("zoom: could not set %.2fx: %s", wanted, exc)
            return False, float("nan"), str(exc)
        if abs(got - wanted) > 0.01:
            msg = ("%.2fx requested, clamped to %.2fx (the lens does "
                   "%.0fx-%.0fx)" % (wanted, got, zoom_map.MIN_RATIO,
                                     zoom_map.MAX_RATIO))
        else:
            msg = ("%.2fx set by hand -- the ch%d knob is ignored until "
                   "~/zoom_to_rc" % (got, self.args.rc_channel))
        log.info("zoom: %s", msg)
        return True, got, msg

    def hand_zoom_to_rc(self):
        if self.servo is None:
            return False, "no VISCA control"
        if self.servo.hand_to_rc():
            msg = "back under the ch%d knob" % self.args.rc_channel
            log.info("zoom: %s", msg)
            return True, msg
        log.warning("zoom: no RC link to hand back to")
        return False, "no RC link to hand back to"

    # -- starting and stopping by hand -----------------------------------

    def request_start_recording(self, why="start_recording"):
        """Start a recording without the RC switch -- no Pixhawk needed.

        The recording holds until it is stopped by hand, or until ch8 (if
        there is one) is flicked up and back down; a switch that is merely
        sitting low does not end it. Video joins on the next frame.
        Returns (ok, message).
        """
        if self.recording:
            log.info("already recording")
            return True, "already recording " + os.path.basename(self.base or "")
        if self._write_failed:
            msg = ("not starting: the last write failed (full disk?) -- free "
                   "space and restart the driver")
            log.error(msg)
            return False, msg
        self._by_hand = True
        self.start_recording(None)
        log.info("recording started by %s", why)
        return True, "recording " + os.path.basename(self.base)

    def request_stop_recording(self, why="stop_recording"):
        """Stop the recording. Returns (ok, message, base).

        A service call is deliberate, so unlike the pane's x it needs no
        second press; fcb_keys keeps the two-press confirmation for keys.
        """
        if not self.recording:
            log.info("not recording -- nothing to stop")
            return False, "not recording", ""
        base = self.base
        # Suppress BEFORE stopping. ch8 is probably still high, and any gap
        # between "no longer recording" and "not allowed to start" is one
        # in which a new session begins and the stop chopped the flight.
        still_high = self.wants_recording()
        if still_high:
            self._start_suppressed = True
        self.stop_recording(why=why)
        msg = "stopped " + os.path.basename(base)
        if still_high:
            log.info("ch%d is still high; flick it low and back to start "
                     "another recording", self.args.rec_channel)
            msg += "; ch%d is still high -- flick it low and back to start " \
                   "another" % self.args.rec_channel
        return True, msg, base

    # -- naming a finished flight ----------------------------------------

    def recorded_files(self, base):
        """Every file that belongs to one recording session."""
        if not base:
            return []
        stem = os.path.basename(base)
        directory = os.path.dirname(base)
        try:
            names = os.listdir(directory)
        except OSError:
            return []
        out = []
        for name in sorted(names):
            if not name.startswith(stem):
                continue
            rest = name[len(stem):]
            # "" + .avi/.csv, or "-2" + .avi -- not "-something-else".
            if re.fullmatch(r"(-\d+)?\.(%s|csv)" % "|".join(CONTAINERS), rest):
                out.append(os.path.join(directory, name))
        return out

    @staticmethod
    def safe_name(text):
        """A filename from whatever was typed, or None if nothing survives.

        Basename first, so a slash cannot walk the rename out of the
        recordings directory; then only characters painless everywhere.
        """
        text = os.path.basename(text.strip()).strip(". ")
        text = re.sub(r"\s+", "_", text)
        text = re.sub(r"[^A-Za-z0-9._-]", "", text)
        return text or None

    def rename_recording(self, base, text):
        """Rename every file of one session, or none of them.

        Returns (ok, message, new_base, files).
        """
        if self.recording and base == self.base:
            return False, "that recording is still running -- stop it first", base, []
        name = self.safe_name(text)
        if name is None:
            log.warning("that name has no usable characters -- keeping %s",
                        os.path.basename(base))
            return False, "no usable characters in that name", base, []
        files = self.recorded_files(base)
        if not files:
            log.warning("nothing left to rename for %s", os.path.basename(base))
            return False, "no files for %s" % base, base, []

        directory = os.path.dirname(base)
        stem = os.path.basename(base)
        # Never overwrite an existing recording, and never half-rename a
        # session: a suffix is found that clears every file at once.
        suffix, target = "", name
        while True:
            planned = [(f, os.path.join(
                directory, target + os.path.basename(f)[len(stem):])) for f in files]
            if not any(os.path.exists(dst) for _, dst in planned):
                break
            suffix = f"-{int(suffix[1:] or 1) + 1}" if suffix else "-2"
            target = name + suffix
        if suffix:
            log.warning("%s was taken -- using %s", name, target)

        done = []
        for src, dst in planned:
            try:
                os.rename(src, dst)
                done.append(dst)
            except OSError as exc:
                log.error("could not rename %s: %s -- the rest of this "
                          "flight keeps its old name",
                          os.path.basename(src), exc)
                return False, str(exc), base, done
        log.info("RENAMED  %s -> %s  (%d file(s))", stem, target, len(planned))
        for _, dst in planned:
            log.info("         %s", os.path.basename(dst))
        new_base = os.path.join(directory, target)
        if self._last_stopped_base == base:
            self._last_stopped_base = new_base
        return True, "renamed %s -> %s" % (stem, target), new_base, done

    # -- snapshots -------------------------------------------------------

    def take_snapshot(self, why="service"):
        """Save and publish the current frame. False if there is none yet."""
        frame = self.last_frame
        if frame is None:
            log.warning("snapshot: no frame yet")
            return False
        # Counted on the press, not on the save, so the status line
        # acknowledges the button immediately.
        self.snapshots += 1
        if self.snap_worker is not None:
            self.snap_worker.submit(frame, why, self.snapshots)
        else:
            draw_snapshot(self.args, frame, why, self.snapshots,
                          on_snapshot=self.on_snapshot)
        return True

    # -- RC switches -----------------------------------------------------

    def poll_rc_switches(self):
        """Service the momentary and selector channels once per frame.

        Both are edge-triggered. A switch that misbehaves must not be able
        to end a recording in progress, so failures are contained.
        """
        try:
            if self.snap_button is not None and self.snap_button.pressed():
                self.take_snapshot(why=f"ch{self.args.snap_channel}")
        except Exception:
            log.exception("snapshot channel failed; continuing")

        try:
            if self.icr_switch is not None:
                mode = self.icr_switch.changed()
                # Only on a transition, which leaves the service usable: the
                # switch reasserts nothing until the pilot moves it.
                if mode is not None and mode != self.icr_mode:
                    self.set_icr(mode, announce=
                                 f"imaging mode (ch{self.args.icr_channel})")
        except Exception:
            log.exception("imaging mode channel failed; continuing")

    # -- recording -------------------------------------------------------

    def start_recording(self, frame, captured_at=None):
        """Begin a flight: open the CSV, then the first video segment."""
        os.makedirs(self.args.record_dir, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        self.base = unique_base(
            os.path.join(self.args.record_dir, f"fcb-{stamp}"))

        self.csv = TelemetryCsv(f"{self.base}.csv")
        # Time zero is the first frame written, not the instant this runs.
        now_mono = time.monotonic()
        self.record_started_mono = now_mono if captured_at is None else captured_at
        self.record_started_wall = datetime.now(timezone.utc) - _delta(
            now_mono - self.record_started_mono
        )
        self.recording = True
        self.segment = 0
        self._last_gap_row = 0.0
        log.info("RECORDING STARTED  %s.csv", os.path.basename(self.base))
        self.open_segment(frame)

    def open_segment(self, frame):
        """Open one video file inside the current session.

        Called again after a camera outage, so a flight that loses video
        for a while comes back as a second numbered file rather than
        ending. Returns True if there is a video to write to.
        """
        if frame is None or self.grabber is None:
            return False
        height, width = frame.shape[:2]
        # The container's frame rate decides playback speed. The camera's
        # own figure where it has one; a measured rate only once enough
        # frames have arrived to mean something.
        measured = self.grabber.fps()
        if self.declared_fps:
            fps, source = self.declared_fps, "camera"
        elif self.repeating_board:
            # Not measured: a measurement taken while the link loses frames
            # tagged one recording 43 fps for a 60 fps camera. Frames lost
            # on the way are filled in instead (see write_frame).
            fps, source = self.args.fps, "camera nominal"
        elif measured >= 1.0:
            fps, source = measured, "measured"
        else:
            fps, source = self.args.fps, "fps default"

        # Retried on every frame while there is no video, so rate-limited,
        # and the segment number only advances on success.
        now = time.monotonic()
        if now - self._segment_failed_at < 5.0:
            return False
        number = self.segment + 1
        suffix = "" if number == 1 else f"-{number}"
        path = f"{self.base}{suffix}.{self.args.container}"
        try:
            self.video = VideoRecorder(
                path, width, height, fps,
                encoder=self.args.encoder, bitrate_mbps=self.args.bitrate,
            )
        except RuntimeError as exc:
            self._segment_failed_at = now
            log.error("could not open video segment %d (%s) -- telemetry "
                      "keeps recording; retrying every 5s", number, exc)
            self.video = None
            return False
        self.segment = number
        self._last_written = None
        self._last_written_at = None

        self.video_path = path
        log.info("  video segment %d: %s (%dx%d @ %.2f fps from %s, "
                 "%s encoder)", self.segment, os.path.basename(path),
                 width, height, fps, source, self.video.encoder)
        return True

    def close_segment(self):
        """Finalise the current video file, leaving the session and CSV open."""
        if self.video is None:
            return
        frames = self.video.frames
        encoder = self.video.encoder
        path = self.video_path
        elapsed = self.video.close()
        self.video = None
        try:
            size_bytes = os.path.getsize(path)
        except OSError:
            size_bytes = 0
        log.info("  video segment %d closed: %s -- %d frames, %.1fs, "
                 "%.1f MB (%.1f fps average)", self.segment,
                 os.path.basename(path), frames, elapsed, size_bytes / 1e6,
                 frames / elapsed if elapsed > 0 else 0.0)

        # Frames counted but nothing on disk: the encoder rejected every
        # buffer. Otherwise completely silent, so it is called out.
        if frames > 0 and size_bytes < 10_000:
            log.error(
                "THE VIDEO IS EMPTY (%d bytes) despite %d frames -- the %s "
                "encoder accepted the file but rejected the frames, so this "
                "recording has no usable video. The CSV is still valid. "
                "Set encoder:=cpu to keep recording while the encoder "
                "problem is investigated.",
                size_bytes, frames, encoder,
            )

    def stop_recording(self, why="stopped"):
        """End the flight: close the last video segment and the CSV.

        Called for ch8 going low (once settled), for a stop by hand, on a
        write failure, and at shutdown. A range dropout does not stop it:
        stale RC reads as "cannot tell", which holds.
        """
        if not self.recording:
            return
        # Marked stopped first, so nothing writes to half-closed files, and
        # each close is attempted whatever happened to the other.
        self.recording = False
        self._by_hand = False
        try:
            self.close_segment()
        except Exception:
            log.exception("could not close the video cleanly")
            self.video = None
        rows = self.csv.rows if self.csv is not None else 0
        if self.csv is not None:
            try:
                self.csv.close()
            except Exception as exc:
                log.error("could not close the CSV cleanly: %s", exc)
        self.csv = None
        self._last_stopped_base = self.base
        elapsed = (time.monotonic() - self.record_started_mono
                   if self.record_started_mono else 0.0)
        log.info("RECORDING STOPPED (%s)  %s -- %.1fs, %d video segment(s), "
                 "%d telemetry rows", why, os.path.basename(self.base or "?"),
                 elapsed, self.segment, rows)

    def write_gap_row(self, now):
        """A telemetry row with no frame behind it (empty frame column).

        Video and telemetry must not take each other down: the camera
        dropping out used to end the CSV too, losing the position track.
        """
        if self.csv is None or self.record_started_mono is None:
            return
        if now - self._last_gap_row < self.args.telemetry_interval:
            return
        self._last_gap_row = now
        t_mono = now - self.record_started_mono
        try:
            self.csv.write(
                None, self.record_started_wall + _delta(t_mono), t_mono,
                self.mavlink.telemetry() if self.mavlink else _NO_TELEMETRY,
                self.servo if self.servo else _NO_ZOOM, now,
            )
        except Exception:
            log.exception("telemetry row failed; continuing")

    def wants_recording(self):
        """Whether the RC switch is asking for recording right now.

        None means "cannot tell", and no flight controller is exactly that
        -- not a considered "no". Returning False here once made the loop
        record nothing and say nothing for a whole flight.
        """
        if self.mavlink is None:
            return None
        age = self.mavlink.rc_age()
        if age is None or age > self.args.rc_timeout:
            return None  # unknown -- RC is stale, hold whatever we are doing
        pwm = self.mavlink.channel(self.args.rec_channel)
        if pwm <= 0:
            return None
        high = pwm >= self.args.rec_channel_threshold
        return not high if self.args.rec_channel_reverse else high

    # -- main loop -------------------------------------------------------

    def run(self):
        """Capture and record until asked to stop."""
        self.snap_worker = SnapshotWorker(self.args, on_snapshot=self.on_snapshot)
        try:
            self.capture_loop()
        finally:
            worker, self.snap_worker = self.snap_worker, None
            worker.stop()
            self._fail_pending_commands()

    def _emit_frame(self, frame, captured_at):
        if self.on_frame is None:
            return
        try:
            self.on_frame(frame, captured_at)
        except Exception:
            log.exception("publishing a frame failed; continuing")

    def capture_loop(self):
        last_seq = 0
        self._last_rc_warning = 0.0
        waiting_logged = False
        last_frame_at = time.monotonic()

        while self.running:
            if self.grabber is None:
                # The last reopen failed, so the camera is gone rather than
                # stalled. Stand in for the frame wait -- same cadence -- so
                # the loop keeps running and retries on the timeout below.
                time.sleep(0.2)
                frame, seq, captured_at = None, last_seq, None
            else:
                frame, seq, captured_at = self.grabber.wait_for_frame(
                    last_seq, timeout=0.2
                )
            now = time.monotonic()
            self.poll_commands()

            if frame is None:
                if not waiting_logged:
                    log.warning("no frames from the camera")
                    waiting_logged = True
                # Give the stream a grace period, then rebuild it -- backing
                # off: 5 s, 10, 20, 40, then every 60 s. A reopen cannot
                # revive a board that has stopped streaming (the Oppila one
                # needs its 12 V cycled), and the churn of retrying every
                # 5 s for ever is itself what wedges it.
                wait = min(60.0, self.args.camera_timeout * (2 ** min(self._reopens, 4)))
                if now - last_frame_at > wait:
                    self._reopens += 1
                    if self.reopen_camera():
                        last_seq = 0
                    if self._reopens >= 2:
                        self.board_is_stuck("no frames after %d reopens" % self._reopens)
                    if self._reopens >= 3:
                        log.warning("video: %d reopens without a frame -- next "
                                    "try in %.0fs. If the camera board was "
                                    "unplugged or reset, cycle its 12 V supply",
                                    self._reopens, min(60.0, self.args.camera_timeout
                                                       * (2 ** min(self._reopens, 4))))
                    last_frame_at = time.monotonic()
                # No video does not mean no flight: the switch is still
                # obeyed, and the position and attitude track keeps going.
                self.apply_record_switch(now, None, None)
                if self.recording:
                    self.write_gap_row(now)
                self.check_disk(now)
                self.update_display(now)
                continue
            last_frame_at = now
            self._reopens = 0
            frame = to_bgr(frame, self.yuv_code)
            if waiting_logged:
                log.info("frames flowing again")
                waiting_logged = False
            self.check_blank(frame, now)
            self.check_disk(now)

            if self.frames_seen == 0:
                log.info("first frame received (%dx%d)",
                         frame.shape[1], frame.shape[0])
            dropped = seq - last_seq - 1
            if dropped > 0:
                self.frames_dropped += dropped
                log.debug("skipped %d frame(s) -- consumer behind the camera",
                          dropped)
            last_seq = seq
            self.frames_seen += 1
            # Held for the snapshot button, which fires between frames.
            self.last_frame = frame
            self._emit_frame(frame, captured_at)
            self.poll_rc_switches()

            self.apply_record_switch(now, frame, captured_at)

            if self.recording:
                # Video coming back mid-flight rejoins the same session as a
                # new numbered segment, against the same CSV.
                if self.video is None:
                    self.open_segment(frame)
                video, sheet = self.video, self.csv
                if video is not None and sheet is not None:
                    try:
                        self.write_frame(video, sheet, frame, captured_at, now,
                                         seq_gap=max(1, dropped + 1))
                    except Exception as exc:
                        self.on_write_failure(exc)
                else:
                    self.write_gap_row(now)

            self.update_display(now)

    def write_frame(self, video, sheet, frame, captured_at, now, seq_gap=1):
        """Write one captured frame, after filling a gap the camera left.

        A frame the link lost leaves a hole in the timeline, and the
        container plays at a fixed rate, so without a fill the video runs
        short and plays fast. Only those holes are filled -- seq_gap == 1,
        the grabber had nothing in between -- and never more than
        MAX_FILL_PER_GAP at once. Filling frames this loop was merely too
        slow to collect once spiralled into 1,795 copies in 35 s.
        """
        telemetry = self.mavlink.telemetry() if self.mavlink else _NO_TELEMETRY
        servo = self.servo if self.servo else _NO_ZOOM
        prev, prev_t = self._last_written, self._last_written_at
        if prev is not None and seq_gap == 1 and prev_t is not None:
            gap = captured_at - prev_t
            # Over 1.75 frame periods: the board's own arrival jitter never
            # reaches that.
            if gap > 1.75 / video.fps:
                missing = min(int(round(gap * video.fps)) - 1,
                              self.MAX_FILL_PER_GAP)
                for i in range(1, missing + 1):
                    t = prev_t + i / video.fps
                    sheet.write(video.frames,
                                self.record_started_wall + _delta(t - self.record_started_mono),
                                t - self.record_started_mono, telemetry, servo, now,
                                filled=1)
                    video.write(prev)
        sheet.write(video.frames,
                    self.record_started_wall + _delta(captured_at - self.record_started_mono),
                    captured_at - self.record_started_mono, telemetry, servo, now)
        video.write(frame)
        self._last_written, self._last_written_at = frame, captured_at

    def settled_switch(self, now):
        """wants_recording(), acted on only once it has held still for
        rec_debounce. "Cannot tell" (None) passes straight through."""
        raw = self.wants_recording()
        if raw is None:
            self._switch_raw = None
            return None
        if raw != self._switch_raw:
            self._switch_raw, self._switch_since = raw, now
        if now - self._switch_since >= getattr(self.args, "rec_debounce", 0.3):
            self._switch_settled = raw
        return self._switch_settled

    def apply_record_switch(self, now, frame, captured_at):
        """Start or stop on ch8, with or without a video frame in hand."""
        wanted = self.settled_switch(now)
        if wanted is None:
            if now - self._last_rc_warning > 5.0:
                if self.mavlink is None:
                    if not (getattr(self.args, "fc_source", "pymavlink") == "none"
                            or getattr(self.args, "no_mavlink", False)
                            or self.recording):
                        log.error("NO FLIGHT CONTROLLER -- channel %d cannot "
                                  "be read, so nothing is being recorded. "
                                  "Call ~/start_recording (r in fcb_keys) to "
                                  "record without it. Check the autopilot's "
                                  "USB lead and power.", self.args.rec_channel)
                elif self._switch_raw is None:
                    log.warning("no RC data -- holding recording state")
                self._last_rc_warning = now
            return
        if not wanted:
            # The switch is genuinely low: whatever was stopped by hand is
            # now stopped by the pilot too, so let it arm again.
            if self._start_suppressed:
                log.info("ch%d is low again -- the switch can start a new "
                         "recording", self.args.rec_channel)
                self._start_suppressed = False
            if self.recording and self._by_hand:
                return  # started without the switch; it has not been used
            if self.recording:
                self.stop_recording(why=f"ch{self.args.rec_channel} low")
            return
        # The switch is up: from here on it owns the recording, so bringing
        # it back down stops one that was started by hand.
        self._by_hand = False
        if not self.recording and not self._start_suppressed:
            if self._write_failed:
                return  # said once already, in on_write_failure
            self.start_recording(frame, captured_at)
            if frame is None:
                log.warning("recording with NO VIDEO yet -- telemetry is being "
                            "written, and video joins as soon as frames arrive")

    def on_write_failure(self, exc):
        """A frame or row could not be written: say so, and stop cleanly.

        Almost always a full disk. No new recording starts until the driver
        is restarted -- one that cannot be written is not a recording.
        """
        free = self.free_gb()
        log.critical("WRITE FAILED: %s -- %s. Stopping the recording so what "
                     "was written stays readable.", exc,
                     f"{free:.1f} GB free" if free is not None else
                     "free space unknown")
        self._write_failed = True
        try:
            self.stop_recording(why="write failed")
        except Exception:
            log.exception("could not close the recording cleanly")

    def free_gb(self):
        try:
            return shutil.disk_usage(self.args.record_dir).free / 1e9
        except (OSError, AttributeError):
            return None

    def check_disk(self, now):
        """Warn as the recordings disk fills, every 30 s while it is low."""
        if now - self._disk_checked < 30.0:
            return
        self._disk_checked = now
        self.disk_free_gb = self.free_gb()
        if self.disk_free_gb is None:
            return
        bytes_per_s = getattr(self.args, "bitrate", 25.0) * 1e6 / 8
        if self.disk_free_gb < 1.0:
            log.critical("DISK NEARLY FULL: %.1f GB left in %s -- about %.0f s "
                         "of video", self.disk_free_gb, self.args.record_dir,
                         self.disk_free_gb * 1e9 / bytes_per_s)
        elif self.disk_free_gb < 5.0:
            log.warning("disk low: %.1f GB left in %s (about %.0f min of video)",
                        self.disk_free_gb, self.args.record_dir,
                        self.disk_free_gb * 1e9 / bytes_per_s / 60)

    def board_is_stuck(self, why):
        """The board's video is wedged: power-cycle it if we can, else say so.

        The Oppila board runs on its own 12 V and does not recover from the
        Orin booting or rebooting while it stays powered; only cutting its
        12 V clears it. With board_power_cycle_cmd set (a relay or MOSFET
        on that line) this does it, at most once every 90 s.
        """
        cmd = getattr(self.args, "board_power_cycle_cmd", "")
        now = time.monotonic()
        if not cmd:
            if not self._board_advice_given:
                self._board_advice_given = True
                log.error("CAMERA BOARD IS STUCK (%s). Cycle its 12 V supply. It "
                          "does not recover from the Orin booting or rebooting "
                          "while it stays powered; set board_power_cycle_cmd "
                          "to have the driver do this itself.", why)
            return
        if now - self._board_cycled_at < 90.0:
            return
        self._board_cycled_at = now
        log.warning("CAMERA BOARD IS STUCK (%s) -- power-cycling it: %s", why, cmd)
        try:
            done = subprocess.run(cmd, shell=True, timeout=60,
                                  capture_output=True, text=True)
            if done.returncode != 0:
                log.error("board power cycle failed (exit %d): %s",
                          done.returncode, (done.stderr or done.stdout).strip()[:200])
        except Exception as exc:
            log.error("board power cycle failed: %s", exc)
        self._reopens = 0
        self._blank_since = None

    def check_blank(self, frame, now):
        """Notice a picture that is one flat colour, every couple of seconds.

        All green is the board streaming zeros: up on USB, VISCA answering,
        but no image from the camera block. A real picture -- even pitch
        dark -- carries sensor noise, so under one grey level is nothing.
        """
        if now - self._blank_checked < 2.0:
            return
        self._blank_checked = now
        try:
            blank = float(frame[::24, ::24].std()) < 1.0
        except Exception:
            return
        if blank:
            if self._blank_since is None:
                self._blank_since = now
            elif now - self._blank_since > 10.0:
                self.board_is_stuck("picture blank for %.0fs" % (now - self._blank_since))
        else:
            self._blank_since = None
        if blank and not self.picture_blank:
            log.error("PICTURE IS BLANK -- every pixel the same colour (all "
                      "green means all-zero frames). The board is streaming "
                      "but receiving no image from the camera: check the "
                      "camera's LVDS cable and power, then cycle the board's "
                      "12 V. Zoom and RGB/IR still respond; that is a "
                      "separate path.")
        elif not blank and self.picture_blank:
            log.info("picture is back")
        self.picture_blank = blank

    def publish_state(self, text):
        """Write the full status line to state_file for the GCS panes.

        Replaced atomically, on a tmpfs by default.
        """
        path = getattr(self.args, "state_file", "")
        if not path or self._state_broken:
            return
        try:
            tmp = f"{path}.{os.getpid()}.tmp"
            with open(tmp, "w") as handle:
                handle.write(text + "\n")
            os.replace(tmp, path)
        except OSError as exc:
            self._state_broken = True
            log.warning("could not publish state to %s: %s", path, exc)

    def update_display(self, now):
        """Publish the status often; write it to the log occasionally."""
        try:
            if now - self._last_live >= self.args.status_interval:
                self._last_live = now
                text = self.status_text(now)
                if self.on_status is not None:
                    self.on_status(text, now)
                self.publish_state(text)

            if now - self._last_status >= self.args.log_status_interval:
                self._last_status = now
                log.info("status: %s", self.status_text(now))
        except Exception:
            if not self._status_broken:
                self._status_broken = True
                log.exception("status display failed; continuing without it")

    def status_text(self, now):
        parts = [f"{self.grabber.fps():.1f} fps" if self.grabber else "no video",
                 f"frames {self.frames_seen}"]
        if self.frames_dropped:
            parts.append(f"skipped {self.frames_dropped}")

        if self.recording:
            elapsed = now - self.record_started_mono
            if self.video is not None:
                parts.append(f"REC {os.path.basename(self.base)} "
                             f"(seg {self.segment}, {self.video.frames} fr, "
                             f"{elapsed:.0f}s)")
            else:
                parts.append(f"REC TELEMETRY ONLY -- NO VIDEO "
                             f"({elapsed:.0f}s)")
        else:
            parts.append("not recording")

        if self.servo is not None and self.servo.ratio is not None:
            target = self.servo.target_ratio
            if (target is not None
                    and abs(target - self.servo.ratio) >= 0.05):
                zoom = f"zoom {self.servo.ratio:.1f}x>{target:.1f}x"
            else:
                zoom = f"zoom {self.servo.ratio:.1f}x"
            if self.servo.source != "rc":
                zoom += " BY HAND"
            parts.append(zoom)

        # First, and on every refresh, for as long as it is true. The one
        # startup error scrolls off in seconds; this cannot.
        if (self.mavlink is None
                and getattr(self.args, "fc_source", "pymavlink") != "none"
                and not getattr(self.args, "no_mavlink", False)):
            parts.insert(0, "NO FC" if self.recording
                         else "NO FC -- r TO RECORD")
        if self.picture_blank and self.grabber is not None:
            parts.insert(0, "BLANK PICTURE")
        if self._write_failed:
            parts.insert(0, "WRITE FAILED -- NOT RECORDING")
        if self.disk_free_gb is not None and self.disk_free_gb < 5.0:
            parts.append(f"DISK {self.disk_free_gb:.1f}GB")

        if self.icr_mode is not None:
            parts.append(ICR_SHORT[self.icr_mode])

        if self.snapshots:
            parts.append(f"{self.snapshots} snap")

        if self.mavlink is not None:
            telemetry = self.mavlink.telemetry()
            if telemetry.lat_deg is not None:
                parts.append(f"GPS {telemetry.lat_deg:.6f},{telemetry.lon_deg:.6f}")
            else:
                parts.append("GPS no fix")
            if telemetry.alt_rel_m is not None:
                parts.append(f"alt {_fmt(telemetry.alt_msl_m, '.1f')}m "
                             f"rel {telemetry.alt_rel_m:.1f}m")
            if telemetry.roll_deg is not None:
                parts.append(f"rpy {telemetry.roll_deg:.0f}/"
                             f"{telemetry.pitch_deg:.0f}/{telemetry.yaw_deg:.0f}")
            if telemetry.groundspeed_ms is not None:
                parts.append(f"gs {telemetry.groundspeed_ms:.1f}m/s "
                             f"hdg {_fmt(telemetry.heading_deg, '.0f')}")
            if not self.mavlink.is_connected():
                parts.append("MAVLINK DOWN")
        return " | ".join(parts)

    # -- shutdown --------------------------------------------------------

    def close(self):
        """Finish the recording and release everything.

        The state file is removed, so a reader that finds it after the
        driver is gone does not show the last frame of the flight as live.
        """
        self.running = False
        path = getattr(self.args, "state_file", "")
        if path:
            try:
                os.unlink(path)
            except OSError:
                pass
        self.stop_recording(why="shutdown")
        self._fail_pending_commands()
        if self.grabber is not None:
            self.grabber.stop()
        if self.servo is not None:
            self.servo.stop()
        if self.rc is not None:
            self.rc.stop()
        if self.mavlink is not None:
            self.mavlink.stop()
        if self.link is not None:
            try:
                self.link.close()
            except Exception:
                pass
        log.info("shut down cleanly -- %d frames seen, %d skipped",
                 self.frames_seen, self.frames_dropped)


def _fmt(value, spec):
    return "?" if value is None else format(value, spec)


def _have_pymavlink():
    try:
        import pymavlink  # noqa: F401
    except ImportError:
        return False
    return True
