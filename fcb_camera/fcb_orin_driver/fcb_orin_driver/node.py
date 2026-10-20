"""ROS 2 node for the Sony FCB-EV9520L: camera, VISCA control and recorder.

One process owns the camera, the VISCA port and the flight controller link,
as fcb_record.py did -- the board cannot be shared, and the recorder must
keep running for the whole session (closing the stream on the Oppila board
needs a 12 V power cycle to recover; see the README).

Topics (all under the node's namespace, ~/):
    image_raw            sensor_msgs/Image, bgr8, every frame (publish_rate_hz)
    camera_info          sensor_msgs/CameraInfo, alongside image_raw
    snapshot/compressed  sensor_msgs/CompressedImage, one JPEG per snapshot
    status               fcb_orin_msgs/Status, status_interval (4 Hz)
    status_text          std_msgs/String, the recorder's status line

Services:
    start_recording      std_srvs/Trigger        r in the pane
    stop_recording       fcb_orin_msgs/StopRecording   x x
    rename_recording     fcb_orin_msgs/RenameRecording the name prompt
    snapshot             std_srvs/Trigger        s, or RC ch9
    set_zoom             fcb_orin_msgs/SetZoom   z 7.3 Enter
    zoom_to_rc           std_srvs/Trigger        a
    set_imaging_mode     fcb_orin_msgs/SetImagingMode  1-4, or RC ch10
    cycle_imaging_mode   std_srvs/Trigger        i
"""
import array
import logging
import math
import os
import signal
import sys
import threading
import time
import traceback
import types
from datetime import datetime

from fcb_orin_driver import prefer_cv2  # noqa: F401  -- before cv2
import cv2

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data

from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

from fcb_orin_msgs.msg import Status
from fcb_orin_msgs.srv import (RenameRecording, SetImagingMode, SetZoom,
                               StopRecording)

from fcb_base_driver import zoom_map
from fcb_orin_driver import recorder as rec
from fcb_orin_driver.recorder import ICR_LABELS, ICR_SHORT, Recorder, log

#: Every parameter, with fcb_record.py's defaults. Names are its command
#: line options with dashes as underscores; "" and 0 stand for "not given"
#: where the option had no default (video, port, baud, fourcc, ...).
PARAMETERS = {
    # camera
    "video": "",                    # UVC device; "" autodetects by USB ID
    "width": 1920,
    "height": 1080,
    "fps": 60.0,                    # nominal rate when the board misreports
    "fourcc": "",                   # force a capture format, e.g. MJPG
    "camera_timeout": 5.0,          # s without frames before reopening
    "board_power_cycle_cmd": os.environ.get("FCB_BOARD_POWER_CYCLE", ""),
    # VISCA
    "port": "",                     # "" autodetects by protocol
    "baud": 0,                      # 0 tries 9600, 38400, 115200
    "address": 1,
    "stabilizer": "on",             # on | off | keep
    "icr": "",                      # rgb | ir | rgb+ir | auto; "" keeps as found
    "curve": "ratio",               # ratio | log | raw
    "zoom_step": zoom_map.DEFAULT_RATIO_STEP,
    # flight controller
    "fc_source": "pymavlink",       # pymavlink | mavros | none
    "mavros_namespace": "/mavros",
    "rc_url": "",                   # "" autodetects by heartbeat
    "rc_baud": 115200,
    "stream_rate": 10,
    "require_mavlink": False,
    "rc_channel": 7,
    "rc_pwm_min": 1000,
    "rc_pwm_max": 2000,
    "rc_deadband": 8,
    "rc_reverse": False,
    "rc_timeout": 2.0,
    "rec_channel": 8,
    "rec_channel_threshold": 1500,
    "rec_channel_reverse": False,
    "rec_debounce": 0.3,
    "snap_channel": 9,
    "snap_threshold": 1500,
    "snap_reverse": False,
    "icr_channel": 10,
    "icr_channel_reverse": False,
    # recording
    "record_dir": os.path.expanduser(
        os.environ.get("FCB_RECORD_DIR", "~/flight_recordings")),
    "autostart": False,
    "container": "avi",             # avi | mp4
    "encoder": "nvenc",             # nvenc (falls back to cpu) | cpu
    "bitrate": 25.0,                # Mbps
    "telemetry_interval": 0.1,      # s between rows while there is no video
    # snapshots
    "snap_dir": "",                 # "" = <record_dir>/snapshots
    "snap_width": 960,
    "snap_quality": 70,
    "preview": False,               # also draw it here, if this process has a terminal
    "preview_cols": 100,
    # status and logs
    "status_interval": 0.25,
    "log_status_interval": 5.0,
    "state_file": os.path.join(os.environ.get("XDG_RUNTIME_DIR") or "/tmp",
                               "fcb_gcs_state"),
    "log_level": "DEBUG",           # detail in the daily log file
    "log_to_file": True,
    # ROS only
    "frame_id": "fcb_camera_optical_frame",
    "publish_image": True,
    "publish_rate_hz": 0.0,         # 0 = every frame
    "publish_rotate": 0,            # 0 | 180; published images only, not recordings
    "camera_info_url": "",          # calibration YAML (file:// or a path)
    "publish_compressed": False,    # ~/image_raw/compressed, a JPEG stream for bags
    "compressed_rate_hz": 10.0,
    "compressed_width": 960,        # 0 = full size
    "compressed_quality": 80,
}

CHOICES = {
    "stabilizer": ("on", "off", "keep"),
    "curve": zoom_map.CURVES,
    "fc_source": ("pymavlink", "mavros", "none"),
    "container": tuple(rec.CONTAINERS),
    "encoder": ("nvenc", "cpu"),
    "log_level": ("DEBUG", "INFO", "WARNING"),
    "publish_rotate": (0, 180),
}


class RosLogHandler(logging.Handler):
    """Hands the recorder's log records to the node's ROS logger."""

    def __init__(self, logger):
        super().__init__(logging.INFO)
        self._logger = logger

    def emit(self, record):
        try:
            text = record.getMessage()
            if record.exc_info:
                text += "\n" + "".join(traceback.format_exception(*record.exc_info))
            if record.levelno >= logging.CRITICAL:
                self._logger.fatal(text)
            elif record.levelno >= logging.ERROR:
                self._logger.error(text)
            elif record.levelno >= logging.WARNING:
                self._logger.warning(text)
            elif record.levelno >= logging.INFO:
                self._logger.info(text)
            else:
                self._logger.debug(text)
        except Exception:
            pass   # a log line is never worth a recording


def setup_logging(node, args):
    """Log to ROS and, in full detail, to one file per day in record_dir.

    One file a day, appended to, keeps every run of that day in one place
    -- a respawning node would otherwise strand a near-empty log per crash.
    """
    root = logging.getLogger("fcb")
    root.setLevel(logging.DEBUG)
    root.handlers.clear()
    root.propagate = False
    root.addHandler(RosLogHandler(node.get_logger()))
    if not args.log_to_file:
        return None
    path = None
    try:
        os.makedirs(args.record_dir, exist_ok=True)
        path = os.path.join(args.record_dir,
                            f"fcb_record-{datetime.now():%Y%m%d}.log")
        if os.path.exists(path) and os.path.getsize(path) > 0:
            with open(path, "a") as fh:
                fh.write("\n===== run started %s =====\n"
                         % datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
        to_file = logging.FileHandler(path, mode="a")
        to_file.setLevel(getattr(logging, args.log_level))
        to_file.setFormatter(logging.Formatter(
            "%(asctime)s.%(msecs)03d %(levelname)-7s %(message)s",
            datefmt="%H:%M:%S"))
        root.addHandler(to_file)
    except OSError as exc:
        node.get_logger().error(f"cannot write the log file {path}: {exc}")
        return None
    return path


def load_camera_info(url):
    """A CameraInfo from a camera_calibration YAML, or None."""
    if not url:
        return None
    path = url[len("file://"):] if url.startswith("file://") else url
    path = os.path.expanduser(path)
    import yaml
    with open(path) as fh:
        calib = yaml.safe_load(fh)
    info = CameraInfo()
    info.width = int(calib.get("image_width", 0))
    info.height = int(calib.get("image_height", 0))
    info.distortion_model = calib.get("distortion_model", "plumb_bob")
    info.d = [float(v) for v in calib.get("distortion_coefficients", {}).get("data", [])]
    for field, key in (("k", "camera_matrix"), ("r", "rectification_matrix"),
                       ("p", "projection_matrix")):
        data = calib.get(key, {}).get("data")
        if data:
            setattr(info, field, [float(v) for v in data])
    return info


class ImagePublisher:
    """Publishes frames off the capture thread, newest wins.

    Turning a 1080p frame into a message is a 6 MB copy, and the capture
    thread's budget is one frame period with the encoder in it, so this
    gets its own thread. A subscriber that cannot keep up costs published
    frames, never recorded ones.
    """

    def __init__(self, node, args):
        self._node = node
        self._args = args
        self._image_pub = node.create_publisher(Image, "~/image_raw",
                                                qos_profile_sensor_data)
        self._info_pub = node.create_publisher(CameraInfo, "~/camera_info",
                                               qos_profile_sensor_data)
        self._info = None
        try:
            self._info = load_camera_info(args.camera_info_url)
        except Exception as exc:
            log.warning("camera_info_url %s could not be read: %s -- "
                        "publishing an uncalibrated CameraInfo",
                        args.camera_info_url, exc)
        self._min_period = 1.0 / args.publish_rate_hz if args.publish_rate_hz > 0 else 0.0
        self._last = 0.0
        self._pending = None
        self._cond = threading.Condition()
        self._running = True
        self._thread = threading.Thread(target=self._run, name="image-pub",
                                        daemon=True)
        self._thread.start()

    def submit(self, frame, captured_at):
        if captured_at is None or captured_at - self._last < self._min_period:
            return
        if (self._image_pub.get_subscription_count() == 0
                and self._info_pub.get_subscription_count() == 0):
            return
        self._last = captured_at
        with self._cond:
            self._pending = (frame, captured_at)
            self._cond.notify()

    def stop(self):
        with self._cond:
            self._running = False
            self._cond.notify()
        self._thread.join(1.0)

    def _run(self):
        while True:
            with self._cond:
                while self._running and self._pending is None:
                    self._cond.wait()
                if not self._running:
                    return
                (frame, captured_at), self._pending = self._pending, None
            try:
                self._publish(frame, captured_at)
            except Exception:
                log.exception("image publish failed; continuing")

    def _publish(self, frame, captured_at):
        if self._args.publish_rotate == 180:
            frame = cv2.rotate(frame, cv2.ROTATE_180)
        stamp = stamp_for(self._node, captured_at)
        height, width = frame.shape[:2]
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = self._args.frame_id
        msg.height, msg.width = height, width
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = width * 3
        msg.data = array.array("B", frame.tobytes())
        self._image_pub.publish(msg)

        info = CameraInfo() if self._info is None else self._info
        info.header = msg.header
        if not info.width:
            info.width, info.height = width, height
        self._info_pub.publish(info)


class CompressedPublisher(ImagePublisher):
    """A downscaled JPEG stream on ~/image_raw/compressed, for rosbags.

    Raw 1080p60 is ~370 MB/s, which no bag keeps up with; the full-quality
    video is the driver's own AVI. This is the reviewable copy alongside the
    telemetry: compressed_width pixels wide, at compressed_rate_hz.
    """

    def __init__(self, node, args):
        self._node = node
        self._args = args
        self._pub = node.create_publisher(CompressedImage, "~/image_raw/compressed",
                                          qos_profile_sensor_data)
        rate = args.compressed_rate_hz
        self._min_period = 1.0 / rate if rate > 0 else 0.0
        self._last = 0.0
        self._pending = None
        self._cond = threading.Condition()
        self._running = True
        self._thread = threading.Thread(target=self._run, name="jpeg-pub",
                                        daemon=True)
        self._thread.start()

    def submit(self, frame, captured_at):
        if captured_at is None or captured_at - self._last < self._min_period:
            return
        if self._pub.get_subscription_count() == 0:
            return
        self._last = captured_at
        with self._cond:
            self._pending = (frame, captured_at)
            self._cond.notify()

    def _publish(self, frame, captured_at):
        if self._args.publish_rotate == 180:
            frame = cv2.rotate(frame, cv2.ROTATE_180)
        width = self._args.compressed_width
        if 0 < width < frame.shape[1]:
            height = int(round(frame.shape[0] * width / frame.shape[1]))
            frame = cv2.resize(frame, (width, height), interpolation=cv2.INTER_AREA)
        ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY),
                                               int(self._args.compressed_quality)])
        if not ok:
            return
        msg = CompressedImage()
        msg.header.stamp = stamp_for(self._node, captured_at)
        msg.header.frame_id = self._args.frame_id
        msg.format = "bgr8; jpeg compressed bgr8"
        msg.data = array.array("B", buf.tobytes())
        self._pub.publish(msg)


def stamp_for(node, captured_at):
    """ROS time of a frame dequeued at monotonic `captured_at`."""
    now_ros = node.get_clock().now()
    age_ns = int(max(0.0, time.monotonic() - captured_at) * 1e9)
    return rclpy.time.Time(nanoseconds=now_ros.nanoseconds - age_ns,
                           clock_type=now_ros.clock_type).to_msg()


def _nan(value):
    return float("nan") if value is None else float(value)


class FcbDriverNode(Node):

    def __init__(self):
        super().__init__("fcb_driver")
        self.args = self._read_parameters()
        self.log_path = setup_logging(self, self.args)

        self.recorder = Recorder(self.args)
        self.recorder.make_mavros = self._make_mavros
        self.recorder.on_status = self._on_status
        self.recorder.on_snapshot = self._on_snapshot

        self.frame_sinks = []
        if self.args.publish_image:
            self.frame_sinks.append(ImagePublisher(self, self.args))
        if self.args.publish_compressed:
            self.frame_sinks.append(CompressedPublisher(self, self.args))
        if self.frame_sinks:
            def on_frame(frame, captured_at):
                for sink in self.frame_sinks:
                    sink.submit(frame, captured_at)
            self.recorder.on_frame = on_frame

        self.status_pub = self.create_publisher(Status, "~/status", 10)
        self.status_text_pub = self.create_publisher(String, "~/status_text", 10)
        self.snapshot_pub = self.create_publisher(
            CompressedImage, "~/snapshot/compressed",
            QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE))

        group = ReentrantCallbackGroup()
        self.create_service(Trigger, "~/start_recording", self._srv_start,
                            callback_group=group)
        self.create_service(StopRecording, "~/stop_recording", self._srv_stop,
                            callback_group=group)
        self.create_service(RenameRecording, "~/rename_recording",
                            self._srv_rename, callback_group=group)
        self.create_service(Trigger, "~/snapshot", self._srv_snapshot,
                            callback_group=group)
        self.create_service(SetZoom, "~/set_zoom", self._srv_set_zoom,
                            callback_group=group)
        self.create_service(Trigger, "~/zoom_to_rc", self._srv_zoom_to_rc,
                            callback_group=group)
        self.create_service(SetImagingMode, "~/set_imaging_mode",
                            self._srv_set_mode, callback_group=group)
        self.create_service(Trigger, "~/cycle_imaging_mode",
                            self._srv_cycle_mode, callback_group=group)

    # -- parameters ------------------------------------------------------

    def _read_parameters(self):
        values = {}
        for name, default in PARAMETERS.items():
            # Dynamically typed and coerced to the default's type, so
            # compressed_rate_hz:=10 or fps:=60 from a command line or a
            # launch file is not refused for being an int.
            value = self.declare_parameter(
                name, default, ParameterDescriptor(dynamic_typing=True)).value
            if isinstance(default, bool):
                value = value if isinstance(value, bool) else \
                    str(value).strip().lower() in ("true", "1", "yes", "on")
            elif isinstance(default, (int, float)):
                value = type(default)(value)
            elif isinstance(default, str) and not isinstance(value, str):
                value = str(value)
            values[name] = value
        for name, allowed in CHOICES.items():
            if values[name] not in allowed:
                raise ValueError(f"parameter {name}={values[name]!r}: expected "
                                 f"one of {', '.join(map(str, allowed))}")
        if values["icr"]:
            rec.resolve_icr(values["icr"])     # fail now, not mid-flight
        if values["fc_source"] == "none" and values["require_mavlink"]:
            raise ValueError("fc_source none and require_mavlink contradict "
                             "each other")
        values["record_dir"] = os.path.expanduser(values["record_dir"])
        if values["snap_dir"]:
            values["snap_dir"] = os.path.expanduser(values["snap_dir"])
        values["no_mavlink"] = values["fc_source"] == "none"
        return types.SimpleNamespace(**values)

    def _make_mavros(self):
        from fcb_orin_driver.mavros_source import MavrosSource
        return MavrosSource(self, self.args.mavros_namespace,
                            on_status=lambda msg: log.info("MAVROS: %s", msg))

    # -- outputs ---------------------------------------------------------

    def _on_status(self, text, now):
        r = self.recorder
        msg = Status()
        msg.stamp = self.get_clock().now().to_msg()
        msg.text = text
        msg.video_open = r.grabber is not None
        msg.fps = float(r.grabber.fps()) if r.grabber else 0.0
        msg.frames_seen = int(r.frames_seen)
        msg.frames_skipped = int(r.frames_dropped)
        msg.repeats_dropped = int(getattr(r.grabber, "repeats_dropped", 0) or 0)
        msg.picture_blank = bool(r.picture_blank)
        msg.video_device = r.video_device or ""

        msg.recording = bool(r.recording)
        msg.recording_has_video = r.recording and r.video is not None
        msg.started_by_hand = bool(r._by_hand)
        msg.write_failed = bool(r._write_failed)
        msg.recording_base = (r.base or "") if r.recording else ""
        msg.segment = int(r.segment) if r.recording else 0
        msg.segment_frames = int(r.video.frames) if r.video is not None else 0
        msg.recording_elapsed_s = (float(now - r.record_started_mono)
                                   if r.recording and r.record_started_mono
                                   else 0.0)
        msg.disk_free_gb = _nan(r.disk_free_gb)

        servo = r.servo
        # The servo owns the live link once it exists (it rebuilds it after
        # faults); r.link is only the one found at startup.
        msg.control_ok = (servo is not None and not getattr(servo, "_faults", 0)
                          and servo.updated_at is not None
                          and time.monotonic() - servo.updated_at < 5.0)
        msg.control_port = r.control_port or ""
        msg.control_baud = int(r.control_baud or -1)
        msg.zoom_ratio = _nan(servo.ratio if servo else None)
        msg.zoom_position = int(servo.position) if servo and servo.position is not None else -1
        msg.zoom_target_ratio = _nan(servo.target_ratio if servo else None)
        msg.zoom_source = servo.source if servo else ""
        msg.imaging_mode = ICR_SHORT.get(r.icr_mode, "")
        msg.snapshots = int(r.snapshots)

        msg.fc_source = self.args.fc_source
        msg.fc_present = r.mavlink is not None
        msg.fc_connected = bool(r.mavlink.is_connected()) if r.mavlink else False
        msg.rc_age_s = _nan(r.mavlink.rc_age() if r.mavlink else None)

        self.status_pub.publish(msg)
        self.status_text_pub.publish(String(data=text))

    def _on_snapshot(self, path, frame):
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.args.frame_id
        msg.format = "jpeg"
        data = None
        if path is not None:
            with open(path, "rb") as fh:
                data = fh.read()
        else:
            ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY),
                                                   int(self.args.snap_quality)])
            data = buf.tobytes() if ok else None
        if data is None:
            return
        # Humble's generated setter walks a bytes object int by int; an
        # array('B') is taken as it is.
        msg.data = array.array("B", data)
        self.snapshot_pub.publish(msg)

    # -- services --------------------------------------------------------
    #
    # Each runs its action on the capture thread (Recorder.submit), where
    # the recorder's state is owned. A loop blocked in a camera reopen, or
    # one still starting up, answers "busy" rather than hanging the caller.

    def _run(self, fn):
        try:
            return self.recorder.submit(fn, timeout=15.0), None
        except TimeoutError:
            return None, ("busy -- the driver is starting up or reopening "
                          "the camera; try again")
        except Exception as exc:
            return None, f"{type(exc).__name__}: {exc}"

    def _srv_start(self, request, response):
        result, error = self._run(lambda: self.recorder.request_start_recording(
            why="~/start_recording"))
        response.success, response.message = result if error is None else (False, error)
        return response

    def _srv_stop(self, request, response):
        def stop():
            ok, message, base = self.recorder.request_stop_recording(
                why="~/stop_recording")
            return ok, message, base, self.recorder.recorded_files(base)
        result, error = self._run(stop)
        if error is not None:
            response.success, response.message = False, error
            return response
        response.success, response.message, response.base, response.files = result
        return response

    def _srv_rename(self, request, response):
        def rename():
            base = request.base or self.recorder._last_stopped_base
            if not base:
                return False, "no finished recording to rename", "", []
            return self.recorder.rename_recording(base, request.name)
        result, error = self._run(rename)
        if error is not None:
            response.success, response.message = False, error
            return response
        response.success, response.message, response.base, response.files = result
        return response

    def _srv_snapshot(self, request, response):
        result, error = self._run(lambda: self.recorder.take_snapshot(why="service"))
        if error is not None:
            response.success, response.message = False, error
        else:
            response.success = bool(result)
            response.message = (f"snapshot {self.recorder.snapshots}" if result
                                 else "no frame yet")
        return response

    def _srv_set_zoom(self, request, response):
        result, error = self._run(lambda: self.recorder.apply_zoom(request.ratio))
        if error is not None:
            response.success, response.message, response.ratio = False, error, float("nan")
        else:
            response.success, response.ratio, response.message = result
        return response

    def _srv_zoom_to_rc(self, request, response):
        result, error = self._run(self.recorder.hand_zoom_to_rc)
        response.success, response.message = result if error is None else (False, error)
        return response

    def _srv_set_mode(self, request, response):
        try:
            mode = rec.resolve_icr(request.mode)
        except ValueError as exc:
            response.success, response.message = False, str(exc)
            return response
        result, error = self._run(lambda: self.recorder.set_icr(mode))
        response.mode = ICR_SHORT[mode]
        if error is not None:
            response.success, response.message = False, error
        else:
            response.success = bool(result)
            response.message = (ICR_LABELS[mode] if result else
                                "the camera did not take it -- see the log")
        return response

    def _srv_cycle_mode(self, request, response):
        result, error = self._run(self.recorder.cycle_icr)
        if error is not None:
            response.success, response.message = False, error
        else:
            ok, mode = result
            response.success = bool(ok)
            response.message = ICR_SHORT[mode] if ok else \
                "the camera did not take it -- see the log"
        return response

    def destroy_node(self):
        for sink in self.frame_sinks:
            sink.stop()
        return super().destroy_node()


def startup(node):
    """fcb_record.py's main(), step for step. Returns an exit status or None."""
    args, recorder = node.args, node.recorder
    log.info("fcb_driver starting -- recordings and logs in %s", args.record_dir)
    if node.log_path:
        log.info("log file: %s", node.log_path)
    log.debug("parameters: %s", vars(args))

    # Control before video: the grab thread streams 1080p continuously over
    # the same USB link the NeoHD board carries VISCA on, and probing baud
    # rates against a busy bus makes the handshake flaky.
    recorder.open_control()
    recorder.open_mavlink()
    if recorder.mavlink is None and args.require_mavlink:
        log.critical("require_mavlink is set and there is no flight "
                     "controller, so this run would record nothing. "
                     "Refusing to start.")
        return 2
    recorder.start_servo()
    if not recorder.open_camera():
        # Not fatal. Exiting only had the supervisor restart it every few
        # seconds -- churn the board does not survive -- while recording
        # nothing. The loop keeps trying.
        log.error("no camera at startup -- carrying on: ch%d still records "
                  "telemetry, and video joins when the camera appears",
                  args.rec_channel)
    if args.fc_source == "none":
        log.info("ready -- no flight controller in use: ~/start_recording "
                 "starts a recording, ~/stop_recording stops it")
    elif recorder.mavlink is None:
        log.error("ready -- NO FLIGHT CONTROLLER: channel %d cannot start a "
                  "recording until one is found. Call ~/start_recording to "
                  "record without it.", args.rec_channel)
    else:
        log.info("ready -- channel %d high starts recording, or call "
                 "~/start_recording", args.rec_channel)
    if args.autostart:
        recorder.request_start_recording(why="autostart")
    return None


def main(argv=None):
    # rclpy's own SIGINT handling raises KeyboardInterrupt wherever the main
    # thread happens to be -- mid-way through writing a frame, say -- and
    # invalidates the context under the executor. The recorder is stopped
    # the way fcb_record.py stopped it instead: the signal only asks, and
    # the loop finishes the recording and returns.
    rclpy.init(args=argv, signal_handler_options=SignalHandlerOptions.NO)
    try:
        node = FcbDriverNode()
    except Exception as exc:
        print(f"fcb_driver: {exc}", file=sys.stderr)
        rclpy.try_shutdown()
        return 1
    recorder = node.recorder

    def on_signal(signum, _frame):
        if recorder.running:
            log.info("caught %s -- finishing the recording and shutting down",
                     signal.Signals(signum).name)
        recorder.running = False

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    # An SSH session that dies of range sends SIGHUP, whose default action
    # kills -- leaving the video unfinalised at exactly the moment a flight
    # ends badly. A dropped link is not a request to stop recording; every
    # deliberate way to stop (Ctrl-C, SIGTERM, ros2 launch) still works.
    signal.signal(signal.SIGHUP, signal.SIG_IGN)

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    def spin():
        try:
            executor.spin()
        except Exception as exc:
            if recorder.running:
                log.error("ROS executor stopped (%s) -- finishing the "
                          "recording and shutting down", exc)
            recorder.running = False

    spinner = threading.Thread(target=spin, name="executor", daemon=True)
    spinner.start()

    status = 0
    try:
        status = startup(node)
        if status is None:
            status = 0
            if recorder.running:
                recorder.run()
    except Exception:
        log.exception("unhandled error")
        status = 1
    finally:
        recorder.close()
        executor.shutdown(timeout_sec=1.0)
        node.destroy_node()
        rclpy.try_shutdown()
    return status


if __name__ == "__main__":
    sys.exit(main())
