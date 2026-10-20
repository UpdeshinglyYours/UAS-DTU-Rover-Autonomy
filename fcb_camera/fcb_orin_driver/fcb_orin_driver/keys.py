"""The recorder pane's keyboard, as a ROS node: fcb_keys.

fcb_record.py read single keys from the tmux pane it ran in. The driver
node has no terminal, so this is that pane: run it in tmux on the drone
and SSH in, and the keys do what they did, by calling the driver's
services. It draws the live status line from ~/status, and a snapshot --
ch9 or s -- is drawn into the pane as colour text, the one picture that
crosses the link.

    1 2 3 4   RGB / IR / RGB+IR / AUTO        i  cycle them
    s         snapshot, drawn here            ?  status line now
    z         type an exact zoom, Enter       a  zoom back to the ch7 knob
    + -       zoom one detent in / out
    r         start; r r stops                x x  stop, then name the flight

Two presses to stop, within five seconds, as before: this is the one
irreversible key on a terminal anyone might lean on.

Unlike the pane, this process is not the recorder. Ctrl-C here, a closed
pane or a dead SSH session ends the keyboard and nothing else -- the
recording carries on under RC control.
"""
import os
import select
import shutil
import sys
import termios
import threading
import time
import tty
import queue
from datetime import datetime

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import CompressedImage
from std_srvs.srv import Trigger

from fcb_orin_msgs.msg import Status
from fcb_orin_msgs.srv import (RenameRecording, SetImagingMode, SetZoom,
                               StopRecording)

#: Seconds in which the second stop key must land.
STOP_CONFIRM_WINDOW = 5.0

MODE_KEYS = {"1": "rgb", "2": "ir", "3": "rgb+ir", "4": "auto"}

#: A status this old means the driver is gone or wedged, and says so.
STATUS_STALE = 3.0


class TerminalKeys:
    """Single keypresses from the terminal, without waiting for Enter.

    cbreak rather than raw, so Ctrl-C still works. Read from the fd, not
    sys.stdin: a buffered read pulls a whole burst into Python where
    select() cannot see it. Escape sequences (arrow keys) are swallowed.
    """

    def __init__(self, on_retire=None):
        self.enabled = sys.stdin.isatty()
        self._fd = sys.stdin.fileno() if self.enabled else None
        self._saved = None
        self._on_retire = on_retire or (lambda why: None)

    def __enter__(self):
        if self.enabled:
            try:
                self._saved = termios.tcgetattr(self._fd)
                tty.setcbreak(self._fd)
            except termios.error:
                self.enabled = False
        return self

    def __exit__(self, *exc):
        self.restore()

    def restore(self):
        if self._saved is not None:
            try:
                termios.tcsetattr(self._fd, termios.TCSADRAIN, self._saved)
            except termios.error:
                pass
            self._saved = None

    def get(self):
        if not self.enabled or not self._ready():
            return None
        try:
            data = os.read(self._fd, 1)
        except OSError as exc:
            self._retire(f"the terminal went away ({exc})")
            return None
        if not data:
            # EOF: the pane was closed. select() now reports this fd
            # readable for ever and every read is empty -- a drain loop on
            # it spins at full tilt -- so the keyboard is retired, once.
            self._retire("the terminal closed")
            return None
        if data != b"\x1b":
            return data.decode("utf-8", "replace")
        if not self._ready():
            return None
        try:
            if os.read(self._fd, 1) not in (b"[", b"O"):
                return None
            while self._ready():
                final = os.read(self._fd, 1)
                if not final or 0x40 <= final[0] <= 0x7E:
                    break
        except OSError:
            pass
        return None

    def pending(self):
        return self.enabled and self._ready()

    def wait(self, timeout):
        """Block until a key is waiting or `timeout` passes."""
        if not self.enabled:
            time.sleep(timeout)
            return False
        return self._ready(timeout)

    def _retire(self, why):
        if not self.enabled:
            return
        self.enabled = False
        self._on_retire(why)

    def _ready(self, timeout=0.0):
        return bool(select.select([self._fd], [], [], timeout)[0])


class LiveLine:
    """One status line that repaints in place; nothing off a terminal.

    Gives up on a terminal that has gone (EIO on write) rather than raise.
    """

    def __init__(self, stream=None):
        self.stream = stream or sys.stdout
        self.enabled = self.stream.isatty()
        self._shown = False
        self.lock = threading.RLock()

    def show(self, text):
        if not self.enabled:
            return
        width = shutil.get_terminal_size((120, 24)).columns
        with self.lock:
            self._write("\r\033[K" + text[:width - 1])
            self._shown = True

    def clear(self):
        if not self.enabled or not self._shown:
            return
        with self.lock:
            self._write("\r\033[K")
            self._shown = False

    def _write(self, text):
        try:
            self.stream.write(text)
            self.stream.flush()
        except (OSError, ValueError):
            self.enabled = False


class KeysNode(Node):

    def __init__(self):
        super().__init__("fcb_keys")
        driver = self.declare_parameter("driver", "/fcb_driver").value.rstrip("/")
        self.preview = self.declare_parameter("preview", True).value
        self.preview_cols = self.declare_parameter("preview_cols", 100).value
        self.zoom_step = self.declare_parameter("zoom_step", 0.5).value

        self.live = LiveLine()
        self.ui = queue.Queue()        # actions from callbacks, run on the key loop
        self.status = None
        self.status_at = None
        self._stop_armed = 0.0
        self.zoom_entry = None
        self.name_entry = None
        self.name_base = None

        self.create_subscription(Status, f"{driver}/status", self._on_status, 10)
        self.create_subscription(
            CompressedImage, f"{driver}/snapshot/compressed", self._on_snapshot,
            QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE))

        def client(kind, name):
            return self.create_client(kind, f"{driver}/{name}")
        self.c_start = client(Trigger, "start_recording")
        self.c_stop = client(StopRecording, "stop_recording")
        self.c_rename = client(RenameRecording, "rename_recording")
        self.c_snap = client(Trigger, "snapshot")
        self.c_zoom = client(SetZoom, "set_zoom")
        self.c_knob = client(Trigger, "zoom_to_rc")
        self.c_mode = client(SetImagingMode, "set_imaging_mode")
        self.c_cycle = client(Trigger, "cycle_imaging_mode")
        self.driver = driver

    # -- output ----------------------------------------------------------

    def say(self, text, *args):
        line = f"{datetime.now():%H:%M:%S}  " + (text % args if args else text)
        with self.live.lock:
            self.live.clear()
            try:
                print(line, flush=True)
            except (OSError, ValueError):
                pass

    def redraw(self):
        if self.zoom_entry is not None:
            self.live.show(f"zoom> {self.zoom_entry}x  [1-30, Enter]")
        elif self.name_entry is not None:
            keep = os.path.basename(self.name_base or "")
            self.live.show(f"name this flight> {self.name_entry}   [Enter keeps {keep}]")
        else:
            self.live.show(self.status_text())

    def status_text(self):
        if self.status is None:
            return f"waiting for {self.driver}/status ..."
        age = time.monotonic() - self.status_at
        if age > STATUS_STALE:
            return f"NO STATUS FROM THE DRIVER FOR {age:.0f}s | " + self.status.text
        return self.status.text

    # -- callbacks (executor thread) -------------------------------------

    def _on_status(self, msg):
        self.status, self.status_at = msg, time.monotonic()

    def _on_snapshot(self, msg):
        if not self.preview:
            return
        self.ui.put(lambda: self._draw_snapshot(bytes(msg.data)))

    def _draw_snapshot(self, data):
        from fcb_orin_driver import prefer_cv2  # noqa: F401
        import cv2
        import numpy as np
        from fcb_orin_driver import snapshot
        frame = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            self.say("snapshot: could not decode the JPEG")
            return
        text, cols, rows = snapshot.render(frame, max_cols=self.preview_cols)
        with self.live.lock:
            self.live.clear()
            try:
                sys.stdout.write(text + "\n")
                sys.stdout.flush()
            except (OSError, ValueError):
                return
        self.say("snapshot: %dx%d preview (%.0f KB)", cols, rows,
                 len(text.encode("utf-8")) / 1024.0)

    def call(self, client, request, then=None):
        """Call a service without blocking the keys; report the answer."""
        if not client.service_is_ready():
            self.say("%s is not available -- is the driver running?",
                     client.srv_name)
            return
        future = client.call_async(request)

        def done(fut):
            try:
                response = fut.result()
            except Exception as exc:
                self.ui.put(lambda: self.say("%s failed: %s", client.srv_name, exc))
                return
            if then is not None:
                self.ui.put(lambda: then(response))
            else:
                self.ui.put(lambda: self.say("%s%s", "" if response.success
                                             else "FAILED: ", response.message))
        future.add_done_callback(done)

    # -- keys (main thread) ----------------------------------------------

    def handle_key(self, key):
        if self.zoom_entry is not None:
            return self.handle_zoom_entry(key)
        if self.name_entry is not None:
            return self.handle_name_entry(key)

        if key in MODE_KEYS:
            self.call(self.c_mode, SetImagingMode.Request(mode=MODE_KEYS[key]))
        elif key in ("i", "I"):
            self.call(self.c_cycle, Trigger.Request())
        elif key in ("r", "R"):
            if self.status is not None and self.status.recording:
                self.request_stop("r")
            else:
                self.call(self.c_start, Trigger.Request())
        elif key in ("x", "X"):
            self.request_stop("x")
        elif key in ("z", "Z"):
            self.zoom_entry = ""
            self.redraw()
        elif key in ("a", "A"):
            self.call(self.c_knob, Trigger.Request())
        elif key in ("+", "=", "-", "_"):
            self.step_zoom(1 if key in ("+", "=") else -1)
        elif key in ("s", "S"):
            self.call(self.c_snap, Trigger.Request())
        elif key == "?":
            self.say("status: %s", self.status_text())
            self.announce_keys()

    def announce_keys(self):
        self.say("keys: 1 RGB  2 IR  3 RGB+IR  4 AUTO  i cycle  s snap  "
                 "z zoom  +/- detent  a knob back  r REC/STOP (r r)  "
                 "x STOP+NAME (twice)  ? status  (Ctrl-C leaves the keys; "
                 "recording carries on)")

    def request_stop(self, key):
        if self.status is not None and not self.status.recording:
            self.say("not recording -- nothing to stop")
            return
        now = time.monotonic()
        if now - self._stop_armed > STOP_CONFIRM_WINDOW:
            self._stop_armed = now
            self.say("press %s again within %.0fs to STOP the recording",
                     key, STOP_CONFIRM_WINDOW)
            return
        self._stop_armed = 0.0

        def stopped(response):
            self.say("%s%s", "" if response.success else "FAILED: ",
                     response.message)
            # Ask now, while the pilot still remembers which pass it was.
            if response.success and response.files:
                self.name_base = response.base
                self.name_entry = ""
                self.redraw()
        self.call(self.c_stop, StopRecording.Request(), then=stopped)

    def step_zoom(self, detents):
        if self.status is None or self.status.zoom_ratio != self.status.zoom_ratio:
            self.say("zoom position unknown -- no camera control yet")
            return
        from fcb_base_driver import zoom_map
        step = self.zoom_step or zoom_map.DEFAULT_RATIO_STEP
        here = self.status.zoom_target_ratio
        if here != here:     # NaN
            here = self.status.zoom_ratio
        target = zoom_map.snap_ratio(here + detents * step, step)
        self.call(self.c_zoom, SetZoom.Request(ratio=float(target)))

    def handle_zoom_entry(self, key):
        if key in ("\r", "\n"):
            text, self.zoom_entry = self.zoom_entry, None
            if not text:
                self.say("zoom: entry cancelled")
                return
            try:
                wanted = float(text)
            except ValueError:
                self.say("zoom: %r is not a number", text)
                return
            self.call(self.c_zoom, SetZoom.Request(ratio=wanted))
            return
        if key in ("\x7f", "\b"):
            self.zoom_entry = self.zoom_entry[:-1]
        elif key.isdigit() or (key == "." and "." not in self.zoom_entry):
            if len(self.zoom_entry) < 4:
                self.zoom_entry += key
        self.redraw()

    def handle_name_entry(self, key):
        if key in ("\r", "\n"):
            text, self.name_entry = self.name_entry, None
            base, self.name_base = self.name_base, None
            if not text.strip():
                self.say("kept the timestamp name: %s", os.path.basename(base or "?"))
                return
            self.call(self.c_rename, RenameRecording.Request(base=base, name=text))
            return
        if key in ("\x7f", "\b"):
            self.name_entry = self.name_entry[:-1]
        elif key.isprintable() and len(self.name_entry) < 60:
            self.name_entry += key
        self.redraw()

    # -- loop ------------------------------------------------------------

    def loop(self):
        retired = []
        keys = TerminalKeys(on_retire=retired.append)
        if not keys.enabled:
            self.say("stdin is not a terminal -- fcb_keys needs one. Run it "
                     "in the tmux pane you SSH into.")
            return 1
        with keys:
            self.announce_keys()
            last_draw = 0.0
            while rclpy.ok() and not retired:
                keys.wait(0.1)
                for _ in range(64):
                    if not keys.pending():
                        break
                    key = keys.get()
                    if key is not None:
                        try:
                            self.handle_key(key)
                        except Exception as exc:
                            self.say("key %r failed: %s", key, exc)
                for _ in range(16):
                    try:
                        action = self.ui.get_nowait()
                    except queue.Empty:
                        break
                    try:
                        action()
                    except Exception as exc:
                        self.say("%s", exc)
                now = time.monotonic()
                if now - last_draw >= 0.25:
                    last_draw = now
                    self.redraw()
        self.live.clear()
        if retired:
            self.say("%s -- the keyboard is gone; the driver and any "
                     "recording carry on", retired[0])
        return 0


def main(argv=None):
    rclpy.init(args=argv)
    node = KeysNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def spin():
        try:
            executor.spin()
        except Exception:
            pass    # shutting down; the key loop notices rclpy.ok()
    threading.Thread(target=spin, daemon=True).start()
    status = 0
    try:
        status = node.loop()
    except KeyboardInterrupt:
        pass
    finally:
        node.live.clear()
        executor.shutdown(timeout_sec=1.0)
        node.destroy_node()
        rclpy.try_shutdown()
    return status


if __name__ == "__main__":
    sys.exit(main())
