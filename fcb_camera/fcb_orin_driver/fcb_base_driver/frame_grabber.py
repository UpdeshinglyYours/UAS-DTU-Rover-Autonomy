"""Threaded capture that keeps only the newest frame.

VideoCapture.read() blocks until the next frame, so calling it inline
would add a frame period to every consumer iteration on top of whatever
else that loop does.

The colour conversion deliberately does not happen on the grab thread.
The FCB camera offers only raw YUYV 4:2:2 at 1920x1080, and read()
converts that to BGR -- 6 MB a frame, ~370 MB/s at 60 fps -- on the same
thread that services the V4L2 buffer queue. Whenever that conversion was
delayed (CPU scheduling on a hybrid P/E-core laptop under the powersave
governor was enough), the queue starved and the stream hitched for
~170 ms several times a minute, while `v4l2-ctl --stream-mmap` on the same
camera -- which never converts -- held a flat 59.94 fps. So the grab
thread only dequeues, and conversion happens on the consumer's thread,
once per captured frame.

Only the newest frame is kept, so a consumer slower than the camera skips
frames rather than falling behind. Sequence numbers are exposed precisely
so that consumer can tell how many it missed.
"""
import threading
import time

import cv2


def _same_frame(a, b):
    """Whether two frames are byte-identical, judged on a sparse grid.

    One pixel in 16 each way is ~0.4% of the frame -- cheap at 80 fps, and
    still thousands of samples, so a real frame's noise always shows.
    """
    return (b is not None and a.shape == b.shape
            and (a[::16, ::16] == b[::16, ::16]).all())


class FrameGrabber:

    def __init__(self, capture, yuv_code=cv2.COLOR_YUV2BGR_YUYV,
                 drop_repeats=False):
        """`yuv_code` decodes raw two-channel frames; it must match the
        packing the camera negotiated (devices.yuv422_to_bgr_code), since
        YUYV and UYVY frames have the same shape.

        `drop_repeats` discards a frame identical to the one before it. The
        Oppila board sends every third camera frame twice -- 78.7 frames a
        second carrying the camera's ~59 -- and passing the copies on would
        both stutter the video and inflate the measured rate it is tagged
        with. Two real frames are never identical, even of a still scene:
        sensor noise alone differed by ~0.67 grey levels on average.
        """
        self._capture = capture
        self._yuv_code = yuv_code
        self._drop_repeats = drop_repeats
        self._dropped_last = False
        self.repeats_dropped = 0
        self._raw = None
        self._seq = 0
        self._captured_at = None
        self._bgr = None
        self._bgr_seq = -1
        self._lock = threading.Lock()
        self._new_frame = threading.Condition(self._lock)
        self._running = True
        self._times = []
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    # -- capture thread --------------------------------------------------

    def _run(self):
        while self._running:
            ok, frame = self._capture.read()
            if not ok:
                time.sleep(0.05)
                continue
            # At most one in a row. The board repeats a single frame, never
            # two; a run of identical frames is the picture itself -- a blank
            # one when the board gets no image from the camera -- and
            # dropping it all made a live, blank camera look like no camera:
            # the recorder saw no frames, reopened the device every 5 s, and
            # that churn wedged the board.
            if (self._drop_repeats and not self._dropped_last
                    and _same_frame(frame, self._raw)):
                self._dropped_last = True
                self.repeats_dropped += 1
                continue
            self._dropped_last = False
            now = time.monotonic()
            with self._new_frame:
                self._raw = frame
                self._seq += 1
                self._captured_at = now
                self._times.append(now)
                self._times = [t for t in self._times if now - t < 2.0]
                self._new_frame.notify_all()

    # -- consumer API ----------------------------------------------------

    def sequence(self):
        """Count of frames captured so far, to spot a repeated latest()."""
        with self._lock:
            return self._seq

    def latest(self):
        """Newest frame as BGR, or None. The caller must not draw on it."""
        frame, _, _ = self.latest_meta()
        return frame

    def latest_meta(self):
        """(frame, sequence, captured_at) for the newest frame.

        `captured_at` is the monotonic time this frame was dequeued, which
        is what a consumer should timestamp it with -- not the time it got
        around to asking.
        """
        with self._lock:
            raw, seq, stamp = self._raw, self._seq, self._captured_at
            if seq == self._bgr_seq:
                return self._bgr, seq, stamp
        if raw is None:
            return None, seq, stamp
        # Two channels per pixel means CAP_PROP_CONVERT_RGB stuck and these
        # are raw 4:2:2 bytes; anything else is already converted.
        if raw.ndim == 3 and raw.shape[2] == 2:
            frame = cv2.cvtColor(raw, self._yuv_code)
        else:
            frame = raw
        with self._lock:
            self._bgr, self._bgr_seq = frame, seq
        return frame, seq, stamp

    def wait_for_frame(self, after_seq, timeout=1.0):
        """Block until a frame newer than `after_seq` arrives.

        Returns (frame, sequence, captured_at), or (None, after_seq, None)
        on timeout. Lets a headless consumer pace itself on the camera
        instead of spinning.
        """
        with self._new_frame:
            if self._seq <= after_seq:
                self._new_frame.wait(timeout)
            if self._seq <= after_seq:
                return None, after_seq, None
        return self.latest_meta()

    #: Frames needed before a measured rate means anything. Two samples an
    #: instant apart imply hundreds of fps, and that figure is not merely
    #: cosmetic -- a recording started in the first moments would stamp it
    #: into the video container and play back at the wrong speed.
    MIN_SAMPLES_FOR_FPS = 5

    def fps(self):
        """Measured capture rate, or 0.0 until there is enough to measure."""
        with self._lock:
            times = list(self._times)
        if len(times) < self.MIN_SAMPLES_FOR_FPS:
            return 0.0
        span = times[-1] - times[0]
        return (len(times) - 1) / span if span > 0 else 0.0

    def stop(self):
        self._running = False
        self._thread.join(timeout=2.0)
        if self._thread.is_alive():
            # Still blocked inside read(). Releasing the capture out from
            # under a thread sitting in a V4L2 ioctl risks leaving the
            # device wedged -- every later open then times out until the
            # camera is replugged. Process teardown closes the fd properly,
            # so leave it to that rather than racing it here.
            return
        self._capture.release()
