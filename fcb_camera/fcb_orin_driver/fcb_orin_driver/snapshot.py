"""Getting a look at what the camera sees, without a video link.

The drone has no downlink for video. The only channel back to the ground
station is the SSH session the recorder is already running in, and that is
a link you are also flying on -- filling it with a stream would cost
telemetry latency for a picture nobody is watching most of the time.

So the picture is sent on demand, once per button press, in two forms:

  - a JPEG written on the drone, downscaled but real, which is there to
    pull off later or right now if you want the detail;
  - a coarse colour rendering printed straight into the tmux pane, which
    is what actually crosses the link. It costs tens of kilobytes once,
    tells you where the camera is pointed and roughly what is in frame,
    and needs nothing installed on the ground station -- the terminal that
    is already open draws it.

The rendering uses the upper-half-block character with a foreground and a
background colour, so one character cell carries two pixels and the image
comes out with square pixels at the aspect ratio the camera shot it at.
"""
import os
import shutil
from datetime import datetime

from fcb_orin_driver import prefer_cv2  # noqa: F401  -- before cv2
import cv2

#: One cell is two stacked pixels: foreground paints the top half,
#: background the bottom.
HALF_BLOCK = "▀"

#: Colour resolution thrown away before rendering. The escape sequence for
#: a cell is only emitted when its colour differs from the previous cell's,
#: so coarsening the colours lengthens the runs and shrinks what crosses
#: the link. It matters more than it sounds: a real frame is noisy, and in
#: a dim scene the noise alone is enough to make almost every cell differ
#: from its neighbour and re-emit. Measured on a frame off the drone at
#: 64x18, this is 38 KB unquantized, 30 KB at 8, and 24 KB at 16 -- for a
#: difference you cannot see in a picture this small.
COLOR_QUANTUM = 16

#: Kept narrow on purpose. Preview cost is quadratic in width, and the
#: point of this is to be cheap enough to press without thinking.
DEFAULT_MAX_COLS = 100


def terminal_size(fallback=(100, 30)):
    """Columns and rows of the pane, falling back when there is no tty."""
    try:
        size = shutil.get_terminal_size(fallback)
        return max(20, size.columns), max(10, size.lines)
    except Exception:
        return fallback


def _as_bgr(frame):
    if frame.ndim == 2:
        return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    if frame.shape[2] == 4:
        return cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
    return frame


def fit(frame_shape, max_cols, max_rows):
    """Character-cell size that preserves the frame's aspect ratio.

    A cell holds two pixels vertically, so a `cols` x `rows` block of cells
    is a `cols` x `2*rows` grid of pixels. Matching that grid's aspect to
    the frame's is what keeps the picture from coming out stretched, which
    matters more than it sounds -- a squashed preview makes it genuinely
    hard to tell what you are looking at.
    """
    height, width = frame_shape[0], frame_shape[1]
    cols = max(8, int(max_cols))
    rows = max(4, int(round(cols * height / (2.0 * width))))
    if rows > max_rows:
        rows = max(4, int(max_rows))
        cols = max(8, int(round(rows * 2.0 * width / height)))
    return cols, rows


def to_ansi(frame, cols, rows):
    """Render a frame as 24-bit-colour half-block text."""
    frame = _as_bgr(frame)
    small = cv2.resize(frame, (cols, rows * 2), interpolation=cv2.INTER_AREA)
    if COLOR_QUANTUM > 1:
        small = (small // COLOR_QUANTUM) * COLOR_QUANTUM

    out = []
    for row in range(rows):
        top = small[row * 2]
        bottom = small[row * 2 + 1]
        last_fg = last_bg = None
        line = []
        for col in range(cols):
            tb, tg, tr = int(top[col][0]), int(top[col][1]), int(top[col][2])
            bb, bg_, br = (int(bottom[col][0]), int(bottom[col][1]),
                           int(bottom[col][2]))
            fg = (tr, tg, tb)
            bg = (br, bg_, bb)
            # Only re-state the colours that actually changed. On a normal
            # scene most neighbouring cells match, and this is where the
            # bulk of the saving comes from.
            if fg != last_fg and bg != last_bg:
                line.append("\x1b[38;2;%d;%d;%d;48;2;%d;%d;%dm" % (fg + bg))
            elif fg != last_fg:
                line.append("\x1b[38;2;%d;%d;%dm" % fg)
            elif bg != last_bg:
                line.append("\x1b[48;2;%d;%d;%dm" % bg)
            line.append(HALF_BLOCK)
            last_fg, last_bg = fg, bg
        line.append("\x1b[0m")
        out.append("".join(line))
    return "\n".join(out)


def render(frame, max_cols=DEFAULT_MAX_COLS, max_rows=None):
    """A preview sized to the pane. Returns (text, cols, rows)."""
    term_cols, term_rows = terminal_size()
    cols = min(int(max_cols), term_cols - 1)
    # Leave room for the status line, the log line announcing the preview,
    # and the shell prompt -- a preview that scrolls its own caption off
    # the top is worse than a slightly smaller one.
    rows_budget = max_rows if max_rows else max(6, term_rows - 6)
    cols, rows = fit(frame.shape, cols, rows_budget)
    return to_ansi(frame, cols, rows), cols, rows


def save(frame, directory, max_width=960, quality=70, prefix="snap"):
    """Write a downscaled JPEG of the frame. Returns (path, bytes).

    Downscaled because the point is a look, not an asset: the recording
    already holds every frame at full resolution, and a 1080p JPEG per
    button press would fill the card the recordings need.
    """
    os.makedirs(directory, exist_ok=True)
    frame = _as_bgr(frame)
    height, width = frame.shape[0], frame.shape[1]
    if width > max_width:
        scale = max_width / float(width)
        frame = cv2.resize(frame, (int(width * scale), int(height * scale)),
                           interpolation=cv2.INTER_AREA)

    stamp = datetime.now().strftime("%Y%m%d-%H%M%S-%f")[:-3]
    path = os.path.join(directory, f"{prefix}-{stamp}.jpg")
    ok = cv2.imwrite(path, frame,
                     [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
    if not ok:
        raise OSError(f"could not write {path}")
    return path, os.path.getsize(path)
