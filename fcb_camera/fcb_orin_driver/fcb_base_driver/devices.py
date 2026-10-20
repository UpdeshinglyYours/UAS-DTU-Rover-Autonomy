"""Finding the camera and the flight controller among the /dev nodes.

The FCB's video node, its VISCA control port, and the Pixhawk's MAVLink
port all appear as generic devices -- /dev/video* and /dev/ttyACM* -- and
which number each gets depends on the order things were plugged in. On a
machine with both, the numbers move whenever either is replugged, so
nothing here trusts a fixed path: a device is identified by answering its
own protocol, or by the name v4l2 reports for it.
"""
import glob
import os
import subprocess
import sys
from pathlib import Path

import cv2

from fcb_base_driver import visca
from fcb_base_driver.visca_link import ViscaLink

#: Substrings (case-insensitive) seen in v4l2's "Card type" for this camera
#: across the boards it ships on -- a bare Sony FCB block reports "FCB", the
#: Active Silicon Harrier board reports "Harrier", and the Twiga board this
#: particular camera turned out to be on reports "NeoHD".
FCB_NAME_HINTS = ("neohd", "fcb", "harrier")

#: USB interface boards that carry the FCB, by USB ID.
#:
#: The Twiga "USB3 NeoHD" board reports 04b4:00f9, or 04b4:00f8 when its
#: link falls back to USB 2 (per github.com/gun29may/sony_fcb-ui).
#:
#: The Oppila LVDS-USB3 board (oppila.in/products/usb-interface) reports
#: 04b4:0040 under the bare name "FX3 CAMERA", sends UYVY only, and has two
#: ACM ports: interface 0 carries VISCA at 9600 baud, the other is debug
#: output from its USB controller (Oppila's Getting Started and Camera
#: Control pages). A name that generic says nothing about which camera it
#: is, so boards are matched by USB ID rather than by adding "fx3" to the
#: name hints, which would claim any FX3-based device on the bus.
CAMERA_BOARD_USB_IDS = {
    "04b4:00f9": "NeoHD",
    "04b4:00f8": "NeoHD (USB 2 fallback)",
    "04b4:0040": "Oppila LVDS-USB3",
}

#: The CDC-ACM interface that carries VISCA, where the board documents it.
#: Oppila: interface 0 is VISCA, the other is the board's debug output.
VISCA_INTERFACE = {"04b4:0040": 0}

#: Boards whose serial bridge holds VISCA replies back until it has a full
#: 32-byte USB packet of them; ViscaLink pads to push them out.
CHUNKED_REPLY_USB_IDS = {"04b4:0040"}

#: Boards that send some camera frames twice and report a frame rate that
#: is not the camera's. The Oppila board reports 30 fps whatever is asked
#: of it, delivers 78.7, and every fourth of those repeats the third -- so
#: the copies are dropped (FrameGrabber drop_repeats) and the rate is
#: measured rather than taken from the board.
REPEATING_FRAME_USB_IDS = {"04b4:0040"}

#: The camera's baud is a persistent register setting (9600 out of the box;
#: 38400/115200 selectable and surviving a power cycle), so autodetection
#: checks all three documented rates rather than only the one requested.
VISCA_BAUDS = (9600, 38400, 115200)


class V4l2Unresponsive(RuntimeError):
    """v4l2-ctl is installed but did not finish listing the devices."""


#: Measured at 5.1 s on an Orin NX with the Oppila board attached (its UVC
#: control queries fail slowly), against the 5 s this
#: used to allow -- so every listing timed out with the camera attached.
V4L2_LIST_TIMEOUT = 20


def list_v4l2_devices():
    """Group /dev/video* nodes by USB device, as `v4l2-ctl --list-devices` sees them.

    Returns {card_name: [device_paths]}, or None if v4l2-ctl is missing. A
    UVC camera creates one node per logical stream (capture, metadata, ...)
    under one physical device, and only the card name -- not the node's own
    properties -- says which physical camera a node belongs to.

    Raises V4l2Unresponsive if it is installed but too slow to answer. That
    is kept apart from "missing" on purpose: missing is what licenses the
    blind scan in autodetect_video, and a slow listing used to trigger that
    scan -- which can hand back the wrong camera -- with the real one
    attached.
    """
    try:
        output = subprocess.run(
            ["v4l2-ctl", "--list-devices"],
            capture_output=True, text=True, timeout=V4L2_LIST_TIMEOUT,
        ).stdout
    except FileNotFoundError:
        return None
    except subprocess.SubprocessError as exc:
        raise V4l2Unresponsive(
            f"v4l2-ctl --list-devices did not answer within "
            f"{V4L2_LIST_TIMEOUT} s") from exc

    groups = {}
    card = None
    for line in output.splitlines():
        if not line.strip():
            card = None
        elif not line.startswith(("\t", " ")):
            card = line.split(" (")[0].strip()
            groups[card] = []
        elif card is not None and line.strip().startswith("/dev/video"):
            groups[card].append(line.strip())
    return groups


def _usb_interface(node):
    """sysfs directory of the USB interface behind a /dev node, or None."""
    if not node:
        return None
    name = os.path.basename(node)
    for subsystem in ("video4linux", "tty"):
        link = Path("/sys/class") / subsystem / name / "device"
        if link.exists():
            return link.resolve()
    return None


def usb_id(node):
    """'vvvv:pppp' of the USB device behind a /dev node, or None if not USB."""
    interface = _usb_interface(node)
    if interface is None:
        return None
    device = interface.parent
    try:
        vendor = (device / "idVendor").read_text().strip()
        product = (device / "idProduct").read_text().strip()
    except OSError:
        return None
    return f"{vendor}:{product}"


def usb_interface_number(node):
    """bInterfaceNumber of the USB interface behind a /dev node, or None."""
    interface = _usb_interface(node)
    try:
        return int((interface / "bInterfaceNumber").read_text().strip(), 16)
    except (OSError, TypeError, ValueError):
        return None


def board_name(card, devices):
    """Which known camera board a v4l2 card is, or None if it is not one.

    By USB ID first, since that is what identifies the Oppila board behind
    its generic "FX3 CAMERA" name; by card name for the others.
    """
    for device in devices:
        board = CAMERA_BOARD_USB_IDS.get(usb_id(device))
        if board:
            return board
    if any(hint in card.lower() for hint in FCB_NAME_HINTS):
        return card
    return None


def camera_serial_ports():
    """ACM ports belonging to a known camera board, VISCA interface first.

    The Oppila board exposes two ACM ports and only interface 0 carries
    VISCA, so ordering by interface number finds the camera on the first
    probe and keeps VISCA bytes off the debug port.
    """
    ports = []
    for p in glob.glob("/dev/ttyACM*"):
        ident = usb_id(p)
        if ident not in CAMERA_BOARD_USB_IDS:
            continue
        # A board whose VISCA interface is documented is held to it. Probing
        # the Oppila board's debug port as well only doubled the slow opens
        # when the board was misbehaving -- each one waits out a USB control
        # timeout in the kernel -- for a port that never answers VISCA.
        wanted = VISCA_INTERFACE.get(ident)
        if wanted is not None and usb_interface_number(p) != wanted:
            continue
        ports.append(p)
    return sorted(ports, key=lambda p: (usb_interface_number(p) or 0, p))


def fourcc_of(capture):
    """The pixel format a capture actually negotiated, e.g. 'YUYV' or 'UYVY'."""
    code = int(capture.get(cv2.CAP_PROP_FOURCC))
    return code.to_bytes(4, "little").decode("ascii", "replace").strip("\0 ")


def yuv422_to_bgr_code(fourcc):
    """cvtColor code for raw two-channel 4:2:2 frames in this pixel format.

    Both packings arrive as the same two-channel image once OpenCV stops
    converting, so the shape cannot tell them apart -- only the negotiated
    format can. Decoding UYVY as YUYV still yields a picture, just with
    the colours wrong, which is easy to miss until after a flight.
    """
    if fourcc == "UYVY":
        return cv2.COLOR_YUV2BGR_UYVY
    return cv2.COLOR_YUV2BGR_YUY2


def capture_capable(device):
    """True if this node reports Video Capture support, not just metadata."""
    try:
        result = subprocess.run(
            ["v4l2-ctl", "-d", device, "--list-formats"],
            capture_output=True, text=True, timeout=5,
        )
        return "Video Capture" in result.stdout
    except (FileNotFoundError, subprocess.SubprocessError):
        return True  # can't tell -- don't rule it out


def opens_and_reads(device):
    capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
    try:
        return capture.isOpened() and capture.read()[0]
    finally:
        capture.release()


def autodetect_video(log=print):
    """Find the FCB's video node by USB identity, not by guessing at /dev order.

    A laptop's own webcam and the FCB camera both just look like "a video
    device that yields a frame" to the naive approach of trying /dev/video0
    first -- and the built-in webcam usually wins that race since it has no
    USB3 negotiation to do. This asks v4l2 which physical camera each node
    belongs to and only considers ones whose name matches this camera's
    known boards.

    Once a card matches by name this commits to it rather than falling
    through to the generic scan, because silently substituting some other
    camera for the real one is worse than reporting no video at all. The
    choice is made on what v4l2 reports, without a trial open-and-read:
    this board wedges after enough open/release churn -- every later open
    then times out until it is replugged -- and probing meant opening it
    twice on every startup.
    """
    try:
        groups = list_v4l2_devices()
    except V4l2Unresponsive as exc:
        log(f"{exc} -- cannot tell which camera is which, so not guessing. "
            f"Pass --video to name the device")
        return None
    if groups is not None:
        for card, devices in groups.items():
            board = board_name(card, devices)
            if board is None:
                continue
            for device in devices:
                if capture_capable(device):
                    label = card if board == card else f"{board}, \"{card}\""
                    log(f"matched {label} -> {device}")
                    return device
            log(f"matched \"{card}\" but none of its nodes report video "
                f"capture -- not falling back to an unrelated camera")
            return None
        # v4l2 answered and nothing it listed is this camera, so this camera
        # is not attached. Scanning every node from here would just find
        # whatever else is (a laptop's built-in webcam, typically) and hand
        # back the wrong picture, which is worse than no picture.
        log(f"no /dev/video* device matched a known FCB board "
            f"(names: {', '.join(FCB_NAME_HINTS)}; NeoHD USB IDs: "
            f"{', '.join(CAMERA_BOARD_USB_IDS)}) -- is the camera plugged in? "
            f"Pass --video to use one anyway")
        return None

    # Only reached when v4l2-ctl is missing, so there is no way to ask which
    # camera is which and a scan is the only option left.
    log("v4l2-ctl not available to identify cameras by name; "
        "falling back to scanning every node")
    for device in sorted(glob.glob("/dev/video*")):
        if opens_and_reads(device):
            return device
    return None


def is_zoom_reply(payload):
    """True if `payload` really is a CAM_ZoomPosInq answer.

    A zoom inquiry is answered with `50 0p 0q 0r 0s`: a completion byte
    then four bytes carrying one nibble each, so every data byte is 0x0F or
    below. Checking that shape matters because the flight controller sits
    on a neighbouring /dev/ttyACM* port, and MAVLink is binary traffic full
    of 0xFF bytes -- which is also the VISCA frame terminator. Slicing a
    telemetry stream at those bytes yields "frames" that decode into a
    plausible-looking zoom position often enough to hijack autodetection,
    and the four-high-nibbles-clear test is what MAVLink noise fails.
    """
    return (
        len(payload) == 5
        and payload[0] == 0x50
        and all(byte <= 0x0F for byte in payload[1:5])
    )


def answers_visca(port, baud, address=1, timeout=None):
    """Whether a real FCB camera is listening on this port and baud.

    Asks twice: a stray byte sequence can pass the structural check once,
    but not twice in a row, and a camera always can.
    """
    if timeout is None:
        # A board that holds replies back needs padding round trips to get
        # one out -- measured at 270-650 ms per answer on the Oppila board.
        timeout = 1.5 if usb_id(port) in CHUNKED_REPLY_USB_IDS else 0.3
    try:
        with ViscaLink(port, baud, address, timeout=timeout) as link:
            for _ in range(2):
                payload = link.inquiry(visca.zoom_pos_inq(address))
                if not is_zoom_reply(payload):
                    return False
                # The digital end, not the optical one: the camera drives on
                # past 30x into digital zoom (neohd-ptz-ros measured it
                # topping out at exactly 0x7AC0), and a camera left zoomed
                # in there is still the camera.
                if not 0 <= visca.parse_zoom_pos(payload) <= visca.ZOOM_DIGITAL_TELE_END:
                    return False
        return True
    except Exception:
        return False


def autodetect_visca(ports=None, bauds=None, address=1):
    """First (port, baud) that actually answers a VISCA zoom position inquiry.

    Only a real protocol exchange -- not merely opening the device -- proves
    a port is the camera's control interface, since a board can expose other
    ACM ports (a debug/config console, for instance) that open fine but
    never speak VISCA. Pass a single-element `ports` and/or `bauds` to pin
    whichever side the caller already knows and sweep only the other.

    A candidate that fails verification does not end the sweep; the camera
    may still be on a later port, and giving up at the first plausible-
    looking-but-wrong answer used to leave the camera undetected entirely.
    """
    if ports is None:
        # The camera board's own ports first: found on the first probe, and
        # the autopilot's port is not sent VISCA bytes needlessly.
        own = camera_serial_ports()
        ports = own + [p for p in serial_candidates() if p not in own]
    if bauds is None:
        bauds = VISCA_BAUDS
    for port in ports:
        for baud in bauds:
            if answers_visca(port, baud, address):
                return port, baud
    return None, None


def autodetect_mavlink(bauds=(115200,), exclude=None, timeout=3.0):
    """First serial candidate that actually answers a MAVLink heartbeat.

    Mirrors autodetect_visca: only a real protocol exchange proves a port is
    the flight controller's link. Pass `exclude` to skip a port already
    claimed for something else -- no point spending a timeout probing the
    port the camera is known to be on.
    """
    try:
        from pymavlink import mavutil
    except ImportError:
        return None, None

    # Never the camera board's ports: its VISCA port is claimed, and its
    # debug port is no autopilot -- probing them only costs a timeout each.
    candidates = [p for p in serial_candidates()
                  if p != exclude and usb_id(p) not in CAMERA_BOARD_USB_IDS]
    for candidate in candidates:
        for baud in bauds:
            connection = None
            try:
                connection = mavutil.mavlink_connection(candidate, baud=baud)
                if connection.wait_heartbeat(timeout=timeout):
                    return stable_serial_path(candidate), baud
            except Exception:
                pass
            finally:
                if connection is not None:
                    try:
                        connection.close()
                    except Exception:
                        pass
    return None, None


def stable_serial_path(node):
    """A name for a serial device that survives it being replugged.

    /dev/ttyACM numbers are handed out in plug order, so an autopilot
    unplugged and replugged mid-session -- or one that resets -- can come
    back as a different ttyACM, and a link that keeps reconnecting to the
    old name never sees it again. The /dev/serial/by-id link is built from
    the USB device's own identity and follows it. Falls back to the node
    itself where there is no such link (the Jetson's UARTs, a pty).
    """
    try:
        target = os.path.realpath(node)
        for link in sorted(glob.glob("/dev/serial/by-id/*")):
            if os.path.realpath(link) == target:
                return link
    except OSError:
        pass
    return node


def serial_candidates():
    """Serial ports worth probing, in a stable order.

    ttyTHS* is where a Jetson's own UARTs land, so a flight controller
    wired to TELEM2 rather than USB is still found.
    """
    return sorted(
        glob.glob("/dev/ttyACM*")
        + glob.glob("/dev/ttyUSB*")
        + glob.glob("/dev/ttyTHS*")
    )


def open_capture(device, width=1920, height=1080, fourcc=None):
    """Open a V4L2 capture set up to hand back raw frames.

    CONVERT_RGB is turned off so the colour conversion happens on the
    consumer's thread instead of inside read() -- see FrameGrabber for why
    that matters. Backends that ignore it still work; FrameGrabber checks
    what it actually got.
    """
    capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not capture.isOpened():
        return None
    if fourcc:
        capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    capture.set(cv2.CAP_PROP_CONVERT_RGB, 0)
    return capture


def describe_camera():
    """(exit status, lines) naming the attached camera board and its nodes.

    For scripts that need to say which board they found without opening
    the video or the VISCA port -- the recorder may already hold both, and
    this board wedges after enough open/release churn.
    """
    try:
        groups = list_v4l2_devices()
    except V4l2Unresponsive as exc:
        return 2, [f"{exc}, so the camera board cannot be identified"]
    if groups is None:
        return 2, ["v4l2-ctl is not installed, so the camera board cannot "
                   "be identified (sudo apt install v4l-utils)"]
    for card, nodes in groups.items():
        board = board_name(card, nodes)
        if board is None:
            continue
        ident = next(filter(None, map(usb_id, nodes)), None)
        video = next((n for n in nodes if capture_capable(n)), None)
        line = board if board == card else f"{board} (\"{card}\")"
        if ident:
            line += f" [{ident}]"
        line += f", video {video or 'none that can capture'}"
        lines = [line]
        if ident in CAMERA_BOARD_USB_IDS:
            control = camera_serial_ports()
            lines[0] += f", control {control[0] if control else 'not found'}"
        if ident == "04b4:00f8":
            # Raw 1080p 4:2:2 needs about 1 Gbit/s; USB 2 carries 480 Mbit/s.
            lines.append("is on a USB 2 link, which cannot carry its 1080p "
                         "video -- expect no frames, though zoom and IR "
                         "control still work. Use a USB 3 port and cable, "
                         "and a powered hub if it goes through one")
        return 0, lines
    return 1, ["no known camera board is attached"]


if __name__ == "__main__":
    status, lines = describe_camera()
    print("\n".join(lines))
    sys.exit(status)
