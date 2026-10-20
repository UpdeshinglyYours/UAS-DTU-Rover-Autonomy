"""VISCA command/reply encoding for the Sony FCB-EV9520L block camera.

Byte sequences here are transcribed from the Sony FCB-EV9520L Technical
Manual (H-292-100-11(1), 2023), "Command List" section. In Sony's notation
`8x` is the command header for camera address x, and `y0` is the reply
header where y = address + 8, so address 1 gives 0x81 out / 0x90 back.

Zoom position is a 16-bit value split across the low nibbles of four bytes:
    0x0000  wide end (1x)
    0x4000  optical tele end (30x)
    0x7AC0  digital tele end (with digital zoom enabled)

This module only encodes/decodes bytes; it does no I/O. See visca_link.py
for the serial transport.
"""

# --- Addressing -------------------------------------------------------

BROADCAST = 0x88


def header(address: int = 1) -> int:
    """Command header byte for a camera address (1-7) -> 0x81-0x87."""
    if not 1 <= address <= 7:
        raise ValueError(f"VISCA address must be 1-7, got {address}")
    return 0x80 | address


def reply_header(address: int = 1) -> int:
    """Expected reply header byte for a camera address -> 0x90-0xF0.

    Sony writes the reply header as `y0` where y = address + 8, so the
    address lands in the high nibble: address 1 replies with 0x90, not 0x89.
    """
    if not 1 <= address <= 7:
        raise ValueError(f"VISCA address must be 1-7, got {address}")
    return (address + 8) << 4


# --- Zoom -------------------------------------------------------------

ZOOM_WIDE_END = 0x0000
ZOOM_OPTICAL_TELE_END = 0x4000  # 30x, the limit without digital zoom
ZOOM_DIGITAL_TELE_END = 0x7AC0  # 360x combined, digital zoom enabled

#: Optical zoom ratio -> VISCA position, from the technical manual's
#: "Zoom Ratio and Zoom Position (for reference)" table. The relationship is
#: strongly non-linear -- 3x is already at 0x2063, i.e. half of full-scale
#: position covers only a tenth of the magnification range -- so mapping a
#: linear input (an RC knob) straight onto position feels badly skewed.
#: zoom_map.py interpolates this table to correct for that.
ZOOM_RATIO_TABLE = (
    (1, 0x0000), (2, 0x16A1), (3, 0x2063), (4, 0x2628), (5, 0x2A1D),
    (6, 0x2D13), (7, 0x2F6D), (8, 0x3161), (9, 0x330D), (10, 0x3486),
    (11, 0x35D7), (12, 0x3709), (13, 0x3820), (14, 0x3920), (15, 0x3A0A),
    (16, 0x3ADD), (17, 0x3B9C), (18, 0x3C46), (19, 0x3CDC), (20, 0x3D60),
    (21, 0x3DD4), (22, 0x3E39), (23, 0x3E90), (24, 0x3EDC), (25, 0x3F1E),
    (26, 0x3F57), (27, 0x3F8A), (28, 0x3FB6), (29, 0x3FDC), (30, 0x4000),
)


def zoom_stop(address: int = 1) -> bytes:
    """CAM_Zoom Stop: 8x 01 04 07 00 FF"""
    return bytes([header(address), 0x01, 0x04, 0x07, 0x00, 0xFF])


def zoom_tele(speed: int, address: int = 1) -> bytes:
    """CAM_Zoom Tele (Variable): 8x 01 04 07 2p FF, p = 0 (low) to 7 (high)."""
    return bytes([header(address), 0x01, 0x04, 0x07, 0x20 | (speed & 0x07), 0xFF])


def zoom_wide(speed: int, address: int = 1) -> bytes:
    """CAM_Zoom Wide (Variable): 8x 01 04 07 3p FF, p = 0 (low) to 7 (high)."""
    return bytes([header(address), 0x01, 0x04, 0x07, 0x30 | (speed & 0x07), 0xFF])


def zoom_direct(position: int, address: int = 1) -> bytes:
    """CAM_Zoom Direct: 8x 01 04 47 0p 0q 0r 0s FF -- absolute position."""
    position = clamp_zoom(position)
    return bytes([
        header(address), 0x01, 0x04, 0x47, *_nibbles(position, 4), 0xFF,
    ])


def zoom_pos_inq(address: int = 1) -> bytes:
    """CAM_ZoomPosInq: 8x 09 04 47 FF -> y0 50 0p 0q 0r 0s FF"""
    return bytes([header(address), 0x09, 0x04, 0x47, 0xFF])


def parse_zoom_pos(payload: bytes) -> int:
    """Decode a CAM_ZoomPosInq completion payload (50 0p 0q 0r 0s) -> position."""
    return _parse_nibble_value(payload, count=4)


def clamp_zoom(position: int, allow_digital: bool = False) -> int:
    limit = ZOOM_DIGITAL_TELE_END if allow_digital else ZOOM_OPTICAL_TELE_END
    return max(ZOOM_WIDE_END, min(limit, int(position)))


# --- Focus ------------------------------------------------------------

FOCUS_NEAR_END = 0xF000
FOCUS_FAR_END = 0x1000  # 0x1000 is "over infinity"


def focus_stop(address: int = 1) -> bytes:
    """CAM_Focus Stop: 8x 01 04 08 00 FF"""
    return bytes([header(address), 0x01, 0x04, 0x08, 0x00, 0xFF])


def focus_far(speed: int, address: int = 1) -> bytes:
    """CAM_Focus Far (Variable): 8x 01 04 08 2p FF"""
    return bytes([header(address), 0x01, 0x04, 0x08, 0x20 | (speed & 0x07), 0xFF])


def focus_near(speed: int, address: int = 1) -> bytes:
    """CAM_Focus Near (Variable): 8x 01 04 08 3p FF"""
    return bytes([header(address), 0x01, 0x04, 0x08, 0x30 | (speed & 0x07), 0xFF])


def focus_auto(enabled: bool, address: int = 1) -> bytes:
    """CAM_Focus Auto Focus / Manual Focus: 8x 01 04 38 02|03 FF"""
    return bytes([header(address), 0x01, 0x04, 0x38, 0x02 if enabled else 0x03, 0xFF])


def focus_one_push(address: int = 1) -> bytes:
    """CAM_Focus One Push Trigger: 8x 01 04 18 01 FF"""
    return bytes([header(address), 0x01, 0x04, 0x18, 0x01, 0xFF])


def focus_direct(position: int, address: int = 1) -> bytes:
    """CAM_Focus Direct: 8x 01 04 48 0p 0q 0r 0s FF"""
    position = max(0x0000, min(0xFFFF, int(position)))
    return bytes([
        header(address), 0x01, 0x04, 0x48, *_nibbles(position, 4), 0xFF,
    ])


def focus_pos_inq(address: int = 1) -> bytes:
    """CAM_FocusPosInq: 8x 09 04 48 FF -> y0 50 0p 0q 0r 0s FF"""
    return bytes([header(address), 0x09, 0x04, 0x48, 0xFF])


def parse_focus_pos(payload: bytes) -> int:
    return _parse_nibble_value(payload, count=4)


# --- Interface / power ------------------------------------------------

def if_clear(address: int = 1) -> bytes:
    """IF_Clear: 8x 01 00 01 FF -- clears the camera's command buffer."""
    return bytes([header(address), 0x01, 0x00, 0x01, 0xFF])


def power(on: bool, address: int = 1) -> bytes:
    """CAM_Power On/Off (Standby): 8x 01 04 00 02|03 FF"""
    return bytes([header(address), 0x01, 0x04, 0x00, 0x02 if on else 0x03, 0xFF])


def image_stabilizer(on: bool, address: int = 1) -> bytes:
    """CAM_Stabilizer On/Off: 8x 01 04 34 02|03 FF"""
    return bytes([header(address), 0x01, 0x04, 0x34, 0x02 if on else 0x03, 0xFF])


def stabilizer_inq(address: int = 1) -> bytes:
    """CAM_StabilizerModeInq: 8x 09 04 34 FF -> y0 50 02 (on) | 03 (off) FF"""
    return bytes([header(address), 0x09, 0x04, 0x34, 0xFF])


#: CAM_StabilizerModeInq reply byte -> state. 0x00 is the manual's "Hold".
STABILIZER_STATES = {0x02: "on", 0x03: "off", 0x00: "hold"}


def parse_stabilizer(payload: bytes) -> str:
    """Decode a CAM_StabilizerModeInq completion payload: on, off or hold."""
    if len(payload) < 2:
        raise ValueError(f"payload too short for a stabilizer state: {payload.hex()}")
    try:
        return STABILIZER_STATES[payload[1]]
    except KeyError:
        raise ValueError(f"unrecognised stabilizer byte: 0x{payload[1]:02X}") from None


# --- IR cut filter (ICR) / day-night ----------------------------------
#
# The FCB-EV9520L is a single visible-light sensor with a mechanically
# removable IR cut filter -- it is not a thermal camera and there is no
# second video stream. "ICR On" *removes* the filter, letting infrared reach
# the sensor for night work; "ICR Off" puts it back for normal daylight
# colour. The naming in the manual is the inverse of what most people
# expect, so the modes below are named from the operator's point of view.

#: Operator-facing mode -> CAM_ICR sub-command (manual's Command List).
ICR_MODES = {
    "day": 0x03,          # CAM_ICR Off       -- IR cut filter engaged, normal colour
    "night": 0x02,        # CAM_ICR On        -- filter removed, IR-sensitive mono
    "night_color": 0x04,  # CAM_ICR On (Color) -- filter removed, colour retained;
                          # the manual warns of false colours under IR light
}

#: Aliases for the same three modes, in the vocabulary people actually use.
ICR_ALIASES = {
    "rgb": "day",
    "colour": "day",
    "color": "day",
    "ir": "night",
    "bw": "night",
    "mono": "night",
    "ir_color": "night_color",
}


def normalize_icr_mode(mode: str) -> str:
    """Resolve an alias to a canonical ICR mode name."""
    mode = str(mode).strip().lower()
    mode = ICR_ALIASES.get(mode, mode)
    if mode not in ICR_MODES and mode != "auto":
        raise ValueError(
            f"unknown ICR mode {mode!r}; expected one of "
            f"{sorted(set(ICR_MODES) | {'auto'} | set(ICR_ALIASES))}"
        )
    return mode


def icr(mode: str, address: int = 1) -> bytes:
    """CAM_ICR: 8x 01 04 01 02|03|04 FF -- manual day/night selection."""
    mode = normalize_icr_mode(mode)
    if mode == "auto":
        raise ValueError("use auto_icr() for automatic day/night switching")
    return bytes([header(address), 0x01, 0x04, 0x01, ICR_MODES[mode], 0xFF])


def auto_icr(enabled: bool, colour: bool = False, address: int = 1) -> bytes:
    """CAM_AutoICR: 8x 01 04 51 02|03|04 FF -- switch on scene brightness."""
    if not enabled:
        sub = 0x03
    else:
        sub = 0x04 if colour else 0x02
    return bytes([header(address), 0x01, 0x04, 0x51, sub, 0xFF])


def auto_icr_threshold(level: int, address: int = 1) -> bytes:
    """CAM_AutoICR Threshold: 8x 01 04 21 00 00 0p 0q FF"""
    level = max(0x00, min(0xFF, int(level)))
    return bytes([
        header(address), 0x01, 0x04, 0x21, 0x00, 0x00, *_nibbles(level, 2), 0xFF,
    ])


def icr_mode_inq(address: int = 1) -> bytes:
    """CAM_ICRModeInq: 8x 09 04 01 FF -> y0 50 02|03|04 FF"""
    return bytes([header(address), 0x09, 0x04, 0x01, 0xFF])


def parse_icr_mode(payload: bytes) -> str:
    """Decode a CAM_ICRModeInq completion payload into a mode name."""
    if len(payload) < 2:
        raise ValueError(f"payload too short for an ICR mode: {payload.hex()}")
    for name, sub in ICR_MODES.items():
        if payload[1] == sub:
            return name
    raise ValueError(f"unrecognised ICR mode byte: 0x{payload[1]:02X}")


# --- Reply classification ---------------------------------------------

ACK = "ack"
COMPLETION = "completion"
ERROR = "error"
UNKNOWN = "unknown"

#: VISCA error codes from the completion/error reply's third byte.
ERROR_CODES = {
    0x01: "message length error",
    0x02: "syntax error",
    0x03: "command buffer full",
    0x04: "command cancelled",
    0x05: "no socket",
    0x41: "command not executable",
}


def classify(frame: bytes) -> tuple:
    """Classify a complete VISCA reply frame (header ... 0xFF).

    Returns (kind, payload) where payload is the frame's bytes between the
    header and the terminating 0xFF. VISCA replies are:
        y0 4z FF        ACK, socket z
        y0 5z FF        command completion, socket z
        y0 50 <data> FF inquiry completion, carrying data
        y0 6z <err> FF  error
    """
    if len(frame) < 3 or frame[-1] != 0xFF:
        return UNKNOWN, b""
    payload = frame[1:-1]
    if not payload:
        return UNKNOWN, b""
    kind_nibble = payload[0] & 0xF0
    if kind_nibble == 0x40:
        return ACK, payload
    if kind_nibble == 0x50:
        return COMPLETION, payload
    if kind_nibble == 0x60:
        return ERROR, payload
    return UNKNOWN, payload


def error_message(payload: bytes) -> str:
    """Human-readable description of an ERROR reply payload."""
    code = payload[1] if len(payload) > 1 else None
    return ERROR_CODES.get(code, f"unknown error code {code!r}")


# --- Helpers ----------------------------------------------------------

def _nibbles(value: int, count: int) -> list:
    """Split a value into `count` big-endian nibbles: (0x1234, 4) -> [1,2,3,4]."""
    return [(value >> (4 * (count - 1 - i))) & 0x0F for i in range(count)]


def _parse_nibble_value(payload: bytes, count: int) -> int:
    """Reassemble the low nibbles of a completion payload's data bytes.

    `payload` is a completion payload (0x50 followed by `count` data bytes).
    """
    if len(payload) < count + 1:
        raise ValueError(f"payload too short for {count} nibbles: {payload.hex()}")
    data = payload[1:count + 1]
    value = 0
    for byte in data:
        value = (value << 4) | (byte & 0x0F)
    return value
