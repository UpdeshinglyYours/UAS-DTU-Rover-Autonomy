"""Serial transport for VISCA, framed properly and safe to share.

VISCA is a framed protocol: every packet ends in 0xFF, and packets vary in
length. Replies to a command arrive as an ACK (`y0 4z FF`) as soon as the
camera accepts it, then a separate completion (`y0 5z FF`) once the action
has actually finished -- which for a zoom movement can be seconds later.

That distinction matters. Waiting for completion after every command would
stall the control loop for the whole duration of the physical movement, so
`command()` waits only for the ACK by default and late completions are
drained on the next exchange.

The link takes an exclusive `flock` on the serial device so a second node
(or a stray CLI tool) cannot interleave bytes into the same stream.

Some USB boards hold replies back. The Oppila LVDS-USB3 board's serial
bridge only passes camera bytes to the host in full 32-byte packets -- on
the drone every read that returned anything was exactly 32 bytes, holding
the replies to several earlier exchanges -- so a lone 3-7 byte reply sits
in the board until enough later ones pile up behind it. Commands still
take effect at once; it is only the answer that is stuck. For those
boards the link pads: while it is waiting on a reply it sends broadcast
Address Set packets, whose 4-byte `88 30 02 FF` answers fill the packet
and push the real reply out. Address Set only renumbers cameras on the
bus, and with the one camera there it is already address 1, so it changes
nothing; its answers classify as UNKNOWN and are dropped like any other
frame the link is not waiting for.
"""
import fcntl
import threading
import time

import serial

from fcb_base_driver import visca


class ViscaError(RuntimeError):
    """The camera replied with a VISCA error packet."""


class ViscaTimeout(RuntimeError):
    """No complete reply frame arrived within the timeout."""


#: Broadcast Address Set; answered with the 4-byte `88 30 02 FF`.
_PAD = bytes([0x88, 0x30, 0x01, 0xFF])
#: Eight answers are 32 bytes: one full packet behind the awaited reply,
#: however much of the previous packet was already sitting in the board.
_PADS = _PAD * 8
#: How long a reply may be quiet before the link pads to push it out.
_PAD_AFTER = 0.05
#: And how long after padding before padding again. Eight pads took
#: 120-180 ms to come back on the drone; padding faster than that floods
#: the camera, and the board then went silent altogether.
_PAD_AGAIN = 0.4


def _check_position_reply(payload: bytes, what: str) -> None:
    """Reject a position reply that is not 50 0p 0q 0r 0s.

    VISCA carries no checksum, and a board that damages bytes in transit
    (the Oppila one does, while streaming) can turn a position into another
    plausible-looking one -- which the zoom servo would then "correct" by
    moving the lens. The one thing that can be checked is the shape: four
    data bytes, each a single nibble. A reply that fails it is an error to
    retry, not a reading.
    """
    if (len(payload) != 5 or payload[0] != 0x50
            or any(b > 0x0F for b in payload[1:])):
        raise ViscaError(f"corrupt {what} reply: {payload.hex(' ')}")


class ViscaLink:
    """Exclusive, framed serial link to a VISCA camera."""

    def __init__(self, port: str, baud: int = 9600, address: int = 1,
                 timeout: float = 0.25, pad_replies: bool = None):
        """`pad_replies` None decides from the port's USB ID: on for boards
        known to hold replies back (devices.CHUNKED_REPLY_USB_IDS)."""
        self.address = address
        self.timeout = timeout
        self._lock = threading.Lock()
        if pad_replies is None:
            # Imported here: devices imports this module.
            from fcb_base_driver import devices
            pad_replies = devices.usb_id(port) in devices.CHUNKED_REPLY_USB_IDS
        self.pad_replies = pad_replies
        if pad_replies:
            # A padded answer takes 120-250 ms on the Oppila board, now and
            # then 500, against the 0.25 s callers pass for a direct board.
            # Too short a limit turns every slow answer into a timeout and a
            # resync, which costs more than the wait it was meant to save.
            self.timeout = timeout = max(timeout, 1.0)
        # Set when a padded exchange times out: its reply may still turn up
        # later, behind the board's packet boundary, and must not be taken
        # for the answer to the next question. Starts set on a padded link,
        # since the board can still be holding bytes from whoever had the
        # port before -- on the drone, the first exchange after opening
        # timed out until this was done.
        self._desynced = pad_replies
        self._padded_at = 0.0

        # Right after a USB camera board re-enumerates, its tty can exist but
        # not yet take ioctls, and pyserial's line setup then fails with a
        # plain OSError rather than a SerialException. That race is
        # sub-second, so it is retried; a SerialException (no such port,
        # permission denied) is not, since retrying cannot fix it and the
        # autodetect sweep would pay for every attempt.
        for attempt in range(1, 4):
            try:
                self._ser = serial.Serial(
                    port=port,
                    baudrate=baud,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=timeout,
                )
                break
            except serial.SerialException:
                raise
            except OSError:
                if attempt == 3:
                    raise
                time.sleep(0.5)
        try:
            fcntl.flock(self._ser.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError as exc:
            self._ser.close()
            raise RuntimeError(
                f"{port} is locked by another process; refusing to share the bus"
            ) from exc

        self._ser.reset_input_buffer()
        self._ser.reset_output_buffer()
        if self.pad_replies:
            # Short reads, so a quiet line is noticed in time to pad rather
            # than only once the whole exchange has already timed out.
            self._ser.timeout = min(timeout, _PAD_AFTER / 2)

    # -- lifecycle -----------------------------------------------------

    def close(self) -> None:
        try:
            fcntl.flock(self._ser.fileno(), fcntl.LOCK_UN)
        except OSError:
            pass
        self._ser.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    # -- framing -------------------------------------------------------

    def _pad(self) -> None:
        self._ser.write(_PADS)
        self._ser.flush()
        self._padded_at = time.monotonic()

    def _read_frame(self, deadline: float) -> bytes:
        """Read bytes up to and including the next 0xFF terminator.

        On a padded link, a quiet line is padded again -- but never sooner
        than _PAD_AGAIN after the last pads, however many frames arrived in
        between. Timing it per frame instead re-padded every _PAD_AFTER
        while the previous pads' answers trickled in, flooding the camera
        until the board went silent.
        """
        frame = bytearray()
        quiet_since = time.monotonic()
        while time.monotonic() < deadline:
            byte = self._ser.read(1)
            if not byte:
                now = time.monotonic()
                if (self.pad_replies and now - quiet_since >= _PAD_AFTER
                        and now - self._padded_at >= _PAD_AGAIN):
                    self._pad()
                continue
            quiet_since = time.monotonic()
            value = byte[0]
            if frame and 0x80 <= value < 0xFF:
                # A header byte inside a frame: the terminator of the frame
                # before was lost, so a new one starts here. In a VISCA reply
                # only headers (0x80-0xFE) and the 0xFF terminator have the
                # top bit set; every data byte is below 0x80. Measured on the
                # Oppila board while it streams: a quarter of replies lost or
                # damaged, many as two run together ("88 30 02 88 30 02 FF").
                # Without this, both were thrown away -- one of them, often,
                # the answer being waited for.
                frame = bytearray()
            frame += byte
            if value == 0xFF:
                return bytes(frame)
            if len(frame) > 16:
                frame = bytearray()     # no VISCA reply is this long: noise
        raise ViscaTimeout(
            f"no complete frame within {self.timeout}s (partial: {frame.hex()})"
        )

    def _await(self, want: str, timeout: float = None,
               min_payload: int = 0) -> bytes:
        """Read frames until one of kind `want` arrives, or time out.

        Frames of other kinds (a stale completion from an earlier movement,
        for instance) are skipped rather than mistaken for this reply.

        `min_payload` separates the two shapes of completion frame. A command
        completes with a bare `y0 5z FF` (one payload byte), while an inquiry
        answers with `y0 50 <data> FF`. Interleaving a position inquiry with
        movement commands means a lens arriving mid-inquiry can drop its
        completion into the stream first, so an inquiry asks for a payload
        long enough to actually carry data and skips the bare one.
        """
        timeout = self.timeout if timeout is None else timeout
        deadline = time.monotonic() + timeout
        while True:
            try:
                frame = self._read_frame(deadline)
            except ViscaTimeout:
                if self.pad_replies:
                    self._desynced = True
                raise
            kind, payload = visca.classify(frame)
            if kind == visca.ERROR:
                raise ViscaError(visca.error_message(payload))
            if kind == want and len(payload) >= min_payload:
                return payload
            # Otherwise: an ACK we are not waiting on, or a late completion
            # from a previous command. Drop it and keep reading.

    def _resync(self) -> None:
        """Discard whatever is queued in the board behind a timed-out reply.

        Sends sixteen pads and reads until eight of their answers are back.
        Their answers come after anything older, so everything ahead of them
        is stale and dropped, and what stays held in the board afterwards is
        only more pad answers -- which the next exchange ignores anyway.
        """
        self._ser.write(_PADS)
        self._pad()
        deadline = time.monotonic() + max(self.timeout, 1.0)
        seen = 0
        while seen < 8:
            frame = self._read_frame(deadline)
            if frame == bytes([0x88, 0x30, 0x02, 0xFF]):
                seen += 1
        self._desynced = False

    def _exchange(self, packet: bytes, want: str, timeout: float = None,
                  min_payload: int = 0) -> bytes:
        """Write `packet`, then wait for a reply frame of kind `want`."""
        if self._desynced:
            self._resync()
        # Pads straight behind the packet, not after a quiet spell: measured
        # on the drone, ~180 ms per answer this way against ~800 ms when
        # the link first waited to see whether the reply came on its own.
        self._ser.write(packet)
        if self.pad_replies:
            self._pad()
        else:
            self._ser.flush()
        return self._await(want, timeout, min_payload)

    # -- public API ----------------------------------------------------

    def command(self, packet: bytes, wait_completion: bool = False,
                expect_ack: bool = True, timeout: float = None) -> None:
        """Send a command packet and wait for the camera to accept it.

        By default this returns once the camera ACKs, not once the movement
        finishes -- see the module docstring. Pass `wait_completion=True`
        for commands whose completion is fast and worth confirming.

        IF_Clear is a documented exception to the normal ACK-then-completion
        pattern -- the manual states plainly that no ACK is returned for it,
        only a bare completion -- so `expect_ack=False` skips straight to
        waiting on the completion instead of timing out on an ACK that is
        never coming.
        """
        with self._lock:
            if expect_ack:
                self._exchange(packet, want=visca.ACK, timeout=timeout)
                if wait_completion:
                    self._await(visca.COMPLETION, timeout=timeout)
            else:
                self._exchange(packet, want=visca.COMPLETION, timeout=timeout)

    def inquiry(self, packet: bytes, timeout: float = None) -> bytes:
        """Send an inquiry packet and return its completion payload.

        Inquiries are answered with a single completion frame carrying data;
        they are not ACKed first.
        """
        with self._lock:
            return self._exchange(
                packet, want=visca.COMPLETION, timeout=timeout, min_payload=2
            )

    # -- convenience wrappers -----------------------------------------

    def zoom_position(self) -> int:
        payload = self.inquiry(visca.zoom_pos_inq(self.address))
        _check_position_reply(payload, "zoom")
        return visca.parse_zoom_pos(payload)

    def zoom_direct(self, position: int) -> None:
        self.command(visca.zoom_direct(position, self.address))

    def zoom_tele(self, speed: int) -> None:
        self.command(visca.zoom_tele(speed, self.address))

    def zoom_wide(self, speed: int) -> None:
        self.command(visca.zoom_wide(speed, self.address))

    def zoom_stop(self) -> None:
        self.command(visca.zoom_stop(self.address))

    def focus_position(self) -> int:
        payload = self.inquiry(visca.focus_pos_inq(self.address))
        _check_position_reply(payload, "focus")
        return visca.parse_focus_pos(payload)

    def if_clear(self) -> None:
        """Clear the camera's command buffer -- useful on startup.

        Per the technical manual: "Acknowledge is not returned for this
        command" -- the camera replies with a bare completion only.
        """
        self.command(visca.if_clear(self.address), expect_ack=False)
