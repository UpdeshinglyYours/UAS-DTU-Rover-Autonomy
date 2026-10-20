"""Closed-loop zoom control over VISCA, on its own thread.

Zoom is driven as a closed loop rather than by firing absolute-position
commands. VISCA gives a command two sockets; an absolute CAM_Zoom Direct
holds its socket until the lens physically arrives, so streaming Direct
commands from a continuously-moving RC knob exhausts the sockets within a
second and the camera starts refusing them with "command buffer full".
Variable-speed drive commands (CAM_Zoom Tele/Wide) complete immediately,
so this polls the zoom position, compares it against the requested target,
and drives towards it at a speed proportional to the error -- stopping
inside a deadband. That tracks a moving knob smoothly and never backs up
the command buffer.

Polling backs off once the lens is holding its target. The VISCA link and
the video stream share one USB3 connection on the NeoHD board, and there
is no reason to keep asking a stationary lens where it is at 20 Hz.

Two commanders are arbitrated: the RC knob, and whatever asks for a target
by hand. Any manual request takes over until control is handed back, since
the RC knob would otherwise immediately win -- it is always publishing.
"""
import threading
import time

from fcb_base_driver import visca, zoom_map
from fcb_base_driver.visca_link import ViscaError, ViscaTimeout

ACTIVE_INTERVAL = 0.05  # 20 Hz while actually driving towards a target
IDLE_INTERVAL = 0.2     # 5 Hz once parked -- just enough to stay current

#: Poll period on a link that pads its replies out of the board (the Oppila
#: board, see visca_link.py). Every exchange there costs a padded round trip
#: of ~0.15-0.25 s and eight extra commands to the camera; polling at the
#: rates above flooded it: an exchange timed out about every 3 s, and four
#: minutes in the camera stopped answering VISCA altogether until
#: power-cycled -- possibly the flood, possibly not, but not worth the risk.
#: Twice a second is plenty once the camera drives to targets itself (below).
PADDED_INTERVAL = 0.5
#: On such a link the lens is not steered with tele/wide/stop: the stop
#: decision rests on a position that is a padded round trip old, several
#: times the INQUIRY_LATENCY the coast model assumes, so it would overshoot.
#: The camera is given the target position instead (CAM_Zoom Direct) and
#: drives the lens there itself. The target is re-sent if the lens has been
#: well off it for this long -- a command lost on the way, or the lens moved
#: by something else.
DIRECT_RESEND_AFTER = 2.0
MAX_SPEED = 7

#: What the lens actually does, measured on an FCB-EV9520L over VISCA at
#: 9600 baud. ZOOM_RATE is position counts per second while a drive command
#: is in effect; ZOOM_COAST is how far it keeps going after zoom_stop is
#: sent.
#:
#: The coast is the whole problem this servo has to solve. Even at the
#: slowest speed the lens travels 25 counts after being told to stop, and
#: at full speed 402. A servo that stops when it is "close enough" and then
#: measures again therefore always finds itself past the target, drives
#: back, overshoots again, and hunts forever -- which is exactly what a
#: parked knob used to do. So the stop is issued *early*, by the distance
#: the lens is about to cover anyway.
ZOOM_RATE = {0: 333, 1: 665, 2: 1321, 3: 1778,
             4: 2616, 5: 3795, 6: 4328, 7: 5179}
ZOOM_COAST = {0: 25, 1: 49, 2: 125, 3: 135,
              4: 202, 5: 293, 6: 335, 7: 402}

#: A position inquiry is a serial round-trip at 9600 baud, measured at
#: ~60 ms. The position acted on is therefore always that stale, and at
#: speed 7 the lens has already moved another 300 counts by the time the
#: answer arrives. Ignoring this is the second half of the overshoot.
INQUIRY_LATENCY = 0.06

#: One full pass of the loop: the sleep plus the inquiry that precedes it.
TICK_PERIOD = ACTIVE_INTERVAL + INQUIRY_LATENCY

#: How far the lens may be from a target it has already parked on before
#: the servo decides something genuinely moved it -- a power cycle, a
#: reconnect -- rather than this being its own landing error. Well above
#: the landing accuracy, so it can never re-trigger a drive on its own.
RESUME_BAND = 250

#: How square on the target the lens has to be before the servo calls it
#: done, and how many slow trims it may spend getting there. The coast
#: model is good but not exact -- the lens does not travel at quite the
#: same rate everywhere in its range -- so a first approach can land half
#: a detent out. A trim fixes that.
#:
#: The count is the whole safety argument. An unbounded "correct until
#: close" is precisely the hunting this replaced; a bounded one improves
#: the landing and then, right or wrong, stops.
FINE_BAND = 50
MAX_TRIMS = 3

#: Retained because other tools import it. The servo no longer uses a
#: symmetric deadband; see ZOOM_COAST above for why one cannot work.
DEADBAND = 24

#: Consecutive failed exchanges before the link is assumed dead rather
#: than merely busy. A camera that is mid-movement can refuse a command or
#: miss a reply occasionally, so one fault means nothing; a run of them
#: means the port went away -- unplugged, re-enumerated, or wedged.
FAULTS_BEFORE_RECONNECT = 10
RECONNECT_INTERVAL = 2.0

#: How often a fault that will not clear repeats itself. Roughly every ten
#: seconds at the idle poll rate -- often enough that a dead zoom cannot be
#: mistaken for a working one, rare enough not to bury the log.
FAULTS_BEFORE_QUIET = 50


class ZoomServo:
    """Drives the lens towards a normalized 0.0-1.0 target.

    Pass `link_factory` to make the servo self-healing: if the VISCA link
    stops answering it is closed and rebuilt from the factory, which
    matters on a drone where nobody can replug anything mid-flight.
    """

    def __init__(self, link, curve="log", rc=None, source="rc", on_status=None,
                 link_factory=None, ratio_step=0.0):
        self.link = link
        self.curve = curve if curve in zoom_map.CURVES else "log"
        self.rc = rc
        # Detented zoom. The knob stays continuously variable; what is
        # discretized is the magnification it asks for, so the detents fall
        # on round numbers (1.0x, 1.5x, ...) regardless of the curve mapping
        # stick travel onto them.
        self.quantizer = zoom_map.RatioQuantizer(ratio_step)
        self.target_ratio = None
        self.source = source if rc is not None else "manual"
        self._status = on_status or (lambda msg: None)
        self._link_factory = link_factory
        self._reconnect_misses = 0
        self._direct_target = None
        self._direct_sent_at = 0.0
        self._faults = 0
        self._last_reconnect = 0.0

        self.unit = 0.0
        self.position = None
        self.ratio = None
        self.updated_at = None
        self.is_driving = False
        self._speed = 0
        # The target the lens is currently sitting on, once it has arrived.
        # While this matches the requested target there is nothing to do:
        # the lens holds its position mechanically, and "correcting" a
        # target that has not moved is what the hunting was.
        self._parked_at = None
        self._trims = 0
        # An exact magnification asked for by hand. Overrides the detents
        # while it is set: the detents are an affordance for a knob, and a
        # number typed in is not a knob.
        self.manual_ratio = None

        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    # -- commanders ------------------------------------------------------

    def set_ratio(self, ratio, source="manual"):
        """Drive to an exact magnification, ignoring the detents.

        Returns the value actually adopted, which is the request clamped to
        what the lens can do. Holds until control is handed back to the
        knob, since the knob is always publishing and would otherwise take
        it back on the next tick.
        """
        ratio = max(zoom_map.MIN_RATIO, min(zoom_map.MAX_RATIO, float(ratio)))
        self.manual_ratio = ratio
        self.unit = zoom_map.position_to_unit(
            zoom_map.ratio_to_position(ratio), self.curve
        )
        self.source = source
        self.quantizer.reset(ratio)
        return ratio

    def set_unit(self, value, source="manual"):
        """Request a normalized 0.0-1.0 zoom, taking over from the RC knob."""
        self.unit = max(0.0, min(1.0, float(value)))
        self.source = source
        self.manual_ratio = None
        # Judge the next knob sample against where the lens is being put
        # now, not against the detent it was last parked on.
        self.quantizer.reset(zoom_map.unit_to_ratio(self.unit, self.curve))
        return self.unit

    def nudge(self, delta):
        return self.set_unit(self.unit + delta)

    def hand_to_rc(self):
        """Give control back to the RC knob. False if there is no RC link."""
        if self.rc is None:
            return False
        self.source = "rc"
        self.manual_ratio = None
        # The knob has not moved while it was ignored, so judge its next
        # sample against where the lens actually is.
        self.quantizer.reset(self.ratio)
        return True

    def sync_from_camera(self):
        """Adopt the lens's current position as the setpoint.

        Called at startup so the first command continues from where the
        lens already is instead of snapping it across the range.
        """
        position = self.link.zoom_position()
        self.position = position
        self.ratio = zoom_map.position_to_ratio(position)
        self.unit = zoom_map.position_to_unit(position, self.curve)
        self.updated_at = time.monotonic()
        self.quantizer.reset(self.ratio)
        return self.ratio

    # -- servo loop ------------------------------------------------------

    def _run(self):
        while self._running:
            try:
                self._tick()
                if self._faults:
                    self._status("zoom: link recovered")
                self._faults = 0
            except Exception as exc:
                # Deliberately every exception, not just the VISCA ones.
                # Anything escaping here kills this thread, and a dead
                # thread means no zoom for the rest of the flight with
                # nothing said about it -- the servo simply stops
                # answering while every other reading stays plausible.
                self._faults += 1
                # Not every fault: a dead link would fill the log at 5 Hz.
                # But not only the first one either -- a fault that never
                # clears has to keep saying so, or the one message scrolls
                # away and the zoom looks fine from then on.
                if (self._faults == 1
                        or self._faults % FAULTS_BEFORE_QUIET == 0):
                    self._status(f"zoom: {type(exc).__name__}: {exc} "
                                 f"({self._faults} in a row)")
                if self._faults >= FAULTS_BEFORE_RECONNECT:
                    self._reconnect()
            if self._direct():
                time.sleep(PADDED_INTERVAL)
            else:
                time.sleep(ACTIVE_INTERVAL if self.is_driving else IDLE_INTERVAL)

    def _reconnect(self):
        """Rebuild a VISCA link that has stopped answering."""
        if self._link_factory is None:
            return
        now = time.monotonic()
        if now - self._last_reconnect < RECONNECT_INTERVAL:
            return
        self._last_reconnect = now

        try:
            self.link.close()
        except Exception:
            pass
        try:
            link = self._link_factory()
        except Exception as exc:
            self._status(f"zoom: VISCA reconnect failed: {exc}")
            return
        if link is None:
            # Said, not swallowed: the sweep can take tens of seconds against
            # a board that has stopped answering USB control requests, and a
            # silent servo looks exactly like a working one with nothing to do.
            self._reconnect_misses += 1
            if self._reconnect_misses == 1 or self._reconnect_misses % 10 == 0:
                self._status("zoom: camera control not found -- still looking "
                             f"(attempt {self._reconnect_misses}). If the "
                             "camera board was unplugged or reset, it may "
                             "need its power cycled")
            return
        self._reconnect_misses = 0
        self.link = link
        self.is_driving = False
        self._faults = 0
        self._direct_target = None
        self._status("zoom: VISCA link re-established")

    def _tick(self):
        # A manual request already set unit and switched source itself;
        # only pull from RC while it is the active commander.
        if self.source == "rc" and self.rc is not None:
            unit = self.rc.unit()
            if unit is not None:
                self.unit = unit

        self.position = self.link.zoom_position()
        self.ratio = zoom_map.position_to_ratio(self.position)
        self.updated_at = time.monotonic()

        if self.manual_ratio is not None:
            self.target_ratio = self.manual_ratio
            target = zoom_map.ratio_to_position(self.target_ratio)
        elif self.quantizer.enabled:
            self.target_ratio = self.quantizer.snap(
                zoom_map.unit_to_ratio(self.unit, self.curve)
            )
            target = zoom_map.ratio_to_position(self.target_ratio)
        else:
            target = zoom_map.unit_to_position(self.unit, self.curve)
            self.target_ratio = zoom_map.position_to_ratio(target)
        error = target - self.position
        distance = abs(error)

        if self._direct():
            self._drive_direct(target, distance)
            return

        if target != self._parked_at:
            self._trims = 0
        else:
            if distance > RESUME_BAND:
                # Too far out to be this servo's own landing error --
                # something else moved the lens. Drive it back.
                self._parked_at = None
            elif distance > FINE_BAND and self._trims < MAX_TRIMS:
                self._trims += 1
                self._parked_at = None      # one more approach, slowly
            else:
                # Sitting on the target. Nothing to correct, and correcting
                # anyway is what made it hunt.
                self._stop_if_driving()
                return

        # Close enough that even the gentlest nudge would sail past it.
        if not self.is_driving and distance <= self._stop_distance(0):
            self._parked_at = target
            return

        # Moving, and the distance left is what the lens will cover on its
        # own once told to stop. Stop now and let it coast onto the target.
        if self.is_driving and distance <= self._stop_distance(self._speed):
            self._stop_if_driving()
            self._parked_at = target
            return

        speed = self._speed_for(distance)
        self._speed = speed
        if error > 0:
            self.link.zoom_tele(speed)
        else:
            self.link.zoom_wide(speed)
        self.is_driving = True
        self._parked_at = None

    def _direct(self):
        """Whether this link is driven by target position (see PADDED_INTERVAL)."""
        return bool(getattr(self.link, "pad_replies", False))

    def _drive_direct(self, target, distance):
        """Hand the camera the target position; it drives the lens there."""
        now = time.monotonic()
        stale = (distance > RESUME_BAND
                 and now - self._direct_sent_at >= DIRECT_RESEND_AFTER)
        if target != self._direct_target or stale:
            self.link.zoom_direct(target)
            self._direct_target = target
            self._direct_sent_at = now
        self.is_driving = distance > FINE_BAND

    @staticmethod
    def _stop_distance(speed):
        """How far the lens still travels once a stop is decided on.

        The coast itself, plus what it covers during the round-trip that
        produced the position this decision is based on.
        """
        return ZOOM_COAST[speed] + ZOOM_RATE[speed] * INQUIRY_LATENCY

    def _speed_for(self, distance):
        """The fastest speed that still leaves room to stop on target.

        Deceleration falls out of this rather than being scheduled: as the
        gap closes, each speed in turn no longer has room for its own coast
        plus one more tick of travel, so the lens steps down through the
        speeds and arrives slowly.
        """
        for speed in range(MAX_SPEED, 0, -1):
            room = self._stop_distance(speed) + ZOOM_RATE[speed] * TICK_PERIOD
            if room <= distance:
                return speed
        return 0

    def _stop_if_driving(self):
        if not self.is_driving:
            return
        try:
            self.link.zoom_stop()
            self.is_driving = False
        except (ViscaError, ViscaTimeout) as exc:
            self._status(f"zoom stop: {exc}")

    # -- other camera commands -------------------------------------------

    def set_icr(self, mode):
        """Switch the IR cut filter: day, night, night_color, or auto."""
        address = self.link.address
        if mode == "auto":
            self.link.command(visca.auto_icr(True, address=address))
        else:
            # Leave automatic switching first, or the camera would override
            # a hand-picked mode at the next light change.
            self.link.command(visca.auto_icr(False, address=address))
            self.link.command(visca.icr(mode, address))

    def one_push_af(self):
        self.link.command(visca.focus_one_push(self.link.address))

    def stop(self):
        self._running = False
        self._thread.join(timeout=1.0)
        try:
            self.link.zoom_stop()
        except Exception:
            pass
