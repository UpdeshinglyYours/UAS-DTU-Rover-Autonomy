#!/usr/bin/env python3
"""Set the FCB-EV9520L zoom over VISCA without waiting for a reply.

The Oppila FX3 board holds the camera's VISCA replies until 32 bytes have
built up, so a single command or inquiry gets no answer in time and both FCB
drivers decide "no VISCA control". The camera does receive and carry out the
commands, though. This sends CAM_Zoom Direct (ratio -> lens position from the
drivers' own zoom table) and then reads the zoom position back by sending
inquiries until the board flushes a batch.

    ./fcb_zoom.py 1          # 1x, the wide end
    ./fcb_zoom.py 4.5        # any ratio 1-30
    ./fcb_zoom.py 1 --no-confirm

Safe to run next to a driver: it only writes zoom commands and inquiries.
"""
import argparse
import os
import re
import sys
import time

import serial

sys.path.insert(0, os.path.expanduser("~/fcb_orin"))
from fcb_base_driver import visca, zoom_map  # noqa: E402

PORT = "/dev/serial/by-id/usb-Cypress_FX3_CAMERA-if00"
ZOOM_REPLY = re.compile(rb"\x90\x50([\x00-\x0f]{4})\xff")


def read_back(link, wait=8.0):
    """Zoom positions in the replies, sending inquiries until a batch arrives."""
    buf, t0 = b"", time.time()
    while time.time() - t0 < wait and len(ZOOM_REPLY.findall(buf)) < 2:
        link.write(visca.zoom_pos_inq())
        t = time.time()
        while time.time() - t < 0.5:
            buf += link.read(64)
    return [int("".join("%x" % b for b in m), 16) for m in ZOOM_REPLY.findall(buf)]


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("ratio", type=float, help="zoom ratio, 1-30")
    parser.add_argument("--port", default=PORT)
    parser.add_argument("--baud", type=int, default=9600)
    parser.add_argument("--no-confirm", action="store_true",
                        help="send the command only, do not read the position back")
    args = parser.parse_args()

    ratio = min(max(args.ratio, zoom_map.MIN_RATIO), zoom_map.MAX_RATIO)
    position = zoom_map.ratio_to_position(ratio)
    with serial.Serial(args.port, args.baud, timeout=0.05) as link:
        link.reset_input_buffer()
        link.write(visca.zoom_direct(position))
        print(f"sent zoom {ratio:g}x (lens position 0x{position:04x})", flush=True)
        if args.no_confirm:
            return
        time.sleep(5.0)          # full-range travel takes a few seconds
        try:
            found = read_back(link)
        except serial.SerialException:
            # a running driver has the port open too and takes the replies
            print("a driver is reading this port: the command was sent; "
                  "check the zoom on the driver's status line")
            return
        if not found:
            print("no zoom reply came back (the command was still sent)")
            return
        now = found[-1]          # earlier ones can be stale, queued before the move
        print(f"camera reports position 0x{now:04x} = "
              f"{zoom_map.position_to_ratio(now):.1f}x")


if __name__ == "__main__":
    main()
