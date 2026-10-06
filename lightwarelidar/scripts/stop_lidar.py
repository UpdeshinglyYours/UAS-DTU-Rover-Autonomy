#!/usr/bin/env python3
"""
Emergency / quick stop script for LightWare SF45/B LiDAR.
Stops streaming (Command 30 = 0) and halts the scanning motor (Command 96 = 0).
"""
import sys
import time
import argparse
import struct
import serial

def create_crc(data: bytes) -> int:
    crc = 0
    for byte in data:
        code = (crc >> 8) & 0xFF
        code ^= byte
        code ^= (code >> 4) & 0xFF
        crc = (crc << 8) & 0xFFFF
        crc ^= code
        code = (code << 5) & 0xFFFF
        crc ^= code
        code = (code << 7) & 0xFFFF
        crc ^= code
    return crc & 0xFFFF

def build_packet(cmd_id: int, write: bool, payload: bytes = b'') -> bytes:
    payload_len = 1 + len(payload)
    flags = (payload_len << 6) | (1 if write else 0)
    header = bytes([0xAA, flags & 0xFF, (flags >> 8) & 0xFF, cmd_id]) + payload
    crc = create_crc(header)
    return header + struct.pack('<H', crc)

def stop_lidar(port: str, baudrate: int):
    print(f"Connecting to SF45/B on {port} at {baudrate} baud...")
    try:
        ser = serial.Serial(port, baudrate, timeout=0.5)
    except Exception as e:
        print(f"Failed to open port {port}: {e}")
        sys.exit(1)

    time.sleep(0.05)

    # 1. Disable streaming (Command 30: Stream = 0)
    pkt_stream = build_packet(30, True, struct.pack('<I', 0))
    ser.write(pkt_stream)
    ser.flush()

    # 2. Disable scanning motor (Command 96: Scan enable = 0)
    pkt_scan = build_packet(96, True, struct.pack('<B', 0))
    ser.write(pkt_scan)
    ser.flush()

    time.sleep(0.1)
    ser.close()
    print("Commands sent! LiDAR streaming stopped and motor halted.")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Halt SF45/B LiDAR motor and stream.")
    parser.add_argument('--port', default='/dev/ttyUSB0', help="Serial port (default: /dev/ttyUSB0)")
    parser.add_argument('--baudrate', type=int, default=115200, help="Baud rate (default: 115200)")
    args = parser.parse_args()

    stop_lidar(args.port, args.baudrate)
