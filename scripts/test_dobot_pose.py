#!/usr/bin/env python3
"""
Test script to read real-time pose and joint states from Dobot Magician Lite
via Dobot Communication Protocol v1.x (ID 0x0A: GetPose).
Does NOT execute any movement or homing.
"""

import sys
import time
import struct
import serial

PORT = "/dev/ttyACM0"
BAUDRATE = 115200
TIMEOUT = 1.0


def calculate_checksum(payload: bytes) -> int:
    """Two's complement checksum of payload: (256 - (sum(payload) % 256)) % 256"""
    return (256 - (sum(payload) % 256)) % 256


def build_get_pose_packet() -> bytes:
    """Builds the immediate read packet for GetPose (ID 0x0A)."""
    header = b"\xaa\xaa"
    # Payload: ID (0x0A), Ctrl (0x00) -> len = 2
    payload = bytes([0x0A, 0x00])
    length = bytes([len(payload)])
    checksum = bytes([calculate_checksum(payload)])
    return header + length + payload + checksum


def read_packet(ser: serial.Serial, expected_id: int = 0x0A, max_attempts: int = 50):
    """Reads and validates a response packet from the Dobot serial stream."""
    for _ in range(max_attempts):
        # Synchronize with header 0xAA 0xAA
        b1 = ser.read(1)
        if not b1 or b1 != b"\xaa":
            continue
        b2 = ser.read(1)
        if not b2 or b2 != b"\xaa":
            continue

        len_byte = ser.read(1)
        if not len_byte:
            continue
        payload_len = len_byte[0]
        if payload_len == 0 or payload_len > 128:
            continue

        payload = ser.read(payload_len)
        if len(payload) != payload_len:
            continue

        checksum_byte = ser.read(1)
        if not checksum_byte:
            continue

        expected_checksum = calculate_checksum(payload)
        if checksum_byte[0] != expected_checksum:
            continue

        cmd_id = payload[0]
        if cmd_id == expected_id:
            return payload

    return None


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else PORT
    baudrate = int(sys.argv[2]) if len(sys.argv) > 2 else BAUDRATE

    print(f"Connecting to Dobot Magician Lite on {port} at {baudrate} baud...")
    try:
        ser = serial.Serial(port, baudrate=baudrate, timeout=TIMEOUT)
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}", file=sys.stderr)
        print("Tip: Ensure the USB cable is connected and permissions are granted (e.g., sudo chmod 666 /dev/ttyACM0).", file=sys.stderr)
        sys.exit(1)

    time.sleep(0.5)
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    packet = build_get_pose_packet()
    print(f"Sending GetPose packet: {packet.hex().upper()}")

    # Send read request
    ser.write(packet)
    ser.flush()

    # Read response
    payload = read_packet(ser, expected_id=0x0A)
    if not payload:
        print("Failed to receive valid GetPose response packet from robot.", file=sys.stderr)
        ser.close()
        sys.exit(2)

    params = payload[2:]
    if len(params) < 32:
        print(f"Unexpected params length: {len(params)} bytes (expected 32)", file=sys.stderr)
        ser.close()
        sys.exit(3)

    x, y, z, r, j1, j2, j3, j4 = struct.unpack("<8f", params[:32])

    print("\n=== Dobot Magician Lite Pose Feedback ===")
    print(f"Cartesian Coordinates (End-Effector):")
    print(f"  X: {x:.2f} mm")
    print(f"  Y: {y:.2f} mm")
    print(f"  Z: {z:.2f} mm")
    print(f"  R (Rotation): {r:.2f}°")
    print(f"Joint Angles:")
    print(f"  Joint 1 (Base):     {j1:.2f}°")
    print(f"  Joint 2 (Rear Arm): {j2:.2f}°")
    print(f"  Joint 3 (Forearm):  {j3:.2f}°")
    print(f"  Joint 4 (Rotation): {j4:.2f}°")
    print("=========================================\n")

    ser.close()
    print("Test completed successfully without any mechanical movement.")


if __name__ == "__main__":
    main()
