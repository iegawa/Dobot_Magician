"""
Dobot Magician Lite Communication Protocol Handler.

Implements packet framing, checksum calculation, and GetPose (0x0A)
deserialization for the Dobot Communication Protocol v1.x.
"""

from dataclasses import dataclass
import struct
from typing import Optional
import serial


@dataclass
class DobotPose:
    # Cartesian coordinates in millimeters
    x: float
    y: float
    z: float
    r: float  # end-effector yaw in degrees

    # Joint angles in degrees
    j1: float  # Base
    j2: float  # Rear Arm
    j3: float  # Forearm
    j4: float  # End-effector rotation


class DobotProtocol:
    HEADER = b"\xaa\xaa"

    # Command IDs
    CMD_GET_POSE = 0x0A
    CMD_GET_ALARMS_STATE = 0x14
    CMD_CLEAR_ALL_ALARMS_STATE = 0x15
    CMD_SET_PTP_JOINT_PARAMS = 0x50
    CMD_SET_PTP_COORDINATE_PARAMS = 0x51
    CMD_SET_PTP_COMMON_PARAMS = 0x53
    CMD_SET_PTP_CMD = 0x54
    CMD_QUEUED_START_EXEC = 0xF0
    CMD_QUEUED_STOP_EXEC = 0xF1
    CMD_QUEUED_CLEAR = 0xF5

    # PTP Modes
    PTP_JUMP_XYZ = 0
    PTP_MOVJ_XYZ = 1
    PTP_MOVL_XYZ = 2
    PTP_JUMP_ANGLE = 3
    PTP_MOVJ_ANGLE = 4
    PTP_MOVL_ANGLE = 5

    @staticmethod
    def calculate_checksum(payload: bytes) -> int:
        """Calculates two's complement checksum: (256 - (sum(payload) % 256)) % 256."""
        return (256 - (sum(payload) % 256)) % 256

    @classmethod
    def _build_packet(cls, cmd_id: int, ctrl: int, params: bytes = b"") -> bytes:
        payload = bytes([cmd_id, ctrl]) + params
        length = bytes([len(payload)])
        checksum = bytes([cls.calculate_checksum(payload)])
        return cls.HEADER + length + payload + checksum

    @classmethod
    def build_get_pose_packet(cls) -> bytes:
        """Builds immediate read packet for GetPose (ID 0x0A)."""
        return cls._build_packet(cls.CMD_GET_POSE, 0x00)

    @classmethod
    def build_get_alarms_state(cls) -> bytes:
        """Builds immediate read packet for GetAlarmsState (ID 0x14)."""
        return cls._build_packet(cls.CMD_GET_ALARMS_STATE, 0x00)

    @classmethod
    def build_clear_all_alarms_state(cls) -> bytes:
        """Builds immediate write packet to clear all alarms (ID 0x15)."""
        return cls._build_packet(cls.CMD_CLEAR_ALL_ALARMS_STATE, 0x01)

    @classmethod
    def build_set_ptp_coordinate_params(
        cls,
        xyz_vel: float = 100.0,
        r_vel: float = 100.0,
        xyz_acc: float = 100.0,
        r_acc: float = 100.0,
        is_queued: bool = False,
    ) -> bytes:
        """Sets Cartesian coordinate velocities (mm/s) and accelerations (mm/s^2)."""
        ctrl = 0x03 if is_queued else 0x01
        params = struct.pack("<4f", float(xyz_vel), float(r_vel), float(xyz_acc), float(r_acc))
        return cls._build_packet(cls.CMD_SET_PTP_COORDINATE_PARAMS, ctrl, params)

    @classmethod
    def build_set_ptp_joint_params(
        cls, vel: float = 100.0, acc: float = 100.0, is_queued: bool = False
    ) -> bytes:
        """Sets Joint velocities (deg/s) and accelerations (deg/s^2)."""
        ctrl = 0x03 if is_queued else 0x01
        params = struct.pack("<8f", *([float(vel)] * 4 + [float(acc)] * 4))
        return cls._build_packet(cls.CMD_SET_PTP_JOINT_PARAMS, ctrl, params)

    @classmethod
    def build_set_ptp_common_params(
        cls, velocity_ratio: float, acceleration_ratio: float, is_queued: bool = False
    ) -> bytes:
        """Sets velocity and acceleration ratio (1..100%)."""
        ctrl = 0x03 if is_queued else 0x01
        params = struct.pack("<2f", float(velocity_ratio), float(acceleration_ratio))
        return cls._build_packet(cls.CMD_SET_PTP_COMMON_PARAMS, ctrl, params)

    @classmethod
    def build_set_ptp_cmd(
        cls, ptp_mode: int, x: float, y: float, z: float, r: float, is_queued: bool = True
    ) -> bytes:
        """Builds SetPTPCmd (ID 0x54) packet."""
        ctrl = 0x03 if is_queued else 0x01
        params = bytes([ptp_mode]) + struct.pack("<4f", float(x), float(y), float(z), float(r))
        return cls._build_packet(cls.CMD_SET_PTP_CMD, ctrl, params)

    @classmethod
    def build_queue_start_exec(cls) -> bytes:
        """Starts executing the command queue."""
        return cls._build_packet(cls.CMD_QUEUED_START_EXEC, 0x01)

    @classmethod
    def build_queue_stop_exec(cls) -> bytes:
        """Stops executing the command queue."""
        return cls._build_packet(cls.CMD_QUEUED_STOP_EXEC, 0x01)

    @classmethod
    def build_queue_clear(cls) -> bytes:
        """Clears the command queue (Emergency abort)."""
        return cls._build_packet(cls.CMD_QUEUED_CLEAR, 0x01)

    @classmethod
    def read_packet(
        cls, ser: serial.Serial, expected_id: int = CMD_GET_POSE, max_attempts: int = 50
    ) -> Optional[bytes]:
        """
        Reads and validates a response packet from the Dobot serial stream.
        """
        for _ in range(max_attempts):
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

            expected_checksum = cls.calculate_checksum(payload)
            if checksum_byte[0] != expected_checksum:
                continue

            cmd_id = payload[0]
            if cmd_id == expected_id:
                return payload

        return None

    @classmethod
    def parse_get_pose_response(cls, payload: bytes) -> Optional[DobotPose]:
        """
        Parses payload from GetPose response into DobotPose.
        Payload layout:
          byte 0: cmd_id (0x0A)
          byte 1: ctrl (0x00)
          bytes 2..33: 8 x float32 (little-endian)
        """
        if len(payload) < 34 or payload[0] != cls.CMD_GET_POSE:
            return None

        params = payload[2:34]
        x, y, z, r, j1, j2, j3, j4 = struct.unpack("<8f", params)
        return DobotPose(x=x, y=y, z=z, r=r, j1=j1, j2=j2, j3=j3, j4=j4)
