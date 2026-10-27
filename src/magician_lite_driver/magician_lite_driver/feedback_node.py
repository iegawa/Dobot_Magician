"""
ROS 2 Jazzy Driver Node for Dobot Magician Lite.

Features:
  - Real-time arm pose and joint angles feedback published on:
      /dobot/joint_states (sensor_msgs/msg/JointState)
      /dobot/current_pose (geometry_msgs/msg/PoseStamped)
  - Safe motion control services with workspace and velocity validation:
      /dobot/move_to_pose (magician_lite_msgs/srv/SetPose)
      /dobot/move_to_joints (magician_lite_msgs/srv/SetJoints)
      /dobot/emergency_stop (std_srvs/srv/Trigger)
"""

import math
import threading
from typing import Optional
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
import serial

from magician_lite_msgs.srv import SetPose, SetJoints
from magician_lite_driver.protocol import DobotProtocol, DobotPose


class MagicianLiteDriverNode(Node):
    def __init__(self):
        super().__init__("magician_lite_driver_node")

        # Serial parameters
        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("poll_rate_hz", 10.0)
        self.declare_parameter("base_frame_id", "dobot_base_link")
        self.declare_parameter(
            "joint_names",
            ["dobot_joint_1", "dobot_joint_2", "dobot_joint_3", "dobot_joint_4"],
        )

        # Safety & motion limits
        self.declare_parameter("speed_ratio", 20.0)      # % of max speed (default 20% for safety)
        self.declare_parameter("accel_ratio", 20.0)      # % of max accel
        self.declare_parameter("min_z_m", -0.01)         # Min Z in meters (-10 mm)
        self.declare_parameter("max_z_m", 0.160)         # Max Z in meters (160 mm)
        self.declare_parameter("min_reach_m", 0.140)     # Min horizontal reach (140 mm)
        self.declare_parameter("max_reach_m", 0.330)     # Max horizontal reach (330 mm)

        self.port = self.get_parameter("port").get_parameter_value().string_value
        self.baudrate = self.get_parameter("baudrate").get_parameter_value().integer_value
        self.poll_rate_hz = self.get_parameter("poll_rate_hz").get_parameter_value().double_value
        self.base_frame_id = (
            self.get_parameter("base_frame_id").get_parameter_value().string_value
        )
        self.joint_names = (
            self.get_parameter("joint_names").get_parameter_value().string_array_value
        )
        self.speed_ratio = self.get_parameter("speed_ratio").get_parameter_value().double_value
        self.accel_ratio = self.get_parameter("accel_ratio").get_parameter_value().double_value
        self.min_z_m = self.get_parameter("min_z_m").get_parameter_value().double_value
        self.max_z_m = self.get_parameter("max_z_m").get_parameter_value().double_value
        self.min_reach_m = self.get_parameter("min_reach_m").get_parameter_value().double_value
        self.max_reach_m = self.get_parameter("max_reach_m").get_parameter_value().double_value

        # Publishers
        self.joint_state_pub = self.create_publisher(JointState, "/dobot/joint_states", 10)
        self.pose_pub = self.create_publisher(PoseStamped, "/dobot/current_pose", 10)

        # Services for motion control
        self.srv_move_pose = self.create_service(
            SetPose, "/dobot/move_to_pose", self.handle_move_to_pose
        )
        self.srv_move_joints = self.create_service(
            SetJoints, "/dobot/move_to_joints", self.handle_move_to_joints
        )
        self.srv_stop = self.create_service(
            Trigger, "/dobot/emergency_stop", self.handle_emergency_stop
        )
        self.srv_clear_alarms = self.create_service(
            Trigger, "/dobot/clear_alarms", self.handle_clear_alarms
        )

        # Serial state and concurrency lock
        self.serial_lock = threading.Lock()
        self.serial_conn: Optional[serial.Serial] = None
        self.get_pose_packet = DobotProtocol.build_get_pose_packet()

        # Connect to hardware
        self._connect_serial()

        # Timer loop for polling feedback
        timer_period = 1.0 / max(1.0, self.poll_rate_hz)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(
            f"Dobot Magician Lite Driver started. Polling at {self.poll_rate_hz:.1f} Hz. "
            f"Safety speed ratio: {self.speed_ratio:.0f}%."
        )

    def _connect_serial(self) -> bool:
        """Attempts to open serial connection and configure safe motion parameters."""
        with self.serial_lock:
            if self.serial_conn and self.serial_conn.is_open:
                try:
                    self.serial_conn.close()
                except Exception:
                    pass

            try:
                self.serial_conn = serial.Serial(
                    self.port,
                    baudrate=self.baudrate,
                    timeout=0.2,
                    write_timeout=0.2,
                )
                self.serial_conn.reset_input_buffer()
                self.serial_conn.reset_output_buffer()

                # Clear any lingering alarms and configure safe velocities
                clear_pkt = DobotProtocol.build_clear_all_alarms_state()
                coord_pkt = DobotProtocol.build_set_ptp_coordinate_params(
                    xyz_vel=100.0, r_vel=100.0, xyz_acc=100.0, r_acc=100.0, is_queued=False
                )
                joint_pkt = DobotProtocol.build_set_ptp_joint_params(
                    vel=100.0, acc=100.0, is_queued=False
                )
                param_pkt = DobotProtocol.build_set_ptp_common_params(
                    self.speed_ratio, self.accel_ratio, is_queued=False
                )
                start_pkt = DobotProtocol.build_queue_start_exec()

                self.serial_conn.write(clear_pkt)
                self.serial_conn.write(coord_pkt)
                self.serial_conn.write(joint_pkt)
                self.serial_conn.write(param_pkt)
                self.serial_conn.write(start_pkt)
                self.serial_conn.flush()

                self.get_logger().info(
                    f"Connected to Dobot on {self.port} ({self.baudrate} baud). "
                    f"Configured PTP params and started queue. Speed ratio: {self.speed_ratio:.0f}%."
                )
                return True
            except serial.SerialException as ex:
                self.get_logger().warn(
                    f"Could not connect to {self.port}: {ex}. Will retry in timer loop.",
                    throttle_duration_sec=5.0,
                )
                self.serial_conn = None
                return False

    def timer_callback(self):
        """Periodically requests and publishes real-time pose and joint states."""
        if self.serial_conn is None or not self.serial_conn.is_open:
            self._connect_serial()
            return

        with self.serial_lock:
            try:
                self.serial_conn.write(self.get_pose_packet)
                self.serial_conn.flush()

                raw_payload = DobotProtocol.read_packet(
                    self.serial_conn, expected_id=DobotProtocol.CMD_GET_POSE, max_attempts=15
                )

                if not raw_payload:
                    return

                pose_data: Optional[DobotPose] = DobotProtocol.parse_get_pose_response(raw_payload)
                if not pose_data:
                    return

                now = self.get_clock().now().to_msg()

                # 1. Publish Joint States (radians)
                joint_msg = JointState()
                joint_msg.header.stamp = now
                joint_msg.header.frame_id = self.base_frame_id
                joint_msg.name = self.joint_names
                joint_msg.position = [
                    math.radians(pose_data.j1),
                    math.radians(pose_data.j2),
                    math.radians(pose_data.j3),
                    math.radians(pose_data.j4),
                ]
                self.joint_state_pub.publish(joint_msg)

                # 2. Publish Current Cartesian Pose (meters)
                pose_msg = PoseStamped()
                pose_msg.header.stamp = now
                pose_msg.header.frame_id = self.base_frame_id
                pose_msg.pose.position.x = pose_data.x / 1000.0
                pose_msg.pose.position.y = pose_data.y / 1000.0
                pose_msg.pose.position.z = pose_data.z / 1000.0

                yaw_rad = math.radians(pose_data.r)
                pose_msg.pose.orientation.x = 0.0
                pose_msg.pose.orientation.y = 0.0
                pose_msg.pose.orientation.z = math.sin(yaw_rad / 2.0)
                pose_msg.pose.orientation.w = math.cos(yaw_rad / 2.0)

                self.pose_pub.publish(pose_msg)

            except (serial.SerialException, OSError) as ex:
                self.get_logger().error(f"Serial communication error: {ex}. Reconnecting...")
                self.serial_conn = None

    def handle_move_to_pose(self, request: SetPose.Request, response: SetPose.Response):
        """Service handler for Cartesian target movement (/dobot/move_to_pose)."""
        if self.serial_conn is None or not self.serial_conn.is_open:
            response.success = False
            response.message = "Robot serial port is not connected."
            return response

        # Check safety envelope (SI units in meters)
        reach = math.hypot(request.x, request.y)
        if not (self.min_reach_m <= reach <= self.max_reach_m):
            response.success = False
            response.message = (
                f"Target horizontal reach {reach * 1000.0:.1f} mm is outside safe limits "
                f"[{self.min_reach_m * 1000.0:.0f}, {self.max_reach_m * 1000.0:.0f}] mm."
            )
            self.get_logger().warn(response.message)
            return response

        if not (self.min_z_m <= request.z <= self.max_z_m):
            response.success = False
            response.message = (
                f"Target Z {request.z * 1000.0:.1f} mm is outside safe limits "
                f"[{self.min_z_m * 1000.0:.0f}, {self.max_z_m * 1000.0:.0f}] mm."
            )
            self.get_logger().warn(response.message)
            return response

        # Convert to Dobot units: mm and degrees
        target_x_mm = request.x * 1000.0
        target_y_mm = request.y * 1000.0
        target_z_mm = request.z * 1000.0
        target_r_deg = math.degrees(request.r)

        # Select mode (1 = MOVJ_XYZ, 2 = MOVL_XYZ)
        ptp_mode = DobotProtocol.PTP_MOVL_XYZ if request.mode == 2 else DobotProtocol.PTP_MOVJ_XYZ

        with self.serial_lock:
            try:
                # Build and send PTP command packet
                ptp_pkt = DobotProtocol.build_set_ptp_cmd(
                    ptp_mode=ptp_mode,
                    x=target_x_mm,
                    y=target_y_mm,
                    z=target_z_mm,
                    r=target_r_deg,
                    is_queued=True,
                )
                start_pkt = DobotProtocol.build_queue_start_exec()

                self.serial_conn.write(ptp_pkt)
                self.serial_conn.write(start_pkt)
                self.serial_conn.flush()

                mode_str = "MOVL (Linear)" if ptp_mode == DobotProtocol.PTP_MOVL_XYZ else "MOVJ (Joint)"
                response.success = True
                response.message = (
                    f"Pose command queued ({mode_str}): X={target_x_mm:.1f}mm, "
                    f"Y={target_y_mm:.1f}mm, Z={target_z_mm:.1f}mm, R={target_r_deg:.1f}°"
                )
                self.get_logger().info(response.message)
                return response

            except (serial.SerialException, OSError) as ex:
                response.success = False
                response.message = f"Failed to send command over serial: {ex}"
                self.get_logger().error(response.message)
                return response

    def handle_move_to_joints(self, request: SetJoints.Request, response: SetJoints.Response):
        """Service handler for Joint angle target movement (/dobot/move_to_joints)."""
        if self.serial_conn is None or not self.serial_conn.is_open:
            response.success = False
            response.message = "Robot serial port is not connected."
            return response

        # Convert radians to degrees
        j1_deg = math.degrees(request.j1)
        j2_deg = math.degrees(request.j2)
        j3_deg = math.degrees(request.j3)
        j4_deg = math.degrees(request.j4)

        # Safety limits for Magician Lite joints (degrees)
        if not (-135.0 <= j1_deg <= 135.0):
            response.success = False
            response.message = f"Joint 1 angle {j1_deg:.1f}° is outside safe limits [-135, +135]°."
            return response
        if not (-15.0 <= j2_deg <= 85.0):
            response.success = False
            response.message = f"Joint 2 angle {j2_deg:.1f}° is outside safe limits [-15, +85]°."
            return response
        if not (-15.0 <= j3_deg <= 90.0):
            response.success = False
            response.message = f"Joint 3 angle {j3_deg:.1f}° is outside safe limits [-15, +90]°."
            return response
        if not (-145.0 <= j4_deg <= 145.0):
            response.success = False
            response.message = f"Joint 4 angle {j4_deg:.1f}° is outside safe limits [-145, +145]°."
            return response

        with self.serial_lock:
            try:
                ptp_pkt = DobotProtocol.build_set_ptp_cmd(
                    ptp_mode=DobotProtocol.PTP_MOVJ_ANGLE,
                    x=j1_deg,
                    y=j2_deg,
                    z=j3_deg,
                    r=j4_deg,
                    is_queued=True,
                )
                start_pkt = DobotProtocol.build_queue_start_exec()

                self.serial_conn.write(ptp_pkt)
                self.serial_conn.write(start_pkt)
                self.serial_conn.flush()

                response.success = True
                response.message = (
                    f"Joint command queued: J1={j1_deg:.1f}°, J2={j2_deg:.1f}°, "
                    f"J3={j3_deg:.1f}°, J4={j4_deg:.1f}°"
                )
                self.get_logger().info(response.message)
                return response

            except (serial.SerialException, OSError) as ex:
                response.success = False
                response.message = f"Failed to send command over serial: {ex}"
                self.get_logger().error(response.message)
                return response

    def handle_emergency_stop(self, request: Trigger.Request, response: Trigger.Response):
        """Service handler to immediately stop and clear robot motion queue (/dobot/emergency_stop)."""
        if self.serial_conn is None or not self.serial_conn.is_open:
            response.success = False
            response.message = "Robot serial port is not connected."
            return response

        with self.serial_lock:
            try:
                stop_pkt = DobotProtocol.build_queue_stop_exec()
                clear_pkt = DobotProtocol.build_queue_clear()

                self.serial_conn.write(stop_pkt)
                self.serial_conn.write(clear_pkt)
                self.serial_conn.flush()

                response.success = True
                response.message = "Emergency stop executed. Command queue halted and cleared."
                self.get_logger().warn(response.message)
                return response

            except (serial.SerialException, OSError) as ex:
                response.success = False
                response.message = f"Failed to execute emergency stop: {ex}"
                self.get_logger().error(response.message)
                return response

    def handle_clear_alarms(self, request: Trigger.Request, response: Trigger.Response):
        """Service handler to clear all robot alarms (/dobot/clear_alarms)."""
        if self.serial_conn is None or not self.serial_conn.is_open:
            response.success = False
            response.message = "Robot serial port is not connected."
            return response

        with self.serial_lock:
            try:
                clear_pkt = DobotProtocol.build_clear_all_alarms_state()
                self.serial_conn.write(clear_pkt)
                self.serial_conn.flush()

                response.success = True
                response.message = "ClearAllAlarmsState command sent to robot."
                self.get_logger().info(response.message)
                return response

            except (serial.SerialException, OSError) as ex:
                response.success = False
                response.message = f"Failed to send clear alarms command: {ex}"
                self.get_logger().error(response.message)
                return response

    def destroy_node(self):
        """Cleanup serial port on shutdown."""
        with self.serial_lock:
            if self.serial_conn and self.serial_conn.is_open:
                try:
                    self.serial_conn.close()
                except Exception:
                    pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MagicianLiteDriverNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
