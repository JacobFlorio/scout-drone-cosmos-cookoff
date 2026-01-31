#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand
try:
    from px4_msgs.msg import VehicleStatusV1 as VehicleStatusMsg
    VEHICLE_STATUS_TOPIC = '/fmu/out/vehicle_status_v1'
except ImportError:
    from px4_msgs.msg import VehicleStatus as VehicleStatusMsg
    VEHICLE_STATUS_TOPIC = '/fmu/out/vehicle_status_v1'



def now_us(node: Node) -> int:
    # PX4 messages expect timestamps in microseconds
    return int(node.get_clock().now().nanoseconds / 1000)


class OffboardCommanderNode(Node):
    """
    Minimal PX4 offboard example for SITL via uXRCE-DDS:
    - Publishes OffboardControlMode + TrajectorySetpoint at 20 Hz
    - Arms immediately
    - Switches to OFFBOARD after 5 seconds of setpoint streaming
    - Holds position at (0,0,-3) in NED, yaw=0
    """

    def __init__(self) -> None:
        super().__init__('offboard_commander_node')

        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('offboard_delay_sec', 5.0)
        self.declare_parameter('hover_z', -3.0)
        self.declare_parameter('arm_retry_sec', 1.0)

        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.offboard_delay = float(self.get_parameter('offboard_delay_sec').value)
        self.hover_z = float(self.get_parameter('hover_z').value)
        self.arm_retry_sec = float(self.get_parameter('arm_retry_sec').value)

        # PX4 bridge topics often want reliable + transient local
        qos_out = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.pub_offboard = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.pub_traj = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.pub_cmd = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos)

        self.sub_status = self.create_subscription(
            VehicleStatusMsg, VEHICLE_STATUS_TOPIC, self._on_status, qos_out
        )
        self._last_status = None

        self.start_time = self.get_clock().now()
        self.sent_arm = False
        self.sent_offboard = False
        self._last_arm_attempt = -1.0

        period = 1.0 / max(self.rate_hz, 1.0)
        self.timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f"OffboardCommander started (rate_hz={self.rate_hz}, offboard_delay_sec={self.offboard_delay}, hover_z={self.hover_z})"
        )

    def _on_status(self, msg: VehicleStatusMsg) -> None:
        self._last_status = msg

        # If PX4 reports ARMED, stop sending ARM retries
        try:
            if int(getattr(msg, 'arming_state', 0)) == 2:
                self.sent_arm = True
        except Exception:
            pass
    def _send_vehicle_command(self, command: int, param1: float = 0.0, param2: float = 0.0) -> None:
        msg = VehicleCommand()
        msg.timestamp = now_us(self)

        msg.param1 = float(param1)
        msg.param2 = float(param2)

        msg.command = int(command)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True

        self.pub_cmd.publish(msg)

    def _publish_offboard_mode(self) -> None:
        msg = OffboardControlMode()
        msg.timestamp = now_us(self)
        # Position control
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.pub_offboard.publish(msg)

    def _publish_trajectory(self) -> None:
        msg = TrajectorySetpoint()
        msg.timestamp = now_us(self)
        msg.position = [0.0, 0.0, float(self.hover_z)]
        msg.velocity = [math.nan, math.nan, math.nan]
        msg.acceleration = [math.nan, math.nan, math.nan]
        msg.yaw = 0.0
        msg.yawspeed = math.nan
        self.pub_traj.publish(msg)

    def _on_timer(self) -> None:
        # Time since start (seconds)
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        # Always stream required offboard topics
        self._publish_offboard_mode()
        self._publish_trajectory()

        # ARM retry
        if not self.sent_arm:
            if (self._last_arm_attempt < 0.0) or ((elapsed - self._last_arm_attempt) >= self.arm_retry_sec):
                self._send_vehicle_command(400, param1=1.0)  # ARM
                self._last_arm_attempt = elapsed
                self.get_logger().info("Sent ARM command (retry).")

        # OFFBOARD after delay
        if (not self.sent_offboard) and elapsed >= self.offboard_delay:
            self._send_vehicle_command(176, param1=1.0, param2=6.0)
            self.sent_offboard = True
            self.get_logger().info("Sent OFFBOARD mode command.")




    def destroy_node(self) -> bool:
        self.get_logger().info("OffboardCommander shutting down.")
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OffboardCommanderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
