#!/usr/bin/env python3
import time
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool


@dataclass
class WatchdogStatus:
    ok: bool
    last_heartbeat_time: float


class SafetyWatchdogNode(Node):
    """
    Minimal safety watchdog for the project.

    - Publishes a heartbeat / system_ok boolean at a fixed rate.
    - Future: subscribe to perception + PX4 status and gate autonomy/offboard.
    """

    def __init__(self) -> None:
        super().__init__('safety_watchdog_node')

        # Parameters (easy to tune from launch files later)
        self.declare_parameter('rate_hz', 2.0)
        self.declare_parameter('log_period_sec', 5.0)

        self._rate_hz = float(self.get_parameter('rate_hz').value)
        self._log_period_sec = float(self.get_parameter('log_period_sec').value)

        self._pub_ok = self.create_publisher(Bool, 'asd/system_ok', 10)

        self._last_log = 0.0
        self._status = WatchdogStatus(ok=True, last_heartbeat_time=time.time())

        period = 1.0 / max(self._rate_hz, 0.1)
        self._timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f"SafetyWatchdog started (rate_hz={self._rate_hz}, log_period_sec={self._log_period_sec})."
        )

    def _on_timer(self) -> None:
        # For now, always OK. Later we’ll set ok=False based on topic timeouts, PX4 failsafes, etc.
        now = time.time()
        self._status.last_heartbeat_time = now

        msg = Bool()
        msg.data = bool(self._status.ok)
        self._pub_ok.publish(msg)

        if (now - self._last_log) >= self._log_period_sec:
            self.get_logger().info(f"system_ok={self._status.ok}")
            self._last_log = now


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SafetyWatchdogNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("SafetyWatchdog shutting down.")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
