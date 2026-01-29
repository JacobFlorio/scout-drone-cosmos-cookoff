#!/usr/bin/env python3
import time
from typing import Dict, List

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class HeartbeatMonitorNode(Node):
    """
    Monitors a list of heartbeat topics (std_msgs/Bool). If any required topic
    hasn't been seen within timeout_sec, system_ok becomes False.
    """

    def __init__(self) -> None:
        super().__init__('heartbeat_monitor_node')

        # Parameters
        self.declare_parameter('required_heartbeats', rclpy.Parameter.Type.STRING_ARRAY)
        self.declare_parameter('timeout_sec', 2.0)
        self.declare_parameter('publish_rate_hz', 2.0)

        self.required: List[str] = list(self.get_parameter('required_heartbeats').value)
        self.timeout_sec: float = float(self.get_parameter('timeout_sec').value)
        self.publish_rate_hz: float = float(self.get_parameter('publish_rate_hz').value)

        self.last_seen: Dict[str, float] = {}
        self.subs = []

        # Publisher for global OK flag
        self.pub_ok = self.create_publisher(Bool, 'asd/system_ok', 10)

        # Subscribe to each heartbeat topic
        for topic in self.required:
            self.last_seen[topic] = 0.0
            self.subs.append(
                self.create_subscription(Bool, topic, self._make_cb(topic), 10)
            )

        period = 1.0 / max(self.publish_rate_hz, 0.1)
        self.timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f"HeartbeatMonitor started. required={self.required}, timeout_sec={self.timeout_sec}, publish_rate_hz={self.publish_rate_hz}"
        )

    def _make_cb(self, topic: str):
        def cb(msg: Bool) -> None:
            # Treat any received message as "alive"
            self.last_seen[topic] = time.time()
        return cb

    def _on_timer(self) -> None:
        now = time.time()

        missing = []
        for topic in self.required:
            t = self.last_seen.get(topic, 0.0)
            if t == 0.0 or (now - t) > self.timeout_sec:
                missing.append(topic)

        ok = (len(missing) == 0)

        out = Bool()
        out.data = ok
        self.pub_ok.publish(out)

        if ok:
            self.get_logger().debug("system_ok=True")
        else:
            self.get_logger().warn(f"system_ok=False missing={missing}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HeartbeatMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("HeartbeatMonitor shutting down.")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
