#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class PublisherHeartbeatNode(Node):
    def __init__(self) -> None:
        super().__init__('px4_heartbeat_node')
        self.declare_parameter('topic', 'asd/px4/heartbeat')
        self.declare_parameter('rate_hz', 2.0)

        topic = str(self.get_parameter('topic').value)
        rate_hz = float(self.get_parameter('rate_hz').value)

        self.pub = self.create_publisher(Bool, topic, 10)
        self.timer = self.create_timer(1.0 / max(rate_hz, 0.1), self._tick)
        self.get_logger().info(f"Publishing heartbeat on {topic} at {rate_hz} Hz")

    def _tick(self) -> None:
        m = Bool()
        m.data = True
        self.pub.publish(m)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PublisherHeartbeatNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
