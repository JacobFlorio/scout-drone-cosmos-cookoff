import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher = self.create_publisher(Image, '/siyi_a8/image_raw', 10)
        self.bridge = CvBridge()
        self.frame = cv2.imread('/home/jacob/scout-drone-cosmos-cookoff/data/frames/test_frame.jpg')
        if self.frame is None:
            self.get_logger().error('Failed to load test_frame.jpg')
            return
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz loop
        self.get_logger().info('Publishing static image loop')

    def timer_callback(self):
        if self.frame is not None:
            msg = self.bridge.cv2_to_imgmsg(self.frame, "bgr8")
            msg.header.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
