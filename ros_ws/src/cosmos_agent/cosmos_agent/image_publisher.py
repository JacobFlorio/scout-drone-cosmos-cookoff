import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher = self.create_publisher(Image, '/siyi_a8/image_raw', 10)
        self.timer = self.create_timer(1.0, self.publish_image)
        self.bridge = CvBridge()
        self.image_path = '/home/jacob/scout-drone-cosmos-cookoff/data/frames/machete_frame.jpg'
        self.get_logger().info('Publishing static image loop')

    def publish_image(self):
        img = cv2.imread(self.image_path)
        if img is not None:
            msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
            self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
