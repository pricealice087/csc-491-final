import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')

        self.bridge = CvBridge()
        self.count = 0
        self.output_dir = "saved_images"
        os.makedirs(self.output_dir, exist_ok=True)

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # change this if your topic is different
            self.callback,
            10
        )

        self.get_logger().info("Saving images from /camera/image_raw")

    def callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        filename = os.path.join(self.output_dir, f"frame_{self.count:05d}.png")
        cv2.imwrite(filename, cv_image)

        self.get_logger().info(f"Saved {filename}")
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    node = ImageSaver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()