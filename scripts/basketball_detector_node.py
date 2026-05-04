import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO

# this node estimates basketball position using bounding box size alone
class BasketballDetector(Node):
    def __init__(self):
        super().__init__("basketball_detector")

        self.bridge = CvBridge()
        self.model = YOLO("yolov8n.pt")

        # COCO class id for sports ball we are just looking for basketballs
        self.SPORTS_BALL_CLASS_ID = 32

        # Calibration from a few sample images
        self.KNOWN_DISTANCE = 2.0
        self.KNOWN_BOX_WIDTH = 139.0

        # Approx camera horizontal field of view
        # Adjust later 
        self.HORIZONTAL_FOV_DEG = 70.0

        self.latest_image = None

        self.subscription = self.create_subscription(
            Image,
            "/camera/image_raw",
            self.image_callback,
            10
        )

        # Run detection once per second
        self.timer = self.create_timer(1.0, self.detect_once)

        self.get_logger().info("Basketball detector node started.")

    def image_callback(self, msg):
        self.latest_image = msg

    def detect_once(self):
        if self.latest_image is None:
            self.get_logger().info("No camera image received yet.")
            return

        frame = self.bridge.imgmsg_to_cv2(
            self.latest_image,
            desired_encoding="bgr8"
        )

        image_height, image_width, _ = frame.shape
        image_center_x = image_width / 2.0

        results = self.model(frame, verbose=False)

        best_detection = None
        best_confidence = 0.0

        for result in results:
            for box in result.boxes:
                class_id = int(box.cls[0])
                confidence = float(box.conf[0])

                if class_id == self.SPORTS_BALL_CLASS_ID and confidence > best_confidence:
                    best_confidence = confidence
                    best_detection = box

        if best_detection is None:
            self.get_logger().info("No basketball detected.")
            return

        x1, y1, x2, y2 = best_detection.xyxy[0].tolist()

        box_center_x = (x1 + x2) / 2.0
        box_width = x2 - x1

        # Pixel offset from center of image
        pixel_offset = box_center_x - image_center_x

        # Convert pixel offset to approximate heading angle
        half_width = image_width / 2.0
        half_fov_rad = math.radians(self.HORIZONTAL_FOV_DEG / 2.0)

        heading_rad = math.atan(
            (pixel_offset / half_width) * math.tan(half_fov_rad)
        )

        heading_deg = math.degrees(heading_rad)

        # Rough distance estimate using bounding box width
        estimated_distance = (
            self.KNOWN_DISTANCE * self.KNOWN_BOX_WIDTH / box_width
        )

        direction = "CENTER"
        if heading_deg < -3:
            direction = "LEFT"
        elif heading_deg > 3:
            direction = "RIGHT"

        self.get_logger().info(
            f"Basketball detected | "
            f"confidence={best_confidence:.3f} | "
            f"heading={heading_deg:.2f} deg {direction} | "
            f"distance≈{estimated_distance:.2f} units | "
            f"bbox_width={box_width:.1f}px"
        )


def main(args=None):
    rclpy.init(args=args)
    node = BasketballDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()