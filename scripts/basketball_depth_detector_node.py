import math
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from message_filters import Subscriber, ApproximateTimeSynchronizer
from ultralytics import YOLO

# this node uses the depth image to estimate basketball position more accurately than just bounding box size alone
class BasketballDepthDetector(Node):
    def __init__(self):
        super().__init__("basketball_depth_detector")

        self.bridge = CvBridge()
        self.model = YOLO("yolov8n.pt")

        self.SPORTS_BALL_CLASS_ID = 32
        self.HORIZONTAL_FOV_DEG = 70.0

        self.latest_rgb = None
        self.latest_depth = None

        # CHANGE THESE idk if these are the right topic names 
        self.rgb_topic = "/camera/image_raw"
        self.depth_topic = "/camera/depth_image"

        self.rgb_sub = Subscriber(self, Image, self.rgb_topic)
        self.depth_sub = Subscriber(self, Image, self.depth_topic)

        self.sync = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub],
            queue_size=10,
            slop=0.2
        )

        self.sync.registerCallback(self.image_callback)

        # Print detection once per second
        self.timer = self.create_timer(1.0, self.detect_once)

        self.get_logger().info("Basketball depth detector started.")
        self.get_logger().info(f"RGB topic: {self.rgb_topic}")
        self.get_logger().info(f"Depth topic: {self.depth_topic}")

    def image_callback(self, rgb_msg, depth_msg):
        self.latest_rgb = rgb_msg
        self.latest_depth = depth_msg

    def get_depth_at_box_center(self, depth_image, depth_msg, cx, cy):
        patch_radius = 8

        h, w = depth_image.shape[:2]

        x_min = max(0, cx - patch_radius)
        x_max = min(w, cx + patch_radius)
        y_min = max(0, cy - patch_radius)
        y_max = min(h, cy + patch_radius)

        patch = depth_image[y_min:y_max, x_min:x_max].astype(np.float32)

        # 1millimeters
        if depth_msg.encoding == "16UC1":
            patch = patch / 1000.0

        # meters
        valid = patch[np.isfinite(patch)]
        valid = valid[valid > 0.0]

        if len(valid) == 0:
            return None

        return float(np.median(valid))

    def detect_once(self):
        if self.latest_rgb is None or self.latest_depth is None:
            self.get_logger().info("Waiting for RGB and depth images...")
            return

        rgb_frame = self.bridge.imgmsg_to_cv2(
            self.latest_rgb,
            desired_encoding="bgr8"
        )

        depth_image = self.bridge.imgmsg_to_cv2(
            self.latest_depth,
            desired_encoding="passthrough"
        )

        image_height, image_width = rgb_frame.shape[:2]
        image_center_x = image_width / 2.0

        results = self.model(rgb_frame, verbose=False)

        best_box = None
        best_confidence = 0.0

        for result in results:
            for box in result.boxes:
                class_id = int(box.cls[0])
                confidence = float(box.conf[0])

                if class_id == self.SPORTS_BALL_CLASS_ID and confidence > best_confidence:
                    best_box = box
                    best_confidence = confidence

        if best_box is None:
            self.get_logger().info("No basketball detected.")
            return

        x1, y1, x2, y2 = best_box.xyxy[0].tolist()

        box_center_x = int((x1 + x2) / 2.0)
        box_center_y = int((y1 + y2) / 2.0)
        box_width = x2 - x1

        pixel_offset = box_center_x - image_center_x

        half_width = image_width / 2.0
        half_fov_rad = math.radians(self.HORIZONTAL_FOV_DEG / 2.0)

        heading_rad = math.atan(
            (pixel_offset / half_width) * math.tan(half_fov_rad)
        )

        heading_deg = math.degrees(heading_rad)

        distance_m = self.get_depth_at_box_center(
            depth_image,
            self.latest_depth,
            box_center_x,
            box_center_y
        )

        direction = "CENTER"
        if heading_deg < -3:
            direction = "LEFT"
        elif heading_deg > 3:
            direction = "RIGHT"

        if distance_m is None:
            distance_text = "unknown"
        else:
            distance_text = f"{distance_m:.2f} m"

        self.get_logger().info(
            f"Basketball detected | "
            f"confidence={best_confidence:.3f} | "
            f"heading={heading_deg:.2f} deg {direction} | "
            f"distance={distance_text} | "
            f"bbox=({x1:.1f}, {y1:.1f}, {x2:.1f}, {y2:.1f}) | "
            f"bbox_width={box_width:.1f}px"
        )


def main(args=None):
    rclpy.init(args=args)
    node = BasketballDepthDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()