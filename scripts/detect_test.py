from ultralytics import YOLO
import sys
from pathlib import Path
# this script detects basketballs in images using a pretrained YOLOv8 model.
# it takes images as command line arguments to test detection performance.

# pretrained model from ultralytics hub
model = YOLO("yolov8n.pt")

SPORTS_BALL_CLASS_ID = 32

def detect_image(image_path):
    results = model(image_path)

    print(f"\nImage: {image_path}")

    found = False

    for result in results:
        for box in result.boxes:
            class_id = int(box.cls[0])
            confidence = float(box.conf[0])

            if class_id == SPORTS_BALL_CLASS_ID:
                found = True

                x1, y1, x2, y2 = box.xyxy[0].tolist()

                print("Basketball / sports ball detected")
                print(f"Confidence: {confidence:.3f}")
                print(f"Bounding box: x1={x1:.1f}, y1={y1:.1f}, x2={x2:.1f}, y2={y2:.1f}")

    if not found:
        print("No basketball / sports ball detected.")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python detect_basketball.py image1.jpg image2.png")
        sys.exit(1)

    for image in sys.argv[1:]:
        image_path = Path(image)

        if not image_path.exists():
            print(f"File not found: {image_path}")
            continue

        detect_image(str(image_path))