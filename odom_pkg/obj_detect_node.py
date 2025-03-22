import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from my_bot_msgs.msg import ObjectDetection, ObjectDetectionArray  # Import custom message
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import numpy as np

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')  # Load YOLO model

        # Subscribers
        self.rgb_sub = self.create_subscription(
            Image, '/camera/image_raw', self.rgb_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_raw', self.depth_callback, 10)

        # Publisher for detected objects and distances
        self.detection_pub = self.create_publisher(ObjectDetectionArray, '/detected_objects', 10)

        self.depth_image = None
        self.color_map = self.generate_color_map()

    def generate_color_map(self):
        """Generates a unique color for each class label."""
        num_classes = len(self.model.names)
        np.random.seed(42)  # Seed for consistent colors
        return {i: tuple(np.random.randint(0, 255, 3).tolist()) for i in range(num_classes)}

    def rgb_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            results = self.model(cv_image)

            # Create an array to hold all detections
            detection_array = ObjectDetectionArray()
            detections = []

            # Process results
            for result in results:
                boxes = result.boxes
                for box in boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    class_id = int(box.cls)
                    label = self.model.names[class_id]
                    confidence = box.conf[0]
                    color = self.color_map[class_id]  # Assign unique color

                    # Draw bounding box
                    cv2.rectangle(cv_image, (x1, y1), (x2, y2), color, 2)
                    cv2.putText(cv_image, f'{label} {confidence:.2f}', (x1, y1 - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)

                    # Calculate distance if depth image is available
                    depth_value = 0.0
                    if self.depth_image is not None:
                        depth_region = self.depth_image[y1:y2, x1:x2]
                        if depth_region.size > 0:
                            depth_values = depth_region[np.nonzero(depth_region)]
                            if depth_values.size > 0:
                                depth_value = np.min(depth_values)
                                cv2.putText(cv_image, f'D: {depth_value:.2f}m', (x1, y2 + 25),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)

                    # Create a detection message
                    detection = ObjectDetection()
                    detection.label = label
                    detection.confidence = float(confidence)
                    detection.distance = float(depth_value)
                    detections.append(detection)

            # Publish all detections
            detection_array.detections = detections
            self.detection_pub.publish(detection_array)

            cv2.imshow('Object Detection', cv_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Error processing RGB image: {e}')

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()