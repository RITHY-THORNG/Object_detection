import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageSubscriber(Node):

    def __init__(self):
        super().__init__('image_subscriber')
        # Subscribe to the /camera/image_raw topic
        self.subscription = self.create_subscription(
            Image,  # Message type
            '/camera/image_raw',  # Topic name
            self.listener_callback,  # Callback function
            10  # QoS (Quality of Service)
        )
        self.subscription  # prevent unused variable warning

        # Initialize CvBridge to convert ROS images to OpenCV images
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        try:
            # Convert the ROS image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Display the image using OpenCV
            cv2.imshow("Camera Image", cv_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().info('Error converting ROS Image to OpenCV: %s' % str(e))

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()

    # Spin the node to keep it running
    rclpy.spin(image_subscriber)

    # Clean up before shutdown
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
