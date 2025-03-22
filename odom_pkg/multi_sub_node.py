import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist  # For /cmd_vel
from nav_msgs.msg import Odometry  # For /odom
from my_bot_msgs.msg import ObjectDetectionArray  # For /detected_objects


class MultiTopicSubscriber(Node):
    def __init__(self):
        super().__init__('multi_topic_subscriber')

        # Subscriber for /cmd_vel (Twist)
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Subscriber for /odom (Odometry)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Subscriber for /detected_objects (ObjectDetectionArray)
        self.detected_objects_sub = self.create_subscription(
            ObjectDetectionArray,
            '/detected_objects',
            self.detected_objects_callback,
            10
        )

        self.get_logger().info('MultiTopicSubscriber node has been started')

    def cmd_vel_callback(self, msg):
        # Log the linear and angular velocities from /cmd_vel
        linear = msg.linear
        angular = msg.angular
        self.get_logger().info(
            f'li_vel=(x: {linear.x:.2f}), '
            f'ang_vel=(z: {angular.z:.2f})'
        )

    def odom_callback(self, msg):
        # Log the robot's position and orientation from /odom
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.get_logger().info(
            f'Received odom: position=(x: {position.x:.2f}, y: {position.y:.2f}), '
            f'orientation=(x: {orientation.x:.2f}, y: {orientation.y:.2f}, z: {orientation.z:.2f}, w: {orientation.w:.2f})'
        )

    def detected_objects_callback(self, msg):
        # Log the number of detections
        num_detections = len(msg.detections)
        self.get_logger().info(f'Received {num_detections} detections on /detected_objects')

        # Log details of each detection
        for detection in msg.detections:
            self.get_logger().info(
                f'Detection: label={detection.label}, confidence={detection.confidence:.2f}, '
                f'distance={detection.distance:.2f})'
            )


def main(args=None):
    rclpy.init(args=args)
    node = MultiTopicSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down MultiTopicSubscriber node')
    finally:
        node.destroy_no
        rclpy.shutdown()


if __name__ == '__main__':
    main()