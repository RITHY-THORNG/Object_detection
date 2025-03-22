import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdomSubscriber(Node):
    def __init__(self):
        super().__init__('odom_subscriber')
        # Create a subscription to the /odom topic
        self.subscription = self.create_subscription(
            Odometry,           # Message type
            '/odom',            # Topic name
            self.odom_callback, # Callback function
            10                  # QoS profile (queue size)
        )
        self.get_logger().info('Subscribed to /odom topic')

    def odom_callback(self, msg):
        # Extract position from the Odometry message
        position = msg.pose.pose.position
        x = position.x
        y = position.y
        #z = position.z
        # Print the position
        self.get_logger().info(f'Position: x={x:.2f}m, y={y:.2f}m')

def main(args=None):
    rclpy.init(args=args)
    odom_subscriber = OdomSubscriber()
    rclpy.spin(odom_subscriber)  # Keep the node running
    odom_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()