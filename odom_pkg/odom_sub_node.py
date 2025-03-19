import rclpy
from rclpy.node import Node 
from nav_msgs.msg import Odometry


class odom_sub(Node):
      def __init__(self):
            super().__init__('odom_sub_node')


            self.odom_sub_ = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)


      def odom_callback(self, msg):
            self.pos = msg.pose.pose.position.x
            self.yaw = msg.pose.pose.orientation.w

            self.get_logger().info(f'postion :{self.pos}, angular velocity: {self.yaw}')


def main():
    rclpy.init()
    
    odom_sub_node = odom_sub()
    
    try:
        rclpy.spin(odom_sub_node)
    except KeyboardInterrupt:
        pass
    
    odom_sub_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
