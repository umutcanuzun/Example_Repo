# from http://docs.ros.org/en/api/geometry_msgs/html/msg/TwistStamped.html
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

class TwistStampedSub(Node):
    
    def __init__(self):
        super().__init__('twist_stamped_subscriber')
        self.create_subscription(
            TwistStamped,
            'mavros/local_position/velocity', 
            self.twist_stamped_callback,
            10 
        )

    def twist_stamped_callback(self, msg):
        self.get_logger().info(f'The linear velocity along the x-axis: {msg.twist.linear.x}')
        self.get_logger().info(f'The linear velocity along the y-axis: {msg.twist.linear.y}')
        self.get_logger().info(f'The linear velocity along the z-axis: {msg.twist.linear.z}')

        self.get_logger().info('The angular velocity around the x-axis: {msg.twist.angular.x}') # roll velocity
        self.get_logger().info('The angular velocity around the y-axis: {msg.twist.angular.y}') # pitch velocity
        self.get_logger().info('The angular velocity around the z-axis: {msg.twist.angular.z}') # yaw velocity

def main(args=None):
    rclpy.init(args=args)
    node = TwistStampedSub()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()