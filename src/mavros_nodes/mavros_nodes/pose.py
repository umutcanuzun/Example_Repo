# http://docs.ros.org/en/api/geometry_msgs/html/msg/PoseStamped.html
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class PoseStampedSub(Node):

    def __init__(self):
        super().__init__('pose_stamped_subscriber')
        self.create_subscription(
            PoseStamped,
            'mavros/local_position/pose',
            self.pose_stamped_callback,
            10
        )

    def pose_stamped_callback(self, msg):

        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z

        self.get_logger().info('Position:')
        self.get_logger().info(f'  X: {x}')
        self.get_logger().info(f'  Y: {y}')
        self.get_logger().info(f'  Z: {z}')

        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w

        self.get_logger().info('Orientation:')
        self.get_logger().info(f'  X: {qx}')
        self.get_logger().info(f'  Y: {qy}')
        self.get_logger().info(f'  Z: {qz}')
        self.get_logger().info(f'  W: {qw}')

def main(args=None):
    rclpy.init(args=args)
    node = PoseStampedSub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
