# from http://docs.ros.org/en/api/mavros_msgs/html/msg/State.html
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State

class StateSub(Node):
    
    def __init__(self):
        super().__init__("state_subscriber")
        self.create_subscription(
            State,
            'mavros/state',
            self.state_callback,
            10
        )
    
    def state_callback(self, msg):

        self.get_logger().info(f'Connected: {msg.connected}') #MAVROS'a bagli miyiz?
        self.get_logger().info(f'Mode: {msg.mode}') # Ucus modu bilgisi
        # bool armed, bool guided, bool manual_input, uint8 system status ihtiyac yok gibi

def main(args=None):
    rclpy.init(args=args)
    node = StateSub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()