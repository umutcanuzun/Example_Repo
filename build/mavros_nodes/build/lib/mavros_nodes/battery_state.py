# from http://docs.ros.org/en/api/sensor_msgs/html/msg/BatteryState.html
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState

class BatteryStateSub(Node):

    def __init__(self):
        super().__init__('battery_state_subscriber')
        self.create_subscription(
            BatteryState,
            'mavros/battery',
            self.battery_state_callback,
            10
        )

    def battery_state_callback(self, msg):
        # Gerekli gorduklerim:
        self.get_logger().info(f'Capacity: {msg.capacity} Ah')  # Belki gosterilmeyebilir
        self.get_logger().info(f'Voltage: {msg.voltage} V')
        self.get_logger().info(f'Current: {msg.current} A')
        self.get_logger().info(f'Percentage: {msg.percentage * 100}%')

        # charge bilgisine gerek yok gibi, percantage o isi goruyor
        # Gerekliyse ekle --> self.get_logger().info(f'Charge: {msg.charge} Ah')

def main(args=None):
    rclpy.init(args=args)
    node = BatteryStateSub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    