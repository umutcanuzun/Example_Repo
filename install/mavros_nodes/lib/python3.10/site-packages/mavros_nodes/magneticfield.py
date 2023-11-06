# from http://docs.ros.org/en/api/sensor_msgs/html/msg/MagneticField.html
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import MagneticField

class MagneticFieldSub(Node):

    def __init__(self):
        super().__init__('magnetic_field_subscriber')
        self.create_subscription(
            MagneticField,
            'mavro/imu/mag',  
            self.magnetic_field_callback,
            10  
        )
        

    def magnetic_field_callback(self, msg):
        self.get_logger().info("Received Magnetic Field Data:")
        self.get_logger().info(f"Magnetic Field: [{msg.magnetic_field.x}, {msg.magnetic_field.y}, {msg.magnetic_field.z}]")
        self.get_logger().info(f"Magnetic Field Covariance: {msg.magnetic_field_covariance}")

def main(args=None):
    rclpy.init(args=args)
    node = MagneticFieldSub()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
