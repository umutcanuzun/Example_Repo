# from http://docs.ros.org/en/api/sensor_msgs/html/msg/Imu.html
import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import Imu

class ImuSub(Node):

    def __init__(self):
        super().__init__('imu_subscriber')
        self.create_subscription(
            Imu,
            'mavros/imu/data',  
            self.imu_callback,
            10 
        )

    def imu_callback(self, msg):
        self.get_logger().info("IMU data:")
        self.get_logger().info(f"Linear Acceleration: x={msg.linear_acceleration.x}, y={msg.linear_acceleration.y}, 
                               z={msg.linear_acceleration.z}")
        self.get_logger().info(f"Angular Velocity: x={msg.angular_velocity.x}, y={msg.angular_velocity.y}, 
                               z={msg.angular_velocity.z}")
        self.get_logger().info(f"Orientation: x={msg.orientation.x}, y={msg.orientation.y}, 
                               z={msg.orientation.z}, w={msg.orientation.w}")
        
        # yaw, roll, pitch angles (Euler angles) from Orientation
        # roll: rotation around x-axis, angle in radians.
        # pitch: rotation around y-axis, angle in radians.
        # yaw: rotation around z-axis, angle in radians.
 
        def euler_from_quaternion(x, y, z, w):
            t0 = +2.0 * (w * x + y * z)
            t1 = +1.0 - 2.0 * (x * x + y * y)
            roll_x = math.atan2(t0, t1)
        
            t2 = +2.0 * (w * y - z * x)
            t2 = +1.0 if t2 > +1.0 else t2
            t2 = -1.0 if t2 < -1.0 else t2
            pitch_y = math.asin(t2)
        
            t3 = +2.0 * (w * z + x * y)
            t4 = +1.0 - 2.0 * (y * y + z * z)
            yaw_z = math.atan2(t3, t4)
        
            return roll_x, pitch_y, yaw_z # in radians
            # in degrees, example --> roll_x_degrees = math.degrees(roll_x)


def main(args=None):
    rclpy.init(args=args)
    node = ImuSub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()