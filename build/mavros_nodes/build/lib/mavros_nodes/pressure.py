# from http://docs.ros.org/en/api/sensor_msgs/html/msg/FluidPressure.html
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure

class FluidPressureSub(Node):

    def __init__(self):
        super().__init__('fluid_pressure_subscriber')
        self.create_subscription(
            FluidPressure,
            'mavros/imu/atm_pressure',
            self.fluid_pressure_callback,
            10 
        )

    def fluid_pressure_callback(self, msg):
        pressure = msg.fluid_pressure
        variance = msg.variance

        water_density = 1000.0  # (kg/m³)
        gravity = 9.80665  # (m/s²)

        # Calculate depth from pressure 
        depth = pressure / (water_density * gravity) # m

        self.get_logger().info(f"Received Pressure info:")
        self.get_logger().info(f"  Pressure: {pressure} Pa")
        self.get_logger().info(f"  Variance: {variance}") # ihtiyac var mı?
        self.get_logger().info(f"  Depth: {depth} meters")

def main(args=None):
    rclpy.init(args=args)
    node = FluidPressureSub()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
