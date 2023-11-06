import rclpy
from rclpy.node import Node

class MyNode(Node): # Bu sinif ismi önemli degil
    def __init__(self):
        super().__init__("first_node") # Buradaki isim olusturulan node adidir
        self.counter_ = 0 
        self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info(f"Hello {self.counter_}")
        self.counter_ += 1

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()