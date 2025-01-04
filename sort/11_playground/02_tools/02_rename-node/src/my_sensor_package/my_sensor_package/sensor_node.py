import rclpy
from rclpy.node import Node
import random

class SensorNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.temperature = random.uniform(20.0, 25.0)
        self.timer = self.create_timer(2.0, self.publish_temperature)

    def publish_temperature(self):
        self.get_logger().info(f"Temperature: {self.temperature:.2f} °C")

def main(args=None):
    rclpy.init(args=args)
    sensor_name = rclpy.get_node_names()[-1] if args else 'sensor_node'
    node = SensorNode(sensor_name)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
