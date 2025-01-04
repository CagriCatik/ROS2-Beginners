import rclpy
from rclpy.node import Node

class MyPythonNode(Node):
    def __init__(self):
        super().__init__('my_python_node')
        self.get_logger().info('Hello ROS2 from Python!')

def main(args=None):
    rclpy.init(args=args)
    node = MyPythonNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
