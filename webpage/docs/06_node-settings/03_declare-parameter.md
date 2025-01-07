# Declaring Parameters in ROS 2

Declaring parameters within your nodes is a crucial step in ROS 2 development. This section explains how to declare parameters and demonstrates why it is essential to declare them within the node code.

## Python Example

Let's start with a Python node. Assume we have a node named `number_publisher`.

1. **Modify the Node to Declare a Parameter:**

   Edit the `number_publisher.py` file to declare a new parameter in the node's constructor.

   ```python
   import rclpy
   from rclpy.node import Node

   class NumberPublisher(Node):
       def __init__(self):
           super().__init__('number_publisher')
           self.declare_parameter('test_param', 0)  # Declare a parameter named 'test_param'

   def main(args=None):
       rclpy.init(args=args)
       node = NumberPublisher()
       rclpy.spin(node)
       node.destroy_node()
       rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

   In this example, the parameter `test_param` is declared with a default value of `0`.

2. **Compile and Run the Node:**

   ```sh
   colcon build --packages-select <your_package_name>
   source install/setup.bash
   ros2 run <your_package_name> number_publisher
   ```

3. **Check the Parameter:**

   Use the command line to list and get the parameter values.

   ```sh
   ros2 param list
   ros2 param get /number_publisher test_param
   ```

   This should display the parameter `test_param` with its default value.

## C++ Example

Similarly, for a C++ node:

1. **Modify the Node to Declare a Parameter:**

   Edit the `number_publisher.cpp` file to declare a new parameter in the node's constructor.

   ```cpp
   #include "rclcpp/rclcpp.hpp"

   class NumberPublisher : public rclcpp::Node
   {
   public:
       NumberPublisher() : Node("number_publisher")
       {
           this->declare_parameter<int>("test_param", 0);  // Declare a parameter named 'test_param'
       }
   };

   int main(int argc, char * argv[])
   {
       rclcpp::init(argc, argv);
       auto node = std::make_shared<NumberPublisher>();
       rclcpp::spin(node);
       rclcpp::shutdown();
       return 0;
   }
   ```

2. **Compile and Run the Node:**

   ```sh
   colcon build --packages-select <your_package_name>
   source install/setup.bash
   ros2 run <your_package_name> number_publisher
   ```

3. **Check the Parameter:**

   ```sh
   ros2 param list
   ros2 param get /number_publisher test_param
   ```

### Setting Parameter Values

Parameters can be set using the command line tool `ros2 param`. This section demonstrates how to set parameter values and verify them.

1. **Set a Parameter Value:**

   ```sh
   ros2 param set /number_publisher test_param 42
   ```

   This sets the parameter `test_param` to `42`.

2. **Verify the Parameter Value:**

   ```sh
   ros2 param get /number_publisher test_param
   ```

   This should return the value `42`.

### Using Parameters in Node Logic

After declaring and setting parameters, you can use them in your node logic. Here's how to read parameter values within the node.

## Python Example

Modify the `NumberPublisher` class to read the parameter value and use it in the logic.

```python
import rclpy
from rclpy.node import Node

class NumberPublisher(Node):
    def __init__(self):
        super().__init__('number_publisher')
        self.declare_parameter('test_param', 0)
        test_param_value = self.get_parameter('test_param').get_parameter_value().integer_value
        self.get_logger().info(f'Test Param Value: {test_param_value}')

def main(args=None):
    rclpy.init(args=args)
    node = NumberPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

This example reads the parameter `test_param` and logs its value.

## C++ Example

Modify the `NumberPublisher` class to read the parameter value and use it in the logic.

```cpp
#include "rclcpp/rclcpp.hpp"

class NumberPublisher : public rclcpp::Node
{
public:
    NumberPublisher() : Node("number_publisher")
    {
        this->declare_parameter<int>("test_param", 0);
        int test_param_value = this->get_parameter("test_param").as_int();
        RCLCPP_INFO(this->get_logger(), "Test Param Value: %d", test_param_value);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

This example reads the parameter `test_param` and logs its value.

### Conclusion

This tutorial covered the essential aspects of working with parameters in ROS 2, including declaring parameters within nodes, setting parameter values using the command line, and utilizing parameters within node logic. By following these practices, you can effectively manage parameters in your ROS 2 applications, ensuring robust and flexible node configurations.