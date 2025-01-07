# Introduction

This tutorial aims to provide an in-depth understanding of ROS2 parameters, focusing on their declaration, retrieval, and usage within ROS2 nodes. Parameters are essential for configuring nodes at runtime, allowing for flexible and dynamic operation. By the end of this tutorial, you will be proficient in using ROS2 parameters in your applications.

## Overview of ROS2 Parameters

ROS2 parameters enable the configuration of nodes by setting values that can be adjusted without modifying the code. These parameters are stored in key-value pairs and can be of various types, such as integers, floats, strings, booleans, and arrays. Parameters are particularly useful for:
- Adjusting node behavior without recompilation.
- Managing configuration data.
- Facilitating reusable and maintainable code.

## Declaring Parameters in ROS2

Parameters must be declared within a node before they can be used. This declaration ensures that the parameters are recognized by the ROS2 parameter server. Here's how to declare parameters in a ROS2 node:

### Example in Python (rclpy)

```python
import rclpy
from rclpy.node import Node

class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')
        # Declare a parameter with a default value
        self.declare_parameter('my_param', 'default_value')
        
        # Declare multiple parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('param_1', 42),
                ('param_2', 3.14),
                ('param_3', True),
            ]
        )

def main(args=None):
    rclpy.init(args=args)
    node = ParameterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Example in C++ (rclcpp)

```cpp
#include "rclcpp/rclcpp.hpp"

class ParameterNode : public rclcpp::Node
{
public:
    ParameterNode()
    : Node("parameter_node")
    {
        // Declare a parameter with a default value
        this->declare_parameter<std::string>("my_param", "default_value");
        
        // Declare multiple parameters
        this->declare_parameter<int>("param_1", 42);
        this->declare_parameter<double>("param_2", 3.14);
        this->declare_parameter<bool>("param_3", true);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ParameterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

## Retrieving Parameters in ROS2

Once parameters are declared, they can be retrieved and used within the node. Here’s how to get the parameters:

### Example in Python (rclpy)

```python
class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')
        self.declare_parameter('my_param', 'default_value')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('param_1', 42),
                ('param_2', 3.14),
                ('param_3', True),
            ]
        )
        
        # Retrieve parameters
        my_param = self.get_parameter('my_param').get_parameter_value().string_value
        param_1 = self.get_parameter('param_1').get_parameter_value().integer_value
        param_2 = self.get_parameter('param_2').get_parameter_value().double_value
        param_3 = self.get_parameter('param_3').get_parameter_value().bool_value
        
        # Use the parameters
        self.get_logger().info(f'my_param: {my_param}')
        self.get_logger().info(f'param_1: {param_1}')
        self.get_logger().info(f'param_2: {param_2}')
        self.get_logger().info(f'param_3: {param_3}')

# Remaining part of the main function stays the same
```

### Example in C++ (rclcpp)

```cpp
class ParameterNode : public rclcpp::Node
{
public:
    ParameterNode()
    : Node("parameter_node")
    {
        this->declare_parameter<std::string>("my_param", "default_value");
        this->declare_parameter<int>("param_1", 42);
        this->declare_parameter<double>("param_2", 3.14);
        this->declare_parameter<bool>("param_3", true);
        
        // Retrieve parameters
        std::string my_param = this->get_parameter("my_param").as_string();
        int param_1 = this->get_parameter("param_1").as_int();
        double param_2 = this->get_parameter("param_2").as_double();
        bool param_3 = this->get_parameter("param_3").as_bool();
        
        // Use the parameters
        RCLCPP_INFO(this->get_logger(), "my_param: %s", my_param.c_str());
        RCLCPP_INFO(this->get_logger(), "param_1: %d", param_1);
        RCLCPP_INFO(this->get_logger(), "param_2: %f", param_2);
        RCLCPP_INFO(this->get_logger(), "param_3: %s", param_3 ? "true" : "false");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ParameterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

## Conclusion

This tutorial has provided a detailed examination of ROS2 parameters, covering their declaration and retrieval in both Python and C++. By utilizing parameters effectively, you can create more flexible and configurable ROS2 nodes, enhancing the overall robustness and maintainability of your applications.