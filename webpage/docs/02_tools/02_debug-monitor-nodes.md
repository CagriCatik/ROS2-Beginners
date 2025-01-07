# Creating and Managing ROS2 Nodes

This tutorial provides a detailed guide on creating and managing ROS2 nodes using Python and C++. It includes instructions on using the ROS2 command-line tools for efficient node management.

## 1. Creating Python and C++ Nodes

## Python Nodes

To create a Python node, you need to follow these steps:

1. **Create a Python Script**: Write your Python script that defines the node. For example, create a file named `my_python_node.py` with the following content:

   ```python
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
   ```

2. **Package Setup**: Ensure your package is correctly set up. The `setup.py` should include the necessary entry points:

   ```python
   from setuptools import setup

   package_name = 'my_python_package'

   setup(
       name=package_name,
       version='0.0.0',
       packages=[package_name],
       install_requires=['setuptools'],
       zip_safe=True,
       maintainer='your_name',
       maintainer_email='your_email@example.com',
       description='Example Python package for ROS2',
       license='Apache License 2.0',
       tests_require=['pytest'],
       entry_points={
           'console_scripts': [
               'my_python_node = my_python_package.my_python_node:main'
           ],
       },
   )
   ```

## C++ Nodes

For a C++ node, follow these steps:

1. **Create a C++ Source File**: Write your C++ node. For example, create a file named `my_cpp_node.cpp` with the following content:

   ```cpp
   #include "rclcpp/rclcpp.hpp"

   class MyCppNode : public rclcpp::Node {
   public:
       MyCppNode() : Node("my_cpp_node") {
           RCLCPP_INFO(this->get_logger(), "Hello ROS2 from C++!");
       }
   };

   int main(int argc, char * argv[]) {
       rclcpp::init(argc, argv);
       auto node = std::make_shared<MyCppNode>();
       rclcpp::spin(node);
       rclcpp::shutdown();
       return 0;
   }
   ```

2. **CMake Setup**: Ensure your `CMakeLists.txt` is properly configured:

   ```cmake
   cmake_minimum_required(VERSION 3.5)
   project(my_cpp_package)

   find_package(ament_cmake REQUIRED)
   find_package(rclcpp REQUIRED)

   add_executable(my_cpp_node src/my_cpp_node.cpp)
   ament_target_dependencies(my_cpp_node rclcpp)

   install(TARGETS
     my_cpp_node
     DESTINATION lib/${PROJECT_NAME})

   ament_package()
   ```

## 2. Building and Installing Nodes

After writing your node scripts, build and install them in your ROS2 workspace:

1. **Build**:

   ```sh
   colcon build --packages-select my_python_package my_cpp_package
   ```

2. **Source the Environment**:

   ```sh
   source install/setup.bash
   ```

## 3. Using ROS2 Command Line Tools

The ROS2 command-line tools provide a powerful way to manage your nodes.

## 3.1 Launching Nodes

You can launch your nodes using the `ros2 run` command:

```sh
ros2 run <package_name> <executable_name>
```

For example:

```sh
ros2 run my_python_package my_python_node
ros2 run my_cpp_package my_cpp_node
```

## 3.2 Listing and Inspecting Nodes

To list all running nodes:

```sh
ros2 node list
```

To get detailed information about a specific node:

```sh
ros2 node info <node_name>
```

## 3.3 Sourcing Environment

Ensure your environment is correctly sourced by adding the following lines to your `.bashrc`:

```sh
source /opt/ros/foxy/setup.bash
source ~/ros2_ws/install/setup.bash
```

If these lines are not sourced, you will encounter errors when running ROS2 commands.

## 4. Handling Common Issues

## 4.1 Sourcing Errors

If you encounter `command not found` errors, it indicates that the environment is not properly sourced. Source it manually:

```sh
source /opt/ros/foxy/setup.bash
source ~/ros2_ws/install/setup.bash
```

## 4.2 Node Conflicts

Avoid running two nodes with the same name to prevent conflicts in the ROS graph. Ensure each node has a unique name.

## 5. Additional Command-Line Tools

Here are some additional useful ROS2 command-line tools:

- `ros2 pkg create <package_name>`: Creates a new package.
- `ros2 topic list`: Lists all topics.
- `ros2 service list`: Lists all services.

## Conclusion

This tutorial has provided a comprehensive guide to creating and managing ROS2 nodes using Python and C++. By following these instructions and utilizing the ROS2 command-line tools, you can efficiently develop and manage your ROS2 applications. Ensure your environment is correctly sourced and avoid node name conflicts for smooth operation.
