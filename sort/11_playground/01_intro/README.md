# ROS2 Project Setup

## Overview

This documentation provides a detailed guide on creating, building, and running ROS2 nodes using Python and C++. It also covers common issues you might encounter and their solutions. The project setup includes creating a ROS2 workspace, defining packages, writing node scripts, building the packages, and using ROS2 command-line tools to manage and troubleshoot nodes.

## 1. Setting Up Your ROS2 Workspace

### 1.1 Create a ROS2 Workspace

Start by setting up a new ROS2 workspace:

```sh
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### 1.2 Directory Structure

Ensure your workspace has the following structure:

```
ros2_ws/
  ├── src/
  │   ├── my_python_package/
  │   │   ├── my_python_package/
  │   │   │   └── my_python_node.py
  │   │   ├── setup.py
  │   │   └── package.xml
  │   └── my_cpp_package/
  │       ├── src/
  │       │   └── my_cpp_node.cpp
  │       ├── CMakeLists.txt
  │       └── package.xml
  ├── build/
  ├── install/
  └── log/
```

## 2. Creating ROS2 Packages

### 2.1 Python Package

Create a Python package:

```sh
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_python_package
```

#### 2.1.1 Write Python Node

Create a Python node script in `my_python_package/my_python_package/my_python_node.py`:

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

#### 2.1.2 Update `setup.py`

Modify the `setup.py` in the root of `my_python_package`:

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

### 2.2 C++ Package

Create a C++ package:

```sh
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake my_cpp_package
```

#### 2.2.1 Write C++ Node

Create a C++ node script in `my_cpp_package/src/my_cpp_node.cpp`:

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

#### 2.2.2 Update `CMakeLists.txt`

Modify the `CMakeLists.txt` in the root of `my_cpp_package`:

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

## 3. Building and Running ROS2 Packages

### 3.1 Build the Packages

Navigate to the root of your workspace and build the packages:

```sh
cd ~/ros2_ws
colcon build --packages-select my_python_package my_cpp_package
```

If you encounter issues with the packages not being recognized, try the following:

1. **Clean the workspace**:
    ```sh
    rm -rf build/ install/ log/
    ```

2. **Rebuild without specifying packages**:
    ```sh
    colcon build
    ```

3. **Source the environment again**:
    ```sh
    source install/setup.bash
    ```

### 3.2 Run the Nodes

Run your nodes using the ROS2 command-line tool:

```sh
ros2 run my_python_package my_python_node
ros2 run my_cpp_package my_cpp_node
```

## 4. Troubleshooting Common Issues

### 4.1 Package Not Found

If you see the error `Package 'my_python_package' not found` or `Package 'my_cpp_package' not found`, ensure:

- The package names are correctly specified in the commands and files.
- The environment is properly sourced:
    ```sh
    source /opt/ros/humble/setup.bash
    source ~/ros2_ws/install/setup.bash
    ```
- The `setup.py`, `package.xml`, and `CMakeLists.txt` files are correctly configured.

### 4.2 Sourcing Environment

Ensure your environment is correctly sourced by adding the following lines to your `.bashrc`:

```sh
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

### 4.3 Verifying Package Structure

Check that your package directories and files are in the correct locations and have the correct names. This includes ensuring that:

- `my_python_node.py` is inside the correct directory.
- The `setup.py` and `CMakeLists.txt` files correctly point to your scripts.

### 4.4 Rebuilding and Cleaning

If issues persist, try cleaning the build environment and performing a fresh build:

```sh
rm -rf build/ install/ log/
colcon build
source install/setup.bash
```

## 5. Additional ROS2 Command-Line Tools

- **List nodes**: `ros2 node list`
- **Inspect a node**: `ros2 node info <node_name>`
- **List topics**: `ros2 topic list`
- **List services**: `ros2 service list`
- **Create a new package**: `ros2 pkg create <package_name>`
