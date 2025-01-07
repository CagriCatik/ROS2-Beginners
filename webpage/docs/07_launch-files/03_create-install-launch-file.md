# Creating and Using a Launch File

Launch files in ROS2 allow you to start multiple nodes and configure their parameters from a single file. This tutorial will guide you through creating a launch file for a ROS2 application consisting of two nodes: a number publisher node and a number counter node. We will also set up a dedicated package for launch files, following best practices for organization and dependency management.

## Step 1: Create a New Package for Launch Files

First, navigate to the source directory of your ROS2 workspace. For demonstration purposes, let's assume we have three packages already: one for Python nodes, one for C++ nodes, and an interface package. We'll create a new package specifically for launch files.

```bash
ros2 pkg create my_robot_bringup
```

Creating a separate package for launch files helps centralize them, making it easier to manage dependencies and configurations.

## Step 2: Configure the Package

Navigate into the newly created package and set up the necessary directories and files:

```bash
cd my_robot_bringup
mkdir launch
```

Modify the `CMakeLists.txt` and `package.xml` files to install the launch directory and declare dependencies.

**CMakeLists.txt**:

```cmake
cmake_minimum_required(VERSION 3.5)
project(my_robot_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
```

**package.xml**:

```xml
<?xml version="1.0"?>
<package format="2">
  <name>my_robot_bringup</name>
  <version>0.0.1</version>
  <description>Launch files for my robot</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <exec_depend>my_python_package</exec_depend>
  <exec_depend>my_cpp_package</exec_depend>

  <buildtool_depend>ament_cmake</buildtool_depend>
</package>
```

## Step 3: Create the Launch File

Create a new launch file in the `launch` directory. The file will be named `number_app.launch.py` and will start both the number publisher and number counter nodes.

```bash
touch launch/number_app.launch.py
chmod +x launch/number_app.launch.py
```

Edit the `number_app.launch.py` file:

```python
#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    number_publisher_node = Node(
        package='my_python_package',
        executable='number_publisher',
        name='number_publisher_node'
    )

    number_counter_node = Node(
        package='my_cpp_package',
        executable='number_counter',
        name='number_counter_node'
    )

    return LaunchDescription([
        number_publisher_node,
        number_counter_node
    ])
```

This script defines a function `generate_launch_description` that returns a `LaunchDescription` object containing the nodes to be launched.

## Step 4: Build and Source the Package

Build the new package and source the workspace:

```bash
cd ~/your_ros2_workspace
colcon build --packages-select my_robot_bringup --symlink-install
source install/setup.bash
```

## Step 5: Launch the Nodes

To launch the nodes using the new launch file, use the following command:

```bash
ros2 launch my_robot_bringup number_app.launch.py
```

This command will start both the number publisher and number counter nodes. You can verify that the nodes are running by listing the active nodes:

```bash
ros2 node list
```

You should see both `number_publisher_node` and `number_counter_node` in the list of active nodes.

## Conclusion

By following this tutorial, you have created a dedicated package for launch files, configured it correctly, and written a launch file to start multiple nodes. This approach not only organizes your project better but also simplifies managing dependencies and configurations across different parts of your application.