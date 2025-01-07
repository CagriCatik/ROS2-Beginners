#  Introduction

Managing multiple nodes in your ROS2 packages, each with unique configurations, can quickly become complex and cumbersome when done manually via the terminal. The solution to this scalability issue is the use of launch files. This tutorial provides a rigorous and detailed examination of ROS2 launch files, their creation, installation, and usage.

By the end of this tutorial, you will have the expertise to initiate all your nodes and set their parameters using a single ROS2 launch file.

## Objectives

- Understand the purpose and utility of launch files in ROS2.
- Learn how to create, install, and execute a launch file.
- Gain practical experience through hands-on activities.

## Understanding ROS2 Launch Files

### What are Launch Files?

Launch files in ROS2 are XML or Python scripts used to automate the process of starting multiple nodes, setting parameters, and remapping topics and services. They streamline the initialization process, ensuring that all configurations are set correctly and consistently.

### When to Use Launch Files

Launch files are particularly useful when:
- You need to start multiple nodes simultaneously.
- Nodes require specific parameters or configurations.
- Topics and services need to be remapped.
- You are working with complex systems that require a structured startup sequence.

## Creating a ROS2 Launch File

ROS2 supports launch files written in both XML and Python. This tutorial focuses on Python launch files due to their flexibility and ease of use.

### Step-by-Step Guide to Creating a Launch File

1. **Directory Structure**:
   Ensure your package has a `launch` directory. If it doesn't, create one:
   ```bash
   mkdir -p your_package_name/launch
   ```

2. **Creating the Launch File**:
   Create a new Python file in the `launch` directory, e.g., `my_launch_file.py`:
   ```python
   #!/usr/bin/env python3

   import launch
   from launch import LaunchDescription
   from launch_ros.actions import Node

   def generate_launch_description():
       return LaunchDescription([
           Node(
               package='your_package_name',
               executable='your_node_executable',
               name='custom_node_name',
               namespace='your_namespace',
               output='screen',
               parameters=[
                   {'param1': 'value1'},
                   {'param2': 'value2'}
               ],
               remappings=[
                   ('/original_topic', '/remapped_topic')
               ]
           ),
           # Add more nodes as needed
       ])
   ```

   - `package`: Name of the package containing the node.
   - `executable`: The executable name of the node.
   - `name`: Custom name for the node.
   - `namespace`: Namespace for the node.
   - `output`: Output mode, e.g., `screen` to print to terminal.
   - `parameters`: List of parameters to set for the node.
   - `remappings`: List of topic/service remappings.

3. **Make the Launch File Executable**:
   Ensure the launch file is executable:
   ```bash
   chmod +x your_package_name/launch/my_launch_file.py
   ```

## Installing and Running the Launch File

### Install the Launch File

Ensure your `setup.py` file includes the launch file in the `data_files` section:
```python
from setuptools import setup

package_name = 'your_package_name'

setup(
    # Other setup parameters
    data_files=[
        ('share/' + package_name + '/launch', ['launch/my_launch_file.py']),
    ],
    # Other setup parameters
)
```

### Running the Launch File

To run the launch file, use the `ros2 launch` command:
```bash
ros2 launch your_package_name my_launch_file.py
```

## Practical Activity

Create a ROS2 package named `my_robot_bringup` and write a launch file to start two nodes:
1. `robot_state_publisher` from the `robot_state_publisher` package.
2. `joint_state_publisher_gui` from the `joint_state_publisher_gui` package.

### Solution

1. **Create the Package**:
   ```bash
   ros2 pkg create my_robot_bringup --build-type ament_python
   ```

2. **Create the Launch File**:
   ```python
   # my_robot_bringup/launch/bringup_launch.py

   import launch
   from launch import LaunchDescription
   from launch_ros.actions import Node

   def generate_launch_description():
       return LaunchDescription([
           Node(
               package='robot_state_publisher',
               executable='robot_state_publisher',
               output='screen',
               parameters=[
                   {'robot_description': 'path/to/your/robot_description.urdf'}
               ]
           ),
           Node(
               package='joint_state_publisher_gui',
               executable='joint_state_publisher_gui',
               output='screen'
           )
       ])
   ```

3. **Make the Launch File Executable**:
   ```bash
   chmod +x my_robot_bringup/launch/bringup_launch.py
   ```

4. **Update `setup.py`**:
   ```python
   from setuptools import setup

   package_name = 'my_robot_bringup'

   setup(
       # Other setup parameters
       data_files=[
           ('share/' + package_name + '/launch', ['launch/bringup_launch.py']),
       ],
       # Other setup parameters
   )
   ```

5. **Build and Source**:
   ```bash
   colcon build
   source install/setup.bash
   ```

6. **Run the Launch File**:
   ```bash
   ros2 launch my_robot_bringup bringup_launch.py
   ```

## Conclusion

Using ROS2 launch files significantly improves the management and initialization of multiple nodes with varied configurations. This tutorial has provided a detailed guide to understanding, creating, installing, and running launch files, equipping you with the skills to streamline your ROS2 development process. Practice creating and using launch files to enhance your proficiency in managing complex ROS2 projects.