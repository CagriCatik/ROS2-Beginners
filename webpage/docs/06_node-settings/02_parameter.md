# ROS 2 Parameters 

## Introduction

In ROS 2 (Robot Operating System 2), parameters play a crucial role in configuring nodes without modifying the source code. This guide will provide an in-depth understanding of ROS 2 parameters, their purpose, and how to utilize them effectively in your projects.

## Understanding the Problem

Consider a camera driver node within a ROS 2 package. This node is responsible for interfacing with a camera, capturing images, and possibly processing them. In a typical scenario, the node might need various configuration settings, such as:

- The name of the USB device to which the camera is connected.
- Frame rate settings (e.g., 30 fps or 60 fps).
- Operating mode (e.g., simulation or real mode).

Hardcoding these settings into your code is not ideal. Each time you need to change a setting, you would have to modify the code and recompile it. This approach is inflexible and inefficient, especially when you want to run multiple instances of the node with different configurations.

## Introduction to ROS 2 Parameters

ROS 2 parameters provide a solution to this problem. They allow you to configure node settings dynamically at runtime without changing the source code. This flexibility makes it easier to manage and deploy nodes with different configurations.

## Declaring Parameters in ROS 2

Before using parameters, you need to declare them in your node. Let's explore this with an example of a camera driver node.

### Example: Camera Driver Node

```python
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

class CameraDriverNode(Node):
    def __init__(self):
        super().__init__('camera_driver')

        # Declare parameters
        self.declare_parameter('usb_device', '/dev/video0')
        self.declare_parameter('frame_rate', 30)
        self.declare_parameter('simulation_mode', False)

        # Retrieve parameter values
        self.usb_device = self.get_parameter('usb_device').get_parameter_value().string_value
        self.frame_rate = self.get_parameter('frame_rate').get_parameter_value().integer_value
        self.simulation_mode = self.get_parameter('simulation_mode').get_parameter_value().bool_value

        self.get_logger().info(f'USB Device: {self.usb_device}')
        self.get_logger().info(f'Frame Rate: {self.frame_rate}')
        self.get_logger().info(f'Simulation Mode: {self.simulation_mode}')

def main(args=None):
    rclpy.init(args=args)
    node = CameraDriverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

In this example, the `CameraDriverNode` class declares three parameters: `usb_device`, `frame_rate`, and `simulation_mode`. These parameters have default values which can be overridden at runtime.

## Setting Parameter Values at Runtime

To run the node with specific parameter values, you can use a YAML file or command-line arguments.

### Using a YAML File

Create a YAML file (e.g., `camera_params.yaml`) with the following content:

```yaml
camera_driver:
  ros__parameters:
    usb_device: '/dev/video1'
    frame_rate: 60
    simulation_mode: true
```

Launch the node with the YAML file:

```bash
ros2 run your_package camera_driver_node --ros-args --params-file camera_params.yaml
```

### Using Command-Line Arguments

You can also set parameter values directly from the command line:

```bash
ros2 run your_package camera_driver_node --ros-args -p usb_device:=/dev/video1 -p frame_rate:=60 -p simulation_mode:=true
```

## Recap

ROS 2 parameters provide a flexible and efficient way to configure nodes. They allow you to set configuration values at runtime, eliminating the need to modify and recompile code for different settings. Each parameter has a name and a data type, such as boolean, integer, double, string, or lists of these types.

## Summary

In summary, ROS 2 parameters are essential for dynamically configuring nodes. They enhance the flexibility and manageability of your ROS 2 applications by allowing you to set and modify node settings at runtime. This guide has covered the basics of declaring and using parameters in ROS 2, providing you with the tools to implement this powerful feature in your projects.