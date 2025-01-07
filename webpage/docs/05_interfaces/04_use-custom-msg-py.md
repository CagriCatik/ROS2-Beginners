# Creating and Using Custom Messages in ROS2 with Python

In this tutorial, we will cover the steps required to create a custom message in ROS2 and use it within a Python node. This will include setting up the custom message, writing the Python publisher node, and configuring the development environment to recognize the new message type. 

## Prerequisites

Before starting, ensure you have the following:
- A working ROS2 installation
- A workspace set up for ROS2
- Basic familiarity with ROS2 concepts such as nodes, topics, and messages
- Basic knowledge of Python

## Step 1: Creating the Custom Message

First, create a new package for your custom messages if you haven't already. Navigate to your ROS2 workspace:

```sh
cd ~/ros2_ws/src
```

Create a new package for your custom interfaces:

```sh
ros2 pkg create --build-type ament_cmake my_robot_interfaces
```

Inside this package, create a `msg` directory and add your custom message file, `HardwareStatus.msg`:

```sh
mkdir -p my_robot_interfaces/msg
```

Create the `HardwareStatus.msg` file with the following content:

```
int32 temperature
bool motor_ready
string debug_message
```

Update the `CMakeLists.txt` file in your `my_robot_interfaces` package to include the new message:

```cmake
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(my_robot_interfaces
  "msg/HardwareStatus.msg"
)
```

Also, update the `package.xml` file to include dependencies on `rosidl_default_generators`:

```xml
<buildtool_depend>ament_cmake</buildtool_depend>
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
```

Now, build your package:

```sh
cd ~/ros2_ws
colcon build --packages-select my_robot_interfaces
source install/setup.bash
```

## Step 2: Creating the Python Node

Create a new Python package if you haven't already:

```sh
ros2 pkg create --build-type ament_python my_python_package
```

Navigate to your package and create a directory for your nodes:

```sh
cd my_python_package
mkdir -p my_python_package
touch my_python_package/hardware_status_publisher.py
```

Edit `setup.py` to include the new script:

```python
entry_points={
    'console_scripts': [
        'hardware_status_publisher = my_python_package.hardware_status_publisher:main'
    ],
},
```

In the `my_python_package` directory, create the `hardware_status_publisher.py` file with the following content:

```python
import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import HardwareStatus

class HardwareStatusPublisher(Node):

    def __init__(self):
        super().__init__('hardware_status_publisher')
        self.publisher_ = self.create_publisher(HardwareStatus, 'hardware_status', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.publish_status)
        self.get_logger().info('Hardware Status Publisher Node has been started.')

    def publish_status(self):
        msg = HardwareStatus()
        msg.temperature = 45
        msg.motor_ready = True
        msg.debug_message = 'All systems operational'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg}')

def main(args=None):
    rclpy.init(args=args)
    node = HardwareStatusPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Step 3: Configuring the Environment

Make sure the `my_robot_interfaces` package is added as a dependency in the `package.xml` of your Python package:

```xml
<exec_depend>my_robot_interfaces</exec_depend>
```

Rebuild your workspace to ensure all dependencies are met:

```sh
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## Step 4: Running the Publisher Node

You can now run the publisher node:

```sh
ros2 run my_python_package hardware_status_publisher
```

Verify the node and topic:

```sh
ros2 node list
ros2 topic list
```

Check the topic information:

```sh
ros2 topic info /hardware_status
```

To see the published messages:

```sh
ros2 topic echo /hardware_status
```

## Common Issues

### Import Errors in VS Code

If you encounter import errors in VS Code, ensure the Python interpreter is aware of your custom messages by adding the appropriate path in VS Code settings:

1. Open VS Code settings (`Ctrl + ,`).
2. Search for `Python: Auto Complete Extra Paths`.
3. Add the path to your ROS2 message packages, e.g., `/path/to/ros2_ws/install/my_robot_interfaces/lib/python3.8/site-packages`.

### Source Your Workspace

Always ensure your workspace is sourced correctly:

```sh
source ~/ros2_ws/install/setup.bash
```

This ensures that your custom messages and nodes are recognized by the ROS2 environment.

By following these steps, you have created a custom message, written a Python node to publish this message, and configured your environment to recognize and use this custom message. This process can be applied to any custom message type you wish to create and use within your ROS2 applications.