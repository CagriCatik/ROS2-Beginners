### Creating a ROS2 Python Subscriber: A Comprehensive Guide

This tutorial provides a detailed walkthrough for creating a Python subscriber node in ROS2, designed to interact with a previously created radio station topic. The node, referred to as the smartphone node, will be equipped with various functionalities, including reading and publishing the battery state, sending text messages, and adjusting speaker volume. This tutorial focuses on the subscriber functionality of the smartphone node.

#### Prerequisites

Before proceeding, ensure you have the following:

- A functioning ROS2 installation
- A ROS2 workspace set up with a Python package where nodes are written
- The necessary dependencies installed

#### Step-by-Step Instructions

##### 1. Setting Up the Workspace

Navigate to the source folder of your ROS2 workspace and into your Python package directory.

```bash
cd ~/ros2_ws/src/my_python_package
```

##### 2. Creating the Subscriber Node

Create a new Python file for the smartphone node and make it executable.

```bash
touch smartphone.py
chmod +x smartphone.py
```

##### 3. Writing the Node Script

Open the `smartphone.py` file in your preferred text editor and start by writing the basic structure of a ROS2 node.

```python
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

class SmartphoneNode(Node):
    def __init__(self):
        super().__init__('smartphone')
        self.get_logger().info('Smartphone node has been started.')
      
        # Creating the subscriber
        self.subscriber = self.create_subscription(
            String,
            'robot_news',
            self.callback_robot_news,
            10
        )

    def callback_robot_news(self, msg):
        self.get_logger().info(f'Received message: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = SmartphoneNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

This script initializes the smartphone node, sets up a subscriber to listen to the `robot_news` topic, and defines a callback function to handle incoming messages.

##### 4. Modifying `setup.py`

Ensure your `setup.py` includes the new node. Modify the entry points to include `smartphone`.

```python
entry_points={
    'console_scripts': [
        'smartphone = my_python_package.smartphone:main',
    ],
},
```

##### 5. Building the Workspace

Build your workspace to integrate the new node.

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

##### 6. Running the Nodes

First, start the publisher node (robot news station) to ensure it is broadcasting messages.

```bash
ros2 run my_python_package robot_news_station
```

In a new terminal, source your workspace, and then run the smartphone node.

```bash
ros2 run my_python_package smartphone
```

You should see log messages in the terminal indicating that the smartphone node has started and is receiving messages from the `robot_news` topic.

##### 7. Testing and Verification

Test the setup by observing the log messages from the smartphone node. You should see messages indicating that the node is receiving the broadcasts from the `robot_news` station.

If you stop the publisher node, the subscriber node should remain active but will not receive any messages until the publisher node is restarted. This demonstrates the decoupled nature of ROS2 communication, where nodes interact through topic interfaces without being directly aware of each other.

#### Conclusion

This tutorial provided a comprehensive guide for creating a Python subscriber node in ROS2. By following these steps, you should be able to set up a functional subscriber node capable of receiving messages from a specified topic. This node can serve as a foundation for more complex functionalities, such as those typically handled by a smartphone in a robotic system.
