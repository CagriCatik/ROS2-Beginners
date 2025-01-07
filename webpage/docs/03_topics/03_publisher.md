### ROS 2 Publisher Node Tutorial with Python

This tutorial provides a comprehensive guide to creating a publisher node in ROS 2 using Python. The goal is to set up a node that publishes messages to a topic, which other nodes can then subscribe to. This tutorial will cover setting up the environment, writing the publisher node, and verifying its functionality.

#### 1. Setting Up the ROS 2 Workspace

First, ensure your ROS 2 workspace is properly set up. Navigate to your ROS 2 workspace source directory:

```bash
cd ~/ros2_ws/src
```

#### 2. Create a Python Package

If you haven't already created a Python package, do so now:

```bash
ros2 pkg create --build-type ament_python my_python_package
```

#### 3. Creating a Publisher Node

Within the `my_python_package` directory, create a new Python file for your publisher node:

```bash
cd my_python_package/my_python_package
touch robot_news_station.py
chmod +x robot_news_station.py
```

#### 4. Writing the Publisher Node

Open `robot_news_station.py` in your preferred text editor and add the following code:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

class RobotNewsStation(Node):
    def __init__(self):
        super().__init__('robot_news_station')
        self.publisher_ = self.create_publisher(String, 'robot_news', 10)
        self.timer = self.create_timer(0.5, self.publish_news)
        self.get_logger().info('Robot News Station has been started.')
        self.robot_name = 'C-3PO'

    def publish_news(self):
        msg = String()
        msg.data = f'Hi, this is {self.robot_name} from the Robot News Station'
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = RobotNewsStation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### 5. Package Configuration

Ensure the necessary dependencies are declared in the `setup.py` and `package.xml` files.

**`setup.py`:**

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
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_news_station = my_python_package.robot_news_station:main',
        ],
    },
)
```

**`package.xml`:**

Add the dependency for `example_interfaces`:

```xml
<package format="2">
  <name>my_python_package</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="your_email@example.com">your_name</maintainer>
  <license>TODO: License declaration</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <build_depend>rclpy</build_depend>
  <build_depend>example_interfaces</build_depend>
  <exec_depend>rclpy</exec_depend>
  <exec_depend>example_interfaces</exec_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

#### 6. Building the Package

Navigate to the root of your workspace and build the package:

```bash
cd ~/ros2_ws
colcon build --packages-select my_python_package
```

#### 7. Running the Publisher Node

Source the workspace and run the node:

```bash
. install/setup.bash
ros2 run my_python_package robot_news_station
```

#### 8. Verifying the Publisher Node

To verify that your node is publishing messages correctly, open a new terminal and use the following commands:

**List active nodes:**

```bash
ros2 node list
```

You should see `robot_news_station` in the list.

**List active topics:**

```bash
ros2 topic list
```

You should see `/robot_news` in the list.

**Echo topic messages:**

```bash
ros2 topic echo /robot_news
```

You should see the messages being published by your node, such as:

```
data: 'Hi, this is C-3PO from the Robot News Station'
```

#### Conclusion

You have successfully created and run a ROS 2 publisher node using Python. This node publishes messages to the `/robot_news` topic, which other nodes can subscribe to for receiving updates. This foundational knowledge is essential for building more complex ROS 2 systems.
