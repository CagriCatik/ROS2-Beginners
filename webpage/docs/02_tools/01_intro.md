# ROS2 Tools

## Overview

This tutorial covers some of the essential tools within the ROS2 (Robot Operating System 2) ecosystem, which are indispensable for developing, managing, and debugging ROS2 applications. By the end of this tutorial, you will have a comprehensive understanding of how to run ROS2 nodes, use the ROS2 Command Line Interface (CLI) for debugging, visualize the ROS2 computational graph using `rqt_graph`, and explore basic simulation using Turtlesim. 

## Objectives

Upon completion of this tutorial, you will have the proficiency to:

1. Execute ROS2 nodes with various options.
2. Debug nodes using ROS2 CLI tools.
3. Visualize the computational graph of your ROS2 application using `rqt_graph`.
4. Utilize Turtlesim, a basic 2D simulation tool, to understand ROS2 concepts.

## Running Nodes with `ros2 run`

The `ros2 run` command is fundamental for launching nodes in ROS2. It is versatile, allowing you to execute nodes with different configurations, including changing the node's name dynamically. Below is a detailed guide on how to use `ros2 run` effectively.

### Basic Node Execution

To run a node in ROS2, use the following command:

```bash
ros2 run <package_name> <node_executable>
```

**Example:**

```bash
ros2 run demo_nodes_cpp talker
```

This command launches the `talker` node from the `demo_nodes_cpp` package. This node is a basic publisher that sends messages to a topic.

### Running with a Custom Node Name

In ROS2, you can change the name of a node dynamically using the `--ros-args` flag followed by the `-r` option:

```bash
ros2 run <package_name> <node_executable> --ros-args -r __node:=custom_node_name
```

**Example:**

```bash
ros2 run demo_nodes_cpp talker --ros-args -r __node:=my_custom_talker
```

This command runs the `talker` node but renames it to `my_custom_talker`.

### Technical Notes

- The `ros2 run` command locates the executable for the specified node within the specified package. 
- The `--ros-args` flag allows additional ROS-specific arguments, such as remapping names, setting parameters, or changing QoS settings, to be passed to the node at runtime.

## Debugging Nodes with ROS2 CLI

The ROS2 CLI provides a suite of commands to interact with and debug nodes. These tools are essential for inspecting the state and behavior of your ROS2 nodes during development.

### Listing Active Nodes

To list all currently running nodes, use:

```bash
ros2 node list
```

This command will display a list of node names currently active in the ROS2 system.

### Getting Node Information

To get detailed information about a specific node, such as its publishers, subscribers, services, and parameters, use:

```bash
ros2 node info <node_name>
```

**Example:**

```bash
ros2 node info /my_custom_talker
```

This command will provide detailed information about the `my_custom_talker` node, including its publishers, subscribers, services, and parameters.

### Introspecting Topics

Topic introspection is critical for understanding the flow of messages between nodes. Here are some essential commands:

- **Listing Topics:**

  ```bash
  ros2 topic list
  ```

  This command lists all active topics.

- **Displaying Topic Information:**

  ```bash
  ros2 topic info <topic_name>
  ```

  **Example:**

  ```bash
  ros2 topic info /chatter
  ```

  This command provides details about the `/chatter` topic, including the type of messages being published and the number of publishers and subscribers.

- **Echoing Topic Data:**

  ```bash
  ros2 topic echo <topic_name>
  ```

  **Example:**

  ```bash
  ros2 topic echo /chatter
  ```

  This command prints the messages being published on the `/chatter` topic to the console, useful for monitoring live data.

### Service Introspection

Services in ROS2 are used for synchronous communication between nodes. The following commands help in inspecting active services:

- **Listing Services:**

  ```bash
  ros2 service list
  ```

  This command lists all active services in the ROS2 system.

- **Displaying Service Information:**

  ```bash
  ros2 service info <service_name>
  ```

  **Example:**

  ```bash
  ros2 service info /spawn
  ```

  This command displays information about the `/spawn` service, including its request and response types.

## Visualizing the ROS2 Graph with `rqt_graph`

The `rqt_graph` tool is invaluable for visualizing the computational graph of your ROS2 system. It provides a graphical representation of nodes, topics, and their interconnections, making it easier to understand and debug complex systems.

### Launching `rqt_graph`

To open `rqt_graph`, use the following command:

```bash
ros2 run rqt_graph rqt_graph
```

This will launch the `rqt_graph` interface, displaying the nodes and their connections in the system.

### Interpreting the Graph

- **Nodes** are represented as rectangles in the graph.
- **Topics** are shown as ellipses.
- **Connections** between nodes and topics represent the flow of messages, with arrows indicating the direction of communication.

### Technical Notes

- `rqt_graph` relies on the underlying ROS2 graph information, which is dynamically updated as nodes are started and stopped.
- This tool is especially useful in complex systems with numerous nodes and topics, allowing you to identify misconfigurations or unintended connections quickly.

## Exploring Turtlesim

Turtlesim is a basic 2D simulation tool included with ROS2, often used for learning and demonstration purposes. It provides a simple environment where you can visualize and interact with basic ROS2 concepts.

### Launching Turtlesim

To start the Turtlesim simulation, use:

```bash
ros2 run turtlesim turtlesim_node
```

This command launches the Turtlesim node, which opens a graphical window displaying the turtle.

### Controlling Turtlesim

To control the turtle using your keyboard, start the teleoperation node:

```bash
ros2 run turtlesim turtle_teleop_key
```

This command allows you to move the turtle around the screen using the arrow keys.

## Practical Activity

### Creating a New ROS2 Package

As a hands-on exercise, you will create a new ROS2 package, write simple publisher and subscriber nodes, and use the tools discussed above to debug and visualize your application.

#### Step 1: Creating a Package

Create a new ROS2 package using the following command:

```bash
ros2 pkg create --build-type ament_cmake my_package
```

This command generates a new package named `my_package` with the CMake build system.

#### Step 2: Writing the Publisher Node (Python)

Create a file named `publisher_node.py` in the `my_package/my_package` directory with the following content:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello, world!'
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Step 3: Writing the Subscriber Node (Python)

Create a file named `subscriber_node.py` in the `my_package/my_package` directory with the following content:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Step 4: Building and Running the Package

- **Build the package:**

  ```bash
  cd ~/ros2_ws
  colcon build
  ```

- **Source the setup file:**

  ```bash
  source install/setup.bash
  ```

- **Run the publisher and subscriber:**

  ```bash
  ros2 run my_package publisher_node
  ros2 run my_package subscriber_node
  ```

#### Step 5: Using the Tools

- **List and get information about the nodes:**

  ```bash
  ros2 node list
  ros2 node info /minimal_publisher
  ros2 node info /minimal_subscriber
  ```

- **Visualize the communication graph:**

  ```bash
  ros2 run rqt_graph rqt_graph
  ```

This will show how

 the `minimal_publisher` and `minimal_subscriber` nodes communicate via the `topic` topic.

## Conclusion

By mastering these ROS2 tools, you are now equipped to develop, debug, and maintain robust ROS2 applications. Understanding how to run nodes with various options, use the ROS2 CLI for debugging, visualize node interactions, and experiment with Turtlesim will enable you to tackle more complex ROS2 projects with confidence.