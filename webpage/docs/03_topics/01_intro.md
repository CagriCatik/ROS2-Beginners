# ROS 2 Topics

## Introduction

This tutorial provides an in-depth guide to ROS 2 topics, a fundamental communication mechanism in the ROS (Robot Operating System) framework. We will cover the conceptual foundations of ROS 2 topics, provide detailed instructions for creating publisher and subscriber nodes in both Python and C++, and introduce tools and techniques for debugging and monitoring topic communication. The tutorial concludes with a practical activity to consolidate your understanding.

## Conceptual Overview

In ROS 2, topics are the primary method for nodes (the basic unit of computation in ROS 2) to communicate with each other in a distributed system. A topic in ROS 2 is a named bus over which nodes exchange messages. This communication follows a publish/subscribe model:

- **Publisher**: A node that sends messages to a topic.
- **Subscriber**: A node that receives messages from a topic.

This architecture decouples the communication between nodes, allowing for modularity and scalability in designing robotic systems. Multiple nodes can publish to and subscribe to the same topic, and nodes do not need to be aware of each other's existence, as long as they know the topic name.

### Real-Life Analogy

Consider a radio station as an analogy. The radio station (publisher) broadcasts on a specific frequency (topic). Anyone with a radio (subscriber) can tune in to that frequency and receive the broadcast. The radio station does not need to know who is listening, and the listeners do not need to know who is broadcasting, as long as they both agree on the frequency.

## Implementing ROS 2 Topics

### Python Implementation

#### Publisher Node

The following is a Python implementation of a minimal publisher node. This node publishes a message of type `std_msgs/msg/String` to a topic named `topic` every 0.5 seconds.

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
        msg.data = 'Hello, ROS 2!'
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Explanation**:
- `MinimalPublisher` is a class derived from `rclpy.node.Node`. It defines a ROS 2 node with a publisher that sends a `String` message to the `topic` every 0.5 seconds.
- The `create_publisher` function creates the publisher, specifying the message type (`String`) and the topic name (`topic`). The third argument, `10`, is the queue size, which defines how many messages can be queued for sending.

#### Subscriber Node

The corresponding subscriber node listens to the `topic` and logs the received messages.

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

**Explanation**:
- `MinimalSubscriber` is another ROS 2 node class that subscribes to the `topic`. It uses the `create_subscription` function to define the subscriber, specifying the message type, topic name, and a callback function (`listener_callback`) that is invoked each time a message is received.

### C++ Implementation

#### Publisher Node

The following C++ code implements a minimal publisher node that sends a string message to a topic every 500 milliseconds.

```cpp
#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node {
public:
    MinimalPublisher() : Node("minimal_publisher"), count_(0) {
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        timer_ = this->create_wall_timer(500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

private:
    void timer_callback() {
        auto message = std_msgs::msg::String();
        message.data = "Hello, ROS 2: " + std::to_string(count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}
```

**Explanation**:
- `MinimalPublisher` is a class derived from `rclcpp::Node`. It defines a ROS 2 node with a publisher that sends a `String` message to the `topic` every 500 milliseconds.
- The `create_publisher` function initializes the publisher. The `create_wall_timer` function creates a timer that calls the `timer_callback` function at a fixed interval.

#### Subscriber Node

The corresponding subscriber node in C++ listens to the `topic` and logs the received messages.

```cpp
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MinimalSubscriber : public rclcpp::Node {
public:
    MinimalSubscriber() : Node("minimal_subscriber") {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1));
    }

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();
    return 0;
}
```

**Explanation**:
- `MinimalSubscriber` is a C++ ROS 2 node class that subscribes to the `topic`. The `create_subscription` function is used to define the subscriber, specifying the message type, topic name, and a callback function (`topic_callback`) that is invoked each time a message is received.

## Debugging and Monitoring ROS 2 Topics

ROS 2 provides several command-line tools and graphical interfaces for debugging and monitoring topics.

### Command-Line Tools

1. **Listing Active Topics**:
   To list all active topics, use the following command:
   ```bash
   ros2 topic list
   ```

2. **Displaying Topic Information**:
   To display detailed information about a specific topic, use:
   ```bash
   ros2 topic info /topic
   ```

3. **Echoing Topic Messages**:
   To view the messages being published on a specific topic, use:
   ```bash
   ros2 topic echo /topic
   ```

### Graphical Tools

1. **Visualizing Data with RQT**:
   RQT is a graphical tool in ROS 2 that allows you to visualize data from topics in real-time. Launch RQT with:
   ```bash
   rqt
   ```

   Within RQT, you can add various plugins to visualize data, plot graphs, and monitor different aspects of your ROS 2 system.

## Experimenting with Turtlesim

Turtlesim is a popular ROS 2 package used to teach the basic concepts of ROS, including topics. It simulates a turtle that can be controlled by publishing messages to specific topics.

### Installing Turtlesim

Before using Turtlesim, you need to install the package:

```bash
sudo apt update
sudo apt install ros-foxy-turtlesim
```

### Launching Turtlesim

Launch the Turtlesim simulation:

```bash
ros2 run turtlesim turtlesim_node
```

### Controlling Turtlesim via Topics

You can control the turtle by publishing velocity commands to the `/turtle1/cmd_vel` topic:

```bash
ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
```

This command moves the turtle forward while rotating it.

## Practical Activity: Average Number Calculator

### Objective

Create a ROS 2 system where one node publishes randomly generated numbers, and another

 node subscribes to these numbers and calculates their average.

### Python Implementation

#### Publisher Node

The publisher node generates random floating-point numbers between 0 and 100 and publishes them to a topic named `numbers`.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import random

class NumberPublisher(Node):
    def __init__(self):
        super().__init__('number_publisher')
        self.publisher_ = self.create_publisher(Float64, 'numbers', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        num = Float64()
        num.data = random.uniform(0, 100)
        self.publisher_.publish(num)
        self.get_logger().info('Publishing: "%f"' % num.data)

def main(args=None):
    rclpy.init(args=args)
    number_publisher = NumberPublisher()
    rclpy.spin(number_publisher)
    number_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Subscriber Node

The subscriber node listens to the `numbers` topic, calculates the running average of the numbers received, and logs the current number along with the average.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class NumberSubscriber(Node):
    def __init__(self):
        super().__init__('number_subscriber')
        self.subscription = self.create_subscription(
            Float64,
            'numbers',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.numbers = []

    def listener_callback(self, msg):
        self.numbers.append(msg.data)
        avg = sum(self.numbers) / len(self.numbers)
        self.get_logger().info('I heard: "%f" - Average: "%f"' % (msg.data, avg))

def main(args=None):
    rclpy.init(args=args)
    number_subscriber = NumberSubscriber()
    rclpy.spin(number_subscriber)
    number_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Running the Activity

1. Run the publisher node:
   ```bash
   python3 number_publisher.py
   ```
2. Run the subscriber node:
   ```bash
   python3 number_subscriber.py
   ```

Watch the output in the terminal to see the published numbers and their running average.

## Conclusion

This tutorial has provided a comprehensive introduction to ROS 2 topics, covering both the theoretical concepts and practical implementation. You have learned how to create publisher and subscriber nodes in Python and C++, monitor and debug topics using ROS 2 tools, and interact with a simulated environment using Turtlesim. The practical activity has reinforced your understanding of inter-node communication in ROS 2, demonstrating how topics can be used to share and process data across a distributed system.