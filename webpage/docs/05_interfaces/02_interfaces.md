# ROS 2 Interfaces

In this guide, we will delve into ROS 2 interfaces, exploring their roles in node communication, and providing detailed steps to create and use custom message and service definitions. We will also critically analyze the existing explanation and rectify any inaccuracies.

## Understanding ROS 2 Interfaces

In ROS 2, interfaces are essential for communication between nodes. These interfaces include:

1. **Messages**: Used in topics for unidirectional communication.
2. **Services**: Used for bidirectional communication with a request-response pattern.

## Topics

Topics are characterized by:
- **Name**: A unique identifier for the topic.
- **Interface (Message Definition)**: The data structure defining the type of message being sent.

## Services

Services are characterized by:
- **Name**: A unique identifier for the service.
- **Interface (Service Definition)**: Comprises two message definitions—one for the request and one for the response.

## Real-Life Analogy

Consider sending a letter via mail:
- **Topics**: Analogous to sending a letter where the content (message) is delivered by the postal service (transport layer).
- **Services**: Analogous to sending a letter and receiving a reply. The initial letter (request) prompts a response letter (response).

## Technical Implementation

When creating a custom message or service definition in ROS 2, the process involves defining the message in a specific format, building it, and using the generated source code in your application.

## Creating Custom Messages and Services

## Step 1: Create a ROS 2 Package

First, create a new ROS 2 package where you will define your custom messages and services.

```bash
ros2 pkg create --build-type ament_cmake my_custom_interfaces
```

## Step 2: Define Message and Service Files

Inside your package, create the necessary directories and files.

```bash
cd my_custom_interfaces
mkdir msg srv
```

Create a message file `msg/MyCustomMessage.msg`:
```
string name
int32 age
bool is_student
```

Create a service file `srv/MyCustomService.srv`:
```
# Request
string name
int32 age

---
# Response
bool success
string message
```

## Step 3: Modify the `CMakeLists.txt`

Edit the `CMakeLists.txt` file to include your message and service definitions.

```cmake
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MyCustomMessage.msg"
  "srv/MyCustomService.srv"
)
```

## Step 4: Modify the `package.xml`

Ensure the `package.xml` includes the necessary dependencies.

```xml
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
```

## Step 5: Build the Package

Build your package to generate the necessary source files.

```bash
colcon build --packages-select my_custom_interfaces
```

## Step 6: Using Custom Messages and Services

Now, you can use your custom messages and services in your ROS 2 nodes.

**Python Example:**

```python
import rclpy
from rclpy.node import Node
from my_custom_interfaces.msg import MyCustomMessage
from my_custom_interfaces.srv import MyCustomService

class CustomNode(Node):

    def __init__(self):
        super().__init__('custom_node')
        self.publisher_ = self.create_publisher(MyCustomMessage, 'custom_topic', 10)
        self.timer_ = self.create_timer(2.0, self.publish_message)
        self.client_ = self.create_client(MyCustomService, 'custom_service')
        while not self.client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

    def publish_message(self):
        msg = MyCustomMessage()
        msg.name = 'John Doe'
        msg.age = 30
        msg.is_student = False
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.name)

    def call_service(self):
        req = MyCustomService.Request()
        req.name = 'Jane Doe'
        req.age = 25
        future = self.client_.call_async(req)
        future.add_done_callback(self.service_callback)

    def service_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info('Response: "%s"' % response.message)
        except Exception as e:
            self.get_logger().error('Service call failed %r' % (e,))

def main(args=None):
    rclpy.init(args=args)
    node = CustomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped cleanly')
    except BaseException:
        node.get_logger().error('Exception in node: %s' % (traceback.format_exc(),))
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**C++ Example:**

```cpp
#include "rclcpp/rclcpp.hpp"
#include "my_custom_interfaces/msg/my_custom_message.hpp"
#include "my_custom_interfaces/srv/my_custom_service.hpp"

class CustomNode : public rclcpp::Node
{
public:
    CustomNode() : Node("custom_node")
    {
        publisher_ = this->create_publisher<my_custom_interfaces::msg::MyCustomMessage>("custom_topic", 10);
        timer_ = this->create_wall_timer(2000ms, std::bind(&CustomNode::publish_message, this));
        client_ = this->create_client<my_custom_interfaces::srv::MyCustomService>("custom_service");

        while (!client_->wait_for_service(1s))
        {
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting...");
        }
    }

private:
    void publish_message()
    {
        auto msg = my_custom_interfaces::msg::MyCustomMessage();
        msg.name = "John Doe";
        msg.age = 30;
        msg.is_student = false;
        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg.name.c_str());
    }

    void call_service()
    {
        auto request = std::make_shared<my_custom_interfaces::srv::MyCustomService::Request>();
        request->name = "Jane Doe";
        request->age = 25;
        auto result = client_->async_send_request(request, std::bind(&CustomNode::service_callback, this, std::placeholders::_1));
    }

    void service_callback(rclcpp::Client<my_custom_interfaces::srv::MyCustomService>::SharedFuture future)
    {
        auto response = future.get();
        RCLCPP_INFO(this->get_logger(), "Response: '%s'", response->message.c_str());
    }

    rclcpp::Publisher<my_custom_interfaces::msg::MyCustomMessage>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<my_custom_interfaces::srv::MyCustomService>::SharedPtr client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CustomNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

## Conclusion

This guide has provided a detailed overview of ROS 2 interfaces, covering their significance, technical implementation, and practical examples. By following these steps, you can create custom message and service definitions for your ROS 2 applications, enhancing the communication capabilities of your robotic systems. Always refer to the official ROS 2 documentation and community resources for additional support and updates.