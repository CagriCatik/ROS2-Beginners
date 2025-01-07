# Services

## Introduction

ROS2 (Robot Operating System 2) provides essential communication mechanisms that facilitate interaction between various nodes in a robotic system. The two primary communication features in ROS2 are Topics and Services. Topics are utilized for continuous data streams, while Services enable client/server interactions, allowing nodes to request and provide specific services.

This tutorial focuses on ROS2 Services. By the end, you will be proficient in implementing and utilizing ROS2 Services for node communication. This will be achieved through the following steps:

1. Understanding ROS2 Services and their appropriate use cases.
2. Writing and implementing a Service (both client and server) in Python and C++.
3. Debugging Services using terminal commands.
4. Completing a practical activity to reinforce the concepts learned.

## Understanding ROS2 Services

### What are ROS2 Services?

ROS2 Services provide a synchronous communication mechanism, enabling nodes to send a request and wait for a response. This is analogous to a function call in programming, where a client node requests a service, and a server node processes this request and returns a response.

### When to Use ROS2 Services

Services are ideal for operations that require a response before proceeding. Examples include requesting the status of a sensor, commanding an actuator to move to a specific position, or fetching specific data from a node. Unlike Topics, which are used for continuous data streaming, Services are suitable for discrete transactions where an immediate response is necessary.

### Real-Life Analogy

Consider a restaurant scenario:

- **Topic:** A waiter continuously updates the chef with the current orders and specials in a stream of information.
- **Service:** A customer requests the bill from the waiter, who then provides the bill after processing the request.

## Writing Your Own Service

### Python Implementation

#### Server Node

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddTwoIntsServer(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        self.get_logger().info('Sending back response: %d' % (response.sum))
        return response

def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Client Node

```python
import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self):
        self.req.a = int(sys.argv[1])
        self.req.b = int(sys.argv[2])
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    client = AddTwoIntsClient()
    response = client.send_request()
    client.get_logger().info('Result of add_two_ints: %d' % response.sum)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### C++ Implementation

#### Server Node

```cpp
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using AddTwoInts = example_interfaces::srv::AddTwoInts;
using namespace std::placeholders;

class AddTwoIntsServer : public rclcpp::Node
{
public:
    AddTwoIntsServer() : Node("add_two_ints_server")
    {
        service_ = create_service<AddTwoInts>("add_two_ints", std::bind(&AddTwoIntsServer::handle_service, this, _1, _2));
    }

private:
    void handle_service(const std::shared_ptr<AddTwoInts::Request> request,
                        std::shared_ptr<AddTwoInts::Response> response)
    {
        response->sum = request->a + request->b;
        RCLCPP_INFO(this->get_logger(), "Incoming request\na: %ld b: %ld", request->a, request->b);
        RCLCPP_INFO(this->get_logger(), "Sending back response: %ld", response->sum);
    }

    rclcpp::Service<AddTwoInts>::SharedPtr service_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AddTwoIntsServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

#### Client Node

```cpp
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using AddTwoInts = example_interfaces::srv::AddTwoInts;
using namespace std::chrono_literals;

class AddTwoIntsClient : public rclcpp::Node
{
public:
    AddTwoIntsClient() : Node("add_two_ints_client")
    {
        client_ = create_client<AddTwoInts>("add_two_ints");
        while (!client_->wait_for_service(1s))
        {
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }
        request_ = std::make_shared<AddTwoInts::Request>();
    }

    void send_request(int64_t a, int64_t b)
    {
        request_->a = a;
        request_->b = b;
        using ServiceResponseFuture = rclcpp::Client<AddTwoInts>::SharedFuture;
        auto response_received_callback = [this](ServiceResponseFuture future)
        {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Result of add_two_ints: %ld", response->sum);
        };
        auto future_result = client_->async_send_request(request_, response_received_callback);
    }

private:
    rclcpp::Client<AddTwoInts>::SharedPtr client_;
    std::shared_ptr<AddTwoInts::Request> request_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto client = std::make_shared<AddTwoIntsClient>();
    client->send_request(std::stoi(argv[1]), std::stoi(argv[2]));
    rclcpp::spin(client);
    rclcpp::shutdown();
    return 0;
}
```

## Debugging Services from the Terminal

### Checking Available Services

To list all available services, use the following command:

```bash
ros2 service list
```

### Calling a Service

To call a service and send a request from the terminal, use:

```bash
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 2, b: 3}"
```

### Describing a Service

To get detailed information about a service, use:

```bash
ros2 service type /add_two_ints
ros2 interface show example_interfaces/srv/AddTwoInts
```

## Practical Activity

Implement the provided Python and C++ service and client nodes. Once implemented, test the communication between the nodes. Use the terminal commands to debug and verify the functionality of your service. Modify the service to handle different types of requests and responses, such as strings or custom data types, to reinforce your understanding of ROS2 Services.

## Conclusion

This tutorial has provided a comprehensive guide to understanding, implementing, and debugging ROS2 Services. Mastery of these concepts is crucial for developing robust and responsive robotic applications. Continue experimenting with different service types and explore advanced features to further enhance your skills in ROS2.
