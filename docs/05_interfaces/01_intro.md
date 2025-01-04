# ROS2 Interfaces

ROS2 (Robot Operating System 2) interfaces are the mechanisms through which data is communicated between nodes in the form of messages and services. These interfaces define the structure of the data being transmitted, ensuring that the sending and receiving nodes understand the format and content of the information being exchanged.

In this tutorial, we will learn how to create and build custom message and service types in ROS2, and how to use these custom interfaces within your nodes, topics, and services. By the end of this section, you will be able to:

- Create and build your own interfaces with message (msg) and service (srv) definitions.
- Use these custom interfaces with your topics and services.
- Implement a complete activity involving nodes, topics, services, and custom interfaces.

## Understanding ROS2 Interfaces

ROS2 interfaces are defined in `.msg` (message) and `.srv` (service) files. These files specify the structure of the data that will be transmitted. A message file defines a data structure for a single message, while a service file defines a request-response communication pattern.

## Creating Custom Messages

1. **Create a New Package:**

   First, create a new ROS2 package where you will define your custom messages and services. Use the `ros2 pkg create` command to generate the package:

   ```bash
   ros2 pkg create --build-type ament_cmake my_custom_interfaces
   ```

2. **Define Message Files:**

   Inside your package, create a directory for message files:

   ```bash
   mkdir -p my_custom_interfaces/msg
   ```

   Create a new message file, for example, `MyMessage.msg`, in the `msg` directory:

   ```bash
   touch my_custom_interfaces/msg/MyMessage.msg
   ```

   Define the content of your message in `MyMessage.msg`:

   ```plaintext
   int32 id
   string content
   ```

3. **Update the CMakeLists.txt:**

   Modify the `CMakeLists.txt` file to include the message generation:

   ```cmake
   find_package(rosidl_default_generators REQUIRED)

   rosidl_generate_interfaces(${PROJECT_NAME}
     "msg/MyMessage.msg"
   )
   ```

4. **Update the package.xml:**

   Ensure the `package.xml` file includes the necessary dependencies for message generation:

   ```xml
   <build_depend>rosidl_default_generators</build_depend>
   <exec_depend>rosidl_default_runtime</exec_depend>
   ```

5. **Build the Package:**

   After defining your message and updating the necessary files, build your package:

   ```bash
   colcon build --packages-select my_custom_interfaces
   ```

## Creating Custom Services

1. **Define Service Files:**

   Similar to message files, create a directory for service files:

   ```bash
   mkdir -p my_custom_interfaces/srv
   ```

   Create a new service file, for example, `MyService.srv`, in the `srv` directory:

   ```bash
   touch my_custom_interfaces/srv/MyService.srv
   ```

   Define the request and response fields in `MyService.srv`:

   ```plaintext
   int32 a
   int32 b
   ---
   int32 sum
   ```

2. **Update the CMakeLists.txt:**

   Modify the `CMakeLists.txt` file to include the service generation:

   ```cmake
   rosidl_generate_interfaces(${PROJECT_NAME}
     "msg/MyMessage.msg"
     "srv/MyService.srv"
   )
   ```

3. **Update the package.xml:**

   Ensure the `package.xml` file includes the necessary dependencies for service generation:

   ```xml
   <build_depend>rosidl_default_generators</build_depend>
   <exec_depend>rosidl_default_runtime</exec_depend>
   ```

4. **Build the Package:**

   After defining your service and updating the necessary files, build your package:

   ```bash
   colcon build --packages-select my_custom_interfaces
   ```

## Using Custom Interfaces in Nodes

Once you have created and built your custom message and service types, you can use them in your ROS2 nodes.

## Publisher Node Example

Here is an example of a simple publisher node using the custom message `MyMessage`:

```python
import rclpy
from rclpy.node import Node
from my_custom_interfaces.msg import MyMessage

class MyPublisher(Node):

    def __init__(self):
        super().__init__('my_publisher')
        self.publisher_ = self.create_publisher(MyMessage, 'my_topic', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = MyMessage()
        msg.id = 1
        msg.content = 'Hello, ROS2!'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.id}, {msg.content}')

def main(args=None):
    rclpy.init(args=args)
    node = MyPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Subscriber Node Example

Here is an example of a simple subscriber node using the custom message `MyMessage`:

```python
import rclpy
from rclpy.node import Node
from my_custom_interfaces.msg import MyMessage

class MySubscriber(Node):

    def __init__(self):
        super().__init__('my_subscriber')
        self.subscription = self.create_subscription(
            MyMessage,
            'my_topic',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: {msg.id}, {msg.content}')

def main(args=None):
    rclpy.init(args=args)
    node = MySubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Service Server Node Example

Here is an example of a simple service server node using the custom service `MyService`:

```python
import rclpy
from rclpy.node import Node
from my_custom_interfaces.srv import MyService

class MyServiceServer(Node):

    def __init__(self):
        super().__init__('my_service_server')
        self.srv = self.create_service(MyService, 'my_service', self.service_callback)

    def service_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Service request: {request.a} + {request.b} = {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = MyServiceServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Service Client Node Example

Here is an example of a simple service client node using the custom service `MyService`:

```python
import rclpy
from rclpy.node import Node
from my_custom_interfaces.srv import MyService

class MyServiceClient(Node):

    def __init__(self):
        super().__init__('my_service_client')
        self.client = self.create_client(MyService, 'my_service')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.request = MyService.Request()

    def send_request(self):
        self.request.a = 10
        self.request.b = 20
        self.future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    node = MyServiceClient()
    response = node.send_request()
    node.get_logger().info(f'Service response: {response.sum}')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Conclusion

By following this tutorial, you have learned how to create and build custom message and service types in ROS2. You also implemented publisher, subscriber, service server, and service client nodes using these custom interfaces. Understanding how to define and use custom interfaces is essential for developing complex ROS2 applications that require specific data structures and communication patterns.