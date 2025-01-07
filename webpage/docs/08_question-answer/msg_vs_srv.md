# Difference Between Srv and Msg in ROS2

In ROS2 (Robot Operating System 2), `srv` (services) and `msg` (messages) are fundamental concepts used for communication between nodes. Here’s a detailed explanation of the difference between them, along with a tutorial to illustrate their usage with a concrete example.

## Messages (`msg`)

Messages in ROS2 are used for unidirectional communication. They are defined data structures that can be sent over topics. Topics are named buses over which nodes exchange messages. A node can publish a message to a topic, and multiple other nodes can subscribe to that topic to receive the message.

**Use Case**: Continuous data streaming, such as sensor data (e.g., a camera image or LIDAR scan).

**Example**: A node publishes temperature readings to a `/temperature` topic, and multiple nodes subscribe to this topic to process the data.

## Services (`srv`)

Services in ROS2 provide a request/reply communication model. They are used for bidirectional communication. A service consists of a request and a response. A node can send a request to another node that offers the service, and then wait for a response.

**Use Case**: On-demand data retrieval or actions, such as querying the current position of a robot or commanding a robot to move to a specific location.

**Example**: A node requests the current position of a robot by calling a `/get_position` service, and the service node replies with the position data.

# Concrete Example: Temperature Query

Let's imagine we have a ROS2 system that should monitor the current temperature. We will use both messages and services to illustrate the difference.

## Messages (Message)

1. **Definition of the Message**

   - Create a message file `Temperature.msg` in the `msg` directory.

   ```plaintext
   float32 temperature
   ```
2. **Publisher Node**

   - A node that publishes temperature values.

   ```python
   import rclpy
   from rclpy.node import Node
   from example_interfaces.msg import Temperature

   class TemperaturePublisher(Node):
       def __init__(self):
           super().__init__('temperature_publisher')
           self.publisher_ = self.create_publisher(Temperature, 'temperature', 10)
           timer_period = 2.0  # seconds
           self.timer = self.create_timer(timer_period, self.publish_temperature)

       def publish_temperature(self):
           msg = Temperature()
           msg.temperature = 24.0  # This could be an actual sensor value
           self.publisher_.publish(msg)
           self.get_logger().info(f'Publishing: "{msg.temperature}"')

   def main(args=None):
       rclpy.init(args=args)
       node = TemperaturePublisher()
       rclpy.spin(node)
       node.destroy_node()
       rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```
3. **Subscriber Node**

   - A node that receives and processes temperature values.

   ```python
   import rclpy
   from rclpy.node import Node
   from example_interfaces.msg import Temperature

   class TemperatureSubscriber(Node):
       def __init__(self):
           super().__init__('temperature_subscriber')
           self.subscription = self.create_subscription(
               Temperature,
               'temperature',
               self.temperature_callback,
               10)
           self.subscription  # prevent unused variable warning

       def temperature_callback(self, msg):
           self.get_logger().info(f'I heard: "{msg.temperature}"')

   def main(args=None):
       rclpy.init(args=args)
       node = TemperatureSubscriber()
       rclpy.spin(node)
       node.destroy_node()
       rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

## Services (Service)

1. **Definition of the Service**

   - Create a service file `GetTemperature.srv` in the `srv` directory.

   ```plaintext
   ---
   float32 temperature
   ```
2. **Service Server Node**

   - A node that provides temperature values on request.

   ```python
   import rclpy
   from rclpy.node import Node
   from example_interfaces.srv import GetTemperature

   class TemperatureService(Node):
       def __init__(self):
           super().__init__('temperature_service')
           self.srv = self.create_service(GetTemperature, 'get_temperature', self.get_temperature_callback)

       def get_temperature_callback(self, request, response):
           response.temperature = 24.0  # This could be an actual sensor value
           self.get_logger().info(f'Returning temperature: {response.temperature}')
           return response

   def main(args=None):
       rclpy.init(args=args)
       node = TemperatureService()
       rclpy.spin(node)
       node.destroy_node()
       rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```
3. **Service Client Node**

   - A node that requests the temperature value from the service.

   ```python
   import rclpy
   from rclpy.node import Node
   from example_interfaces.srv import GetTemperature

   class TemperatureClient(Node):
       def __init__(self):
           super().__init__('temperature_client')
           self.cli = self.create_client(GetTemperature, 'get_temperature')
           while not self.cli.wait_for_service(timeout_sec=1.0):
               self.get_logger().info('Service not available, waiting again...')
           self.req = GetTemperature.Request()

       def send_request(self):
           self.future = self.cli.call_async(self.req)
           rclpy.spin_until_future_complete(self, self.future)
           return self.future.result()

   def main(args=None):
       rclpy.init(args=args)
       node = TemperatureClient()
       response = node.send_request()
       node.get_logger().info(f'Result: {response.temperature}')
       node.destroy_node()
       rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

# Summary

- **Messages (`msg`)**: For continuous data transmission between publishers and subscribers over topics.
- **Services (`srv`)**: For bidirectional communication with requests and responses.

In this example, `msg` is used to continuously publish temperature data, which can be received by multiple nodes. `srv` is used to perform a temperature query where a client sends a request to a server and receives a response. This distinction is crucial for choosing the right form of communication for different use cases in ROS2.
