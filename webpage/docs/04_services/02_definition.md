# ROS 2 Services

## Introduction to ROS 2 Services

ROS 2 services provide a synchronous communication mechanism between nodes in a ROS 2 system. This tutorial will introduce ROS 2 services through a detailed explanation and practical examples, comparing them with real-life scenarios and typical applications in robotics.

## Real-Life Analogy: Online Weather Service

To understand ROS 2 services, consider the analogy of an online weather service:

- **Client-Server Relationship**: Imagine you are using a computer to get the local weather from an online service. Your computer acts as the client, and the weather service acts as the server.
- **Request-Response Pattern**: The client sends a request to the server with specific information (e.g., your location). The server processes the request and sends back a response (e.g., the weather information for the given location).

In this analogy, the URL used for the HTTP request is akin to the service name in ROS 2. The client sends a well-formed request (location data), and the server responds with the appropriate data (weather information).

## ROS 2 Services in Practice

In ROS 2, services facilitate a client-server architecture where nodes can request specific actions from other nodes. This section will delve into the implementation details and practical usage of ROS 2 services.

## Key Concepts

1. **Service Name**: A unique identifier for the service, similar to the URL in the analogy.
2. **Request Message**: The data structure sent by the client to the server.
3. **Response Message**: The data structure sent by the server back to the client.

## Example: LED Control in Robotics

Consider a scenario in a robotics application where you need to control an LED panel:

1. **LED Panel Node**:

   - **Role**: Controls the hardware to power on and off LEDs.
   - **Service Server**: Implements a service named `set_led` that handles requests to change the state of specific LEDs.
2. **Battery Monitoring Node**:

   - **Role**: Monitors the battery level and triggers actions based on its state.
   - **Service Client**: Sends requests to the `set_led` service to turn LEDs on or off based on battery status.

## Service Definition

For the `set_led` service, you would define the request and response messages as follows:

- **Request Message**: Contains the ID of the LED and the desired state (on or off).
- **Response Message**: Contains a success flag indicating whether the operation was successful.

```python
# set_led.srv
int32 id
bool state
---
bool success
```

## Implementation in ROS 2 (Python Example)

1. **Service Server (LED Panel Node)**:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetLed

class LedPanelNode(Node):

    def __init__(self):
        super().__init__('led_panel')
        self.srv = self.create_service(SetLed, 'set_led', self.set_led_callback)

    def set_led_callback(self, request, response):
        led_id = request.id
        state = request.state
        # Code to control the LED hardware
        success = self.control_led(led_id, state)
        response.success = success
        return response

    def control_led(self, led_id, state):
        # Hardware control logic here
        # For the sake of example, let's assume it always succeeds
        self.get_logger().info(f'LED {led_id} set to {"on" if state else "off"}')
        return True

def main(args=None):
    rclpy.init(args=args)
    node = LedPanelNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

2. **Service Client (Battery Monitoring Node)**:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetLed

class BatteryMonitorNode(Node):

    def __init__(self):
        super().__init__('battery_monitor')
        self.cli = self.create_client(SetLed, 'set_led')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        self.battery_check_timer = self.create_timer(10.0, self.check_battery)

    def check_battery(self):
        # Dummy check for battery level
        battery_low = self.is_battery_low()
        if battery_low:
            self.send_led_request(3, True)  # Turn on LED 3
        else:
            self.send_led_request(3, False)  # Turn off LED 3

    def is_battery_low(self):
        # Replace with actual battery check logic
        return True  # Assume battery is low for example

    def send_led_request(self, led_id, state):
        request = SetLed.Request()
        request.id = led_id
        request.state = state
        self.future = self.cli.call_async(request)
        self.future.add_done_callback(self.led_response_callback)

    def led_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('LED control successful')
            else:
                self.get_logger().warn('LED control failed')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = BatteryMonitorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Summary

ROS 2 services provide a powerful mechanism for implementing client-server communication in robotic systems. They enable nodes to request specific actions from other nodes and receive responses, ensuring a structured and reliable exchange of information. This tutorial has covered the fundamental concepts and provided practical examples to illustrate how to define, implement, and use ROS 2 services effectively.

## Conclusion

In conclusion, ROS 2 services complement the unidirectional data streams of topics by offering synchronous communication suitable for request-response interactions. They are essential for scenarios where nodes need to perform specific actions based on requests and provide a robust framework for building complex robotic applications.
