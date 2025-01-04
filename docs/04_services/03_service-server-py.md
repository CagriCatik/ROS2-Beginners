## Creating a Service Server in ROS 2

### Introduction

In this tutorial, we will develop a service server in Python using ROS 2. This server will accept two integers as input and return their sum as output. We will walk through the process step by step, ensuring all concepts are thoroughly explained and accurately implemented.

### Step 1: Understanding ROS 2 Services

A service in ROS 2 is defined by two main components:

1. **Service Name:** This acts as a common interface between nodes, allowing them to find and utilize the service.
2. **Service Type:** This defines the request and response structure for the service.

### Step 2: Using an Existing Service Type

For this tutorial, we will use an existing service type, `AddTwoInts`, provided by ROS 2. You can inspect available interfaces using:

```
ros2 interface show example_interfaces/srv/AddTwoInts
```

This command will display the request and response structure:

- Request: Two integers, `a` and `b`.
- Response: One integer, `sum`.

### Step 3: Setting Up the Workspace

Navigate to your ROS 2 workspace and create a new directory for the service:

```sh
cd ~/ros2_ws/src/
mkdir -p my_service_package/my_service
cd my_service_package/my_service
```

### Step 4: Creating the Service Server Node

Create a new Python file for the service server:

```sh
touch add_two_ints_server.py
chmod +x add_two_ints_server.py
```

### Step 5: Writing the Service Server Code

Open `add_two_ints_server.py` in your preferred text editor and add the following code:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddTwoIntsServer(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.get_logger().info('AddTwoInts server has been started.')

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Request: {request.a} + {request.b} = {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 6: Explanation of the Code

1. **Imports:** Import the necessary modules and service type.
2. **Class Definition:** Define the `AddTwoIntsServer` class, inheriting from `Node`.
3. **Service Creation:** In the constructor, create the service using `self.create_service`. The arguments are:
   - `AddTwoInts`: The service type.
   - `'add_two_ints'`: The service name.
   - `self.add_two_ints_callback`: The callback function to handle the request.
4. **Callback Function:** The callback function `add_two_ints_callback` processes the request by adding the two integers and returning the sum.
5. **Main Function:** Initialize the ROS 2 system, create the node, and keep it running until interrupted.

### Step 7: Building and Running the Service

Navigate back to your workspace root and build the package:

```sh
cd ~/ros2_ws/
colcon build --packages-select my_service_package
```

Source the setup file and run the service:

```sh
. install/setup.bash
ros2 run my_service_package add_two_ints_server
```

You should see a log message indicating the server has started.

### Step 8: Testing the Service

In a new terminal, call the service using the ROS 2 command-line tool:

```sh
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 3, b: 4}"
```

You should see the server log the request and response:

```
Request: 3 + 4 = 7
```

### Conclusion

You have successfully created and tested a ROS 2 service server. This server can handle requests to add two integers and return their sum. This fundamental understanding can now be extended to creating more complex services and custom service types in ROS 2.
