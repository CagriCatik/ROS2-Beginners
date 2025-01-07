# Creating and Building a Custom ROS2 Service

In this tutorial, we will provide a detailed guide on how to create and build a custom service in ROS2. This process assumes you have some familiarity with creating and building messages in ROS2, as the steps for services are quite similar.

## Step 1: Set Up Your Package

First, we will use the existing package `my_robot_interfaces`. Ensure you have this package created and properly set up. 

## Step 2: Create the Service Definition File

Navigate to the `msg` folder within your package and create a new subfolder named `srv`. Inside this `srv` folder, we will define our custom service.

Create a file named `ComputeRectangleArea.srv` with the following content:

```plaintext
# Request part
float64 length
float64 width
---
# Response part
float64 area
```

This service will take the length and width of a rectangle as input and return the area as output.

## Step 3: Update `CMakeLists.txt`

Ensure that your `CMakeLists.txt` file is configured to process the service definitions. Add the service file to the `rosidl_generate_interfaces` section. Your `CMakeLists.txt` should include:

```cmake
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ComputeRectangleArea.srv"
)
```

## Step 4: Update `package.xml`

Make sure your `package.xml` includes the necessary dependencies. Add the following lines if they are not already present:

```xml
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
```

## Step 5: Build the Package

With the service definition in place and the necessary configurations in your `CMakeLists.txt` and `package.xml`, you can now build your package. Use the following command to build the package:

```bash
colcon build --packages-select my_robot_interfaces
```

After building, source the setup file:

```bash
source install/setup.bash
```

## Step 6: Verify the Service

You can verify that your service has been created correctly by using the `ros2 interface show` command:

```bash
ros2 interface show my_robot_interfaces/srv/ComputeRectangleArea
```

This command should display the definition of your service.

## Step 7: Implementing the Service in Python

Next, we will create a Python service node to handle requests for calculating the area of a rectangle.

Create a new Python script named `rectangle_area_service.py` in the `my_robot_interfaces` package's `src` directory with the following content:

```python
import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import ComputeRectangleArea

class RectangleAreaService(Node):

    def __init__(self):
        super().__init__('rectangle_area_service')
        self.srv = self.create_service(ComputeRectangleArea, 'compute_rectangle_area', self.compute_rectangle_area_callback)

    def compute_rectangle_area_callback(self, request, response):
        response.area = request.length * request.width
        self.get_logger().info(f'Request: length={request.length}, width={request.width}, area={response.area}')
        return response

def main(args=None):
    rclpy.init(args=args)
    rectangle_area_service = RectangleAreaService()
    rclpy.spin(rectangle_area_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Step 8: Implementing the Client in Python

Similarly, create a client node to send requests to the service. Create a file named `rectangle_area_client.py` with the following content:

```python
import sys
import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import ComputeRectangleArea

class RectangleAreaClient(Node):

    def __init__(self):
        super().__init__('rectangle_area_client')
        self.client = self.create_client(ComputeRectangleArea, 'compute_rectangle_area')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        self.request = ComputeRectangleArea.Request()

    def send_request(self):
        self.request.length = float(sys.argv[1])
        self.request.width = float(sys.argv[2])
        future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main(args=None):
    rclpy.init(args=args)
    rectangle_area_client = RectangleAreaClient()
    response = rectangle_area_client.send_request()
    rectangle_area_client.get_logger().info(f'Result: {response.area}')
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Step 9: Running the Service and Client

To run the service node, use the following command:

```bash
ros2 run my_robot_interfaces rectangle_area_service
```

In a new terminal, run the client node with the length and width as arguments:

```bash
ros2 run my_robot_interfaces rectangle_area_client 5.0 3.0
```

The client will send the length and width to the service, and the service will respond with the area of the rectangle.

## Conclusion

In this tutorial, we created a custom service in ROS2, defined its message structure, updated the package configuration, built the package, and implemented both a service and a client in Python. This provides a solid foundation for creating and using custom services in your ROS2 projects.