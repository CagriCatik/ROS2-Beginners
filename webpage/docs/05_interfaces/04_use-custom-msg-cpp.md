# ROS2 Custom Message: Python and C++ Integration

This tutorial demonstrates how to create a custom ROS2 message and use it in both Python and C++. The process involves creating a custom message, setting up publisher nodes in both languages, and ensuring proper integration with VS Code. This guide assumes a basic understanding of ROS2, Python, and C++.

## Step 1: Create a Custom Message

First, create a custom message for your ROS2 package. Assume the package is named `my_robot_interfaces`.

1. **Define the Message:**
   Create a directory for your messages within your package:
   ```sh
   mkdir -p ~/ros2_ws/src/my_robot_interfaces/msg
   ```

2. **Create a Message File:**
   Create a file named `HardwareStatus.msg` in the `msg` directory:
   ```plaintext
   # HardwareStatus.msg
   bool is_operational
   float32 temperature
   int32 error_code
   ```

3. **Update the Package:**
   Modify `package.xml` to include the message generation dependencies:
   ```xml
   <build_depend>rosidl_default_generators</build_depend>
   <exec_depend>rosidl_default_runtime</exec_depend>
   ```

   Update `CMakeLists.txt` to generate the custom messages:
   ```cmake
   find_package(rosidl_default_generators REQUIRED)

   rosidl_generate_interfaces(${PROJECT_NAME}
     "msg/HardwareStatus.msg"
   )
   ```

4. **Build the Package:**
   Build your package to generate the necessary message headers and libraries:
   ```sh
   colcon build --packages-select my_robot_interfaces
   ```

## Step 2: Python Publisher Node

Create a Python node to publish the `HardwareStatus` message.

1. **Create the Node:**
   Create a directory for your Python nodes if it doesn't exist:
   ```sh
   mkdir -p ~/ros2_ws/src/my_python_package/my_python_package
   ```

2. **Python Publisher Script:**
   Create a script named `hardware_status_publisher.py`:
   ```python
   import rclpy
   from rclpy.node import Node
   from my_robot_interfaces.msg import HardwareStatus

   class HardwareStatusPublisher(Node):
       def __init__(self):
           super().__init__('hardware_status_publisher')
           self.publisher_ = self.create_publisher(HardwareStatus, 'hardware_status', 10)
           self.timer = self.create_timer(1.0, self.publish_status)

       def publish_status(self):
           msg = HardwareStatus()
           msg.is_operational = True
           msg.temperature = 36.5
           msg.error_code = 0
           self.publisher_.publish(msg)
           self.get_logger().info(f'Published: {msg}')

   def main(args=None):
       rclpy.init(args=args)
       node = HardwareStatusPublisher()
       rclpy.spin(node)
       node.destroy_node()
       rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

3. **Update `setup.py`:**
   Ensure your package is properly set up to include the script:
   ```python
   entry_points={
       'console_scripts': [
           'hardware_status_publisher = my_python_package.hardware_status_publisher:main',
       ],
   },
   ```

4. **Build and Run:**
   Build and run the Python node:
   ```sh
   colcon build --packages-select my_python_package
   . install/setup.bash
   ros2 run my_python_package hardware_status_publisher
   ```

## Step 3: C++ Publisher Node

Create a C++ node to publish the `HardwareStatus` message.

1. **Create the Node:**
   Create a directory for your C++ nodes if it doesn't exist:
   ```sh
   mkdir -p ~/ros2_ws/src/my_cpp_package/src
   ```

2. **C++ Publisher Source File:**
   Create a file named `hardware_status_publisher.cpp`:
   ```cpp
   #include <chrono>
   #include <rclcpp/rclcpp.hpp>
   #include "my_robot_interfaces/msg/hardware_status.hpp"

   using namespace std::chrono_literals;

   class HardwareStatusPublisher : public rclcpp::Node
   {
   public:
       HardwareStatusPublisher()
       : Node("hardware_status_publisher")
       {
           publisher_ = this->create_publisher<my_robot_interfaces::msg::HardwareStatus>("hardware_status", 10);
           timer_ = this->create_wall_timer(1s, std::bind(&HardwareStatusPublisher::publish_status, this));
       }

   private:
       void publish_status()
       {
           auto message = my_robot_interfaces::msg::HardwareStatus();
           message.is_operational = true;
           message.temperature = 36.5;
           message.error_code = 0;
           RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.is_operational ? "true" : "false");
           publisher_->publish(message);
       }

       rclcpp::Publisher<my_robot_interfaces::msg::HardwareStatus>::SharedPtr publisher_;
       rclcpp::TimerBase::SharedPtr timer_;
   };

   int main(int argc, char * argv[])
   {
       rclcpp::init(argc, argv);
       rclcpp::spin(std::make_shared<HardwareStatusPublisher>());
       rclcpp::shutdown();
       return 0;
   }
   ```

3. **Update `CMakeLists.txt`:**
   Modify the `CMakeLists.txt` to build the C++ node:
   ```cmake
   find_package(rclcpp REQUIRED)
   find_package(my_robot_interfaces REQUIRED)

   add_executable(hardware_status_publisher src/hardware_status_publisher.cpp)
   ament_target_dependencies(hardware_status_publisher rclcpp my_robot_interfaces)
   install(TARGETS hardware_status_publisher
     DESTINATION lib/${PROJECT_NAME})
   ```

4. **Build and Run:**
   Build and run the C++ node:
   ```sh
   colcon build --packages-select my_cpp_package
   . install/setup.bash
   ros2 run my_cpp_package hardware_status_publisher
   ```

## Step 4: Integrate with VS Code

Ensure your VS Code environment is correctly set up to work with ROS2 and your custom message.

1. **VS Code Settings:**
   Add the include directories for the custom message to the `c_cpp_properties.json` file:
   ```json
   {
       "configurations": [
           {
               "name": "Linux",
               "includePath": [
                   "${workspaceFolder}/**",
                   "~/ros2_ws/install/my_robot_interfaces/include/**"
               ],
               "defines": [],
               "compilerPath": "/usr/bin/gcc",
               "cStandard": "c11",
               "cppStandard": "c++14"
           }
       ],
       "version": 4
   }
   ```

2. **Package Configuration:**
   Ensure the package dependencies are correctly specified in `package.xml` and `CMakeLists.txt`:
   ```xml
   <exec_depend>my_robot_interfaces</exec_depend>
   ```

   ```cmake
   find_package(my_robot_interfaces REQUIRED)
   ```

## Step 5: Verify Integration

1. **Run the Python Node:**
   ```sh
   ros2 run my_python_package hardware_status_publisher
   ```

2. **Run the C++ Node:**
   ```sh
   ros2 run my_cpp_package hardware_status_publisher
   ```

3. **Verify Messages:**
   Use `ros2 topic echo` to verify messages:
   ```sh
   ros2 topic echo /hardware_status
   ```

If everything is set up correctly, you should see messages being published from both the Python and C++ nodes.

This tutorial provides a comprehensive guide to creating and using custom ROS2 messages in both Python and C++. By following these steps, you can ensure proper integration and functionality in a robust and scientifically accurate manner.