# Creating a Custom ROS 2 Message

In this tutorial, we will create a custom ROS 2 message. Custom messages allow you to define specific data structures that your nodes can use to communicate. To maintain a clean and manageable codebase, we recommend creating a dedicated package for all your custom message definitions.

## Step 1: Create a Dedicated Package for Custom Messages

First, let's create a new package dedicated to our custom message interfaces. This package will only contain message definitions and no other code.

1. **Create the Package**

   Open a terminal and navigate to your ROS 2 workspace (typically named `ros2_ws`), then create the package:

   ```sh
   cd ~/ros2_ws/src
   ros2 pkg create my_robot_interfaces
   ```

   Replace `my_robot` with the name of your robot or project.

2. **Clean Up Unnecessary Files**

   Remove the `include` and `src` directories as they are not needed for this package:

   ```sh
   cd my_robot_interfaces
   rm -rf include src
   mkdir msg
   ```

## Step 2: Configure Package Files

We need to configure the `package.xml` and `CMakeLists.txt` files to enable message generation.

1. **Edit `package.xml`**

   Add the following dependencies inside the `<dependencies>` tag:

   ```xml
   <build_depend>rosidl_default_generators</build_depend>
   <exec_depend>rosidl_default_runtime</exec_depend>
   <member_of_group>rosidl_interface_packages</member_of_group>
   ```

   Your `package.xml` should look something like this:

   ```xml
   <package format="3">
     <name>my_robot_interfaces</name>
     <version>0.0.0</version>
     <description>The my_robot_interfaces package</description>

     <maintainer email="user@todo.todo">TODO: Maintainer Name</maintainer>
     <license>TODO: License declaration</license>

     <buildtool_depend>ament_cmake</buildtool_depend>

     <build_depend>rosidl_default_generators</build_depend>
     <exec_depend>rosidl_default_runtime</exec_depend>
     <member_of_group>rosidl_interface_packages</member_of_group>
   </package>
   ```

2. **Edit `CMakeLists.txt`**

   Make the following changes to configure the build process for message generation:

   ```cmake
   cmake_minimum_required(VERSION 3.5)
   project(my_robot_interfaces)

   find_package(ament_cmake REQUIRED)
   find_package(rosidl_default_generators REQUIRED)

   rosidl_generate_interfaces(${PROJECT_NAME}
     "msg/HardwareStatus.msg"
   )

   ament_package()
   ```

## Step 3: Define Your Custom Message

Create a new message definition file inside the `msg` directory.

1. **Create the Message File**

   Navigate to the `msg` directory and create a file named `HardwareStatus.msg`:

   ```sh
   cd msg
   touch HardwareStatus.msg
   ```

2. **Define the Message Fields**

   Edit `HardwareStatus.msg` to include the following fields:

   ```plaintext
   int64 temperature
   bool are_motors_ready
   string debug_message
   ```

## Step 4: Build the Package

Now, we will build the package to generate the message source code.

1. **Build the Package**

   Navigate back to the root of your workspace and build the package:

   ```sh
   cd ~/ros2_ws
   colcon build --packages-select my_robot_interfaces
   ```

2. **Source the Workspace**

   After building, source your workspace to make the new message available:

   ```sh
   source install/setup.bash
   ```

## Step 5: Verify the Message

You can verify that your message has been generated and is available by using the `ros2 interface show` command:

```sh
ros2 interface show my_robot_interfaces/msg/HardwareStatus
```

You should see the following output:

```plaintext
int64 temperature
bool are_motors_ready
string debug_message
```

## Step 6: Use the Custom Message in Your Code

You can now use your custom message in your ROS 2 nodes.

1. **Python Example**

   Here is an example of how to publish and subscribe to the `HardwareStatus` message in Python:

   **Publisher:**

   ```python
   import rclpy
   from rclpy.node import Node
   from my_robot_interfaces.msg import HardwareStatus

   class HardwareStatusPublisher(Node):
       def __init__(self):
           super().__init__('hardware_status_publisher')
           self.publisher_ = self.create_publisher(HardwareStatus, 'hardware_status', 10)
           self.timer = self.create_timer(1.0, self.publish_status)
           self.get_logger().info('Hardware Status Publisher started')

       def publish_status(self):
           msg = HardwareStatus()
           msg.temperature = 42
           msg.are_motors_ready = True
           msg.debug_message = "All systems go"
           self.publisher_.publish(msg)
           self.get_logger().info(f'Publishing: {msg}')

   def main(args=None):
       rclpy.init(args=args)
       node = HardwareStatusPublisher()
       rclpy.spin(node)
       node.destroy_node()
       rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

   **Subscriber:**

   ```python
   import rclpy
   from rclpy.node import Node
   from my_robot_interfaces.msg import HardwareStatus

   class HardwareStatusSubscriber(Node):
       def __init__(self):
           super().__init__('hardware_status_subscriber')
           self.subscription = self.create_subscription(
               HardwareStatus,
               'hardware_status',
               self.listener_callback,
               10)
           self.subscription  # prevent unused variable warning

       def listener_callback(self, msg):
           self.get_logger().info(f'Received: temperature={msg.temperature}, are_motors_ready={msg.are_motors_ready}, debug_message={msg.debug_message}')

   def main(args=None):
       rclpy.init(args=args)
       node = HardwareStatusSubscriber()
       rclpy.spin(node)
       node.destroy_node()
       rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

2. **C++ Example**

   Here is an example of how to publish and subscribe to the `HardwareStatus` message in C++:

   **Publisher:**

   ```cpp
   #include "rclcpp/rclcpp.hpp"
   #include "my_robot_interfaces/msg/hardware_status.hpp"

   class HardwareStatusPublisher : public rclcpp::Node
   {
   public:
       HardwareStatusPublisher()
       : Node("hardware_status_publisher")
       {
           publisher_ = this->create_publisher<my_robot_interfaces::msg::HardwareStatus>("hardware_status", 10);
           timer_ = this->create_wall_timer(
               std::chrono::seconds(1),
               std::bind(&HardwareStatusPublisher::publish_status, this));
           RCLCPP_INFO(this->get_logger(), "Hardware Status Publisher started");
       }

   private:
       void publish_status()
       {
           auto msg = my_robot_interfaces::msg::HardwareStatus();
           msg.temperature = 42;
           msg.are_motors_ready = true;
           msg.debug_message = "All systems go";
           publisher_->publish(msg);
           RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg.debug_message.c_str());
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

   **Subscriber:**

   ```cpp
   #include "rclcpp/rclcpp.hpp"
   #include "my_robot_interfaces/msg/hardware_status.hpp"

   class HardwareStatusSubscriber : public rclcpp::Node
   {
   public:
       HardwareStatusSubscriber()
       : Node("hardware_status_subscriber")
       {
           subscription_ = this->create_subscription<my_robot_interfaces::msg::HardwareStatus>(
               "hardware_status",
               10,
               std::bind(&HardwareStatusSubscriber::topic_callback, this, std::placeholders::_1));
       }

   private:
       void topic_callback(const my_robot_interfaces::msg::HardwareStatus::SharedPtr msg) const
       {
           RCLCPP_INFO(this->get_logger(), "Received: temperature=%ld, are_motors_ready=%s, debug_message=%s",
                       msg->temperature,
                       msg->are_motors_ready ? "true" : "false",
                       msg->debug_message.c_str());
       }

       rclcpp::Subscription<my_robot_interfaces::msg::HardwareStatus>::SharedPtr subscription_;
   };

   int main(int argc, char * argv[])
   {
       rclcpp::init(argc, argv);
       rclcpp::spin(std::make_shared<HardwareStatusSubscriber>());
       rclcpp::shutdown();
       return 0;
   }
   ```

## Conclusion

You have successfully created a custom ROS 2 message and learned how to use it in both Python and C++ nodes. This setup ensures that your custom messages are well-organized and easily maintainable.