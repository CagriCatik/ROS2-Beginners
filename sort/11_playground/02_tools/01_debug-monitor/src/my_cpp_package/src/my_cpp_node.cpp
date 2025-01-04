#include "rclcpp/rclcpp.hpp"

class MyCppNode : public rclcpp::Node {
public:
    MyCppNode() : Node("my_cpp_node") {
        RCLCPP_INFO(this->get_logger(), "Hello ROS2 from C++!");
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyCppNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
