cmake_minimum_required(VERSION 3.5)
project(my_cpp_package)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(my_cpp_node src/my_cpp_node.cpp)
ament_target_dependencies(my_cpp_node rclcpp)

install(TARGETS
  my_cpp_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
