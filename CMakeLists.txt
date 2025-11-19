find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(f110_msgs REQUIRED)

add_executable(detect_cpp_node src/detect_cpp_node.cpp)
ament_target_dependencies(detect_cpp_node
  rclcpp
  sensor_msgs
  f110_msgs
)

install(TARGETS
  detect_cpp_node
  DESTINATION lib/${PROJECT_NAME})
