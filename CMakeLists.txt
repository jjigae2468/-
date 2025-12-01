cmake_minimum_required(VERSION 3.5)
project(simple_perception_cpp)

# 기본 설정
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 필요한 패키지 의존성
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(f110_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# -------------------------------
# perception_cpp 노드
# -------------------------------
add_executable(simple_detector_node
  src/simple_perception.cpp
)

ament_target_dependencies(simple_detector_node
  rclcpp
  sensor_msgs
  f110_msgs
)

# -------------------------------
# tracking_cpp 노드
# -------------------------------
add_executable(tracking_cpp
  src/tracking.cpp
)

ament_target_dependencies(tracking_cpp
  rclcpp
  sensor_msgs
  f110_msgs
)

# -------------------------------
# 설치 (launch / config / 실행파일)
# -------------------------------
install(TARGETS
  simple_detector_node
  tracking_cpp
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
