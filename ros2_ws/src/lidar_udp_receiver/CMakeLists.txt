cmake_minimum_required(VERSION 3.8)
project(lidar_udp_receiver)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Add include directories
include_directories(include)

# Add the UDP receiver node executable
add_executable(lidar_udp_receiver_node src/lidar_udp_receiver_node.cpp)
ament_target_dependencies(lidar_udp_receiver_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
)

# Install executables
install(TARGETS
  lidar_udp_receiver_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and RViz files
install(DIRECTORY
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()