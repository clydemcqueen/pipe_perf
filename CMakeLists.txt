cmake_minimum_required(VERSION 3.5)
project(pipe_perf)

# Default to C99
if (NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif ()

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif ()

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(time_send_node
  src/time_send.cpp)

ament_target_dependencies(time_send_node
  std_msgs
  rclcpp)

add_executable(time_receive_node
  src/time_receive.cpp)

ament_target_dependencies(time_receive_node
  std_msgs
  rclcpp)

# ament_export_include_directories(include)

# install(DIRECTORY include/${PROJECT_NAME} DESTINATION include)

install(TARGETS time_send_node time_receive_node
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_package()