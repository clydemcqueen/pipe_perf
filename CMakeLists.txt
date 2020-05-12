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
find_package(fiducial_vlam_msgs REQUIRED)
find_package(h264_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(image_transport_plugins REQUIRED)
find_package(orca_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_executable(time_send_node src/time_send.cpp)
ament_target_dependencies(time_send_node std_msgs rclcpp)

add_executable(time_receive_node src/time_receive.cpp)
ament_target_dependencies(time_receive_node std_msgs rclcpp)

add_executable(driver_sub_node src/driver_sub.cpp)
ament_target_dependencies(driver_sub_node orca_msgs rclcpp)

add_executable(fiducial_sub_node src/fiducial_sub.cpp)
ament_target_dependencies(fiducial_sub_node fiducial_vlam_msgs rclcpp)

add_executable(h264_sub_node src/h264_sub.cpp)
ament_target_dependencies(h264_sub_node h264_msgs rclcpp)

add_executable(image_sub_node src/image_sub.cpp)
ament_target_dependencies(image_sub_node sensor_msgs rclcpp)

add_executable(image_transport_sub_node src/image_transport_sub.cpp)
ament_target_dependencies(image_transport_sub_node image_transport image_transport_plugins sensor_msgs rclcpp)

# ament_export_include_directories(include)

# install(DIRECTORY include/${PROJECT_NAME} DESTINATION include)

install(TARGETS
  time_send_node
  time_receive_node
  driver_sub_node
  fiducial_sub_node
  h264_sub_node
  image_sub_node
  image_transport_sub_node
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_package()