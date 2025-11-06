cmake_minimum_required(VERSION 3.8)
project(balance_controller_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(servo_rs485_ros2 REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include)

add_executable(balance_controller_node
  src/balance_controller.cpp
)

ament_target_dependencies(balance_controller_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  servo_rs485_ros2
  ament_index_cpp
  tf2
)

target_link_libraries(balance_controller_node
  yaml-cpp
)

install(TARGETS balance_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()