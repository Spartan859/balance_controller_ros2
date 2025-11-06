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
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)

include_directories(include)

add_library(balance_controller SHARED
  src/balance_controller.cpp
)

ament_target_dependencies(balance_controller
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  servo_rs485_ros2
  ament_index_cpp
  tf2
  controller_interface
  hardware_interface
  pluginlib
  rclcpp_lifecycle
  realtime_tools
)

target_link_libraries(balance_controller
  yaml-cpp
)

pluginlib_export_plugin_description_file(controller_interface balance_controller.xml)

install(TARGETS balance_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()