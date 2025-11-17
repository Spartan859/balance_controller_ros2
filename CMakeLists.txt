cmake_minimum_required(VERSION 3.8)
project(balance_controller_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(imu_ros2 REQUIRED)
find_package(servo_rs485_ros2 REQUIRED)

add_library(balance_controller SHARED
  src/balance_controller.cpp
)
target_include_directories(balance_controller PRIVATE
  include
)
ament_target_dependencies(balance_controller
  rclcpp
  rclcpp_lifecycle
  std_msgs
  sensor_msgs
  geometry_msgs
  pluginlib
  controller_interface
  hardware_interface
  realtime_tools
  tf2
  tf2_geometry_msgs
  imu_ros2
  servo_rs485_ros2
)

pluginlib_export_plugin_description_file(controller_interface balance_controller_plugin.xml)

install(DIRECTORY include/
  DESTINATION include
)
install(FILES balance_controller_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME}
)
install(TARGETS balance_controller
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

include_directories(include)

ament_export_dependencies(rclcpp controller_interface hardware_interface pluginlib)
ament_package()
