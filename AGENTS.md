# AGENTS.md - Balance Controller ROS2

## 项目概述

**balance_controller_ros2** 是一个ROS2包，用于控制平衡自行车（balance bike）的平衡系统。该包实现了串级PID控制算法，通过ODrive电机驱动飞轮和驱动轮，实现动态平衡，同时支持舵机转向控制。

## 运行环境 (Runtime Environment)

**重要提醒**：本项目的代码在本地开发环境中编写和维护，但ROS2相关代码的实际运行需要在远程服务器上执行。

### 部署说明

- **本地开发**：代码编辑、编译和测试在本地Windows环境（D:\GitHub\bike_workspace_ros2）进行
- **远程运行**：ROS2节点和控制器的实际执行需要在SSH远程服务器 `opi_lixiangyu` 上进行
- **连接方式**：使用SSH连接到远程服务器：`ssh opi_lixiangyu`
- **同步代码**：开发完成后，通过Git或其他方式将代码同步到远程服务器进行测试和运行

### 远程服务器要求

- 安装ROS2 Humble或其他兼容版本
- 配置ODrive硬件接口和相关依赖
- 确保网络连接允许SSH访问和ROS2通信
- **Shell环境**：远程服务器使用zsh shell，请注意命令语法兼容性。加载ROS环境时使用 `source /opt/ros/humble/setup.zsh`

请在部署前确保远程服务器环境已正确配置。

## 问题
```
[INFO] [launch]: All log files can be found below /root/.ros/log/2025-11-06-21-38-34-616827-orangepiaipro-20t-42488
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [robot_state_publisher-1]: process started with pid [42516]
[INFO] [ros2_control_node-2]: process started with pid [42518]
[INFO] [spawner-3]: process started with pid [42520]
[INFO] [spawner-4]: process started with pid [42522]
[robot_state_publisher-1] [INFO] [1762436315.196726408] [robot_state_publisher]: got segment base_link
[robot_state_publisher-1] [INFO] [1762436315.196985347] [robot_state_publisher]: got segment drive_link
[robot_state_publisher-1] [INFO] [1762436315.197010284] [robot_state_publisher]: got segment flywheel_link
[ros2_control_node-2] [WARN] [1762436315.211718822] [controller_manager]: [Deprecated] Passing the robot description parameter directly to the control_manager node is deprecated. Use '~/robot_description' topic from 'robot_state_publisher' instead.
[ros2_control_node-2] [INFO] [1762436315.212066491] [resource_manager]: Loading hardware 'bike_robot'
[ros2_control_node-2] [INFO] [1762436315.213783250] [resource_manager]: Initialize hardware 'bike_robot'
[ros2_control_node-2] [ERROR] [1762436315.222601756] [ODriveHardwareInterface]: LIBUSB_ERROR_NO_DEVICE
[ros2_control_node-2] [INFO] [1762436315.222691236] [resource_manager]: Failed to initialize hardware 'bike_robot'
[ros2_control_node-2] [WARN] [1762436315.222705799] [resource_manager]: System hardware component 'bike_robot' from plugin 'odrive_hardware_interface/ODriveHardwareInterface' failed to initialize.
[ros2_control_node-2] [WARN] [1762436315.222770653] [resource_manager]: hardware 'bike_robot' is in finalized state and can be only destroyed.
[ros2_control_node-2] terminate called after throwing an instance of 'std::runtime_error'
[ros2_control_node-2]   what():  Failed to set the initial state of the component : bike_robot to active
[ros2_control_node-2] Stack trace (most recent call last):
[ros2_control_node-2] #13   Object "/usr/lib/aarch64-linux-gnu/ld-linux-aarch64.so.1", at 0xffffffffffffffff, in 
[ros2_control_node-2] #12   Object "/opt/ros/humble/lib/controller_manager/ros2_control_node", at 0xaaaac0ce472f, in 
[ros2_control_node-2] #11   Source "../csu/libc-start.c", line 392, in __libc_start_main_impl [0xe7ffd79c74cb]
[ros2_control_node-2] #10   Source "../sysdeps/nptl/libc_start_call_main.h", line 58, in __aarch64_ldadd4_acq [0xe7ffd79c73fb]
[ros2_control_node-2] #9    Object "/opt/ros/humble/lib/controller_manager/ros2_control_node", at 0xaaaac0ce3ceb, in
[ros2_control_node-2] #8    Object "/opt/ros/humble/lib/libcontroller_manager.so", at 0xe7ffd8091c0f, in controller_manager::ControllerManager::ControllerManager(std::shared_ptr<rclcpp::Executor>, std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > const&, std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > const&, rclcpp::NodeOptions const&)
[ros2_control_node-2] #7    Object "/opt/ros/humble/lib/libcontroller_manager.so", at 0xe7ffd808d8db, in controller_manager::ControllerManager::init_resource_manager(std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > const&)
[ros2_control_node-2] #6    Object "/usr/lib/aarch64-linux-gnu/libstdc++.so.6.0.30", at 0xe7ffd7c22e03, in __cxa_throw
[ros2_control_node-2] #5    Object "/usr/lib/aarch64-linux-gnu/libstdc++.so.6.0.30", at 0xe7ffd7c22b1f, in std::terminate()
[ros2_control_node-2] #4    Object "/usr/lib/aarch64-linux-gnu/libstdc++.so.6.0.30", at 0xe7ffd7c22abb, in
[ros2_control_node-2] #3    Object "/usr/lib/aarch64-linux-gnu/libstdc++.so.6.0.30", at 0xe7ffd7c262db, in __gnu_cxx::__verbose_terminate_handler()
[ros2_control_node-2] #2    Source "./stdlib/abort.c", line 79, in abort [0xe7ffd79c712f]
[ros2_control_node-2] #1    Source "../sysdeps/posix/raise.c", line 26, in raise [0xe7ffd79da67b]
[ros2_control_node-2] #0    Source "./nptl/pthread_kill.c", line 44, in __pthread_kill_implementation [0xe7ffd7a1f1f0]
[ros2_control_node-2] Aborted (Signal sent by tkill() 42518 0)
```