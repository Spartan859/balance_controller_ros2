# AGENTS.md - Balance Controller ROS2

## 项目概述

**balance_controller_ros2** 是一个ROS2包，用于控制平衡自行车（balance bike）的平衡系统。该包实现了串级PID控制算法，通过ODrive电机驱动飞轮和驱动轮，实现动态平衡，同时支持舵机转向控制。

### 主要特性

- **串级PID控制**：角度PID + 角速度PID + 飞轮速度PID，实现精确平衡控制
- **动态零点校正**：根据转向和速度自动调整平衡点
- **安全机制**：IMU超时检测、倾角限制、电机急停保护
- **硬件接口**：集成odrive_ros2_control，支持USB连接ODrive电机控制器
- **ROS2控制框架**：基于ros2_control实现标准化硬件接口