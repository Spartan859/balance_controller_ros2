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