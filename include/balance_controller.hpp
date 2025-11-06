#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <servo_rs485_ros2/srv/set_angle.hpp>

#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <mutex>
#include <optional>

// 简单串口封装，用于和 ODrive ASCII 接口通信
class OdriveSerial {
public:
  explicit OdriveSerial(const std::string &port = "/dev/ttyACM0", int baud = 115200);
  ~OdriveSerial();

  bool is_ready() const { return ready_; }
  bool set_state(int axis, int state);
  bool clear_errors(int axis);
  bool set_velocity(int axis, float vel);
  bool get_velocity(int axis, float &vel_out);

private:
  bool write_line(const std::string &cmd);
  bool read_response(std::string &out, int timeout_ms = 50);

  int fd_ = -1;
  bool ready_ = false;
  std::string port_;
  int baud_;
  mutable std::mutex io_mutex_;
};

// PID 结构体
struct PIDController {
  double kp = 0.0, ki = 0.0, kd = 0.0;
  double integral = 0.0, last_error = 0.0;
  double integral_min = -1.0, integral_max = 1.0;

  double calculate(double target, double current, double derivative_val = 0.0) {
    double error = target - current;
    integral += error;
    integral = std::clamp(integral, integral_min, integral_max);
    double derivative = error - last_error;
    last_error = error;
    return kp * error + ki * integral +
           kd * (derivative_val != 0.0 ? derivative_val : derivative);
  }
};

class BikeControllerNode : public rclcpp::Node {
public:
  BikeControllerNode();
  ~BikeControllerNode();

private:
  // 回调
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void servo_angle_callback(const std_msgs::msg::Float64::SharedPtr msg);
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void control_loop();

  // 辅助
  void initialize_parameters();
  void initialize_odrive();
  void set_drive_velocity(float velocity);
  void stop_all_motors();
  double get_roll_from_quaternion(const geometry_msgs::msg::Quaternion &q);

  template<typename T>
  T load_param(const YAML::Node &node, const std::string &key, const T &fallback);

  // ROS 接口
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr servo_angle_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Client<servo_rs485_ros2::srv::SetAngle>::SharedPtr servo_set_angle_client_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  // ODrive 控制
  std::unique_ptr<OdriveSerial> odrive_serial_;
  int axis_drive_id_;
  int axis_flywheel_id_;
  std::string odrive_port_;
  int odrive_baud_;

  // 状态
  double current_roll_ = 0.0;
  double current_roll_velocity_ = 0.0;
  double current_steer_angle_ = 0.0;
  double current_drive_speed_ = 0.0;
  double last_flywheel_vel_cmd_ = 0.0;
  rclcpp::Time last_imu_time_;
  bool is_imu_ok_ = false;

  // PID
  PIDController angle_pid_;
  PIDController angle_velocity_pid_;
  PIDController flywheel_speed_pid_;

  // 参数
  double machine_middle_angle_init_;
  double bike_turn_scale_deg_;
  double bike_speed_scale_deg_;
  double middle_angle_recitfy_limit_deg_;
  double servo_center_deg_;
  double servo_middle_range_;
  double march_velocity_;
  double drive_accel_;
  double flywheel_speed_limit_;
  double flywheel_accel_limit_;
  double safety_timeout_s_;
  double max_roll_angle_for_safety_;
  std::string params_file_;
};