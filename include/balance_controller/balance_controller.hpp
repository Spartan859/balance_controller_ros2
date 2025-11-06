#pragma once

#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <servo_rs485_ros2/srv/set_angle.hpp>
#include <realtime_tools/realtime_buffer.h>

#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <mutex>

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

namespace balance_controller
{
class BalanceController : public controller_interface::ControllerInterface
{
public:
  BalanceController();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

private:
  // 回调
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void servo_angle_callback(const std_msgs::msg::Float64::SharedPtr msg);
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

  // 辅助
  void initialize_parameters();
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

  // 硬件接口
  std::string drive_joint_name_;
  std::string flywheel_joint_name_;
  std::vector<std::string> joint_names_;
};
}  // namespace balance_controller