#pragma once

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <chrono>
#include <string>
#include <vector>

namespace balance_controller_ros2 {

struct PIDState {
  double kp{0.0};
  double ki{0.0};
  double kd{0.0};
  double integral{0.0};
  double last_error{0.0};
  double integral_min{-1e9};
  double integral_max{1e9};
};

class BalanceController : public controller_interface::ControllerInterface {
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update() override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & prev_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & prev_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & prev_state) override;

private:
  // Params
  std::string drive_joint_name_;
  std::string flywheel_joint_name_;
  double angle_integral_limit_{30.0};
  double flywheel_speed_limit_{20.0};
  double flywheel_accel_limit_{5.0};
  double roll_diff_limit_{0.1};
  double machine_middle_angle_init_{0.0};
  double machine_middle_angle_{0.0};
  double bike_turn_scale_deg_{0.06};
  double bike_speed_scale_deg_{0.002};
  double middle_angle_recitfy_limit_deg_{3.0};
  double servo_center_deg_{0.0};
  double servo_middle_range_{2.0};

  // PID states
  PIDState angle_pid_{};      // outer (angle)
  PIDState angle_vel_pid_{};  // middle (angular velocity)
  PIDState flywheel_zero_pid_{}; // slow ring for zero speed correction
  double flywheel_zero_integral_limit_min_{-200.0};
  double flywheel_zero_integral_limit_max_{200.0};

  // Cached sensor values
  double last_roll_deg_{0.0};
  double last_roll_gyro_degps_{0.0};
  double last_drive_speed_{0.0};
  double last_servo_angle_{0.0};
  rclcpp::Time last_imu_stamp_{};

  // Command state
  double last_flywheel_command_{0.0};
  size_t loop_counter_{0};

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr servo_angle_sub_;

  // Helpers
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void servoAngleCallback(const std_msgs::msg::Float64::SharedPtr msg);

  double computeAnglePID(double current_angle, double target_angle, double current_gyro);
  double computeAngularVelocityPID(double current_gyro, double target_gyro);
  double computeFlywheelZeroPID(double current_speed);
  double clamp(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }
};

} // namespace balance_controller_ros2

PLUGINLIB_EXPORT_CLASS(balance_controller_ros2::BalanceController, controller_interface::ControllerInterface)