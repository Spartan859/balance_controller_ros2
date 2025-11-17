#include "balance_controller_ros2/balance_controller.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <sstream>
#include <iomanip>
#include <algorithm>

namespace balance_controller_ros2 {

controller_interface::CallbackReturn BalanceController::on_init() {
  // Declare parameters
  auto node = get_node();
  node->declare_parameter("drive_joint_name", std::string("joint0"));
  node->declare_parameter("flywheel_joint_name", std::string("joint1"));
  node->declare_parameter("angle_kp", 350.0);
  node->declare_parameter("angle_ki", 0.0);
  node->declare_parameter("angle_kd", 0.7);
  node->declare_parameter("angle_v_kp", 0.5);
  node->declare_parameter("angle_v_ki", 0.0);
  node->declare_parameter("angle_v_kd", 0.1);
  node->declare_parameter("flywheel_speed_kp", 0.1);
  node->declare_parameter("flywheel_speed_ki", 0.45);
  node->declare_parameter("flywheel_speed_integral_limit_min", -200.0);
  node->declare_parameter("flywheel_speed_integral_limit_max", 200.0);
  node->declare_parameter("flywheel_speed_limit", 20.0);
  node->declare_parameter("flywheel_accel_limit", 5.0);
  node->declare_parameter("roll_diff_limit", 0.1);
  node->declare_parameter("machine_middle_angle_init", -2.11);
  node->declare_parameter("bike_turn_scale_deg", 0.06);
  node->declare_parameter("bike_speed_scale_deg", 0.002);
  node->declare_parameter("middle_angle_recitfy_limit_deg", 3.0);
  node->declare_parameter("servo_center_deg", -10.0);
  node->declare_parameter("servo_middle_range", 2.0);
  // Logging params
  node->declare_parameter("log_enable", true);
  node->declare_parameter("log_hz", 10.0);
  // Fetch initial joint names so interface configuration has them
  drive_joint_name_ = node->get_parameter("drive_joint_name").as_string();
  flywheel_joint_name_ = node->get_parameter("flywheel_joint_name").as_string();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn BalanceController::on_configure(const rclcpp_lifecycle::State &) {
  auto node = get_node();
  drive_joint_name_ = node->get_parameter("drive_joint_name").as_string();
  flywheel_joint_name_ = node->get_parameter("flywheel_joint_name").as_string();
  angle_pid_.kp = node->get_parameter("angle_kp").as_double();
  angle_pid_.ki = node->get_parameter("angle_ki").as_double();
  angle_pid_.kd = node->get_parameter("angle_kd").as_double();
  angle_vel_pid_.kp = node->get_parameter("angle_v_kp").as_double();
  angle_vel_pid_.ki = node->get_parameter("angle_v_ki").as_double();
  angle_vel_pid_.kd = node->get_parameter("angle_v_kd").as_double();
  flywheel_zero_pid_.kp = node->get_parameter("flywheel_speed_kp").as_double();
  flywheel_zero_pid_.ki = node->get_parameter("flywheel_speed_ki").as_double();
  flywheel_zero_integral_limit_min_ = node->get_parameter("flywheel_speed_integral_limit_min").as_double();
  flywheel_zero_integral_limit_max_ = node->get_parameter("flywheel_speed_integral_limit_max").as_double();
  flywheel_speed_limit_ = node->get_parameter("flywheel_speed_limit").as_double();
  flywheel_accel_limit_ = node->get_parameter("flywheel_accel_limit").as_double();
  roll_diff_limit_ = node->get_parameter("roll_diff_limit").as_double();
  machine_middle_angle_init_ = node->get_parameter("machine_middle_angle_init").as_double();
  bike_turn_scale_deg_ = node->get_parameter("bike_turn_scale_deg").as_double();
  bike_speed_scale_deg_ = node->get_parameter("bike_speed_scale_deg").as_double();
  middle_angle_recitfy_limit_deg_ = node->get_parameter("middle_angle_recitfy_limit_deg").as_double();
  servo_center_deg_ = node->get_parameter("servo_center_deg").as_double();
  servo_middle_range_ = node->get_parameter("servo_middle_range").as_double();
  log_enable_ = node->get_parameter("log_enable").as_bool();
  log_hz_ = node->get_parameter("log_hz").as_double();
  machine_middle_angle_ = machine_middle_angle_init_;

  // Subscribers
  imu_sub_ = node->create_subscription<sensor_msgs::msg::Imu>(
      "imu/data", rclcpp::SensorDataQoS(),
      std::bind(&BalanceController::imuCallback, this, std::placeholders::_1));
  servo_angle_sub_ = node->create_subscription<std_msgs::msg::Float64>(
      "servo_angle", 10, std::bind(&BalanceController::servoAngleCallback, this, std::placeholders::_1));

  RCLCPP_INFO(node->get_logger(), "BalanceController configured.");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn BalanceController::on_activate(const rclcpp_lifecycle::State &) {
  // Reset integrals
  angle_pid_.integral = 0.0;
  angle_vel_pid_.integral = 0.0;
  flywheel_zero_pid_.integral = 0.0;
  last_flywheel_command_ = 0.0;
  loop_counter_ = 0;
  RCLCPP_INFO(get_node()->get_logger(), "BalanceController activated.");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn BalanceController::on_deactivate(const rclcpp_lifecycle::State &) {
  // Zero commands on deactivate
  for (auto & cmd : command_interfaces_) {
    cmd.set_value(0.0);
  }
  RCLCPP_INFO(get_node()->get_logger(), "BalanceController deactivated.");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration BalanceController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  cfg.names = {
    flywheel_joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY,
    drive_joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY
  };
  return cfg;
}

controller_interface::InterfaceConfiguration BalanceController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  cfg.names = {
    flywheel_joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY,
    drive_joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY
  };
  return cfg;
}

void BalanceController::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  last_imu_stamp_ = msg->header.stamp;
  // Convert quaternion to roll (rad -> deg)
  tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  last_roll_deg_ = roll * 180.0 / M_PI;
  // Use angular velocity x as roll gyro (assumed deg/s already, else convert if needed)
  last_roll_gyro_degps_ = msg->angular_velocity.x; // if in rad/s: *180/M_PI, but assume deg/s per original parser
}

void BalanceController::servoAngleCallback(const std_msgs::msg::Float64::SharedPtr msg) {
  last_servo_angle_ = msg->data;
}

double BalanceController::computeAnglePID(double current_angle, double target_angle, double current_gyro) {
  double error = current_angle - target_angle;
  angle_pid_.integral += error;
  angle_pid_.integral = clamp(angle_pid_.integral, -angle_integral_limit_, angle_integral_limit_);
  return angle_pid_.kp * error + angle_pid_.ki * angle_pid_.integral + angle_pid_.kd * current_gyro;
}

double BalanceController::computeAngularVelocityPID(double current_gyro, double target_gyro) {
  double error = current_gyro - target_gyro;
  angle_vel_pid_.integral += error;
  angle_vel_pid_.integral = clamp(angle_vel_pid_.integral, -10000.0, 10000.0);
  double derivative = error - angle_vel_pid_.last_error;
  double out = angle_vel_pid_.kp * error + angle_vel_pid_.ki * angle_vel_pid_.integral + angle_vel_pid_.kd * derivative;
  angle_vel_pid_.last_error = error;
  return out;
}

double BalanceController::computeFlywheelZeroPID(double current_speed) {
  double target_speed = 0.0;
  double error = current_speed - target_speed;
  flywheel_zero_pid_.integral += error;
  flywheel_zero_pid_.integral = clamp(flywheel_zero_pid_.integral, flywheel_zero_integral_limit_min_, flywheel_zero_integral_limit_max_);
  // match original scaling
  return error * (flywheel_zero_pid_.kp / 10.0) + flywheel_zero_pid_.integral * (flywheel_zero_pid_.ki / 1000.0);
}

controller_interface::return_type BalanceController::update(const rclcpp::Time & time, const rclcpp::Duration & period) {
  ++loop_counter_;
  // IMU timeout safety (1s)
  if (!last_imu_stamp_.nanoseconds()) {
    return controller_interface::return_type::OK;
  }
  // Use provided time instead of creating a new one for deterministic behavior
  if ((time - last_imu_stamp_).seconds() > 1.0) {
    // stop outputs
    for (auto & cmd : command_interfaces_) cmd.set_value(0.0);
    return controller_interface::return_type::OK;
  }

  // Read flywheel & drive speeds from state interfaces
  double flywheel_speed = state_interfaces_[0].get_value();
  double drive_speed = state_interfaces_[1].get_value();
  last_drive_speed_ = drive_speed;

  // Slow loop every ~160ms (assuming 500Hz -> 80 cycles, approximate with modulo 80)
  static double pwm_accel = 0.0;
  bool slow_loop = (loop_counter_ % 80) == 0;
  if (slow_loop) {
    pwm_accel = computeFlywheelZeroPID(flywheel_speed);
  }

  // Turn & speed bias every slow loop
  static double turn_bias = 0.0;
  static double speed_bias = 0.0;
  if (slow_loop) {
    double speed_bias_direction = 0.0;
    if (std::abs(last_servo_angle_ - servo_center_deg_) < servo_middle_range_) {
      speed_bias_direction = 0.0;
    } else if (last_servo_angle_ > servo_center_deg_) {
      speed_bias_direction = 1.0;
    } else {
      speed_bias_direction = -1.0;
    }
    speed_bias = std::abs(drive_speed) * std::abs(drive_speed) * bike_speed_scale_deg_ * speed_bias_direction;
    turn_bias = (last_servo_angle_ - servo_center_deg_) * bike_turn_scale_deg_;
    last_speed_bias_ = speed_bias;
  }

  // Middle loop every ~30ms (mod 15)
  static double pwm_x = 0.0;
  bool middle_loop = (loop_counter_ % 15) == 0;
  if (middle_loop) {
    double dynamic_zero = machine_middle_angle_ + turn_bias + speed_bias + pwm_accel;
    dynamic_zero = clamp(dynamic_zero,
                         machine_middle_angle_ - middle_angle_recitfy_limit_deg_,
                         machine_middle_angle_ + middle_angle_recitfy_limit_deg_);
    pwm_x = computeAnglePID(last_roll_deg_, dynamic_zero, last_roll_gyro_degps_);
    last_dynamic_zero_ = dynamic_zero;
  }

  // Inner loop every cycle (angular velocity PID)
  double final_flywheel_speed = computeAngularVelocityPID(last_roll_gyro_degps_, pwm_x);
  // Clamp speed & acceleration
  final_flywheel_speed = clamp(final_flywheel_speed, -flywheel_speed_limit_, flywheel_speed_limit_);
  final_flywheel_speed = clamp(final_flywheel_speed,
                               last_flywheel_command_ - flywheel_accel_limit_,
                               last_flywheel_command_ + flywheel_accel_limit_);

  // Safety: angle deviation > 3 deg
  if (std::abs(last_roll_deg_ - machine_middle_angle_) > 3.0) {
    final_flywheel_speed = 0.0;
    angle_pid_.integral = 0.0;
    angle_vel_pid_.integral = 0.0;
    flywheel_zero_pid_.integral = 0.0;
  }

  // Write commands: flywheel first, drive left unchanged (0 for now)
  command_interfaces_[0].set_value(final_flywheel_speed);
  command_interfaces_[1].set_value(0.0); // march velocity integration TBD
  last_flywheel_command_ = final_flywheel_speed;
  last_final_flywheel_speed_ = final_flywheel_speed;

  // Throttled real-time style log (controlled by params)
  if (log_enable_ && log_hz_ > 0.0) {
    const uint64_t period_ms = static_cast<uint64_t>(std::max(1.0, 1000.0 / log_hz_));
    std::ostringstream ss;
    ss.setf(std::ios::fixed);
    ss << "steer=" << std::setw(4) << std::setprecision(1) << last_servo_angle_;
    ss << ", roll=" << std::setw(7) << std::setprecision(2) << last_roll_deg_ << "\u00B0";
    ss << ", dy_zero=" << std::setw(5) << std::setprecision(2) << last_dynamic_zero_ << "\u00B0";
    ss << ", spd_bias=" << std::setw(7) << std::setprecision(4) << last_speed_bias_;
    ss << ", tgt_vel=" << std::setw(7) << std::setprecision(2) << last_final_flywheel_speed_;
    ss << ", fw_s=" << std::setw(6) << std::setprecision(1) << flywheel_zero_pid_.integral;
    RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), period_ms, "%s", ss.str().c_str());
  }

  return controller_interface::return_type::OK;
}

} // namespace balance_controller_ros2
