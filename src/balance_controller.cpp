#include "balance_controller/balance_controller.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <algorithm>
#include <memory>
#include <string>
#include <utility>

using namespace std::chrono_literals;  // 新增

namespace balance_controller
{

BalanceController::BalanceController() : controller_interface::ControllerInterface() {}

controller_interface::InterfaceConfiguration
BalanceController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint_name : joint_names_)
  {
    config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  return config;
}

controller_interface::InterfaceConfiguration
BalanceController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint_name : joint_names_)
  {
    config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  return config;
}

controller_interface::return_type BalanceController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // 控制循环逻辑，与原control_loop类似
  if (!is_imu_ok_) return controller_interface::return_type::OK;

  if ((time - last_imu_time_).seconds() > safety_timeout_s_)
  {
    if (is_imu_ok_)
    {
      RCLCPP_WARN(get_node()->get_logger(), "IMU timeout, stopping.");
      stop_all_motors();
      is_imu_ok_ = false;
    }
    return controller_interface::return_type::OK;
  }

  if (std::abs(current_roll_) > max_roll_angle_for_safety_)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Roll %.2f exceeds limit %.2f, stopping.",
                 current_roll_, max_roll_angle_for_safety_);
    stop_all_motors();
    angle_pid_.integral = 0;
    angle_velocity_pid_.integral = 0;
    flywheel_speed_pid_.integral = 0;
    return controller_interface::return_type::OK;
  }

  static int loop_counter = 0;
  loop_counter++;

  static double turn_bias = 0.0;
  static double speed_bias = 0.0;
  if (loop_counter % 80 == 0)
  {
    // 读取当前驱动速度
    auto drive_state = std::find_if(
      state_interfaces_.begin(), state_interfaces_.end(),
      [this](const auto & interface) {
        return interface.get_name() == drive_joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY;
      });
    if (drive_state != state_interfaces_.end())
    {
      current_drive_speed_ = drive_state->get_value();
    }

    double speed_bias_direction = 0.0;
    if (std::abs(current_steer_angle_ - servo_center_deg_) > servo_middle_range_)
    {
      speed_bias_direction = (current_steer_angle_ > servo_center_deg_) ? -1.0 : 1.0;
    }
    speed_bias =
        std::abs(current_drive_speed_) * std::abs(current_drive_speed_) *
        bike_speed_scale_deg_ * speed_bias_direction;

    turn_bias =
        -(current_steer_angle_ - servo_center_deg_) * bike_turn_scale_deg_;
  }

  static double flywheel_zero_correction = 0.0;
  if (loop_counter % 80 == 1)
  {
    // 读取飞轮速度并计算校正
    auto flywheel_state = std::find_if(
      state_interfaces_.begin(), state_interfaces_.end(),
      [this](const auto & interface) {
        return interface.get_name() == flywheel_joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY;
      });
    if (flywheel_state != state_interfaces_.end())
    {
      flywheel_zero_correction = flywheel_speed_pid_.calculate(0.0, flywheel_state->get_value());
    }
    else
    {
      flywheel_zero_correction = 0.0;
    }
  }

  static double target_ang_vel = 0.0;
  if (loop_counter % 15 == 0)
  {
    double dynamic_zero =
        machine_middle_angle_init_ + turn_bias + speed_bias + flywheel_zero_correction;
    dynamic_zero =
        std::clamp(dynamic_zero,
                   machine_middle_angle_init_ - middle_angle_recitfy_limit_deg_,
                   machine_middle_angle_init_ + middle_angle_recitfy_limit_deg_);

    target_ang_vel =
        angle_pid_.calculate(dynamic_zero, current_roll_, -current_roll_velocity_);
  }

  double final_flywheel_speed =
      angle_velocity_pid_.calculate(target_ang_vel, -current_roll_velocity_);

  final_flywheel_speed =
      std::clamp(final_flywheel_speed, -flywheel_speed_limit_, flywheel_speed_limit_);

  double accel_step = flywheel_accel_limit_ * period.seconds();
  final_flywheel_speed = std::clamp(final_flywheel_speed,
                                    last_flywheel_vel_cmd_ - accel_step,
                                    last_flywheel_vel_cmd_ + accel_step);
  last_flywheel_vel_cmd_ = final_flywheel_speed;

  // 设置飞轮速度命令
  auto flywheel_command = std::find_if(
    command_interfaces_.begin(), command_interfaces_.end(),
    [this](const auto & interface) {
      return interface.get_name() == flywheel_joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY;
    });
  if (flywheel_command != command_interfaces_.end())
  {
    flywheel_command->set_value(final_flywheel_speed);
  }

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn BalanceController::on_init()
{
  try
  {
    // 声明参数
    auto_declare<std::string>("params_file", "");
    auto_declare<std::string>("drive_joint_name", "drive_joint");
    auto_declare<std::string>("flywheel_joint_name", "flywheel_joint");
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn BalanceController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  drive_joint_name_ = get_node()->get_parameter("drive_joint_name").as_string();
  flywheel_joint_name_ = get_node()->get_parameter("flywheel_joint_name").as_string();
  joint_names_ = {drive_joint_name_, flywheel_joint_name_};

  initialize_parameters();

  // 创建订阅者
  imu_sub_ = get_node()->create_subscription<sensor_msgs::msg::Imu>(
      "imu/data", 50,
      std::bind(&BalanceController::imu_callback, this, std::placeholders::_1));

  servo_angle_sub_ = get_node()->create_subscription<std_msgs::msg::Float64>(
      "servo_angle", 10,
      std::bind(&BalanceController::servo_angle_callback, this, std::placeholders::_1));

  cmd_vel_sub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      std::bind(&BalanceController::cmd_vel_callback, this, std::placeholders::_1));

  servo_set_angle_client_ =
      get_node()->create_client<servo_rs485_ros2::srv::SetAngle>("set_angle");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn BalanceController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  last_imu_time_ = get_node()->now();
  is_imu_ok_ = false;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn BalanceController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  stop_all_motors();
  return controller_interface::CallbackReturn::SUCCESS;
}

void BalanceController::initialize_parameters()
{
  auto pkg_path = ament_index_cpp::get_package_share_directory("balance_controller_ros2");
  params_file_ = get_node()->get_parameter("params_file").as_string();
  if (params_file_.empty())
  {
    params_file_ = pkg_path + "/config/params.yaml";
  }

  YAML::Node root;
  try
  {
    root = YAML::LoadFile(params_file_);
  }
  catch (const std::exception &e)
  {
    RCLCPP_FATAL(get_node()->get_logger(), "Failed to load %s: %s",
                 params_file_.c_str(), e.what());
    throw;
  }

  auto params_node = root["balance_controller"];

  angle_pid_.kp = load_param<double>(params_node, "angle_kp", 350.0);
  angle_pid_.ki = load_param<double>(params_node, "angle_ki", 0.0);
  angle_pid_.kd = load_param<double>(params_node, "angle_kd", 0.7);
  double angle_int_min = load_param<double>(params_node, "angle_integral_min", -400.0);
  double angle_int_max = load_param<double>(params_node, "angle_integral_max", 400.0);
  angle_pid_.integral_min = angle_int_min;
  angle_pid_.integral_max = angle_int_max;

  angle_velocity_pid_.kp = load_param<double>(params_node, "angle_v_kp", 0.45);
  angle_velocity_pid_.ki = load_param<double>(params_node, "angle_v_ki", 0.0);
  angle_velocity_pid_.kd = load_param<double>(params_node, "angle_v_kd", 0.1);
  double ang_vel_int_min = load_param<double>(params_node, "angle_v_integral_min", -1000.0);
  double ang_vel_int_max = load_param<double>(params_node, "angle_v_integral_max", 1000.0);
  angle_velocity_pid_.integral_min = ang_vel_int_min;
  angle_velocity_pid_.integral_max = ang_vel_int_max;

  flywheel_speed_pid_.kp = load_param<double>(params_node, "flywheel_speed_kp", 0.1);
  flywheel_speed_pid_.ki = load_param<double>(params_node, "flywheel_speed_ki", 0.45);
  double flywheel_int_min = load_param<double>(params_node, "flywheel_integral_min", -400.0);
  double flywheel_int_max = load_param<double>(params_node, "flywheel_integral_max", 400.0);
  flywheel_speed_pid_.integral_min = flywheel_int_min;
  flywheel_speed_pid_.integral_max = flywheel_int_max;

  machine_middle_angle_init_ = load_param<double>(params_node, "machine_middle_angle_init", -1.77);
  bike_turn_scale_deg_ = load_param<double>(params_node, "bike_turn_scale_deg", 0.057);
  bike_speed_scale_deg_ = load_param<double>(params_node, "bike_speed_scale_deg", 0.19);
  middle_angle_recitfy_limit_deg_ = load_param<double>(params_node, "middle_angle_recitfy_limit_deg", 3.0);

  servo_center_deg_ = load_param<double>(params_node, "servo_center_deg", 8.0);
  servo_middle_range_ = load_param<double>(params_node, "servo_middle_range_deg", 2.0);

  march_velocity_ = load_param<double>(params_node, "march_velocity", 0.4);
  drive_accel_ = load_param<double>(params_node, "drive_accel", 0.1);
  flywheel_speed_limit_ = load_param<double>(params_node, "flywheel_speed_limit", 20.0);
  flywheel_accel_limit_ = load_param<double>(params_node, "flywheel_accel_limit", 15.0);

  safety_timeout_s_ = load_param<double>(params_node, "safety_timeout_s", 1.0);
  max_roll_angle_for_safety_ = load_param<double>(params_node, "max_roll_angle_for_safety", 15.0);
}

void BalanceController::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  current_roll_ = get_roll_from_quaternion(msg->orientation);
  current_roll_velocity_ = msg->angular_velocity.y;
  last_imu_time_ = get_node()->now();
  is_imu_ok_ = true;
}

void BalanceController::servo_angle_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
  current_steer_angle_ = msg->data;
}

void BalanceController::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  float target_vel = static_cast<float>(msg->linear.x * march_velocity_);
  set_drive_velocity(target_vel);
}

void BalanceController::set_drive_velocity(float velocity)
{
  auto drive_command = std::find_if(
    command_interfaces_.begin(), command_interfaces_.end(),
    [this](const auto & interface) {
      return interface.get_name() == drive_joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY;
    });
  if (drive_command != command_interfaces_.end())
  {
    drive_command->set_value(velocity);
  }
}

void BalanceController::stop_all_motors()
{
  set_drive_velocity(0.0f);
  auto flywheel_command = std::find_if(
    command_interfaces_.begin(), command_interfaces_.end(),
    [this](const auto & interface) {
      return interface.get_name() == flywheel_joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY;
    });
  if (flywheel_command != command_interfaces_.end())
  {
    flywheel_command->set_value(0.0);
  }
}

double BalanceController::get_roll_from_quaternion(const geometry_msgs::msg::Quaternion &q)
{
  tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
  tf2::Matrix3x3 m(tf_q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return roll * 180.0 / M_PI;
}

template<typename T>
T BalanceController::load_param(const YAML::Node &node, const std::string &key, const T &fallback)
{
  if (node && node[key])
  {
    try
    {
      return node[key].as<T>();
    }
    catch (const YAML::Exception &e)
    {
      RCLCPP_WARN(get_node()->get_logger(), "Param %s parse error: %s. Using default.",
                  key.c_str(), e.what());
    }
  }
  return fallback;
}

}  // namespace balance_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  balance_controller::BalanceController,
  controller_interface::ControllerInterface)
