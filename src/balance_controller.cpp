#include "balance_controller.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <poll.h>
#include <sstream>

using namespace std::chrono_literals;  // 新增

namespace {
inline std::string format_float(float value) {
  std::ostringstream oss;
  oss.setf(std::ios::fixed);
  oss.precision(6);
  oss << value;
  return oss.str();
}
constexpr int AXIS_STATE_CLOSED_LOOP = 8;
}  // namespace

// ---------------- OdriveSerial ----------------
OdriveSerial::OdriveSerial(const std::string &port, int baud)
    : port_(port), baud_(baud) {
  fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (fd_ < 0) {
    perror("open ODrive serial");
    return;
  }

  termios tty{};
  if (tcgetattr(fd_, &tty) != 0) {
    perror("tcgetattr");
    ::close(fd_);
    fd_ = -1;
    return;
  }

  cfmakeraw(&tty);
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  speed_t speed = B115200;
  switch (baud) {
    case 230400: speed = B230400; break;
    case 460800: speed = B460800; break;
    case 921600: speed = B921600; break;
    default: break;
  }
  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 1;

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    perror("tcsetattr");
    ::close(fd_);
    fd_ = -1;
    return;
  }
  tcflush(fd_, TCIOFLUSH);
  ready_ = true;
}

OdriveSerial::~OdriveSerial() {
  if (fd_ >= 0) {
    ::close(fd_);
  }
}

bool OdriveSerial::write_line(const std::string &cmd) {
  if (!ready_) return false;
  std::lock_guard<std::mutex> lk(io_mutex_);
  std::string out = cmd;
  if (out.empty() || out.back() != '\n') out.push_back('\n');
  ssize_t written = ::write(fd_, out.c_str(), out.size());
  return written == static_cast<ssize_t>(out.size());
}

bool OdriveSerial::read_response(std::string &out, int timeout_ms) {
  out.clear();
  if (!ready_) return false;

  std::lock_guard<std::mutex> lk(io_mutex_);
  struct pollfd pfd { fd_, POLLIN, 0 };
  int ret = ::poll(&pfd, 1, timeout_ms);
  if (ret <= 0) return false;

  char buf[128];
  ssize_t n = ::read(fd_, buf, sizeof(buf));
  if (n <= 0) return false;
  out.assign(buf, buf + n);
  return true;
}

bool OdriveSerial::set_state(int axis, int state) {
  return write_line("w axis" + std::to_string(axis) +
                    ".requested_state " + std::to_string(state));
}

bool OdriveSerial::clear_errors(int axis) {
  return write_line("w axis" + std::to_string(axis) + ".error 0");
}

bool OdriveSerial::set_velocity(int axis, float vel) {
  return write_line("w axis" + std::to_string(axis) +
                    ".controller.input_vel " + format_float(vel));
}

bool OdriveSerial::get_velocity(int axis, float &vel_out) {
  if (!write_line("r axis" + std::to_string(axis) + ".encoder.vel_estimate"))
    return false;
  std::string resp;
  if (!read_response(resp, 25))
    return false;
  try {
    vel_out = std::stof(resp);
    return true;
  } catch (...) {
    return false;
  }
}

// ---------------- BikeControllerNode ----------------
BikeControllerNode::BikeControllerNode()
     : Node("balance_controller_node"), last_imu_time_(this->now()) {
   RCLCPP_INFO(this->get_logger(), "Balance Controller Node starting...");
   initialize_parameters();
   initialize_odrive();

   imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
       "imu/data", 50,
       std::bind(&BikeControllerNode::imu_callback, this, std::placeholders::_1));

   servo_angle_sub_ = this->create_subscription<std_msgs::msg::Float64>(
       "servo_angle", 10,
       std::bind(&BikeControllerNode::servo_angle_callback, this, std::placeholders::_1));

   cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
       "cmd_vel", 10,
       std::bind(&BikeControllerNode::cmd_vel_callback, this, std::placeholders::_1));

   servo_set_angle_client_ =
       this->create_client<servo_rs485_ros2::srv::SetAngle>("set_angle");
   while (!servo_set_angle_client_->wait_for_service(1s)) {
     if (!rclcpp::ok()) {
       RCLCPP_ERROR(this->get_logger(), "Interrupted waiting for servo service.");
       return;
     }
     RCLCPP_INFO(this->get_logger(), "Waiting for servo service...");
   }

   control_timer_ = this->create_wall_timer(
       2ms, std::bind(&BikeControllerNode::control_loop, this));
   RCLCPP_INFO(this->get_logger(), "Balance Controller Node initialized.");
}

BikeControllerNode::~BikeControllerNode() {
  RCLCPP_INFO(this->get_logger(), "Shutting down, stopping motors.");
  stop_all_motors();
}

template<typename T>
T BikeControllerNode::load_param(const YAML::Node &node,
                                 const std::string &key,
                                 const T &fallback) {
  if (node && node[key]) {
    try {
      return node[key].as<T>();
    } catch (const YAML::Exception &e) {
      RCLCPP_WARN(this->get_logger(), "Param %s parse error: %s. Using default.",
                  key.c_str(), e.what());
    }
  }
  return fallback;
}

void BikeControllerNode::initialize_parameters() {
  auto pkg_path =
      ament_index_cpp::get_package_share_directory("balance_controller_ros2");
  this->declare_parameter<std::string>("params_file",
                                       pkg_path + "/config/params.yaml");
  params_file_ = this->get_parameter("params_file").as_string();

  YAML::Node root;
  try {
    root = YAML::LoadFile(params_file_);
  } catch (const std::exception &e) {
    RCLCPP_FATAL(this->get_logger(), "Failed to load %s: %s",
                 params_file_.c_str(), e.what());
    throw;
  }

  auto params_node = root["balance_controller_node"]["ros__parameters"];

  axis_drive_id_ = load_param<int>(params_node, "axis_drive_id", 0);
  axis_flywheel_id_ = load_param<int>(params_node, "axis_flywheel_id", 1);
  odrive_port_ = load_param<std::string>(params_node, "odrive_port", "/dev/ttyACM0");
  odrive_baud_ = load_param<int>(params_node, "odrive_baud", 115200);

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

  machine_middle_angle_init_ = load_param<double>(params_node, "machine_middle_angle_init", -1.7);
  bike_turn_scale_deg_ = load_param<double>(params_node, "bike_turn_scale_deg", 0.057);
  bike_speed_scale_deg_ = load_param<double>(params_node, "bike_speed_scale_deg", 0.19);
  middle_angle_recitfy_limit_deg_ = load_param<double>(params_node, "middle_angle_recitfy_limit_deg", 3.0);

  servo_center_deg_ = load_param<double>(params_node, "servo_center_deg", 8.0);
  servo_middle_range_ = load_param<double>(params_node, "servo_middle_range", 2.0);

  march_velocity_ = load_param<double>(params_node, "march_velocity", 0.4);
  drive_accel_ = load_param<double>(params_node, "drive_accel", 0.1);
  flywheel_speed_limit_ = load_param<double>(params_node, "flywheel_speed_limit", 20.0);
  flywheel_accel_limit_ = load_param<double>(params_node, "flywheel_accel_limit", 5.0);

  safety_timeout_s_ = load_param<double>(params_node, "safety_timeout_s", 1.0);
  max_roll_angle_for_safety_ = load_param<double>(params_node, "max_roll_angle_for_safety", 15.0);
}

void BikeControllerNode::initialize_odrive() {
  odrive_serial_ = std::make_unique<OdriveSerial>(odrive_port_, odrive_baud_);
  if (!odrive_serial_ || !odrive_serial_->is_ready()) {
    RCLCPP_FATAL(this->get_logger(), "Failed to open ODrive serial %s",
                 odrive_port_.c_str());
    throw std::runtime_error("ODrive serial unavailable");
  }

  odrive_serial_->clear_errors(axis_drive_id_);
  odrive_serial_->clear_errors(axis_flywheel_id_);
  odrive_serial_->set_state(axis_drive_id_, AXIS_STATE_CLOSED_LOOP);
  odrive_serial_->set_state(axis_flywheel_id_, AXIS_STATE_CLOSED_LOOP);
  RCLCPP_INFO(this->get_logger(), "ODrive serial ready.");
}

void BikeControllerNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  current_roll_ = get_roll_from_quaternion(msg->orientation);
  current_roll_velocity_ = msg->angular_velocity.y;
  last_imu_time_ = this->now();
  is_imu_ok_ = true;
}

void BikeControllerNode::servo_angle_callback(
    const std_msgs::msg::Float64::SharedPtr msg) {
  current_steer_angle_ = msg->data;
}

void BikeControllerNode::cmd_vel_callback(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
  float target_vel = static_cast<float>(msg->linear.x * march_velocity_);
  set_drive_velocity(target_vel);
}

void BikeControllerNode::control_loop() {
  if (!odrive_serial_ || !odrive_serial_->is_ready()) return;

  static int loop_counter = 0;
  loop_counter++;

  if ((this->now() - last_imu_time_).seconds() > safety_timeout_s_) {
    if (is_imu_ok_) {
      RCLCPP_WARN(this->get_logger(), "IMU timeout, stopping.");
      stop_all_motors();
      is_imu_ok_ = false;
    }
    return;
  }

  if (std::abs(current_roll_) > max_roll_angle_for_safety_) {
    RCLCPP_ERROR(this->get_logger(), "Roll %.2f exceeds limit %.2f, stopping.",
                 current_roll_, max_roll_angle_for_safety_);
    stop_all_motors();
    angle_pid_.integral = 0;
    angle_velocity_pid_.integral = 0;
    flywheel_speed_pid_.integral = 0;
    return;
  }

  static double turn_bias = 0.0;
  static double speed_bias = 0.0;
  if (loop_counter % 80 == 0) {
    float drive_vel = 0.0f;
    if (odrive_serial_->get_velocity(axis_drive_id_, drive_vel))
      current_drive_speed_ = drive_vel;
    else
      current_drive_speed_ = 0.0;

    double speed_bias_direction = 0.0;
    if (std::abs(current_steer_angle_ - servo_center_deg_) > servo_middle_range_) {
      speed_bias_direction =
          (current_steer_angle_ > servo_center_deg_) ? -1.0 : 1.0;
    }
    speed_bias =
        std::abs(current_drive_speed_) * std::abs(current_drive_speed_) *
        bike_speed_scale_deg_ * speed_bias_direction;

    turn_bias =
        -(current_steer_angle_ - servo_center_deg_) * bike_turn_scale_deg_;
  }

  static double flywheel_zero_correction = 0.0;
  if (loop_counter % 80 == 1) {
    float flywheel_vel = 0.0f;
    if (odrive_serial_->get_velocity(axis_flywheel_id_, flywheel_vel))
      flywheel_zero_correction = flywheel_speed_pid_.calculate(0.0, flywheel_vel);
    else
      flywheel_zero_correction = 0.0;
  }

  static double target_ang_vel = 0.0;
  if (loop_counter % 15 == 0) {
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

  double accel_step = flywheel_accel_limit_ * 0.002;
  final_flywheel_speed = std::clamp(final_flywheel_speed,
                                    last_flywheel_vel_cmd_ - accel_step,
                                    last_flywheel_vel_cmd_ + accel_step);
  last_flywheel_vel_cmd_ = final_flywheel_speed;

  odrive_serial_->set_velocity(axis_flywheel_id_,
                               static_cast<float>(final_flywheel_speed));
}

void BikeControllerNode::set_drive_velocity(float velocity) {
  if (!odrive_serial_ || !odrive_serial_->is_ready()) return;
  odrive_serial_->set_velocity(axis_drive_id_, velocity);
}

void BikeControllerNode::stop_all_motors() {
  if (!odrive_serial_ || !odrive_serial_->is_ready()) return;
  odrive_serial_->set_velocity(axis_drive_id_, 0.0f);
  odrive_serial_->set_velocity(axis_flywheel_id_, 0.0f);
}

double BikeControllerNode::get_roll_from_quaternion(
    const geometry_msgs::msg::Quaternion &q) {
  tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
  tf2::Matrix3x3 m(tf_q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return roll * 180.0 / M_PI;
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<BikeControllerNode>());
  } catch (const std::exception &e) {
    std::cerr << "Node aborted: " << e.what() << std::endl;
  }
  rclcpp::shutdown();
  return 0;
}
