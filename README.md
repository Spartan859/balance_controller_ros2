# Balance Controller ROS2

## Overview
The Balance Controller ROS2 package implements a bicycle self-balancing control system using ROS2 and C++. It utilizes sensor data from an Inertial Measurement Unit (IMU) to maintain balance and control the steering through a servo motor.

## Package Structure
The package consists of the following files and directories:

- **CMakeLists.txt**: Configures the build process for the ROS2 package, specifying dependencies and build options.
- **package.xml**: Contains metadata about the package, including its name, version, description, maintainers, and dependencies.
- **README.md**: Provides documentation for the project, including build and run instructions.
- **launch/**: Contains the launch file `balance_controller.launch.py` to start the balance controller node.
- **config/**: Contains the configuration file `params.yaml` for setting parameters like PID gains and servo settings.
- **include/balance_controller_ros2/**: Contains header files for the `BalanceController` and `PIDController` classes.
- **src/**: Contains the implementation files for the balance controller node, balance controller logic, and PID controller.

## Building the Package
To build the package, navigate to the root of the workspace and run the following commands:

```bash
colcon build --packages-select balance_controller_ros2
```

## Running the Package
After building the package, you can run the balance controller node using the provided launch file:

```bash
ros2 launch balance_controller_ros2 balance_controller.launch.py
```

## Configuration
The configuration parameters for the balance controller can be set in the `config/params.yaml` file. This includes PID controller gains, servo settings, and IMU settings.

## Functionality
The balance controller node subscribes to IMU data, computes the necessary adjustments to maintain balance, and publishes commands to the servo motor. The PID controller is used to calculate the control outputs based on the errors between the desired and actual states.

## Dependencies
Ensure that the following dependencies are installed in your ROS2 environment:

- rclcpp
- std_msgs
- sensor_msgs

## Maintainers
For any issues or contributions, please contact the maintainers of this package.