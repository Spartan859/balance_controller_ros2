from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro


def launch_setup(context, *args, **kwargs):
    use_mock_value = LaunchConfiguration('use_mock_hardware').perform(context)
    servo_port = LaunchConfiguration('servo_port').perform(context)
    servo_id = LaunchConfiguration('servo_id').perform(context)
    imu_port = LaunchConfiguration('imu_port').perform(context)
    imu_baud = LaunchConfiguration('imu_baud').perform(context)
    servo_publish_interval_ms = LaunchConfiguration('servo_publish_interval_ms').perform(context)
    pkg_share = get_package_share_directory('balance_controller_ros2')
    odrive_xacro = os.path.join(pkg_share, 'urdf', 'bike_robot.ros2_control.xacro')
    mock_xacro = os.path.join(pkg_share, 'urdf', 'bike_robot_mock.ros2_control.xacro')
    xacro_file = mock_xacro if use_mock_value.lower() == 'true' else odrive_xacro
    doc = xacro.process_file(xacro_file)
    robot_description = {'robot_description': doc.toxml()}
    controllers_path = os.path.join(pkg_share, 'config', 'controllers.yaml')

    nodes = [
        # Servo node (must start before controller to provide steering feedback)
        Node(
            package='servo_rs485_ros2',
            executable='servo_node',
            name='servo_rs485_node',
            output='screen',
            parameters=[
                {'port': servo_port},
                {'servo_id': int(servo_id)},
                {'baudrate': 115200},
                {'publish_interval_ms': int(servo_publish_interval_ms)},
            ]
        ),
        # IMU nodes (serial reader + optional processing node)
        Node(
            package='imu_ros2',
            executable='imu_serial_node.py',
            name='imu_serial_node',
            output='screen',
            parameters=[
                {'port': imu_port},
                {'baud': int(imu_baud)}
            ]
        ),
        Node(
            package='imu_ros2',
            executable='imu_node',
            name='imu_node',
            output='screen'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[robot_description]
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[robot_description, controllers_path],
            output='screen'
        ),
        Node(
            package='controller_manager', executable='spawner',
            arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
            output='screen'
        ),
        Node(
            package='controller_manager', executable='spawner',
            arguments=['balance_controller', '-c', '/controller_manager'],
            output='screen'
        ),
    ]
    return nodes


def generate_launch_description():
    use_mock_arg = DeclareLaunchArgument('use_mock_hardware', default_value='false',
                                         description='使用 mock ros2_control 硬件而非 ODrive')
    servo_port_arg = DeclareLaunchArgument('servo_port', default_value='/dev/ttyUSB1',
                                           description='RS485 舵机串口 (Linux: /dev/ttyUSB*, Windows: COM*)')
    servo_id_arg = DeclareLaunchArgument('servo_id', default_value='1',
                                         description='舵机 ID')
    imu_port_arg = DeclareLaunchArgument('imu_port', default_value='/dev/ttyUSB0',
                                         description='IMU 串口设备')
    imu_baud_arg = DeclareLaunchArgument('imu_baud', default_value='115200',
                                         description='IMU 波特率')
    servo_publish_interval_arg = DeclareLaunchArgument('servo_publish_interval_ms', default_value='30',
                                                      description='舵机角度发布间隔 (毫秒)')
    return LaunchDescription([
        use_mock_arg,
        servo_port_arg,
        servo_id_arg,
        imu_port_arg,
        imu_baud_arg,
        servo_publish_interval_arg,
        OpaqueFunction(function=launch_setup)
    ])
