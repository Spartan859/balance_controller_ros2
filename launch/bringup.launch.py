from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os
import xacro


def generate_launch_description():
    use_mock = LaunchConfiguration('use_mock_hardware')
    use_mock_arg = DeclareLaunchArgument('use_mock_hardware', default_value='false')

    pkg_share = get_package_share_directory('balance_controller_ros2')
    odrive_xacro = os.path.join(pkg_share, 'urdf', 'bike_robot.ros2_control.xacro')
    mock_xacro = os.path.join(pkg_share, 'urdf', 'bike_robot_mock.ros2_control.xacro')

    xacro_file = mock_xacro if use_mock.perform({'use_mock_hardware': 'true'}) == 'true' else odrive_xacro
    doc = xacro.process_file(xacro_file)
    robot_description = {'robot_description': doc.toxml()}

    controllers_path = os.path.join(pkg_share, 'config', 'controllers.yaml')

    return LaunchDescription([
        use_mock_arg,
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
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            output='screen'
        ),
        Node(
            package='controller_manager', executable='spawner',
            arguments=['balance_controller', '--controller-manager', '/controller_manager'],
            output='screen'
        ),
    ])
