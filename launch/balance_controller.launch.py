from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = FindPackageShare('balance_controller_ros2')
    odrive_pkg_share = FindPackageShare('odrive_ros2_control')

    urdf_file = PathJoinSubstitution([pkg_share, 'urdf', 'bike_robot.ros2_control.xacro'])
    controllers_file = PathJoinSubstitution([pkg_share, 'config', 'controllers.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'urdf_file',
            default_value=urdf_file,
            description='Path to URDF file'
        ),
        DeclareLaunchArgument(
            'controllers_file',
            default_value=controllers_file,
            description='Path to controllers configuration file'
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', LaunchConfiguration('urdf_file')])
            }]
        ),

        # Controller Manager
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[LaunchConfiguration('controllers_file'), {
                'robot_description': Command(['xacro ', LaunchConfiguration('urdf_file')])
            }],
            output='screen'
        ),

        # Joint State Broadcaster
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            output='screen'
        ),

        # Balance Controller
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['balance_controller', '--controller-manager', '/controller_manager'],
            output='screen'
        )
    ])