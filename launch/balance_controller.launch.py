from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('balance_controller_ros2')
    params_file = os.path.join(pkg_share, 'config', 'params.yaml')

    return LaunchDescription([
        Node(
            package='balance_controller_ros2',
            executable='balance_controller_node',
            name='bike_controller_node',
            output='screen',
            parameters=[params_file]
        )
    ])