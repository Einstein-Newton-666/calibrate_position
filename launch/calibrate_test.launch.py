import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node
from launch.substitutions import EnvironmentVariable
from launch.actions import TimerAction

def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('calibrate_position'), 'config', 'calibrate_position.yaml')

    calibrate_test_node = Node(
        package='calibrate_position',
        executable='calibrate_test',
        name='calibrate_position',
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription([
        calibrate_test_node
        ])