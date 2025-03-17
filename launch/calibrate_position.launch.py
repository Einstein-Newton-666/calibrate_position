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
    
    install_path = os.path.join(get_package_share_directory('calibrate_position'), 'data.txt')
    print(install_path)
    path_parts = os.path.normpath(install_path).split(os.sep)

    install_idx = path_parts.index("install")
    share_idx = path_parts.index("share", install_idx)

    data_dir = os.path.join("/",
    *(
        path_parts[:install_idx] +                    # 保留 install 前的路径
        ["src"] +                                       # 替换 install 为 src
        path_parts[install_idx+1 : share_idx] +        # 保留包名
        path_parts[share_idx+2:]                       # 跳过 share/pkg_name
    ))

    calibrate_position_node = Node(
        package='calibrate_position',
        executable='calibrate_position',
        name='calibrate_position',
        output='screen',
        parameters=[params_file,
                    {'data_dir': data_dir}],
    )

    calibrate_test_node = Node(
        package='calibrate_position',
        executable='calibrate_test',
        name='calibrate_position',
        output='screen',
        parameters=[params_file],
    )
    
    delay_calibrate_position_node = TimerAction(
        period=2.0,
        actions=[calibrate_position_node],
    )


    return LaunchDescription([
        calibrate_position_node,
        # delay_calibrate_position_node,
        # calibrate_test_node
        ])