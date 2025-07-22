import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取配置文件路径
    pkg_dir = get_package_share_directory('serial_driver')
    config_path = os.path.join(pkg_dir, 'config', 'serial_config.yaml')

    serial_node = launch_ros.actions.Node(
        package='serial_driver',
        executable='serial_node',
        name='serial_node',
        output='screen',
        parameters=[config_path]
    )

    return launch.LaunchDescription([
        serial_node
    ])
