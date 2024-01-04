import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('ros2_ipcam_publisher'),
        'config',
        'params.yaml'
    )

    node = Node(
        package = 'ros2_ipcam_publisher',
        name = 'ipcam_publisher_node',
        executable = 'ipcam_publisher_node',
        parameters=[config]
    )

    ld.add_action(node)
    return ld