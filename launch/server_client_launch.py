from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    namespace="example_segfault"

    return LaunchDescription([
        Node(
            package='segfault_pkg',
            executable='segfault_pkg_node',
            output='screen',
            namespace=namespace
        ),
    ])
