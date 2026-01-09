from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='autoware_default_adapi_poc',
            executable='adapi_node',
            output='screen'
        )
    ])
