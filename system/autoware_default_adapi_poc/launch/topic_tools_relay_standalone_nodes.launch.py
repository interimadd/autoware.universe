"""
topic_toolsのRelayノードをstandaloneで複数起動するLaunchファイル
立ち上げるノードの数を引数で指定可能

使い方：
ros2 launch autoware_default_adapi_poc topic_tools_relay_standalone_nodes.launch.py total_nodes:=10
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def generate_message_publisher_node(serial_number: int):
    return ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', 
            '--rate', '10', 
            f"/input_topic_{serial_number}", 
            'std_msgs/msg/String', 
            '{data: "Hello ROS2 from Launch"}'
        ]
    )


def generate_topic_tools_relay_standalone_nodes(serial_number: int) -> Node:
    return Node(
        package='topic_tools',
        executable='relay',
        name=f'topic_tools_relay_{serial_number}',
        parameters=[
            {'input_topic': f'/input_topic_{serial_number}'},
            {'output_topic': f'/output_topic_{serial_number}'}
        ],
        output='screen'
    )

def generate_launch_description():
    # Declare launch argument for total_nodes
    declare_total_nodes = DeclareLaunchArgument(
        'total_nodes',
        default_value='5',
        description='Number of standalone nodes to launch'
    )
    
    def launch_setup(context, *args, **kwargs):
        total_nodes = int(LaunchConfiguration('total_nodes').perform(context))
        nodes = [
            generate_topic_tools_relay_standalone_nodes(i) for i in range(1, total_nodes + 1)
        ]
        nodes += [
            generate_message_publisher_node(i) for i in range(1, total_nodes + 1)
        ]
        return nodes
    
    return LaunchDescription([
        declare_total_nodes,
        OpaqueFunction(function=launch_setup)
    ])
