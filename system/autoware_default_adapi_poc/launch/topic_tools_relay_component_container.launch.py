"""
topic_toolsのRelayノードを1つのcomponent containerにcomposable nodeとして全てloadして起動するLaunchファイル
受信するtopicの数を引数で指定可能

使い方：
ros2 launch autoware_default_adapi_poc topic_tools_relay_component_container.launch.py topic_num:=10
"""
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
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


def generate_topic_tools_relay_composable_node(serial_number: int) -> ComposableNode:
    return ComposableNode(
        package='topic_tools',
        plugin='topic_tools::RelayNode',
        name=f'topic_tools_relay_{serial_number}',
        parameters=[
            {'input_topic': f'/input_topic_{serial_number}'},
            {'output_topic': f'/output_topic_{serial_number}'}
        ],
    )


def generate_launch_description():
    # Declare launch argument for topic_num
    declare_topic_num = DeclareLaunchArgument(
        'topic_num',
        default_value='5',
        description='Number of topics to receive and publish'
    )

    def launch_setup(context, *args, **kwargs):
        topic_num = int(LaunchConfiguration('topic_num').perform(context))
        
        # Create composable nodes for relay
        composable_nodes = [
            generate_topic_tools_relay_composable_node(i) for i in range(1, topic_num + 1)
        ]
        
        # Create component container with all relay nodes
        container = ComposableNodeContainer(
            name='topic_tools_relay_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=composable_nodes,
            output='screen',
        )
        
        # Create message publishers
        publishers = [
            generate_message_publisher_node(i) for i in range(1, topic_num + 1)
        ]
        
        return [container] + publishers

    return LaunchDescription([
        declare_topic_num,
        OpaqueFunction(function=launch_setup)
    ])
