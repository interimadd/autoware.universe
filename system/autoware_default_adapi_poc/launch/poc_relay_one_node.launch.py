"""
自作のRelayノードを1つ立ち上げ、そのノードで全てのtopicを受信してpublishするLaunchファイル
受信するtopicの数を引数で指定可能

使い方：
ros2 launch autoware_default_adapi_poc poc_relay_one_node.launch.py topic_num:=10
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


def generate_poc_relay_node(topic_num: int) -> Node:
    return Node(
        package='autoware_default_adapi_poc',
        executable='adapi_node',
        parameters=[
            {"num_relay_topics": topic_num}
        ],
        output='screen'
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
        nodes = [
            generate_poc_relay_node(topic_num)
        ]
        nodes += [
            generate_message_publisher_node(i) for i in range(1, topic_num + 1)
        ]
        return nodes

    return LaunchDescription([
        declare_topic_num,
        OpaqueFunction(function=launch_setup)
    ])
