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


def generate_poc_relay_standalone_nodes(serial_number: int) -> Node:
    return Node(
        package='autoware_default_adapi_poc',
        executable='adapi_node',
        arguments=[
            "--ros-args",
            "--remap", f"__node:=poc_relay_{serial_number}",
            "--remap", f"/input_topic_1:=/input_topic_{serial_number}",
            "--remap", f"/output_topic_1:=/output_topic_{serial_number}"
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
            generate_poc_relay_standalone_nodes(i) for i in range(1, total_nodes + 1)
        ]
        nodes += [
            generate_message_publisher_node(i) for i in range(1, total_nodes + 1)
        ]
        return nodes
    
    return LaunchDescription([
        declare_total_nodes,
        OpaqueFunction(function=launch_setup)
    ])
