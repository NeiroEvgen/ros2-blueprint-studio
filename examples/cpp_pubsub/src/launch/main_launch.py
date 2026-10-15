import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    ld = LaunchDescription()

    # === C++ NODES (Compiled) ===
    ld.add_action(Node(
        package='cpp_pubsub',
        executable='talker',
        name='talker',
        output='screen',
        emulate_tty=True
    ))

    ld.add_action(Node(
        package='cpp_pubsub',
        executable='listener',
        name='listener',
        output='screen',
        emulate_tty=True
    ))

    return ld
