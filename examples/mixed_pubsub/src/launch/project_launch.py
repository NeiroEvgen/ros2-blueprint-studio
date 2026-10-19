import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    ld = LaunchDescription()

    # === C++ NODES (Compiled) ===
    ld.add_action(Node(
        package='mixed_pubsub',
        executable='listener',
        name='listener',
        output='screen',
        emulate_tty=True
    ))

    # === PYTHON NODES (Direct Scripts) ===
    ld.add_action(ExecuteProcess(
        cmd=['python3', '-u', '/root/ros2_ws/src/user_project/python/talker.py'],
        output='screen',
        shell=True,
        emulate_tty=True
    ))

    return ld
