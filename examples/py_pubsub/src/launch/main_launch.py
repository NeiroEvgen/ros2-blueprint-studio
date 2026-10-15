import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    ld = LaunchDescription()

    # === PYTHON NODES (Direct Scripts) ===
    ld.add_action(ExecuteProcess(
        cmd=['python3', '-u', '/root/ros2_ws/src/user_project/python/talker.py'],
        output='screen',
        shell=True,
        emulate_tty=True
    ))

    ld.add_action(ExecuteProcess(
        cmd=['python3', '-u', '/root/ros2_ws/src/user_project/python/listener.py'],
        output='screen',
        shell=True,
        emulate_tty=True
    ))

    return ld
