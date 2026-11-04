import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

PARAMS_FILE = '/root/ros2_ws/src/user_project/config/params.yaml'

def generate_launch_description():
    ld = LaunchDescription()
    # values saved from the Parameters window; checked at launch time, not at generation
    params = [PARAMS_FILE] if os.path.exists(PARAMS_FILE) else []

    # === C++ NODES (Compiled) ===
    ld.add_action(Node(
        package='cpp_pubsub',
        executable='talker',
        name='talker',
        parameters=params,
        output='screen',
        emulate_tty=True
    ))

    ld.add_action(Node(
        package='cpp_pubsub',
        executable='listener',
        name='listener',
        parameters=params,
        output='screen',
        emulate_tty=True
    ))

    return ld
