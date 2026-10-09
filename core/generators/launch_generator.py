"""Генератор launch-файла проекта. Чистая логика: без Qt и без ui/."""
import os

CONTAINER_PYTHON_DIR = "/root/ros2_ws/src/user_project/python"


class LaunchCompiler:
    def __init__(self, scripts, package_name):
        self.scripts = scripts
        self.package_name = package_name

    def compile(self):
        r = repr  # корректно экранирует кавычки/юникод в строковых литералах
        code = "import os\n"
        code += "from launch import LaunchDescription\n"
        code += "from launch_ros.actions import Node\n"
        code += "from launch.actions import ExecuteProcess\n\n"
        code += "def generate_launch_description():\n"
        code += "    ld = LaunchDescription()\n\n"

        cpp_nodes = [s for s in self.scripts if s['language'] == 'cpp']
        py_nodes = [s for s in self.scripts if s['language'] == 'python']

        if cpp_nodes:
            code += "    # === C++ NODES (Compiled) ===\n"
            for node in cpp_nodes:
                exec_name = os.path.splitext(node['filename'])[0]
                code += "    ld.add_action(Node(\n"
                code += f"        package={r(self.package_name)},\n"
                code += f"        executable={r(exec_name)},\n"
                code += f"        name={r(exec_name)},\n"
                code += "        output='screen',\n"
                code += "        emulate_tty=True\n"
                code += "    ))\n\n"

        if py_nodes:
            code += "    # === PYTHON NODES (Direct Scripts) ===\n"
            for node in py_nodes:
                path = f"{CONTAINER_PYTHON_DIR}/{node['filename']}"
                code += "    ld.add_action(ExecuteProcess(\n"
                code += f"        cmd=['python3', '-u', {r(path)}],\n"
                code += "        output='screen',\n"
                code += "        shell=True,\n"
                code += "        emulate_tty=True\n"
                code += "    ))\n\n"

        code += "    return ld\n"
        return code
