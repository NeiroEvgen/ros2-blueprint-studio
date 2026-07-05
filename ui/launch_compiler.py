import os

class LaunchCompiler:
    def __init__(self, scripts, package_name):
        # Исправляем инициализацию: scripts — это и есть список созданных файлов
        self.scripts = scripts 
        self.package_name = package_name

    def compile(self):
        # Шапка файла launch
        code =  "import os\n"
        code += "from launch import LaunchDescription\n"
        code += "from launch_ros.actions import Node\n"
        code += "from launch.actions import ExecuteProcess\n\n"
        
        code += "def generate_launch_description():\n"
        code += "    ld = LaunchDescription()\n\n"
        
        # Разделяем на списки на основе данных из project_manager.py
        cpp_nodes = [s for s in self.scripts if s['language'] == 'cpp']
        py_nodes = [s for s in self.scripts if s['language'] == 'python']

        # === 1. C++ НОДЫ (Используем стандартный запуск через Node) ===
        if cpp_nodes:
            code += "    # === C++ NODES (Compiled) ===\n"
            for node in cpp_nodes:
                # Имя исполняемого файла (exec) в ROS 2 обычно совпадает с именем .cpp файла без расширения
                exec_name = os.path.splitext(node['filename'])[0]
                
                code += f"    ld.add_action(Node(\n"
                code += f"        package='{self.package_name}',\n"
                code += f"        executable='{exec_name}',\n"
                code += f"        name='{exec_name}',\n"
                code += f"        output='screen',\n"
                code += f"        emulate_tty=True\n"
                code += f"    ))\n\n"

        # === 2. PYTHON НОДЫ (Запуск напрямую через интерпретатор) ===
        if py_nodes:
            code += "    # === PYTHON NODES (Direct Scripts) ===\n"
            for node in py_nodes:
                filename = node['filename']
                
                
                container_path = f"/root/ros2_ws/src/user_project/python/{filename}"
                
                # Команда запуска для ExecuteProcess
                cmd = f"['python3', '-u', '{container_path}']"
                
                code += f"    ld.add_action(ExecuteProcess(\n"
                code += f"        cmd={cmd},\n"
                code += f"        output='screen',\n"
                code += f"        shell=True,\n"
                code += f"        emulate_tty=True\n"
                code += f"    ))\n\n"

        code += "    return ld\n"
        return code