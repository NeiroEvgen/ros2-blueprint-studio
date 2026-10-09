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


def group_files(created_files):
    """{группа: [файлы]} — порядок групп по первому появлению, дефолт 'main'."""
    groups = {}
    for cf in created_files:
        groups.setdefault(cf.get('group', 'main'), []).append(cf)
    return groups


def build_launch_files(created_files, package_name):
    """
    Чистая функция: {имя_файла: содержимое} для всех launch-файлов проекта —
    общий project_launch.py и по одному <группа>_launch.py на деплой-группу.
    """
    out = {"project_launch.py": LaunchCompiler(created_files, package_name).compile()}
    for gname, gfiles in group_files(created_files).items():
        out[f"{gname}_launch.py"] = LaunchCompiler(gfiles, package_name).compile()
    return out


def write_launch_files(launch_dir, created_files, package_name):
    """Пишет launch-файлы в launch_dir и чистит __pycache__. Возвращает группы (для compose)."""
    import shutil
    os.makedirs(launch_dir, exist_ok=True)
    pycache = os.path.join(launch_dir, "__pycache__")
    if os.path.isdir(pycache):
        shutil.rmtree(pycache, ignore_errors=True)
    for name, content in build_launch_files(created_files, package_name).items():
        with open(os.path.join(launch_dir, name), 'w', encoding='utf-8') as f:
            f.write(content)
    return group_files(created_files)
