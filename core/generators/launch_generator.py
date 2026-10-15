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
                path = os.path.normpath(f"{CONTAINER_PYTHON_DIR}/{node.get('relpath') or node['filename']}").replace(os.sep, "/")
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
    files = build_launch_files(created_files, package_name)
    for name, content in files.items():
        with open(os.path.join(launch_dir, name), 'w', encoding='utf-8') as f:
            f.write(content)
    remove_stale_group_launches(launch_dir, set(files))
    return group_files(created_files)


_GENERATED_HEADER = ("import os\nfrom launch import LaunchDescription\n"
                     "from launch_ros.actions import Node\nfrom launch.actions import ExecuteProcess\n")


def remove_stale_group_launches(launch_dir, keep):
    """
    Удаляет <группа>_launch.py удалённых групп. Трогаем только файлы с заголовком,
    который пишет сам генератор: launch-файлы, написанные вручную, остаются.
    """
    removed = []
    try:
        names = os.listdir(launch_dir)
    except OSError:
        return removed
    for name in names:
        if not name.endswith("_launch.py") or name in keep:
            continue
        path = os.path.join(launch_dir, name)
        try:
            with open(path, 'r', encoding='utf-8') as f:
                head = f.read(len(_GENERATED_HEADER))
            if head == _GENERATED_HEADER:
                os.remove(path)
                removed.append(name)
        except OSError:
            pass
    return removed
