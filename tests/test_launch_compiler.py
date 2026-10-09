"""
Характеризационные тесты генератора launch-файлов (LaunchCompiler).

Пинят ТЕКУЩЕЕ поведение, включая спорные места (помечены "ФИКСИРУЕМ").
Без Qt, Docker и ROS: сгенерированный файл исполняется на подставных
модулях launch / launch_ros, которые просто записывают аргументы.
"""
import ast
import sys
import types

import pytest


def _compile(files, pkg="my_pkg"):
    from core.generators.launch_generator import LaunchCompiler
    return LaunchCompiler(files, package_name=pkg).compile()


def cpp(name, group="main"):
    return {"language": "cpp", "filename": f"{name}.cpp", "group": group}


def py(name, group="main"):
    return {"language": "python", "filename": f"{name}.py", "group": group}


class _Rec:
    """Подставной launch-объект: запоминает kwargs."""
    kind = "?"

    def __init__(self, **kw):
        self.kw = kw


class FakeNode(_Rec):
    kind = "Node"


class FakeExecuteProcess(_Rec):
    kind = "ExecuteProcess"


class FakeLaunchDescription:
    def __init__(self):
        self.actions = []

    def add_action(self, a):
        self.actions.append(a)


def run_launch(src):
    """Исполняет сгенерированный launch-файл на подставных модулях и
    возвращает список действий (Node / ExecuteProcess)."""
    fake_launch = types.ModuleType("launch")
    fake_launch.LaunchDescription = FakeLaunchDescription
    fake_actions = types.ModuleType("launch.actions")
    fake_actions.ExecuteProcess = FakeExecuteProcess
    fake_ros = types.ModuleType("launch_ros")
    fake_ros_actions = types.ModuleType("launch_ros.actions")
    fake_ros_actions.Node = FakeNode
    saved = {k: sys.modules.get(k) for k in
             ("launch", "launch.actions", "launch_ros", "launch_ros.actions")}
    sys.modules.update({"launch": fake_launch, "launch.actions": fake_actions,
                        "launch_ros": fake_ros, "launch_ros.actions": fake_ros_actions})
    try:
        ns = {}
        exec(compile(src, "<launch>", "exec"), ns)
        return ns["generate_launch_description"]().actions
    finally:
        for k, v in saved.items():
            if v is None:
                sys.modules.pop(k, None)
            else:
                sys.modules[k] = v


class TestValidity:
    @pytest.mark.parametrize("files", [
        [],
        [cpp("a")],
        [py("a")],
        [cpp("a"), py("b"), cpp("c")],
    ])
    def test_output_is_valid_python(self, files):
        ast.parse(_compile(files))

    def test_has_generate_launch_description(self):
        tree = ast.parse(_compile([cpp("a")]))
        assert any(isinstance(n, ast.FunctionDef) and n.name == "generate_launch_description"
                   for n in tree.body)

    def test_empty_graph_gives_empty_description(self):
        assert run_launch(_compile([])) == []


class TestCppNodes:
    def test_node_args(self):
        (a,) = run_launch(_compile([cpp("planner")], pkg="g1_stage"))
        assert a.kind == "Node"
        assert a.kw["package"] == "g1_stage"
        assert a.kw["executable"] == "planner"
        assert a.kw["name"] == "planner"            # ФИКСИРУЕМ: имя ноды = имя исполняемого
        assert a.kw["output"] == "screen"

    def test_each_cpp_file_is_one_node(self):
        acts = run_launch(_compile([cpp("a"), cpp("b"), cpp("c")]))
        assert [x.kw["executable"] for x in acts] == ["a", "b", "c"]

    def test_executable_is_filename_without_extension(self):
        (a,) = run_launch(_compile([{"language": "cpp", "filename": "my.node.cpp", "group": "main"}]))
        assert a.kw["executable"] == "my.node"

    def test_executable_matches_cmake_target(self, tmp_path):
        """Главный инвариант: executable в launch == add_executable в CMakeLists."""
        from core.project_manager import ProjectManager
        src = tmp_path / "p" / "src"
        (src / "cpp").mkdir(parents=True)
        main = '#include "rclcpp/rclcpp.hpp"\nint main(int c,char**v){return 0;}\n'
        (src / "cpp" / "alpha_node.cpp").write_text(main)
        (src / "cpp" / "beta_node.cpp").write_text(main)
        ProjectManager(None, None)._generate_cpp_build_files(
            str(src), [cpp("alpha_node"), cpp("beta_node")])
        cmake = (src / "CMakeLists.txt").read_text()
        acts = run_launch(_compile([cpp("alpha_node"), cpp("beta_node")], pkg="p"))
        for a in acts:
            assert f"add_executable({a.kw['executable']} " in cmake, a.kw["executable"]


class TestPythonNodes:
    def test_python_node_is_execute_process(self):
        (a,) = run_launch(_compile([py("viewer")]))
        assert a.kind == "ExecuteProcess"
        assert a.kw["cmd"][:2] == ["python3", "-u"]
        assert a.kw["cmd"][2].endswith("/python/viewer.py")

    def test_python_path_is_hardcoded_container_path(self):
        # ФИКСИРУЕМ: путь в контейнере зашит; при смене точки монтирования сломается
        (a,) = run_launch(_compile([py("viewer")]))
        assert a.kw["cmd"][2] == "/root/ros2_ws/src/user_project/python/viewer.py"

    def test_mixed_order_cpp_first_then_python(self):
        acts = run_launch(_compile([py("p1"), cpp("c1"), py("p2"), cpp("c2")]))
        assert [x.kind for x in acts] == ["Node", "Node", "ExecuteProcess", "ExecuteProcess"]


class TestInputRobustness:
    def test_unknown_language_is_ignored(self):
        acts = run_launch(_compile([{"language": "rust", "filename": "x.rs", "group": "main"}, cpp("a")]))
        assert [x.kw["executable"] for x in acts] == ["a"]

    def test_cyrillic_filename_still_valid_python(self):
        ast.parse(_compile([cpp("нода")]))

    def test_filename_with_quote_does_not_break_syntax(self):
        # Апостроф в имени не должен ломать синтаксис сгенерированного файла.
        ast.parse(_compile([cpp("a'b")]))
        ast.parse(_compile([py("a'b.py")]))


class TestLayering:
    def test_core_generator_does_not_import_ui(self):
        import core.generators.launch_generator as m
        src = open(m.__file__, encoding="utf8").read()
        assert "ui." not in src and "PyQt" not in src and "PySide" not in src

    def test_ui_shim_reexports_same_class(self):
        from ui.launch_compiler import LaunchCompiler as A
        from core.generators.launch_generator import LaunchCompiler as B
        assert A is B
