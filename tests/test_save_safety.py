"""
Save никогда не должен удалять то, что ему не принадлежит.

Принадлежит генератору: <src>/<cpp|python>/*.(cpp|py) нод графа, файлы
project_launch.py и <группа>_launch.py, AUTO-зоны CMakeLists/package.xml.
Всё остальное (lib/, scripts/, python/ у C++-проекта, include/, test/, чужие launch,
USER-зона CMake) — файлы человека.
"""
import json
import os
import pytest
from core.project_manager import ProjectManager
from core.layout import ProjectLayout


class FakeGraph:
    def __init__(self, session=None):
        self.session = session or {"nodes": {}, "connections": []}

    def all_nodes(self):
        return []

    def serialize_session(self):
        return self.session


def cpp_node(nid, code):
    return {"id": nid, "type_": "ros.cpp.CppCustomNode", "name": nid,
            "custom": {"node_name": nid, "code_content": code, "source_file": ""},
            "ports": []}


MAIN = "#include <rclcpp/rclcpp.hpp>\nint main(int a,char**b){return 0;}\n"


@pytest.fixture
def project(tmp_path):
    root = tmp_path / "my_proj"
    for d in ("src/cpp", "src/launch", ".blueprint", "docker"):
        (root / d).mkdir(parents=True)
    (root / "project_config.json").write_text(json.dumps({"name": "my_proj", "type": "cpp"}))
    return root


def save(project, nodes=(), py_nodes=()):
    cpp = FakeGraph({"nodes": list(nodes), "connections": []})
    py = FakeGraph({"nodes": list(py_nodes), "connections": []})
    res = ProjectManager(py, cpp).save_project(str(project))
    assert res and res[0] is True, "save_project failed"
    return res


HUMAN_FILES = {
    "src/lib/g1_core/petri_node.py": "# petri",
    "src/lib/g1_core/mission_core.cpp": "// core",
    "src/python/petri/tracker.py": "# tracker",
    "src/scripts/petri_node.py": "# script",
    "src/include/g1_stage/types.hpp": "// types",
    "src/kinematics_lib/trajectory_solver.cpp": "// ktraj",
    "src/test/test_core.cpp": "// test",
    "src/test/test_petri_tracker.py": "# t",
    "src/launch/petri.launch.py": "# user launch",
    "src/launch/pc_sim.launch.py": "# user launch 2",
    "src/executor_core.cpp": "// top-level helper",
    "README_NODES.md": "# readme",
}


def seed(project):
    for rel, text in HUMAN_FILES.items():
        p = project / rel
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(text, encoding="utf-8")


def test_human_files_survive_save(project):
    seed(project)
    save(project, nodes=[cpp_node("talker", MAIN)])
    for rel, text in HUMAN_FILES.items():
        assert (project / rel).read_text(encoding="utf-8") == text, rel


def test_human_files_survive_repeated_saves_and_node_removal(project):
    seed(project)
    save(project, nodes=[cpp_node("a", MAIN), cpp_node("b", MAIN)])
    save(project, nodes=[cpp_node("a", MAIN)])
    save(project, nodes=[])
    for rel, text in HUMAN_FILES.items():
        assert (project / rel).read_text(encoding="utf-8") == text, rel


def test_removed_node_ghost_file_is_deleted_only_in_code_dir(project):
    save(project, nodes=[cpp_node("a", MAIN), cpp_node("b", MAIN)])
    assert (project / "src/cpp/b.cpp").exists()
    save(project, nodes=[cpp_node("a", MAIN)])
    assert not (project / "src/cpp/b.cpp").exists()
    assert (project / "src/cpp/a.cpp").exists()


def test_user_cmake_zone_survives(project):
    save(project, nodes=[cpp_node("a", MAIN)])
    cm = project / "src" / "CMakeLists.txt"
    text = cm.read_text(encoding="utf-8")
    from core.generators.build_generator import CMAKE_USER_END
    text = text.replace(CMAKE_USER_END, "find_package(yaml-cpp REQUIRED)\n" + CMAKE_USER_END)
    cm.write_text(text, encoding="utf-8")
    save(project, nodes=[cpp_node("a", MAIN), cpp_node("c", MAIN)])
    new = cm.read_text(encoding="utf-8")
    assert "find_package(yaml-cpp REQUIRED)" in new and "add_executable(c " in new


def test_save_writes_only_into_known_places(project):
    """После Save в корне проекта не появляется ROS-мусор."""
    save(project, nodes=[cpp_node("a", MAIN)])
    top = set(os.listdir(project))
    assert not top & {"launch", "CMakeLists.txt", "package.xml", "cpp", "include"}
    assert not ProjectLayout(str(project)).is_legacy()


def test_state_roundtrip_file_written(project):
    save(project, nodes=[cpp_node("a", MAIN)])
    assert (project / ".blueprint" / "state.yaml").is_file()


def test_launch_files_generated_and_valid(project):
    import ast
    save(project, nodes=[cpp_node("a", MAIN)])
    ld = project / "src" / "launch"
    ast.parse((ld / "project_launch.py").read_text(encoding="utf-8"))
    assert "executable='a'" in (ld / "project_launch.py").read_text(encoding="utf-8")


def test_library_helper_without_main_not_in_launch(project):
    save(project, nodes=[cpp_node("a", MAIN), cpp_node("helper", "void f(){}\n")])
    launch = (project / "src/launch/project_launch.py").read_text(encoding="utf-8")
    assert "executable='a'" in launch and "helper" not in launch
    assert (project / "src/cpp/helper.cpp").exists()   # файл остаётся, это библиотека
