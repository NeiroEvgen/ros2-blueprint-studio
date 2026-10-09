"""Один проект — C++ и Python ноды вместе (project_type = "mixed")."""
import ast
import json
import os
import pytest
from core.project_manager import ProjectManager
from core.workspace_manager import WorkspaceManager
from tests.test_save_safety import FakeGraph, cpp_node, MAIN


def py_node(nid, code=""):
    return {"id": nid, "type_": "ros.py.PyCustomNode", "name": nid,
            "custom": {"node_name": nid, "code_content": code, "source_file": ""}, "ports": []}


PY_CODE = "import rclpy\nprint('hi')\n"


@pytest.fixture
def mixed(tmp_path):
    root = tmp_path / "mix"
    for d in ("src", ".blueprint", "docker"):
        (root / d).mkdir(parents=True)
    (root / "project_config.json").write_text(json.dumps({"name": "mix", "type": "mixed"}))
    return root


def save(root, cpp_nodes=(), py_nodes=()):
    cpp = FakeGraph({"nodes": list(cpp_nodes), "connections": []})
    py = FakeGraph({"nodes": list(py_nodes), "connections": []})
    res = ProjectManager(py, cpp).save_project(str(root))
    assert res and res[0] is True
    return res


def launch(root):
    return (root / "src/launch/project_launch.py").read_text(encoding="utf-8")


def test_both_languages_generated_into_own_dirs(mixed):
    save(mixed, [cpp_node("drv", MAIN)], [py_node("vision", PY_CODE)])
    assert (mixed / "src/cpp/drv.cpp").is_file()
    assert (mixed / "src/python/vision.py").read_text(encoding="utf-8") == PY_CODE


def test_launch_runs_both(mixed):
    save(mixed, [cpp_node("drv", MAIN)], [py_node("vision", PY_CODE)])
    text = launch(mixed)
    ast.parse(text)
    assert "executable='drv'" in text
    assert "/root/ros2_ws/src/user_project/python/vision.py" in text
    assert text.index("executable='drv'") < text.index("vision.py")        # сначала C++, потом Python


def test_cmake_exists_and_contains_only_cpp_targets(mixed):
    save(mixed, [cpp_node("drv", MAIN)], [py_node("vision", PY_CODE)])
    cm = (mixed / "src/CMakeLists.txt").read_text(encoding="utf-8")
    assert "add_executable(drv " in cm and "vision" not in cm


def test_cpp_helper_without_main_excluded_but_python_kept(mixed):
    save(mixed, [cpp_node("drv", MAIN), cpp_node("helper", "void f(){}\n")], [py_node("vision", PY_CODE)])
    text = launch(mixed)
    assert "helper" not in text and "vision.py" in text


def test_ghost_cleanup_is_per_language(mixed):
    save(mixed, [cpp_node("a", MAIN), cpp_node("b", MAIN)], [py_node("p1", PY_CODE), py_node("p2", PY_CODE)])
    save(mixed, [cpp_node("a", MAIN)], [py_node("p1", PY_CODE)])
    assert (mixed / "src/cpp/a.cpp").exists() and not (mixed / "src/cpp/b.cpp").exists()
    assert (mixed / "src/python/p1.py").exists() and not (mixed / "src/python/p2.py").exists()


def test_cpp_files_never_removed_by_python_cleanup_and_vice_versa(mixed):
    save(mixed, [cpp_node("a", MAIN)], [py_node("p1", PY_CODE)])
    save(mixed, [cpp_node("a", MAIN)], [])
    assert (mixed / "src/cpp/a.cpp").exists() and not (mixed / "src/python/p1.py").exists()


def test_python_only_mixed_has_no_cmake_executables(mixed):
    save(mixed, [], [py_node("p1", PY_CODE)])
    assert "vision" not in launch(mixed) and "p1.py" in launch(mixed)


def test_dockerfile_gets_deps_of_both_languages(mixed):
    save(mixed, [cpp_node("a", "#include <yaml-cpp/yaml.h>\n" + MAIN)],
         [py_node("p", "import numpy\nimport requests\n")])
    text = (mixed / "Dockerfile").read_text(encoding="utf-8")
    assert "libyaml-cpp-dev" in text


def test_group_files_for_both_languages_in_groups(mixed):
    save(mixed, [cpp_node("a", MAIN)], [py_node("p", PY_CODE)])
    assert (mixed / "src/launch/main_launch.py").is_file()
    text = (mixed / "src/launch/main_launch.py").read_text(encoding="utf-8")
    assert "executable='a'" in text and "p.py" in text


class TestWorkspaceMixed:
    @pytest.fixture
    def ws(self, tmp_path, monkeypatch):
        monkeypatch.setattr(WorkspaceManager, "get_workspace_root", staticmethod(lambda: str(tmp_path)))
        return tmp_path

    def test_create_mixed_project(self, ws):
        path = WorkspaceManager.create_project("m", "mixed")
        assert WorkspaceManager.get_project_type(path) == "mixed"

    def test_layout_code_dirs(self, tmp_path):
        from core.layout import ProjectLayout
        L = ProjectLayout(str(tmp_path))
        assert [os.path.basename(d) for d in L.code_dirs("mixed")] == ["cpp", "python"]
        assert [os.path.basename(d) for d in L.code_dirs("cpp")] == ["cpp"]
        L.ensure("mixed")
        assert os.path.isdir(L.code_dir("cpp")) and os.path.isdir(L.code_dir("python"))
