"""
0.1 — Страховочная сетка на генераторы.

ПРАВИЛО ЭТОГО ФАЙЛА: тесты фиксируют СУЩЕСТВУЮЩЕЕ поведение и должны
проходить на текущем коде без его правок. Если тест падает на работающем
коде — неверен тест, а не код.

Известные дыры (поведение, которого ещё нет) оформлены как xfail(strict=True):
они документируют пробел и автоматически «покраснеют», когда фича появится —
это сигнал снять пометку xfail.
"""
import re
import pytest

from tests.conftest import MAIN_CPP, HELPER_CPP


def _cmake(src):
    return (src / "CMakeLists.txt").read_text(encoding="utf-8")


def _executables(cmake_text):
    return re.findall(r"add_executable\((\w+)\s+([^)]*)\)", cmake_text)


# ------------------------------------------------------------------
# CMake-генератор: что становится executable
# ------------------------------------------------------------------

class TestCmakeExecutables:

    def test_helper_without_main_is_not_executable(self, pm, cpp_project):
        src, add = cpp_project
        add("Node.cpp", MAIN_CPP)
        add("Helper.cpp", HELPER_CPP)

        _, execs = pm._generate_cpp_build_files(str(src), [])

        assert execs == {"Node"}
        names = [name for name, _ in _executables(_cmake(src))]
        assert "Helper" not in names

    def test_single_main_gives_single_executable(self, pm, cpp_project):
        src, add = cpp_project
        add("Node.cpp", MAIN_CPP)

        _, execs = pm._generate_cpp_build_files(str(src), [])

        assert execs == {"Node"}
        assert _executables(_cmake(src)) == [("Node", "cpp/Node.cpp")]

    def test_two_mains_give_two_executables(self, pm, cpp_project):
        src, add = cpp_project
        add("Alpha.cpp", MAIN_CPP)
        add("Beta.cpp", MAIN_CPP)

        _, execs = pm._generate_cpp_build_files(str(src), [])

        assert execs == {"Alpha", "Beta"}
        names = sorted(name for name, _ in _executables(_cmake(src)))
        assert names == ["Alpha", "Beta"]

    def test_no_nodes_gives_valid_cmake_without_executables(self, pm, cpp_project):
        src, _ = cpp_project

        _, execs = pm._generate_cpp_build_files(str(src), [])

        text = _cmake(src)
        assert execs == set()
        assert "add_executable" not in text
        assert "cmake_minimum_required" in text
        assert text.rstrip().endswith("ament_package()")

    def test_ament_package_is_last(self, pm, cpp_project):
        src, add = cpp_project
        add("Node.cpp", MAIN_CPP)

        pm._generate_cpp_build_files(str(src), [])

        assert _cmake(src).rstrip().endswith("ament_package()")

    def test_package_name_is_sanitized(self, pm, tmp_path):
        src = tmp_path / "My-Robot Proj" / "src"
        (src / "cpp").mkdir(parents=True)
        (src / "cpp" / "Node.cpp").write_text(MAIN_CPP, encoding="utf-8")

        pm._generate_cpp_build_files(str(src), [])

        assert "project(my_robot_proj)" in (src / "CMakeLists.txt").read_text()
        assert "<name>my_robot_proj</name>" in (src / "package.xml").read_text()

    def test_unchanged_content_is_not_rewritten(self, pm, cpp_project):
        src, add = cpp_project
        add("Node.cpp", MAIN_CPP)
        pm._generate_cpp_build_files(str(src), [])
        cm = src / "CMakeLists.txt"
        mtime = cm.stat().st_mtime_ns

        pm._generate_cpp_build_files(str(src), [])

        assert cm.stat().st_mtime_ns == mtime


# ------------------------------------------------------------------
# Резолвер зависимостей
# ------------------------------------------------------------------

class TestDependencyResolver:

    @pytest.fixture
    def resolver(self):
        from core.generators.dependency_resolver import DependencyResolver
        return DependencyResolver()

    def test_moveit_include_resolves_to_planning_interface(self, pm, cpp_project):
        src, add = cpp_project
        add("Plan.cpp", '#include "moveit/move_group_interface/move_group_interface.h"\n'
                        + MAIN_CPP)

        apt, _ = pm._generate_cpp_build_files(str(src), [])

        text = _cmake(src)
        assert "find_package(moveit_ros_planning_interface REQUIRED)" in text
        assert any("moveit" in p for p in apt)

    def test_stdlib_includes_are_ignored(self, resolver):
        code = "#include <vector>\n#include <string>\n#include <memory>\n"
        assert resolver.scan_includes(code, "cpp") == []

    def test_unknown_include_goes_to_unknown(self, resolver):
        from core.generators.dependency_resolver import DepSet
        deps = DepSet()
        resolver.resolve_code('#include "totally_unknown_lib/thing.hpp"\n', "cpp", deps)
        assert "totally_unknown_lib/thing.hpp" in deps.unknown

    def test_unknown_include_does_not_break_generation(self, pm, cpp_project):
        src, add = cpp_project
        add("Node.cpp", '#include "totally_unknown_lib/thing.hpp"\n' + MAIN_CPP)

        _, execs = pm._generate_cpp_build_files(str(src), [])

        assert execs == {"Node"}
        assert (src / "CMakeLists.txt").exists()

    def test_python_stdlib_imports_are_ignored(self, resolver):
        code = "import os\nimport sys\nfrom typing import List\n"
        assert resolver.scan_includes(code, "python") == []

    # ---- известные дыры: поведения ещё нет ----

    @pytest.mark.xfail(strict=True, reason="yaml-cpp отсутствует в dependency_registry.json "
                                          "— ручная правка CMakeLists стирается при регенерации")
    def test_yaml_cpp_gives_find_package_and_link(self, pm, cpp_project):
        src, add = cpp_project
        add("Node.cpp", '#include <yaml-cpp/yaml.h>\n' + MAIN_CPP)

        pm._generate_cpp_build_files(str(src), [])

        text = _cmake(src)
        assert "find_package(yaml-cpp REQUIRED)" in text
        assert re.search(r"target_link_libraries\(Node[^)]*yaml-cpp", text)

    @pytest.mark.xfail(strict=True, reason="соседний .cpp локального заголовка не добавляется "
                                          "в add_executable (multi-file executable)")
    def test_local_header_pulls_companion_cpp(self, pm, cpp_project):
        src, add = cpp_project
        add("Node.cpp", '#include "solver.hpp"\n' + MAIN_CPP)
        (src / "cpp" / "solver.hpp").write_text("double solve();\n", encoding="utf-8")
        add("solver.cpp", "double solve() { return 1.0; }\n")

        pm._generate_cpp_build_files(str(src), [])

        execs = dict(_executables(_cmake(src)))
        assert "cpp/solver.cpp" in execs["Node"]


# ------------------------------------------------------------------
# _flatten_nodes
# ------------------------------------------------------------------

def _node(nid, ntype="ros.cpp.CppCustomNode", **custom):
    return {"id": nid, "type_": ntype, "name": nid, "custom": custom}


class TestFlattenNodes:

    def test_plain_nodes_pass_through(self, pm):
        data = {"nodes": [_node("a"), _node("b")]}
        flat = pm._flatten_nodes(data)
        assert [e["node"]["id"] for e in flat] == ["a", "b"]
        assert all(e["namespace"] == "" for e in flat)
        assert all(e["container_group"] == "main" for e in flat)

    def test_note_node_is_filtered(self, pm):
        data = {"nodes": [_node("real"),
                          _node("note", ntype="ros.nodes.meta.NoteNode")]}
        flat = pm._flatten_nodes(data)
        assert [e["node"]["id"] for e in flat] == ["real"]

    def test_group_is_flattened_with_namespace(self, pm):
        group = _node("g", ntype="ros.nodes.RosGroup",
                      node_name="arm", internal_nodes=["c1", "c2"])
        c1 = _node("c1", parent_group_id="g")
        c2 = _node("c2", parent_group_id="g")
        flat = pm._flatten_nodes({"nodes": [group, c1, c2]})

        ids = sorted(e["node"]["id"] for e in flat)
        assert ids == ["c1", "c2"]
        assert all(e["namespace"] == "/arm" for e in flat)

    def test_group_children_not_duplicated_at_root(self, pm):
        group = _node("g", ntype="ros.nodes.RosGroup",
                      node_name="arm", internal_nodes=["c1"])
        c1 = _node("c1", parent_group_id="g")
        flat = pm._flatten_nodes({"nodes": [group, c1]})
        assert [e["node"]["id"] for e in flat] == ["c1"]

    def test_empty_group_does_not_break(self, pm):
        group = _node("g", ntype="ros.nodes.RosGroup",
                      node_name="empty", internal_nodes=[])
        flat = pm._flatten_nodes({"nodes": [group, _node("x")]})
        assert [e["node"]["id"] for e in flat] == ["x"]

    def test_dict_shaped_session_is_accepted(self, pm):
        data = {"nodes": {"a": {"type_": "ros.cpp.CppCustomNode", "custom": {}}}}
        flat = pm._flatten_nodes(data)
        assert [e["node"]["id"] for e in flat] == ["a"]

    def test_subgraph_terminals_are_skipped(self, pm):
        data = {"nodes": [
            _node("in", ntype="ros.nodes.internal.SubGraphInputNode"),
            _node("out", ntype="ros.nodes.internal.SubGraphOutputNode"),
            _node("real"),
        ]}
        flat = pm._flatten_nodes(data)
        assert [e["node"]["id"] for e in flat] == ["real"]
