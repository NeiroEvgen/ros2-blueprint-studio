import os

import pytest
from tests.test_save_safety import project  # noqa: F401  (fixture)

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
from core import build_marks as bm
from core.generators import build_generator as bg
from core.project_manager import ProjectManager


@pytest.fixture(scope="module")
def app():
    from PySide6 import QtWidgets
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


def _project(tmp_path):
    proj = tmp_path / "proj"
    cpp = proj / "src" / "cpp"
    (cpp / "kin").mkdir(parents=True)
    (proj / ".blueprint").mkdir()
    (cpp / "node.cpp").write_text("#include <rclcpp/rclcpp.hpp>\nint main(){}\n", encoding="utf-8")
    (cpp / "kin" / "solver.hpp").write_text("#pragma once\n", encoding="utf-8")
    (cpp / "kin" / "solver.cpp").write_text('#include "solver.hpp"\n#include <opencv2/core.hpp>\n', encoding="utf-8")
    (cpp / "kin" / "tool.cpp").write_text("int main(){}\n", encoding="utf-8")
    return proj, proj / "src"


def _gen(src):
    ProjectManager(None, None)._generate_cpp_build_files(str(src), [])
    return (src / "CMakeLists.txt").read_text(encoding="utf-8")


def test_marks_roundtrip_and_cycle(tmp_path):
    proj, _ = _project(tmp_path)
    assert bm.load(str(proj)) == {}
    bm.set_mark(str(proj), "cpp/kin", bm.YELLOW)
    bm.set_mark(str(proj), "cpp\\kin\\solver.cpp", bm.GREEN)
    assert bm.load(str(proj)) == {"cpp/kin": "yellow", "cpp/kin/solver.cpp": "green"}
    bm.set_mark(str(proj), "cpp/kin", None)
    bm.set_mark(str(proj), "cpp/kin/solver.cpp", None)
    assert not os.path.exists(proj / ".blueprint" / "build_marks.json")
    assert [bm.next_state(s) for s in (None, "yellow", "green")] == ["yellow", "green", None]


def test_effective_nearest_wins(tmp_path):
    marks = {"cpp/kin": "yellow", "cpp/kin/solver.cpp": "green"}
    assert bm.effective(marks, "cpp/kin/solver.cpp") == "green"
    assert bm.effective(marks, "cpp/kin/solver.hpp") == "yellow"
    assert bm.effective(marks, "cpp/node.cpp") is None


def test_no_marks_cmake_unchanged(tmp_path):
    _, src = _project(tmp_path)
    cmake = _gen(src)
    assert "include_directories(include)\n" in cmake
    assert "kin/" not in cmake


def test_marked_source_goes_into_every_executable(tmp_path):
    proj, src = _project(tmp_path)
    bm.set_mark(str(proj), "cpp/kin/solver.cpp", bm.YELLOW)
    bm.set_mark(str(proj), "cpp/kin/solver.hpp", bm.YELLOW)
    cmake = _gen(src)
    assert "add_executable(node cpp/node.cpp cpp/kin/solver.cpp)" in cmake
    assert "include_directories(include cpp/kin)" in cmake
    assert "find_package(OpenCV REQUIRED)" in cmake      # dependency from the marked file


def test_green_behaves_like_yellow_in_cmake(tmp_path):
    proj, src = _project(tmp_path)
    bm.set_mark(str(proj), "cpp/kin/solver.cpp", bm.GREEN)
    assert "cpp/kin/solver.cpp" in _gen(src)


def test_marked_main_becomes_own_executable(tmp_path):
    proj, src = _project(tmp_path)
    bm.set_mark(str(proj), "cpp/kin/tool.cpp", bm.YELLOW)
    bm.set_mark(str(proj), "cpp/kin/solver.cpp", bm.YELLOW)
    cmake = _gen(src)
    assert "add_executable(tool cpp/kin/tool.cpp cpp/kin/solver.cpp)" in cmake
    assert "install(TARGETS tool DESTINATION" in cmake
    _, execs = ProjectManager(None, None)._generate_cpp_build_files(str(src), [])
    assert execs == {"node", "tool"}


def test_folder_mark_expands_and_inner_none_is_not_possible(tmp_path):
    proj, src = _project(tmp_path)
    bm.set_mark(str(proj), "cpp/kin", bm.YELLOW)
    files = bm.marked_files(str(src), bm.load(str(proj)))
    assert files == ["cpp/kin/solver.cpp", "cpp/kin/solver.hpp", "cpp/kin/tool.cpp"]


def test_missing_marked_file_is_ignored(tmp_path):
    proj, src = _project(tmp_path)
    bm.set_mark(str(proj), "cpp/gone.cpp", bm.YELLOW)
    assert "gone" not in _gen(src)


def test_duplicate_executable_name_skipped(tmp_path):
    proj, src = _project(tmp_path)
    (src / "cpp" / "kin" / "node.cpp").write_text("int main(){}\n", encoding="utf-8")
    bm.set_mark(str(proj), "cpp/kin/node.cpp", bm.YELLOW)
    assert _gen(src).count("add_executable(node ") == 1


def test_file_browser_marks_column(app, tmp_path):
    from ui.file_browser import FileBrowser, COL_MARK, ROLE_MARK
    proj, src = _project(tmp_path)
    fb = FileBrowser()
    fb.set_sources(str(src), None)
    assert not fb.tree.isColumnHidden(COL_MARK)
    seen = []
    fb.marks_changed.connect(lambda: seen.append(1))
    top = fb.tree.topLevelItem(0)
    from PySide6 import QtCore, QtWidgets
    path = str(src / "cpp" / "node.cpp")
    it = QtWidgets.QTreeWidgetItem(["node.cpp", "", ""])
    it.setData(0, QtCore.Qt.UserRole, path)
    top.addChild(it)
    fb._on_click(it, COL_MARK)
    assert bm.load(str(proj)) == {"cpp/node.cpp": "yellow"} and it.data(COL_MARK, ROLE_MARK) == ("yellow", False)
    fb._on_click(it, COL_MARK)
    fb._on_click(it, COL_MARK)
    assert bm.load(str(proj)) == {} and it.data(COL_MARK, ROLE_MARK) == (None, False) and len(seen) == 3
    fb._on_click(it, 1)                       # not the marks column
    assert len(seen) == 3


def test_marks_hidden_for_non_project_source(app):
    from ui.file_browser import FileBrowser, COL_MARK
    fb = FileBrowser()
    assert fb.tree.isColumnHidden(COL_MARK)


def test_save_keeps_marked_top_level_file(project):
    """A file in cpp/ without a node but with a mark is not a "ghost" and is not removed on Save."""
    from tests.test_save_safety import save, cpp_node, MAIN
    save(project, nodes=[cpp_node("a", MAIN), cpp_node("b", MAIN)])
    bm.set_mark(str(project), "cpp/b.cpp", bm.YELLOW)
    save(project, nodes=[cpp_node("a", MAIN)])
    assert (project / "src/cpp/b.cpp").exists()
    bm.set_mark(str(project), "cpp/b.cpp", None)
    save(project, nodes=[cpp_node("a", MAIN)])
    assert not (project / "src/cpp/b.cpp").exists()


# ---------- green marks -> canvas nodes ----------

def _mixed_project(tmp_path):
    import json
    root = tmp_path / "mix"
    for d in ("src/cpp", "src/python", ".blueprint", "docker"):
        (root / d).mkdir(parents=True)
    (root / "project_config.json").write_text(json.dumps({"name": "mix", "type": "mixed"}), encoding="utf-8")
    (root / "src/cpp/solver.cpp").write_text("#include <rclcpp/rclcpp.hpp>\nint main(){}\n// mine\n", encoding="utf-8")
    (root / "src/python/cam.py").write_text("print('mine')\n", encoding="utf-8")
    (root / "src/cpp/sub").mkdir()
    (root / "src/cpp/sub/deep.cpp").write_text("int main(){}\n", encoding="utf-8")
    return root


def test_green_node_files_split_code_and_plain(tmp_path):
    root = _mixed_project(tmp_path)
    (root / "src/cpp/sub/deep.hpp").write_text("#pragma once\n", encoding="utf-8")
    (root / "src/CMakeLists.txt").write_text("project(x)\n", encoding="utf-8")
    for rel in ("cpp/solver.cpp", "python/cam.py", "cpp/sub/deep.cpp", "cpp/sub/deep.hpp", "CMakeLists.txt"):
        bm.set_mark(str(root), rel, bm.GREEN)
    code, plain = bm.green_node_files(str(root), ("cpp", "python"))
    assert code == [("cpp", "solver.cpp", "cpp/solver.cpp"), ("cpp", "deep.cpp", "cpp/sub/deep.cpp"),
                    ("python", "cam.py", "python/cam.py")]
    assert plain == ["CMakeLists.txt", "cpp/sub/deep.hpp"]
    code, plain = bm.green_node_files(str(root), ("python",))       # .cpp in a python project is just a file
    assert [n[1] for n in code] == ["cam.py"] and "cpp/solver.cpp" in plain
    bm.set_mark(str(root), "cpp/solver.cpp", bm.YELLOW)
    assert "solver.cpp" not in [n[1] for n in bm.green_node_files(str(root), ("cpp",))[0]]


def test_bound_nested_node_keeps_file_in_place_and_launches_it(tmp_path):
    from core import node_sources
    from core.generators.launch_generator import LaunchCompiler
    root = _mixed_project(tmp_path)
    (root / "src/python/tools").mkdir()
    code = "print('nested mine')\n"
    (root / "src/python/tools/deep.py").write_text(code, encoding="utf-8")
    node = {"node": {"name": "deep", "ports": [], "custom": {
        "bound_file": "python/tools/deep.py", "source_file": "deep.py", "code_content": ""}},
        "namespace": "", "container_group": "main"}
    created = node_sources.process_flat_nodes([node], str(root / "src/python"), "python")
    assert (root / "src/python/tools/deep.py").read_text(encoding="utf-8") == code        # neither overwritten nor copied
    assert not (root / "src/python/deep.py").exists()
    assert created[0]["relpath"] == "tools/deep.py"
    launch = LaunchCompiler(created, "mix").compile()
    assert "/python/tools/deep.py" in launch


@pytest.mark.parametrize("single", [True, False])
def test_green_file_becomes_node_bound_to_disk_file(app, tmp_path, single):
    from core import settings
    from ui import main_window as mw
    settings.set("single_canvas", single)
    w = mw.RosVisualRunner()
    try:
        root = _mixed_project(tmp_path)
        w.current_project_path = str(root)
        bm.set_mark(str(root), "cpp/solver.cpp", bm.GREEN)
        bm.set_mark(str(root), "python/cam.py", bm.GREEN)
        bm.set_mark(str(root), "cpp/sub/deep.cpp", bm.GREEN)
        assert w.sync_green_nodes() == 3
        assert w.sync_green_nodes() == 0                     # no duplicates the second time
        assert {n.get_property("bound_file") for g in w.graphs.all() for n in g.all_nodes()} \
            == {"cpp/solver.cpp", "python/cam.py", "cpp/sub/deep.cpp"}
        if not single:
            assert sorted(n.name() for n in w.graph_cpp.all_nodes()) == ["deep", "solver"]
            assert [n.name() for n in w.graph_py.all_nodes()] == ["cam"]

        res = w.project_manager.save_project(str(root))
        assert res and res[0] is True
        # the file's code is not overwritten by a template, the file stays in place
        assert "// mine" in (root / "src/cpp/solver.cpp").read_text(encoding="utf-8")
        assert "mine" in (root / "src/python/cam.py").read_text(encoding="utf-8")
        cm = (root / "src/CMakeLists.txt").read_text(encoding="utf-8")
        assert "add_executable(solver cpp/solver.cpp)" in cm and "add_executable(deep cpp/sub/deep.cpp)" in cm
        assert (root / "src/cpp/sub/deep.cpp").exists() and not (root / "src/cpp/deep.cpp").exists()
    finally:
        w._mark_clean(); w.close(); w.deleteLater()   # this test is not about "unsaved changes"


@pytest.mark.parametrize("single", [True, False])
def test_green_cmake_file_becomes_editable_file_node(app, tmp_path, single):
    from core import settings
    from ui import main_window as mw
    settings.set("single_canvas", single)
    w = mw.RosVisualRunner()
    try:
        root = _mixed_project(tmp_path)
        w.current_project_path = str(root)
        bm.set_mark(str(root), "cpp/solver.cpp", bm.YELLOW)
        assert w.project_manager.save_project(str(root))
        cmake = root / "src/CMakeLists.txt"
        assert cmake.exists()
        bm.set_mark(str(root), "CMakeLists.txt", bm.GREEN)
        bm.set_mark(str(root), "package.xml", bm.GREEN)
        assert w.sync_green_nodes() == 2 and w.sync_green_nodes() == 0
        nodes = [n for g in w.graphs.all() for n in g.all_nodes() if n.type_ == w.project_manager.FILE_NODE_TYPE]
        assert sorted(n.get_property("bound_file") for n in nodes) == ["CMakeLists.txt", "package.xml"]
        node = next(n for n in nodes if n.get_property("bound_file") == "CMakeLists.txt")
        assert node.get_property("code_content") == cmake.read_text(encoding="utf-8")

        # node edit -> file on Save (the USER zone lives, AUTO is regenerated)
        from core.generators.build_generator import CMAKE_USER_END
        edited = node.get_property("code_content").replace(
            CMAKE_USER_END, "find_package(yaml-cpp REQUIRED)\n" + CMAKE_USER_END)
        node.set_property("code_content", edited)
        assert w.project_manager.save_project(str(root))
        assert "find_package(yaml-cpp REQUIRED)" in cmake.read_text(encoding="utf-8")
        assert node.get_property("code_content") == cmake.read_text(encoding="utf-8")       # node updated from the file
        # file nodes went into neither the code nor the launch file
        assert not (root / "src/cpp/CMakeLists.cpp").exists()
        assert "CMakeLists" not in (root / "src/launch/project_launch.py").read_text(encoding="utf-8")

        # edit on disk -> node
        cmake.write_text(cmake.read_text(encoding="utf-8") + "# from disk\n", encoding="utf-8")
        w.project_manager.sync_file_nodes(str(root / "src"))
        assert "# from disk" in node.get_property("code_content")
    finally:
        w._mark_clean(); w.close(); w.deleteLater()   # this test is not about "unsaved changes"


@pytest.mark.parametrize("single", [True, False])
def test_file_node_survives_save_and_reload(app, tmp_path, single):
    from core import settings
    from ui import main_window as mw
    settings.set("single_canvas", single)
    w = mw.RosVisualRunner()
    try:
        root = _mixed_project(tmp_path)
        w.current_project_path = str(root)
        (root / "src/notes.yaml").write_text("a: 1\n", encoding="utf-8")
        bm.set_mark(str(root), "notes.yaml", bm.GREEN)
        assert w.sync_green_nodes() == 1
        assert w.project_manager.save_project(str(root))
        for g in w.graphs.all():
            g.clear_session()
        assert w.project_manager.load_project(str(root))
        found = [n for g in w.graphs.all() for n in g.all_nodes() if n.type_ == w.project_manager.FILE_NODE_TYPE]
        assert [n.get_property("bound_file") for n in found] == ["notes.yaml"]
        assert w.sync_green_nodes() == 0                      # no duplicate after loading
    finally:
        w._mark_clean(); w.close(); w.deleteLater()   # this test is not about "unsaved changes"


# ---------- regression: folder marks do not break CMake or launch ----------

def _g1_like(project):
    """A project like a real one: three nodes, helpers, kinematics_lib, test/ with main()."""
    from tests.test_save_safety import cpp_node
    src = project / "src"
    for d in ("include", "kinematics_lib", "test", "scripts", "python"):
        (src / d).mkdir(exist_ok=True)
    main = "#include <rclcpp/rclcpp.hpp>\n{inc}int main(int a,char**b){{return 0;}}\n"
    code = {
        "exec_node": main.format(inc='#include "executor_core.hpp"\n'),
        "plan_node": main.format(inc='#include "mission_core.hpp"\n'),
        "sim_node": main.format(inc=""),
    }
    (src / "cpp/executor_core.hpp").write_text("#pragma once\n", encoding="utf-8")
    (src / "cpp/executor_core.cpp").write_text('#include "executor_core.hpp"\n', encoding="utf-8")
    (src / "cpp/mission_core.hpp").write_text("#pragma once\n", encoding="utf-8")
    (src / "cpp/mission_core.cpp").write_text('#include "mission_core.hpp"\n', encoding="utf-8")
    (src / "include/api.hpp").write_text("#pragma once\n", encoding="utf-8")
    (src / "kinematics_lib/solver.hpp").write_text("#pragma once\n", encoding="utf-8")
    (src / "kinematics_lib/solver.cpp").write_text('#include "solver.hpp"\n', encoding="utf-8")
    (src / "test/test_solver.cpp").write_text("#include <gtest/gtest.h>\nint main(){}\n", encoding="utf-8")
    (src / "scripts/tool.py").write_text("print(1)\n", encoding="utf-8")
    return [cpp_node(n, c) for n, c in code.items()]


def test_folder_marks_do_not_change_launch_or_spawn_executables(project):
    from tests.test_save_safety import save
    nodes = _g1_like(project)
    save(project, nodes=nodes)
    before_launch = (project / "src/launch/project_launch.py").read_text(encoding="utf-8")
    before_cmake = (project / "src/CMakeLists.txt").read_text(encoding="utf-8")
    assert "kinematics_lib/solver.cpp" not in before_cmake

    assert (project / "src/cpp/executor_core.cpp").exists()        # helper written by hand, not by Studio: not a ghost
    _g1_like(project)                                              # restore the files and mark the folders
    for d in ("cpp", "include", "kinematics_lib", "launch", "scripts", "python", "test"):
        bm.set_mark(str(project), d, bm.YELLOW)
    bm.set_mark(str(project), "CMakeLists.txt", bm.GREEN)
    save(project, nodes=nodes)

    assert (project / "src/launch/project_launch.py").read_text(encoding="utf-8") == before_launch
    cmake = (project / "src/CMakeLists.txt").read_text(encoding="utf-8")
    assert "include_directories(include cpp kinematics_lib test)" in cmake      # include/ is not repeated
    assert "add_executable(test_solver" not in cmake                            # main() from a folder is not a binary
    assert cmake.count("add_executable(") == 3
    # every node: its own helpers (via #include) + kinematics_lib, no other node's helpers
    assert "add_executable(exec_node cpp/exec_node.cpp cpp/executor_core.cpp kinematics_lib/solver.cpp)" in cmake
    assert "add_executable(plan_node cpp/plan_node.cpp cpp/mission_core.cpp kinematics_lib/solver.cpp)" in cmake
    assert "add_executable(sim_node cpp/sim_node.cpp kinematics_lib/solver.cpp)" in cmake
    # marked files are still in place
    for rel in ("cpp/executor_core.cpp", "cpp/mission_core.cpp", "test/test_solver.cpp"):
        assert (project / "src" / rel).exists(), rel


def test_explicit_file_mark_still_makes_executable_from_main(project):
    from tests.test_save_safety import save
    nodes = _g1_like(project)
    bm.set_mark(str(project), "test/test_solver.cpp", bm.YELLOW)       # the file itself, not the folder
    save(project, nodes=nodes)
    assert "add_executable(test_solver test/test_solver.cpp" in \
        (project / "src/CMakeLists.txt").read_text(encoding="utf-8")


def test_hand_written_helper_without_node_is_not_ghost(project):
    """.cpp in cpp/ that Studio did not generate is left alone on Save, even without a mark."""
    from tests.test_save_safety import save
    nodes = _g1_like(project)
    (project / "src/cpp/stray.cpp").write_text("int helper(){return 1;}\n", encoding="utf-8")
    save(project, nodes=nodes)
    assert (project / "src/cpp/stray.cpp").exists()
