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
    (cpp / "node.cpp").write_text("#include <rclcpp/rclcpp.hpp>\nint main(){}\n")
    (cpp / "kin" / "solver.hpp").write_text("#pragma once\n")
    (cpp / "kin" / "solver.cpp").write_text('#include "solver.hpp"\n#include <opencv2/core.hpp>\n')
    (cpp / "kin" / "tool.cpp").write_text("int main(){}\n")
    return proj, proj / "src"


def _gen(src):
    ProjectManager(None, None)._generate_cpp_build_files(str(src), [])
    return (src / "CMakeLists.txt").read_text()


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
    assert "find_package(OpenCV REQUIRED)" in cmake      # зависимость из отмеченного файла


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
    (src / "cpp" / "kin" / "node.cpp").write_text("int main(){}\n")
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
    fb._on_click(it, 1)                       # не колонка отметок
    assert len(seen) == 3


def test_marks_hidden_for_non_project_source(app):
    from ui.file_browser import FileBrowser, COL_MARK
    fb = FileBrowser()
    assert fb.tree.isColumnHidden(COL_MARK)


def test_save_keeps_marked_top_level_file(project):
    """Файл в cpp/ без ноды, но с отметкой, не считается «призраком» и не удаляется при Save."""
    from tests.test_save_safety import save, cpp_node, MAIN
    save(project, nodes=[cpp_node("a", MAIN), cpp_node("b", MAIN)])
    bm.set_mark(str(project), "cpp/b.cpp", bm.YELLOW)
    save(project, nodes=[cpp_node("a", MAIN)])
    assert (project / "src/cpp/b.cpp").exists()
    bm.set_mark(str(project), "cpp/b.cpp", None)
    save(project, nodes=[cpp_node("a", MAIN)])
    assert not (project / "src/cpp/b.cpp").exists()


# ---------- зелёные отметки → ноды на столе ----------

def _mixed_project(tmp_path):
    import json
    root = tmp_path / "mix"
    for d in ("src/cpp", "src/python", ".blueprint", "docker"):
        (root / d).mkdir(parents=True)
    (root / "project_config.json").write_text(json.dumps({"name": "mix", "type": "mixed"}))
    (root / "src/cpp/solver.cpp").write_text("#include <rclcpp/rclcpp.hpp>\nint main(){}\n// mine\n")
    (root / "src/python/cam.py").write_text("print('mine')\n")
    (root / "src/cpp/sub").mkdir()
    (root / "src/cpp/sub/deep.cpp").write_text("int main(){}\n")
    return root


def test_green_node_files_only_top_level(tmp_path):
    root = _mixed_project(tmp_path)
    for rel in ("cpp/solver.cpp", "python/cam.py", "cpp/sub/deep.cpp"):
        bm.set_mark(str(root), rel, bm.GREEN)
    assert bm.green_node_files(str(root), ("cpp", "python")) == [
        ("cpp", "solver.cpp", "cpp/solver.cpp"), ("python", "cam.py", "python/cam.py")]
    bm.set_mark(str(root), "cpp/solver.cpp", bm.YELLOW)
    assert [f for _, f, _ in bm.green_node_files(str(root), ("cpp", "python"))] == ["cam.py"]


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
        assert w.sync_green_nodes() == 2
        assert w.sync_green_nodes() == 0                     # повторно дублей нет
        assert {n.get_property("source_file") for g in w.graphs.all() for n in g.all_nodes()} \
            == {"solver.cpp", "cam.py"}
        if not single:
            assert [n.name() for n in w.graph_cpp.all_nodes()] == ["solver"]
            assert [n.name() for n in w.graph_py.all_nodes()] == ["cam"]

        res = w.project_manager.save_project(str(root))
        assert res and res[0] is True
        # код файла не затёрт шаблоном, файл остался на месте
        assert "// mine" in (root / "src/cpp/solver.cpp").read_text()
        assert "mine" in (root / "src/python/cam.py").read_text()
        assert "add_executable(solver cpp/solver.cpp)" in (root / "src/CMakeLists.txt").read_text()
    finally:
        w.close(); w.deleteLater()
