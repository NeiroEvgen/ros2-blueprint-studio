"""
Save deleted other people's code: ghost cleanup removed any .cpp/.py in src/cpp|python
that was not in the graph. Open a folder without state.yaml and press Save/Run = lose src/cpp.
Now only what Studio generated itself (generated.json) counts as a ghost,
and it goes to .blueprint/trash/ instead of being deleted.
"""
import glob
import os

import pytest

from core import generated_files
from core.project_manager import ProjectManager
from tests.test_save_safety import FakeGraph, cpp_node, save, MAIN, project  # noqa: F401


def _trashed(project, rel):
    return glob.glob(str(project / ".blueprint" / "trash" / "*" / rel))


def test_hand_written_code_survives_save_without_state(project):
    """A folder with someone else's code, no state.yaml yet, empty graph -> Save deletes nothing."""
    drv = project / "src/cpp/my_driver.cpp"
    drv.write_text("// human code\nint main(){}\n", encoding="utf-8")
    save(project, nodes=[])
    assert drv.read_text(encoding="utf-8").startswith("// human code")


def test_hand_written_code_survives_next_to_nodes(project):
    (project / "src/cpp/my_driver.cpp").write_text("int helper(){return 1;}\n", encoding="utf-8")
    save(project, nodes=[cpp_node("a", MAIN)])
    save(project, nodes=[cpp_node("a", MAIN)])
    assert (project / "src/cpp/my_driver.cpp").exists()


def test_generated_ghost_goes_to_trash(project):
    save(project, nodes=[cpp_node("a", MAIN), cpp_node("b", MAIN)])
    assert generated_files.load(str(project)) == {"cpp/a.cpp", "cpp/b.cpp"}
    save(project, nodes=[cpp_node("a", MAIN)])
    assert not (project / "src/cpp/b.cpp").exists()
    assert len(_trashed(project, "cpp/b.cpp")) == 1
    assert generated_files.load(str(project)) == {"cpp/a.cpp"}


def test_legacy_project_without_manifest_uses_previous_state(project):
    """Project before 0.6.0: no generated.json, ghosts come from the previous state.yaml."""
    save(project, nodes=[cpp_node("a", MAIN), cpp_node("b", MAIN)])
    os.remove(generated_files.manifest_path(str(project)))
    (project / "src/cpp/mine.cpp").write_text("int helper(){return 1;}\n", encoding="utf-8")
    save(project, nodes=[cpp_node("a", MAIN)])
    assert not (project / "src/cpp/b.cpp").exists() and _trashed(project, "cpp/b.cpp")
    assert (project / "src/cpp/mine.cpp").exists()


def test_broken_manifest_deletes_nothing(project):
    save(project, nodes=[cpp_node("a", MAIN), cpp_node("b", MAIN)])
    with open(generated_files.manifest_path(str(project)), "w", encoding="utf-8") as f:
        f.write("{not json")
    save(project, nodes=[cpp_node("a", MAIN)])
    assert (project / "src/cpp/b.cpp").exists()


class ClearableGraph(FakeGraph):
    def __init__(self, session=None):
        super().__init__(session)
        self.cleared = 0

    def clear_session(self):
        self.cleared += 1


def test_unreadable_state_blocks_save(project):
    """state.yaml exists but cannot be read -> empty canvas; Save must not overwrite the project."""
    save(project, nodes=[cpp_node("a", MAIN)])
    state = project / ".blueprint/state.yaml"
    state.write_text("graphs: [this: is: not yaml", encoding="utf-8")
    pm = ProjectManager(ClearableGraph(), ClearableGraph())
    assert pm.load_project(str(project)) is False
    assert pm.save_project(str(project)) is False
    assert pm.last_warnings
    assert (project / "src/cpp/a.cpp").exists()
    assert state.read_text(encoding="utf-8") == "graphs: [this: is: not yaml"


def test_load_without_state_clears_canvas(tmp_path):
    """Folder without state.yaml: the canvas is still cleared (otherwise the previous project's nodes move into it)."""
    g = ClearableGraph()
    pm = ProjectManager(g, g)
    assert pm.load_project(str(tmp_path)) is False
    assert g.cleared >= 1 and pm.load_failed_path is None


# ---------- the same through the main window (Qt offscreen) ----------

QtWidgets = None


@pytest.fixture
def window(tmp_path):
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    qtw = pytest.importorskip("PySide6.QtWidgets")
    pytest.importorskip("NodeGraphQt")
    app = qtw.QApplication.instance() or qtw.QApplication([])  # noqa: F841
    from ui import main_window as mw
    w = mw.RosVisualRunner()
    w.system_log = lambda *_: None
    yield w
    w.deleteLater()


def test_open_foreign_folder_after_project_keeps_its_code(window, tmp_path, monkeypatch):
    import json
    import shutil
    from PySide6 import QtWidgets as qtw
    monkeypatch.setattr(qtw.QMessageBox, "question", staticmethod(lambda *a, **k: qtw.QMessageBox.No))
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    a = tmp_path / "proj_a"
    shutil.copytree(os.path.join(root, "examples", "cpp_pubsub"), a)
    b = tmp_path / "proj_b"
    (b / "src/cpp").mkdir(parents=True)
    (b / "project_config.json").write_text(json.dumps({"type": "cpp"}), encoding="utf-8")
    drv = b / "src/cpp/my_driver.cpp"
    drv.write_text("// human code\nint main(){}\n", encoding="utf-8")

    def nodes():
        return [n for g in window.graphs.all() for n in g.all_nodes()]

    window.on_load_project_init(str(a))
    assert nodes()
    window.on_load_project_init(str(b))
    assert nodes() == []                              # project A's nodes did not move into B
    window.on_save_project()
    assert drv.exists()
    assert not (b / "src/cpp/talker.cpp").exists()
