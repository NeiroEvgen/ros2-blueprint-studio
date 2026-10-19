"""
A code edit in the properties window reaches the file (the disk used to always win and the edit was lost),
while a file edit in an IDE still beats an unchanged node. Conflict -> disk.
"""
import json
import os

import pytest

from core.node_sources import choose_code, code_hash, process_flat_nodes

H = code_hash("old")


@pytest.mark.parametrize("node,disk,synced,want,why", [
    ("old", "old", H, "old", "same"),
    ("new", "old", H, "new", "node"),            # edit in Studio
    ("old", "ide", H, "ide", "disk"),            # edit in the IDE
    ("new", "ide", H, "ide", "conflict"),        # both changed
    ("new", "old", None, "old", "disk"),         # old project without a fingerprint: never overwrite the disk
    ("x", "", H, "x", "no_disk"),
    ("", "disk", H, "disk", "same"),
    ("a\r\nb", "a\nb", H, "a\nb", "same"),        # line endings do not count as an edit
])
def test_choose_code(node, disk, synced, want, why):
    assert choose_code(node, disk, synced) == (want, why)


def _node(code, h=None):
    custom = {"node_name": "n", "code_content": code, "source_file": "n.py"}
    if h is not None:
        custom["code_sync_hash"] = h
    return {"id": "1", "name": "n", "type_": "ros.py.PyCustomNode", "ports": [], "custom": custom}


def _run(tmp_path, node):
    process_flat_nodes([{"node": node, "namespace": ""}], str(tmp_path), "python")
    return (tmp_path / "n.py").read_text(encoding="utf-8")


def test_ui_edit_reaches_file_and_hash_is_recorded(tmp_path):
    n = _node("print(1)\n")
    assert _run(tmp_path, n) == "print(1)\n"
    h1 = n["custom"]["code_sync_hash"]
    n2 = _node("print(2)\n", h1)                   # edit in the properties window
    assert _run(tmp_path, n2) == "print(2)\n"
    assert n2["custom"]["code_sync_hash"] == code_hash("print(2)\n")


def test_external_edit_wins_over_unchanged_node(tmp_path):
    n = _node("print(1)\n")
    _run(tmp_path, n)
    (tmp_path / "n.py").write_text("print('ide')\n", encoding="utf-8")
    n2 = _node("print(1)\n", n["custom"]["code_sync_hash"])
    assert _run(tmp_path, n2) == "print('ide')\n"


def test_conflict_keeps_disk(tmp_path):
    n = _node("print(1)\n")
    _run(tmp_path, n)
    (tmp_path / "n.py").write_text("print('ide')\n", encoding="utf-8")
    n2 = _node("print('studio')\n", n["custom"]["code_sync_hash"])
    assert _run(tmp_path, n2) == "print('ide')\n"


# ---------- live canvases ----------
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
QtWidgets = pytest.importorskip("PySide6.QtWidgets")
pytest.importorskip("NodeGraphQt")


@pytest.fixture(scope="module")
def app():
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


@pytest.fixture
def win(app):
    from ui import main_window as mw
    w = mw.RosVisualRunner()
    yield w
    w._mark_clean(); w.close(); w.deleteLater()   # this test is not about "unsaved changes"


def test_properties_edit_survives_save_and_write_back(win, tmp_path):
    root = tmp_path / "p"
    for d in ("src", ".blueprint", "docker"):
        (root / d).mkdir(parents=True)
    (root / "project_config.json").write_text(json.dumps({"name": "p", "type": "cpp"}))
    node = win.graph_cpp.create_node("ros.cpp.CppCustomNode", name="n", pos=[0, 0])
    node.set_property("code_content", "int main(){return 1;}\n")
    assert win.project_manager.save_project(str(root))[0] is True
    (cpp,) = list((root / "src" / "cpp").glob("*.cpp"))
    assert cpp.read_text(encoding="utf-8") == "int main(){return 1;}\n"

    node.set_property("code_content", "int main(){return 2;}\n")       # edit in the properties window
    assert win.project_manager.save_project(str(root))[0] is True
    assert cpp.read_text(encoding="utf-8") == "int main(){return 2;}\n"

    cpp.write_text("int main(){return 3;}\n", encoding="utf-8")       # edit in the IDE
    assert win.project_manager.save_project(str(root))[0] is True
    assert cpp.read_text(encoding="utf-8") == "int main(){return 3;}\n"
    assert node.get_property("code_content") == "int main(){return 3;}\n"   # the node caught up with the file

    node.set_property("code_content", "int main(){return 4;}\n")       # and another edit in Studio
    assert win.project_manager.save_project(str(root))[0] is True
    assert cpp.read_text(encoding="utf-8") == "int main(){return 4;}\n"


def test_loaded_project_edit_wins_after_first_save(win, tmp_path):
    """After opening a project a Studio edit wins right away: the fingerprint is set on load."""
    root = tmp_path / "p"
    for d in ("src", ".blueprint", "docker"):
        (root / d).mkdir(parents=True)
    (root / "project_config.json").write_text(json.dumps({"name": "p", "type": "cpp"}))
    node = win.graph_cpp.create_node("ros.cpp.CppCustomNode", name="n", pos=[0, 0])
    node.set_property("code_content", "int main(){return 1;}\n")
    assert win.project_manager.save_project(str(root))[0] is True
    win.graph_cpp.clear_session()
    assert win.project_manager.load_project(str(root))
    (node,) = win.graph_cpp.all_nodes()
    node.set_property("code_content", "int main(){return 9;}\n")
    assert win.project_manager.save_project(str(root))[0] is True
    (cpp,) = list((root / "src" / "cpp").glob("*.cpp"))
    assert cpp.read_text(encoding="utf-8") == "int main(){return 9;}\n"
