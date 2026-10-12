"""
Режим одного холста: единый граф Python+C++ сохраняется в state v4, генерирует код
в обе директории и открывается обратно (в одном графе и в двух вкладках).
"""
import json
import os

import pytest
import yaml

from core import graph_ir, state_store
from core.project_manager import ProjectManager
from tests.test_save_safety import FakeGraph, cpp_node, MAIN
from tests.test_mixed_project import py_node, PY_CODE


@pytest.fixture
def mixed(tmp_path):
    root = tmp_path / "mix"
    for d in ("src", ".blueprint", "docker"):
        (root / d).mkdir(parents=True)
    (root / "project_config.json").write_text(json.dumps({"name": "mix", "type": "mixed"}))
    return root


def test_single_meta_splits_by_language_and_keeps_groups_with_children():
    merged = {"nodes": [
        py_node("p1"), cpp_node("c1", MAIN),
        {"id": "g", "type_": "ros.nodes.RosGroupNode", "name": "g",
         "custom": {"group_uid": "u1"}, "ports": []},
        dict(cpp_node("c2", MAIN), custom={"node_name": "c2", "code_content": MAIN,
                                           "source_file": "", "parent_group_id": "u1"}),
    ], "connections": []}
    meta = graph_ir.single_meta(merged)
    assert set(meta["cpp_ids"]) == {"c1", "c2", "g"}
    assert meta["cpp_dy"] == 0.0 and meta["remap"] == {}


def test_single_save_writes_v4_and_generates_both_languages(mixed):
    g = FakeGraph({"nodes": [py_node("p1", PY_CODE), cpp_node("c1", MAIN)], "connections": []})
    res = ProjectManager(g, g).save_project(str(mixed))
    assert res and res[0] is True
    raw = yaml.safe_load((mixed / ".blueprint/state.yaml").read_text(encoding="utf-8"))
    assert str(raw["meta"]["version"]).startswith("4")
    assert len(raw["graph"]["nodes"]) == 2
    assert any((mixed / "src/python").rglob("*.py"))
    assert any((mixed / "src/cpp").rglob("*.cpp"))


def test_v4_state_reads_back_into_two_languages(mixed):
    g = FakeGraph({"nodes": [py_node("p1", PY_CODE), cpp_node("c1", MAIN)], "connections": []})
    ProjectManager(g, g).save_project(str(mixed))
    st = state_store.read_state(str(mixed))
    ids = lambda s: {n["id"] for n in s["nodes"]}
    assert ids(st.py) == {"p1"} and ids(st.cpp) == {"c1"}


def test_two_graph_save_still_v36(mixed):
    py = FakeGraph({"nodes": [py_node("p1", PY_CODE)], "connections": []})
    cpp = FakeGraph({"nodes": [cpp_node("c1", MAIN)], "connections": []})
    assert ProjectManager(py, cpp).save_project(str(mixed))[0] is True
    raw = yaml.safe_load((mixed / ".blueprint/state.yaml").read_text(encoding="utf-8"))
    assert "graph" not in raw or "meta" not in raw


def test_first_v4_write_keeps_v36_backup(mixed):
    py = FakeGraph({"nodes": [py_node("p1", PY_CODE)], "connections": []})
    cpp = FakeGraph({"nodes": [cpp_node("c1", MAIN)], "connections": []})
    ProjectManager(py, cpp).save_project(str(mixed))
    g = FakeGraph({"nodes": [py_node("p1", PY_CODE), cpp_node("c1", MAIN)], "connections": []})
    ProjectManager(g, g).save_project(str(mixed))
    assert (mixed / ".blueprint/state.v3.6.yaml.bak").exists()


# ---------- реальные холсты (Qt offscreen) ----------

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
QtWidgets = pytest.importorskip("PySide6.QtWidgets")
pytest.importorskip("NodeGraphQt")


@pytest.fixture(scope="module")
def app():
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


def _make_win(single):
    from core import settings
    from ui import main_window as mw
    settings.set("single_canvas", single)
    return mw.RosVisualRunner()


@pytest.fixture
def single_win(app):
    w = _make_win(True)
    yield w
    w.close(); w.deleteLater()


@pytest.fixture
def two_win(app):
    w = _make_win(False)
    yield w
    w.close(); w.deleteLater()


def _names(graph):
    return sorted(n.name() for n in graph.all_nodes())


def test_single_window_has_one_graph_and_one_canvas_tab(single_win):
    w = single_win
    assert w.graph_py is w.graph_cpp
    assert w.graphs.is_single
    assert w.ui.tabs.indexOf(w.graph_py.widget) >= 0
    w.ui.set_visible_graph("mixed")          # не должен падать при одной вкладке
    w.ui.set_visible_graph("cpp")


def test_default_window_keeps_two_graphs(two_win):
    assert two_win.graph_py is not two_win.graph_cpp


def test_single_round_trip_and_reopen_in_two_tabs(single_win, two_win, tmp_path):
    root = tmp_path / "mix"
    for d in ("src", ".blueprint", "docker"):
        (root / d).mkdir(parents=True)
    (root / "project_config.json").write_text(json.dumps({"name": "mix", "type": "mixed"}))

    w = single_win
    w.graph_py.create_node("ros.py.PyStringPubNode", name="py_pub", pos=[0, 0])
    w.graph_py.create_node("ros.cpp.CppCustomNode", name="cpp_node", pos=[100, 0])
    res = w.project_manager.save_project(str(root))
    assert res and res[0] is True
    raw = yaml.safe_load((root / ".blueprint/state.yaml").read_text(encoding="utf-8"))
    assert str(raw["meta"]["version"]).startswith("4")

    w.graph_py.clear_session()
    assert w.project_manager.load_project(str(root))
    assert _names(w.graph_py) == ["cpp_node", "py_pub"]

    assert two_win.project_manager.load_project(str(root))
    assert _names(two_win.graph_py) == ["py_pub"]
    assert _names(two_win.graph_cpp) == ["cpp_node"]
