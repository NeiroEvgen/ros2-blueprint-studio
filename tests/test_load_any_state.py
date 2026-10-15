"""
Загрузка проекта принимает ЛЮБУЮ версию state.yaml: 3.6 (две вкладки) и 4 (один общий граф).
Проект, сохранённый новым режимом, поэтому всегда открывается и в режиме двух вкладок.
Реальные холсты NodeGraphQt (Qt offscreen).
"""
import os
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
QtWidgets = pytest.importorskip("PySide6.QtWidgets")
pytest.importorskip("NodeGraphQt")
import yaml  # noqa: E402

from core import graph_ir, state_store  # noqa: E402


@pytest.fixture(scope="module")
def app():
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


@pytest.fixture
def win(app):
    from ui import main_window as mw
    w = mw.RosVisualRunner()
    yield w
    w.close(); w.deleteLater()


def _names(graph):
    return sorted(n.name() for n in graph.all_nodes())


def _fill(win):
    a = win.graph_py.create_node("ros.py.PyStringPubNode", name="py_pub", pos=[0, 0])
    b = win.graph_cpp.create_node("ros.cpp.CppCustomNode", name="cpp_node", pos=[0, 0])
    return a, b


def _project(tmp_path, doc):
    proj = tmp_path / "p"
    (proj / ".blueprint").mkdir(parents=True)
    (proj / "src").mkdir()
    with open(proj / ".blueprint" / "state.yaml", "w", encoding="utf-8") as f:
        yaml.dump(doc, f, sort_keys=False, allow_unicode=True)
    return str(proj)


def test_loads_legacy_v36(win, tmp_path):
    _fill(win)
    doc = {"meta": {"version": "3.6"},
           "graphs": {"python": win.graph_py.serialize_session(), "cpp": win.graph_cpp.serialize_session()}}
    path = _project(tmp_path, doc)
    win.graph_py.clear_session(); win.graph_cpp.clear_session()
    assert win.project_manager.load_project(path)
    assert _names(win.graph_py) == ["py_pub"] and _names(win.graph_cpp) == ["cpp_node"]


def test_loads_v4_into_two_tabs(win, tmp_path):
    _fill(win)
    merged, meta = graph_ir.merge_sessions(win.graph_py.serialize_session(), win.graph_cpp.serialize_session())
    path = _project(tmp_path, state_store.build_v4(merged, meta))
    win.graph_py.clear_session(); win.graph_cpp.clear_session()
    assert win.project_manager.load_project(path)
    assert _names(win.graph_py) == ["py_pub"] and _names(win.graph_cpp) == ["cpp_node"]


def test_v4_cpp_layout_offset_is_undone(win, tmp_path):
    a, b = _fill(win)
    a.set_pos(10, 20); b.set_pos(10, 20)             # одинаковые позиции в двух вкладках
    merged, meta = graph_ir.merge_sessions(win.graph_py.serialize_session(), win.graph_cpp.serialize_session())
    path = _project(tmp_path, state_store.build_v4(merged, meta))
    win.graph_py.clear_session(); win.graph_cpp.clear_session()
    win.project_manager.load_project(path)
    (py_node,) = win.graph_py.all_nodes(); (cpp_node,) = win.graph_cpp.all_nodes()
    assert list(py_node.pos()) == [10, 20] and list(cpp_node.pos()) == [10, 20]


def test_missing_state_returns_false(win, tmp_path):
    (tmp_path / "empty" / ".blueprint").mkdir(parents=True)
    assert win.project_manager.load_project(str(tmp_path / "empty")) is False
