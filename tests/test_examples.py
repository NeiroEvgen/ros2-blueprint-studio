"""Учебные примеры examples/{py,cpp,mixed}_pubsub: валидны, открываются в обоих режимах, без личных путей."""
import ast
import os
import re
import shutil

import pytest

from core import state_store

ROOT = os.path.join(os.path.dirname(os.path.dirname(__file__)), "examples")
NAMES = ["py_pubsub", "cpp_pubsub", "mixed_pubsub"]


def _files(name, ext):
    base = os.path.join(ROOT, name, "src")
    return [os.path.join(dp, f) for dp, _, fs in os.walk(base) for f in fs if f.endswith(ext)]


@pytest.mark.parametrize("name", NAMES)
def test_state_has_two_nodes_and_one_wire(name):
    st = state_store.read_state(os.path.join(ROOT, name))
    assert st is not None
    nodes = st.merged["nodes"]
    nodes = nodes.values() if isinstance(nodes, dict) else nodes
    assert len(list(nodes)) == 2
    assert len(st.merged.get("connections", [])) == 1


@pytest.mark.parametrize("name", NAMES)
def test_python_sources_compile(name):
    for p in _files(name, ".py"):
        ast.parse(open(p, encoding="utf-8").read(), filename=p)


@pytest.mark.parametrize("name", ["cpp_pubsub", "mixed_pubsub"])
def test_cpp_sources_are_filled_in(name):
    for p in _files(name, ".cpp"):
        text = open(p, encoding="utf-8").read()
        assert not re.search(r"\{(class_name|node_name|topic_name|EXEC_\w+)\}", text), p
        assert '"/chatter"' in text


def test_mixed_has_both_languages():
    assert _files("mixed_pubsub", ".py") and _files("mixed_pubsub", ".cpp")
    assert any(p.endswith("talker.py") for p in _files("mixed_pubsub", ".py"))


@pytest.mark.parametrize("name", NAMES)
def test_no_personal_paths_or_machine_ids(name):
    for dp, _, fs in os.walk(os.path.join(ROOT, name)):
        for f in fs:
            text = open(os.path.join(dp, f), encoding="utf-8", errors="ignore").read()
            assert not re.search(r"/home/|/Users/|C:\\\\|C:/Users", text), os.path.join(dp, f)


# ---------- открытие в Studio ----------
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
QtWidgets = pytest.importorskip("PySide6.QtWidgets")
pytest.importorskip("NodeGraphQt")


@pytest.fixture(scope="module")
def app():
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


def _win(single):
    from core import settings
    from ui import main_window as mw
    settings.set("single_canvas", single)
    return mw.RosVisualRunner()


def _names(g):
    return sorted(n.name() for n in g.all_nodes())


@pytest.mark.parametrize("single", [True, False])
@pytest.mark.parametrize("name", NAMES)
def test_example_opens(app, tmp_path, name, single):
    dst = tmp_path / name
    shutil.copytree(os.path.join(ROOT, name), dst)
    w = _win(single)
    try:
        assert w.project_manager.load_project(str(dst))
        total = sum(len(g.all_nodes()) for g in w.graphs.all())
        assert total == 2
        if name == "mixed_pubsub" and not single:
            assert _names(w.graph_py) == ["talker"] and _names(w.graph_cpp) == ["listener"]
    finally:
        w.close(); w.deleteLater()
