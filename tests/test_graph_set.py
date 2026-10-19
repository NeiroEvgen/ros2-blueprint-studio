"""GraphSet: the single access point to the canvases (groundwork for one shared canvas)."""
import os
import pytest

from ui.graph_set import GraphSet


class G:
    def __init__(self, *ids):
        self._nodes = [type("N", (), {"id": i})() for i in ids]

    def all_nodes(self):
        return self._nodes


class TestTwoGraphs:
    def test_all_and_current(self):
        py, cpp = G("a"), G("b")
        gs = GraphSet(py, cpp)
        assert not gs.is_single
        assert gs.all() == [py, cpp]
        assert gs.current(0) is py and gs.current(1) is cpp

    def test_for_language(self):
        py, cpp = G(), G()
        gs = GraphSet(py, cpp)
        assert gs.for_language("python") is py and gs.for_language("cpp") is cpp

    def test_find_node_in_either_graph(self):
        gs = GraphSet(G("a"), G("b"))
        assert gs.find_node("b").id == "b" and gs.find_node("a").id == "a"
        assert gs.find_node("zzz") is None


class TestSingleGraph:
    def test_same_graph_for_everything(self):
        g = G("x")
        gs = GraphSet(g, g)
        assert gs.is_single and gs.all() == [g]
        assert gs.current(0) is g and gs.current(1) is g
        assert gs.for_language("cpp") is g and gs.for_language("python") is g
        assert gs.find_node("x").id == "x"


def test_window_uses_graph_set(tmp_path):
    pytest.importorskip("PySide6.QtWidgets")
    pytest.importorskip("NodeGraphQt")
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    from PySide6 import QtWidgets
    QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    from ui import main_window as mw
    w = mw.RosVisualRunner()
    try:
        assert w.graphs.py is w.graph_py and w.graphs.cpp is w.graph_cpp
        assert w.graphs.current(0) is w.graph_py and w.graphs.current(1) is w.graph_cpp
        assert w.graphs.all() == [w.graph_py, w.graph_cpp]
    finally:
        w.close(); w.deleteLater()
