"""
Palettes lived at the relative path nodes/user_palettes: starting Studio from outside the
application folder (shortcut, another cwd) made saved palettes "disappear" or get written elsewhere.
"""
import os

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
QtWidgets = pytest.importorskip("PySide6.QtWidgets")
pytest.importorskip("NodeGraphQt")

from core import palette_share  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def test_palettes_root_is_absolute_and_inside_app():
    assert os.path.isabs(palette_share.PALETTES_ROOT)
    assert palette_share.PALETTES_ROOT == os.path.join(ROOT, "nodes", "user_palettes")


@pytest.fixture
def win(tmp_path, monkeypatch):
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])  # noqa: F841
    from ui import main_window as mw
    monkeypatch.setattr(palette_share, "PALETTES_ROOT", str(tmp_path / "palettes"))
    w = mw.RosVisualRunner()
    w.system_log = lambda *_: None
    elsewhere = tmp_path / "elsewhere"
    elsewhere.mkdir()
    monkeypatch.chdir(elsewhere)                   # Studio started outside its folder
    yield w
    w._mark_clean()
    w.close()


def _palette_codes(w):
    pal = w.ui.node_palette
    from PySide6 import QtCore
    return [pal.item(i).data(QtCore.Qt.UserRole) for i in range(pal.count())]


def test_saved_palette_node_found_from_other_cwd(win, tmp_path, monkeypatch):
    monkeypatch.setattr(QtWidgets.QInputDialog, "getText",
                        staticmethod(lambda *a, **k: ("MyPal", True)))
    graph = win.graphs.all()[0]
    node = graph.create_node("ros.py.PyStringPubNode", name="Talker")
    win.on_save_node_to_palette(node)

    assert (tmp_path / "palettes" / "MyPal" / "Talker" / "node.yaml").exists()
    assert not (tmp_path / "elsewhere" / "nodes").exists()
    assert "USER_NODE:MyPal/Talker" in _palette_codes(win)

    before = len(graph.all_nodes())
    win.add_user_node_logic("USER_NODE:MyPal/Talker")
    assert len(graph.all_nodes()) == before + 1
