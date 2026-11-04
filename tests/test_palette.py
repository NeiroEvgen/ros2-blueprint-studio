"""
Node palette: search, language filter, favorites, A-Z order. The model (ui/palette_model.py) has no Qt;
the panel is checked on the real main window offscreen.
"""
import os
import shutil

import pytest

from ui import palette_model as pm
from ui.palette_model import Entry

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def all_classes():
    pytest.importorskip("NodeGraphQt")
    from nodes import ALL_NODE_CLASSES
    return ALL_NODE_CLASSES


# ---------- model ----------

class TestEntries:
    def test_mixed_project_shows_both_languages_single_shows_one(self):
        mixed = pm.builtin_entries(all_classes(), "mixed")
        py = pm.builtin_entries(all_classes(), "python")
        langs = lambda es: {e.language for e in es}
        assert {"python", "cpp"} <= langs(mixed)
        assert "cpp" not in langs(py) and "python" in langs(py)
        assert all(not e.id.endswith(".FileNode") and "internal" not in e.id for e in mixed)

    def test_categories(self):
        cats = {e.id: e.category for e in pm.builtin_entries(all_classes(), "mixed")}
        assert cats["ros.py.PyStringPubNode"] == "publishers"
        assert cats["ros.cpp.CppStringSubNode"] == "subscribers"
        assert cats["ros.py.PyServiceClientNode"] == "services"
        assert cats["ros.py.PyTimerNode"] == "logic"

    def test_user_palette_language_comes_from_node_yaml(self, tmp_path):
        for palette, node, type_ in (("arm", "Render", "ros.cpp.CppCustomNode"),
                                     ("arm", "Viewer", "ros.py.PyCustomNode"), ("arm", "Readme", "ros.nodes.NoteNode")):
            d = tmp_path / palette / node
            d.mkdir(parents=True)
            (d / "node.yaml").write_text(f"name: {node}\ntype: {type_}\n", encoding="utf-8")
        (tmp_path / "arm" / "broken").mkdir()
        (tmp_path / "arm" / "broken" / "node.yaml").write_text("[not yaml", encoding="utf-8")
        mixed = {e.name: e for e in pm.user_entries(str(tmp_path), "mixed")}
        assert mixed["Render"].language == "cpp" and mixed["Readme"].language == ""
        assert mixed["Render"].id == "USER_NODE:arm/Render" and mixed["Render"].category == "palette:arm"
        py = {e.name for e in pm.user_entries(str(tmp_path), "python")}
        assert "Render" not in py and {"Viewer", "Readme", "broken"} <= py


E = [Entry("ros.py.PyStringPubNode", "StringPub", "publishers", "python", "Publishes text"),
     Entry("ros.cpp.CppStringPubNode", "StringPub", "publishers", "cpp"),
     Entry("ros.py.PyServiceServerNode", "ServiceServer", "services", "python", "Answers AddTwoInts"),
     Entry("ros.nodes.NoteNode", "Note", "tools"),
     Entry("USER_NODE:arm/Render", "Render", "palette:arm", "cpp", "Palette: arm")]


def ids(groups):
    return [(key, [e.id for e in items]) for key, items in groups]


class TestArrange:
    def test_search_is_case_insensitive_and_all_words_must_match(self):
        assert ids(pm.arrange(E, "SERV")) == [("services", ["ros.py.PyServiceServerNode"])]
        assert ids(pm.arrange(E, "string c++")) == [("publishers", ["ros.cpp.CppStringPubNode"])]
        assert ids(pm.arrange(E, "addtwoints")) == [("services", ["ros.py.PyServiceServerNode"])]
        assert ids(pm.arrange(E, "arm")) == [("palette:arm", ["USER_NODE:arm/Render"])]
        assert pm.arrange(E, "nothing like this") == []

    def test_language_filter_keeps_neutral_nodes(self):
        got = dict(ids(pm.arrange(E, language="cpp")))
        assert got["publishers"] == ["ros.cpp.CppStringPubNode"]
        assert got["tools"] == ["ros.nodes.NoteNode"] and "services" not in got

    def test_favorites_first_and_only_once(self):
        got = ids(pm.arrange(E, favorites={"ros.nodes.NoteNode", "USER_NODE:arm/Render"}))
        assert got[0] == ("favorites", ["ros.nodes.NoteNode", "USER_NODE:arm/Render"])
        assert all("ros.nodes.NoteNode" not in items for key, items in got[1:])

    def test_favorites_only(self):
        assert ids(pm.arrange(E, favorites={"ros.nodes.NoteNode"}, favorites_only=True)) == \
            [("favorites", ["ros.nodes.NoteNode"])]

    def test_groups_in_fixed_order_and_a_z_inside(self):
        entries = E + [Entry("ros.py.A", "Alpha", "publishers", "python")]
        got = ids(pm.arrange(entries))
        assert [k for k, _ in got] == ["publishers", "services", "tools", "palette:arm"]
        assert got[0][1][0] == "ros.py.A"

    def test_label_shows_language_only_when_asked(self):
        assert E[1].label(True) == "StringPub  · C++" and E[1].label(False) == "StringPub"
        assert E[3].label(True) == "Note"


# ---------- panel ----------

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")


@pytest.fixture
def win(tmp_path):
    qtw = pytest.importorskip("PySide6.QtWidgets")
    pytest.importorskip("NodeGraphQt")
    qtw.QApplication.instance() or qtw.QApplication([])
    from ui import main_window as mw
    p = tmp_path / "proj"
    shutil.copytree(os.path.join(ROOT, "examples", "mixed_pubsub"), p)
    w = mw.RosVisualRunner()
    w.on_load_project_init(str(p))
    yield w
    w._mark_clean()
    w.close()


def rows(w):
    from PySide6 import QtCore
    pal = w.ui.node_palette
    return [(pal.item(i).text(), pal.item(i).data(QtCore.Qt.UserRole)) for i in range(pal.count())]


def node_ids(w):
    return [i for _, i in rows(w) if i]


def test_mixed_project_labels_language_and_filters(win):
    texts = [t for t, i in rows(win) if i]
    assert "StringPub  · Py" in texts and "StringPub  · C++" in texts
    assert win.ui.palette_lang.isVisibleTo(win)
    win.ui.palette_lang.setCurrentIndex(2)                                    # C++
    assert all(not i.startswith("ros.py.") for i in node_ids(win))


def test_single_language_project_hides_the_language_filter(win, tmp_path):
    p = tmp_path / "py"
    shutil.copytree(os.path.join(ROOT, "examples", "py_pubsub"), p)
    win.on_load_project_init(str(p))
    assert not win.ui.palette_lang.isVisibleTo(win)
    assert "StringPub" in [t for t, i in rows(win) if i]                      # no language tag needed


def test_search_and_nothing_found(win):
    win.ui.palette_search.setText("service server")
    assert sorted(node_ids(win)) == ["ros.cpp.CppServiceServerNode", "ros.py.PyServiceServerNode"]
    win.ui.palette_search.setText("zzz-no-such-node")
    assert node_ids(win) == [] and rows(win)[0][0] == "No nodes match"


def test_star_persists_and_moves_to_top(win):
    from core import settings
    win.ui.toggle_palette_favorite("ros.cpp.CppServiceClientNode")
    assert settings.get("palette_favorites") == ["ros.cpp.CppServiceClientNode"]
    assert rows(win)[0][0] == "--- ★ Favorites ---"
    assert rows(win)[1] == ("★ ServiceClient  · C++", "ros.cpp.CppServiceClientNode")
    win.ui.palette_fav_only.setChecked(True)
    assert node_ids(win) == ["ros.cpp.CppServiceClientNode"]
    win.ui.palette_fav_only.setChecked(False)
    win.ui.toggle_palette_favorite("ros.cpp.CppServiceClientNode")
    assert settings.get("palette_favorites") == [] and rows(win)[0][0] != "--- ★ Favorites ---"


def test_filtered_items_still_drop_their_node_type(win):
    """Drag and double-click read the node type from the item: filtering must not change it."""
    win.ui.palette_search.setText("StringSub")
    assert set(node_ids(win)) == {"ros.py.PyStringSubNode", "ros.cpp.CppStringSubNode"}
