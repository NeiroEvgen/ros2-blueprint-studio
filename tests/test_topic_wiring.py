"""
Wires and topics. ROS connects nodes by topic name, and Studio writes the Topic field into the code,
so a wire between a StringPub and a StringSub with different Topic fields connected nothing:
the subscriber stayed silent while the canvas showed a connection. Now the wire makes the
subscriber take the publisher's topic, a changed publisher topic follows its wires, and Save
warns about wired nodes whose topics still differ. Qt offscreen.
"""
import ast
import os
import shutil

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
QtWidgets = pytest.importorskip("PySide6.QtWidgets")
pytest.importorskip("NodeGraphQt")

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


@pytest.fixture(scope="module")
def app():
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


@pytest.fixture
def win(app, tmp_path):
    from ui import main_window as mw
    p = tmp_path / "proj"
    shutil.copytree(os.path.join(ROOT, "examples", "py_pubsub"), p)
    w = mw.RosVisualRunner()
    w.on_load_project_init(str(p))
    logs = []
    w.system_log = logs.append
    w._logs = logs
    yield w
    w._mark_clean()
    w.close()


def pub_sub(win, pub_topic="/a", sub_topic="/b"):
    g = win.graphs.all()[0]
    pub = g.create_node("ros.py.PyStringPubNode", name="p1")
    sub = g.create_node("ros.py.PyStringSubNode", name="s1")
    pub.set_property("topic_name", pub_topic)
    sub.set_property("topic_name", sub_topic)
    return g, pub, sub


def test_wire_gives_subscriber_the_publisher_topic(win):
    g, pub, sub = pub_sub(win)
    pub.outputs()["out"].connect_to(sub.inputs()["in"])
    assert sub.get_property("topic_name") == "/a"
    assert any("s1" in m and "/a" in m for m in win._logs)


def test_publisher_topic_change_follows_the_wire(win):
    g, pub, sub = pub_sub(win, "/a", "/a")
    pub.outputs()["out"].connect_to(sub.inputs()["in"])
    pub.set_property("topic_name", "/cmd")
    assert sub.get_property("topic_name") == "/cmd"


def test_unwired_nodes_are_left_alone(win):
    g, pub, sub = pub_sub(win, "/a", "/b")
    pub.set_property("topic_name", "/c")
    assert sub.get_property("topic_name") == "/b"


def test_save_warns_about_wired_nodes_with_different_topics(win):
    g, pub, sub = pub_sub(win, "/a", "/a")
    pub.outputs()["out"].connect_to(sub.inputs()["in"])
    sub.set_property("topic_name", "/typo")            # the subscriber side does not push back
    assert win._topic_mismatches() == [("topic_name", "p1", "s1", "/a", "/typo")]
    win._save_now()
    assert any("p1" in m and "s1" in m and "/typo" in m for m in win._logs)


def test_loading_a_project_does_not_rewrite_topics(win, tmp_path):
    """A saved project with different topics on a wire (code edited by hand) loads as saved."""
    g, pub, sub = pub_sub(win, "/a", "/a")
    pub.outputs()["out"].connect_to(sub.inputs()["in"])
    sub.set_property("topic_name", "/kept")
    path = win.current_project_path
    win._save_now()
    win.on_load_project_init(path)
    loaded = {n.name(): n for n in win.graphs.all()[0].all_nodes()}
    assert loaded["s1"].get_property("topic_name") == "/kept"
    assert loaded["p1"].get_property("topic_name") == "/a"


# ---------- the Python publisher has its own timer ----------

def test_python_publisher_template_has_own_timer():
    from core.template_render import render_pristine_template
    from nodes.templates import PYTHON_TEMPLATES
    tmpl = PYTHON_TEMPLATES["Publisher"]
    code = render_pristine_template(tmpl, {"template_key": "py_string_pub", "node_name": "talker",
                                           "topic_name": "/chatter", "interval": "0.5"}, "talker", False)
    ast.parse(code)
    assert "self.create_timer(0.5, self.timer_callback)" in code
    assert "create_publisher(String, '/chatter', 10)" in code
    assert "topic_1" not in code                      # no longer waits for a Timer (Start) node


def test_publisher_node_has_interval_field(win):
    g = win.graphs.all()[0]
    pub = g.create_node("ros.py.PyStringPubNode", name="p2")
    assert pub.get_property("interval") == "0.5"


def test_cpp_publisher_template_uses_interval_field(win):
    """Like the Python one; compiled and run against ROS 2 Humble for docs/tutorials chapter 3."""
    from core.template_render import render_pristine_template
    from nodes.templates import CPP_TEMPLATES
    code = render_pristine_template(CPP_TEMPLATES["Publisher"],
                                    {"template_key": "cpp_string_pub", "node_name": "talker",
                                     "topic_name": "/chatter", "interval": "0.5"}, "talker", True)
    assert "create_wall_timer(std::chrono::duration<double>(0.5)" in code
    pub = win.graphs.all()[0].create_node("ros.cpp.CppStringPubNode", name="c1")
    assert pub.get_property("interval") == "0.5"
