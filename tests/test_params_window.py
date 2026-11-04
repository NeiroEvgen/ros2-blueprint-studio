"""
Parameters window (ui/params_window.py) and its wiring in the main window.
Qt offscreen; the bridge is a fake client (the real one is covered in test_param_bridge.py).
"""
import os
import threading

import pytest
import yaml

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
QtWidgets = pytest.importorskip("PySide6.QtWidgets")
from PySide6 import QtCore  # noqa: E402

from core import param_store  # noqa: E402
from core.param_client import BridgeHolder, ParamBridgeError, ParamInfo  # noqa: E402


@pytest.fixture(scope="module")
def app():
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


def wait(cond, ms=3000):
    timer = QtCore.QElapsedTimer()
    timer.start()
    while not cond() and timer.elapsed() < ms:
        QtWidgets.QApplication.processEvents()
        QtCore.QThread.msleep(5)
    return cond()


def pause(ms):
    wait(lambda: False, ms)


PARAMS = [
    ParamInfo("enabled", "bool", True),
    ParamInfo("frame", "string", "base", read_only=True),
    ParamInfo("gain", "double", 0.5),                                   # no range: no slider
    ParamInfo("kp", "double", 1.0, range={"min": 0.0, "max": 10.0, "step": 0.0}),
    ParamInfo("rate_hz", "int", 50, range={"min": 1, "max": 500, "step": 1}),
    ParamInfo("weights", "double_array", [1.0, 2.0]),
]


class FakeClient:
    def __init__(self, nodes=("/ns/pid_demo", "/other")):
        self.nodes = list(nodes)
        self.values = {p.name: p.value for p in PARAMS}
        self.calls = []
        self.alive = True
        self.lock = threading.Lock()

    def list_nodes(self):
        return self.nodes

    def list_params(self, node):
        return [ParamInfo(p.name, p.type, self.values[p.name], p.read_only, p.description, p.range)
                for p in PARAMS]

    def set_param(self, node, name, value):
        with self.lock:
            self.calls.append((node, name, value))
        if name == "kp" and value == 6.5:
            raise ParamBridgeError("rejected", "6.5 is cursed")
        self.values[name] = float(value) if name in ("kp", "gain") else value
        return self.values[name]


@pytest.fixture
def make_window(app, tmp_path):
    from ui.params_window import ParamsWindow
    made = []

    def make(client=None, real_robot=False, initial="pid_demo"):
        client = client or FakeClient()
        w = ParamsWindow(lambda: client, str(tmp_path), real_robot, "test", initial_node=initial)
        w.show()
        made.append(w)
        assert wait(lambda: len(w.rows) == len(PARAMS)), w.lbl_state.text()
        return w, client
    yield make
    for w in made:
        try:
            w.close()
        except RuntimeError:
            pass


def editor(w, name):
    return w.rows[name].value_widget


class TestEditors:
    def test_one_editor_per_type(self, make_window):
        w, _ = make_window()
        assert isinstance(editor(w, "enabled"), QtWidgets.QCheckBox)
        assert isinstance(editor(w, "kp"), QtWidgets.QDoubleSpinBox) and w.rows["kp"].slider
        assert isinstance(editor(w, "rate_hz"), QtWidgets.QSpinBox) and w.rows["rate_hz"].slider
        assert w.rows["gain"].slider is None                    # no declared range: no invented limits
        assert isinstance(editor(w, "weights"), QtWidgets.QLineEdit)
        assert not w.rows["frame"].isEnabled()

    def test_graph_node_is_matched_to_running_node(self, make_window):
        w, _ = make_window(initial="PID_DEMO")
        assert w.node == "/ns/pid_demo" and w.cb_node.currentText() == "/ns/pid_demo"

    def test_other_node_can_be_picked(self, make_window):
        w, client = make_window()
        w.cb_node.setCurrentText("/other")
        assert wait(lambda: w.node == "/other" and w.rows)
        editor(w, "enabled").setChecked(False)
        assert wait(lambda: client.calls)
        assert client.calls[-1] == ("/other", "enabled", False)


class TestSimulation:
    def test_dragging_sends_only_latest_values_and_final_on_release(self, make_window):
        w, client = make_window()
        s = w.rows["kp"].slider
        s.setSliderDown(True)
        for i in range(0, 1001, 10):                # 100 moves in a few ms
            s.setValue(i)
        s.setSliderDown(False)                      # emits sliderReleased, like the mouse
        assert wait(lambda: client.values["kp"] == 10.0)
        kp_calls = [c for c in client.calls if c[1] == "kp"]
        assert len(kp_calls) <= 5, kp_calls          # coalesced, not 100 requests
        assert kp_calls[-1][2] == 10.0

    def test_int_slider_sends_ints(self, make_window):
        w, client = make_window()
        editor(w, "rate_hz").setValue(120)
        assert wait(lambda: client.values["rate_hz"] == 120)
        assert isinstance(client.calls[-1][2], int)

    def test_rejected_value_reverts_and_shows_reason(self, make_window):
        w, client = make_window()
        editor(w, "kp").setValue(6.5)
        assert wait(lambda: w.rows["kp"].status.text() == "✗")
        assert editor(w, "kp").value() == 1.0 and w.rows["kp"].confirmed == 1.0
        assert "cursed" in w.rows["kp"].status.toolTip()

    def test_bad_list_is_not_sent(self, make_window):
        w, client = make_window()
        e = editor(w, "weights")
        e.setText("1, 2")
        e.editingFinished.emit()
        pause(100)
        assert not client.calls and w.rows["weights"].status.text() == "✗"
        e.setText("[3, 4]")
        e.editingFinished.emit()
        assert wait(lambda: client.values["weights"] == [3, 4])


class TestRealRobot:
    def answer(self, monkeypatch, button, asked):
        def q(*a, **k):
            asked.append(a[2])
            return button
        monkeypatch.setattr(QtWidgets.QMessageBox, "question", staticmethod(q))

    def test_nothing_is_sent_while_dragging(self, make_window, monkeypatch):
        asked = []
        self.answer(monkeypatch, QtWidgets.QMessageBox.Yes, asked)
        w, client = make_window(real_robot=True)
        s = w.rows["kp"].slider
        s.setSliderDown(True)
        for i in range(100, 160, 10):                # 1.0 -> 1.5: small change
            s.setValue(i)
        pause(300)
        assert not client.calls
        s.setSliderDown(False)                      # emits sliderReleased, like the mouse
        assert wait(lambda: client.calls)
        assert [c[2] for c in client.calls] == [1.5] and not asked

    def test_big_jump_asks_and_no_keeps_robot_value(self, make_window, monkeypatch):
        asked = []
        self.answer(monkeypatch, QtWidgets.QMessageBox.No, asked)
        w, client = make_window(real_robot=True)
        editor(w, "kp").setValue(8.0)                 # 70% of the range in one step
        pause(200)
        assert asked and "70" in asked[0]
        assert not client.calls and editor(w, "kp").value() == 1.0

    def test_big_jump_confirmed_is_sent(self, make_window, monkeypatch):
        self.answer(monkeypatch, QtWidgets.QMessageBox.Yes, [])
        w, client = make_window(real_robot=True)
        editor(w, "kp").setValue(8.0)
        assert wait(lambda: client.values["kp"] == 8.0)

    def test_unbounded_value_asks(self, make_window, monkeypatch):
        asked = []
        self.answer(monkeypatch, QtWidgets.QMessageBox.No, asked)
        w, client = make_window(real_robot=True)
        editor(w, "gain").setValue(0.6)
        pause(200)
        assert asked and not client.calls


class TestButtons:
    def test_save_writes_only_changed_values(self, make_window, tmp_path):
        w, client = make_window()
        editor(w, "kp").setValue(2.5)
        assert wait(lambda: w.rows["kp"].confirmed == 2.5)
        w.save()
        data = yaml.safe_load((tmp_path / "src" / "config" / "params.yaml").read_text(encoding="utf-8"))
        assert data == {"/ns/pid_demo": {"ros__parameters": {"kp": 2.5}}}
        assert "kp" in w.lbl_state.text()

    def test_save_without_changes(self, make_window, tmp_path):
        w, _ = make_window()
        w.save()
        assert not (tmp_path / "src" / "config" / "params.yaml").exists()

    def test_save_never_overwrites_a_broken_file(self, make_window, tmp_path):
        w, _ = make_window()
        p = tmp_path / "src" / "config" / "params.yaml"
        p.parent.mkdir(parents=True)
        p.write_text("[broken: yaml", encoding="utf-8")
        editor(w, "kp").setValue(2.5)
        assert wait(lambda: w.rows["kp"].confirmed == 2.5)
        w.save()
        assert p.read_text(encoding="utf-8") == "[broken: yaml"
        assert "params.yaml" in w.lbl_state.text()

    def test_revert_sends_original_values(self, make_window):
        w, client = make_window()
        editor(w, "kp").setValue(3.0)
        editor(w, "rate_hz").setValue(200)
        assert wait(lambda: w.rows["rate_hz"].confirmed == 200 and w.rows["kp"].confirmed == 3.0)
        w.revert()
        assert wait(lambda: client.values["kp"] == 1.0 and client.values["rate_hz"] == 50)
        assert editor(w, "kp").value() == 1.0


class TestFailures:
    def test_bridge_start_failure_is_shown_and_refresh_retries(self, app, tmp_path):
        from ui.params_window import ParamsWindow
        attempts = []
        client = FakeClient()

        def get():
            attempts.append(1)
            if len(attempts) == 1:
                raise ParamBridgeError("no_rclpy", "No module named 'rclpy'")
            return client
        w = ParamsWindow(get, str(tmp_path), False, "test", initial_node="pid_demo")
        try:
            assert wait(lambda: "rclpy" in w.lbl_state.text())
            w.refresh()
            assert wait(lambda: len(w.rows) == len(PARAMS))
        finally:
            w.close()

    def test_no_running_nodes(self, app, tmp_path):
        from ui.params_window import ParamsWindow
        w = ParamsWindow(lambda: FakeClient(nodes=()), str(tmp_path), False, "test")
        try:
            from core.i18n import t
            assert wait(lambda: w.lbl_state.text() == t("params.no_nodes"))
        finally:
            w.close()

    def test_close_sends_last_value_and_late_answers_do_not_crash(self, make_window):
        w, client = make_window()
        slow = threading.Event()
        orig = client.set_param

        def slow_set(*a):
            slow.wait(1)
            return orig(*a)
        client.set_param = slow_set
        s = w.rows["kp"].slider
        s.setSliderDown(True)
        s.setValue(700)                               # pending in the coalescer
        w.close()
        slow.set()
        assert wait(lambda: client.values["kp"] == 7.0)
        pause(100)                                    # answers arrive after the window is gone


# ---------- BridgeHolder ----------

class FakeBridgeClient:
    started = []

    def __init__(self, transport):
        self.transport = transport
        self.alive = False
        self.closed = False

    def start(self):
        self.alive = True
        FakeBridgeClient.started.append(self)
        return self

    def close(self):
        self.closed = True
        self.alive = False


class TestBridgeHolder:
    @pytest.fixture(autouse=True)
    def fake(self, monkeypatch):
        from core import param_client
        FakeBridgeClient.started = []
        monkeypatch.setattr(param_client, "ParamClient", FakeBridgeClient)

    def test_reuse_same_target(self):
        h = BridgeHolder()
        a = h.get(("local", "s"), lambda: "t1")
        assert h.get(("local", "s"), lambda: "t2") is a and len(FakeBridgeClient.started) == 1

    def test_new_target_closes_old(self):
        h = BridgeHolder()
        a = h.get(("local", "s"), lambda: "t1")
        b = h.get(("remote", 1), lambda: "t2")
        assert a.closed and b is not a and b.transport == "t2"

    def test_dead_bridge_is_restarted(self):
        h = BridgeHolder()
        a = h.get(("local", "s"), lambda: "t1")
        a.alive = False
        assert h.get(("local", "s"), lambda: "t1") is not a

    def test_close(self):
        h = BridgeHolder()
        a = h.get(("local", "s"), lambda: "t1")
        h.close()
        assert a.closed


# ---------- main window ----------

@pytest.fixture
def win(app, tmp_path):
    import shutil
    pytest.importorskip("NodeGraphQt")
    from ui import main_window as mw
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    p = tmp_path / "proj"
    shutil.copytree(os.path.join(root, "examples", "py_pubsub"), p)
    w = mw.RosVisualRunner()
    w.on_load_project_init(str(p))
    w.system_log = lambda *_: None
    yield w
    w._mark_clean()
    w.close()


class TestMainWindow:
    def test_no_target_without_run_or_robot(self, win, monkeypatch):
        win.container_manager = type("CM", (), {"container": None, "container_name": "s"})()
        assert win._param_target() is None
        shown = []
        monkeypatch.setattr(QtWidgets.QMessageBox, "information", staticmethod(lambda *a: shown.append(a[2])))
        assert win.on_params_selected(win.graphs.all()[0]) is None and shown

    def test_local_session_target(self, win):
        win.container_manager = type("CM", (), {"container": object(), "container_name": "sess"})()
        key, make, where, remote = win._param_target()
        assert key == ("local", "sess") and not remote
        assert make().argv[:4] == ["docker", "exec", "-i", "sess"]

    def test_connected_robot_wins_and_counts_as_real_robot(self, win, monkeypatch):
        from core.remote.profile import RemoteProfile
        win.container_manager = type("CM", (), {"container": object(), "container_name": "sess"})()
        session = type("S", (), {"ssh": object(), "connected": True,
                                 "profile": RemoteProfile(host="g1", user="u", remote_workspace="/ws",
                                                          container="g1pilot_run")})()
        monkeypatch.setattr(type(win.remote_panel), "connected", property(lambda self: True))
        win.remote_panel.session = session
        key, make, where, remote = win._param_target()
        assert key[0] == "remote" and remote and "g1" in where
        assert "docker exec -i g1pilot_run" in make().command

        opened = win.on_params_selected(win.graphs.all()[0])
        try:
            assert opened.real_robot                      # robot connected: robot policy even if the flag is off
        finally:
            opened.close()
            win.remote_panel.session = None

    def test_bridge_is_closed_on_stop_and_target_change(self, win):
        closed = []
        win._param_bridge = type("B", (), {"close": lambda self: closed.append(1)})()
        win.on_stop_project()
        win._on_remote_changed(False, "", "")
        assert wait(lambda: len(closed) >= 2)

    def test_menu_has_parameters_entry(self, win):
        from core.i18n import t
        graph = win.graphs.all()[0]
        try:
            menu = graph.context_menu()
        except TypeError:
            menu = graph.context_menu
        labels = [a.text() for a in menu.qmenu.actions()]
        assert any(t("menu.parameters").strip() in x for x in labels)


# ---------- params.yaml ----------

class TestParamStore:
    def test_merge_keeps_other_nodes_and_values(self, tmp_path):
        param_store.save_values(str(tmp_path), "pid", {"kp": 1.5})
        param_store.save_values(str(tmp_path), "/arm", {"speed": 2})
        param_store.save_values(str(tmp_path), "/pid", {"ki": 0.1})
        assert param_store.load(str(tmp_path)) == {"/pid": {"kp": 1.5, "ki": 0.1}, "/arm": {"speed": 2}}
        text = (tmp_path / "src" / "config" / "params.yaml").read_text(encoding="utf-8")
        assert text.startswith("# ROS 2 parameters") and "ros__parameters" in text

    def test_hand_written_keys_survive(self, tmp_path):
        p = tmp_path / "src" / "config" / "params.yaml"
        p.parent.mkdir(parents=True)
        p.write_text("pid:\n  ros__parameters:\n    kd: 3\n  other_key: 1\n", encoding="utf-8")
        param_store.save_values(str(tmp_path), "/pid", {"kp": 2.0})
        data = yaml.safe_load(p.read_text(encoding="utf-8"))
        assert data == {"pid": {"ros__parameters": {"kd": 3, "kp": 2.0}, "other_key": 1}}

    def test_broken_file_raises_and_stays(self, tmp_path):
        p = tmp_path / "src" / "config" / "params.yaml"
        p.parent.mkdir(parents=True)
        p.write_text("- just\n- a list\n", encoding="utf-8")
        with pytest.raises(param_store.ParamFileError):
            param_store.save_values(str(tmp_path), "/pid", {"kp": 2.0})
        with pytest.raises(param_store.ParamFileError):
            param_store.load(str(tmp_path))
        assert p.read_text(encoding="utf-8") == "- just\n- a list\n"

    def test_no_file(self, tmp_path):
        assert param_store.load(str(tmp_path)) == {}


# ---------- launch passes the file ----------

class TestLaunchParams:
    def launch(self, monkeypatch, exists):
        from tests.test_launch_compiler import run_launch, _compile, cpp, py
        from core.generators.launch_generator import CONTAINER_PARAMS_FILE
        real = os.path.exists
        monkeypatch.setattr(os.path, "exists", lambda p: exists if p == CONTAINER_PARAMS_FILE else real(p))
        return run_launch(_compile([cpp("ctrl"), py("viewer")])), CONTAINER_PARAMS_FILE

    def test_file_present(self, monkeypatch):
        (node, proc), path = self.launch(monkeypatch, True)
        assert node.kw["parameters"] == [path]
        assert proc.kw["cmd"][-3:] == ["--ros-args", "--params-file", path]

    def test_file_absent_changes_nothing(self, monkeypatch):
        (node, proc), _ = self.launch(monkeypatch, False)
        assert node.kw["parameters"] == []
        assert proc.kw["cmd"] == ["python3", "-u", "/root/ros2_ws/src/user_project/python/viewer.py"]

    def test_file_is_under_src_so_it_is_mounted_and_synced(self):
        from core.generators.launch_generator import CONTAINER_PARAMS_FILE
        from core.layout import ProjectLayout
        rel = os.path.relpath(ProjectLayout("/p").params_file, "/p/src").replace(os.sep, "/")
        assert CONTAINER_PARAMS_FILE == "/root/ros2_ws/src/user_project/" + rel


# ---------- the window against a real ROS 2 node ----------

class TestRealRos:
    def test_slider_reaches_the_node_and_rejection_comes_back(self, app, ros_container, tmp_path):
        from core import param_client as pc
        from ui.params_window import ParamsWindow
        holder = pc.BridgeHolder()
        get = lambda: holder.get(("local", ros_container), lambda: pc.PipeTransport(pc.local_argv(ros_container)))
        w = ParamsWindow(get, str(tmp_path), False, "test", initial_node="pid_demo")
        try:
            assert wait(lambda: "kp" in w.rows, 20000), w.lbl_state.text()
            s = w.rows["kp"].slider
            s.setSliderDown(True)
            for i in range(0, 501, 25):
                s.setValue(i)
            s.setSliderDown(False)
            assert wait(lambda: w.rows["kp"].confirmed == 5.0, 5000)
            assert {p.name: p.value for p in get().list_params("/pid_demo")}["kp"] == 5.0

            editor(w, "kp").setValue(6.66)               # the demo node refuses this value itself
            assert wait(lambda: w.rows["kp"].status.text() == "✗", 5000)
            assert "cursed" in w.rows["kp"].status.toolTip() and editor(w, "kp").value() == 5.0

            w.save()
            assert param_store.load(str(tmp_path)) == {"/pid_demo": {"kp": 5.0}}
        finally:
            w.close()
            holder.close()
