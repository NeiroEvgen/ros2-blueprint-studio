"""
Run and window-close lifecycle.

Run -> Stop -> Run while the first DeployWorker was still running dropped the reference to a live
QThread: "QThread: Destroyed while thread is still running" and the whole process aborted.
Closing the window during Run crashed the same way (there was no closeEvent at all).
Qt offscreen, Docker faked.
"""
import os
import shutil
import threading

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
QtWidgets = pytest.importorskip("PySide6.QtWidgets")
pytest.importorskip("NodeGraphQt")
from PySide6 import QtCore  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


@pytest.fixture(scope="module")
def app():
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


class SlowDocker:
    """ensure_image waits until the test releases it; stop() releases it (like stopping the container)."""
    container_name = "fake"
    extra_env = {}

    def __init__(self):
        self.release = threading.Event()
        self.stopped = 0
        outer = self

        class _C:
            def stop(self, *a, **k):
                outer.stopped += 1
                outer.release.set()
        self.container = _C()

    def ensure_image(self, cb):
        self.release.wait(10)

    def start_session(self, *a, **k):
        pass

    def run_project_launch(self, **k):
        return True


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
    w.container_manager = SlowDocker()
    yield w
    w.container_manager.release.set()
    if getattr(w, "deploy_worker", None) is not None:
        w.deploy_worker.wait(5000)


def _wait(cond, ms=5000):
    timer = QtCore.QElapsedTimer()
    timer.start()
    while not cond() and timer.elapsed() < ms:
        QtWidgets.QApplication.processEvents()
        QtCore.QThread.msleep(10)
    return cond()


def test_run_is_blocked_until_previous_worker_finishes(win):
    win.on_run_project()
    first = win.deploy_worker
    assert first.isRunning()

    win.container_manager.release.clear()          # Stop will not release the worker right away
    win.container_manager.container.stop = lambda *a, **k: None
    win.on_stop_project()
    assert not win.ui.actions['run'].isEnabled()   # Run used to be enabled immediately

    win.on_run_project()                           # pressed anyway (or via a shortcut)
    assert win.deploy_worker is first              # the live QThread is not lost
    assert any("still stopping" in m for m in win._logs)

    win.container_manager.release.set()
    assert _wait(lambda: not first.isRunning())
    assert _wait(lambda: win.ui.actions['run'].isEnabled())


def test_stop_without_running_worker_reenables_run(win):
    win.ui.actions['run'].setEnabled(False)
    win.on_stop_project()
    assert win.ui.actions['run'].isEnabled()


# ---------- window close ----------

def _answer(monkeypatch, button, asked):
    def question(*a, **k):
        asked.append(a[2] if len(a) > 2 else "")
        return button
    monkeypatch.setattr(QtWidgets.QMessageBox, "question", staticmethod(question))


def test_close_after_load_does_not_ask(win, monkeypatch):
    asked = []
    _answer(monkeypatch, QtWidgets.QMessageBox.Cancel, asked)
    assert win.close() is True
    assert asked == []


def test_close_with_unsaved_changes_cancel_keeps_window(win, monkeypatch):
    asked = []
    _answer(monkeypatch, QtWidgets.QMessageBox.Cancel, asked)
    win.graphs.all()[0].create_node("ros.nodes.RosGroupNode", name="Extra")
    assert win.close() is False
    assert len(asked) == 1


def test_close_with_unsaved_changes_save_saves(win, monkeypatch):
    asked, saved = [], []
    _answer(monkeypatch, QtWidgets.QMessageBox.Save, asked)
    win.project_manager.save_project = lambda p: saved.append(p) or (True, set())
    win.graphs.all()[0].create_node("ros.nodes.RosGroupNode", name="Extra")
    assert win.close() is True
    assert saved == [win.current_project_path]


def test_close_with_failed_save_keeps_window(win, monkeypatch):
    _answer(monkeypatch, QtWidgets.QMessageBox.Save, [])
    win.project_manager.save_project = lambda p: False
    win.graphs.all()[0].create_node("ros.nodes.RosGroupNode", name="Extra")
    assert win.close() is False


def test_close_during_run_stops_and_waits_for_worker(win, monkeypatch):
    _answer(monkeypatch, QtWidgets.QMessageBox.Discard, [])
    win.on_run_project()
    worker = win.deploy_worker
    assert worker.isRunning()
    assert win.close() is True
    assert win.container_manager.stopped == 1     # the container is stopped, not abandoned
    assert not worker.isRunning()                 # the thread was waited for: destroying it is safe


def test_remote_panel_shutdown_cancels_build_and_waits(app):
    import time
    from ui.remote_panel import RemotePanel
    panel = RemotePanel()
    cancelled, disconnected = [], []
    panel.session = type("S", (), {
        "cancel": lambda self: cancelled.append(1),
        "disconnect": lambda self: disconnected.append(1),
        "connected": True})()
    panel._building = True
    panel._tasks.start(lambda emit: time.sleep(0.3), parent=panel)
    assert panel.shutdown(5000) is True
    assert cancelled == [1] and disconnected == [1] and not panel._tasks.busy
