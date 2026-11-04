"""
Remote Build checks the target first. colcon builds the files that are ON the target: after an edit
without Sync, Build used to compile the old code without a word. Now a dry-run Sync finds the
difference and the user chooses: Sync and build / Build as is / Cancel. Qt offscreen, fake session.
"""
import os
import time

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
QtWidgets = pytest.importorskip("PySide6.QtWidgets")

from core.remote.sync import SyncReport  # noqa: E402


@pytest.fixture(scope="module")
def app():
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


class Session:
    connected = True
    profile = type("P", (), {"display": lambda self: "u@robot", "container": ""})()

    def __init__(self, pending=(), conflicts=()):
        self.pending, self.conflicts, self.calls = list(pending), list(conflicts), []

    def sync(self, on_line=None, dry_run=False):
        self.calls.append("dry-run" if dry_run else "sync")
        r = SyncReport(dry_run=dry_run)
        r.uploaded = list(self.pending)
        if not dry_run:
            r.conflicts = list(self.conflicts)
            self.pending = []
        return r

    def build(self, on_line, packages=None):
        self.calls.append("build")
        return 0


def wait(cond, ms=5000):
    end = time.time() + ms / 1000
    while not cond() and time.time() < end:
        QtWidgets.QApplication.processEvents()
        time.sleep(0.01)
    return cond()


def panel_with(session, answer=None, asked=None):
    from ui.remote_panel import RemotePanel
    p = RemotePanel()
    p.session = session
    if answer is not None:
        p._ask_sync_before_build = lambda files: (asked.append(files) if asked is not None else None) or answer
    return p


def finished(p):
    return not p._tasks.busy and not p._building


def test_up_to_date_builds_without_asking(app):
    s = Session()
    p = panel_with(s, answer="cancel", asked=(asked := []))
    p.on_build()
    assert wait(lambda: "build" in s.calls and finished(p))
    assert s.calls == ["dry-run", "build"] and not asked


def test_outdated_sync_and_build(app):
    s = Session(pending=["cpp/node.cpp"])
    asked = []
    p = panel_with(s, answer="sync", asked=asked)
    p.on_build()
    assert wait(lambda: "build" in s.calls and finished(p))
    assert s.calls == ["dry-run", "sync", "build"] and asked == [["cpp/node.cpp"]]


def test_outdated_build_as_is(app):
    s = Session(pending=["cpp/node.cpp"])
    p = panel_with(s, answer="build")
    p.on_build()
    assert wait(lambda: "build" in s.calls and finished(p))
    assert s.calls == ["dry-run", "build"]


def test_outdated_cancel_builds_nothing(app):
    s = Session(pending=["cpp/node.cpp"])
    p = panel_with(s, answer="cancel")
    p.on_build()
    assert wait(lambda: finished(p) and s.calls)
    time.sleep(0.1)
    assert s.calls == ["dry-run"]


def test_sync_conflict_stops_before_build(app, monkeypatch):
    shown = []
    monkeypatch.setattr(QtWidgets.QMessageBox, "warning", staticmethod(lambda *a: shown.append(a[2])))
    s = Session(pending=["cpp/node.cpp"], conflicts=[("cpp/node.cpp", "changed on both sides")])
    p = panel_with(s, answer="sync")
    p.on_build()
    assert wait(lambda: finished(p) and shown)
    assert s.calls == ["dry-run", "sync"] and "changed on both sides" in shown[0]
