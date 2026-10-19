"""
FileWatcher: edits in an editor -> node. Editors with atomic save (vim, some IDEs)
delete the file and create it again; if the signal came at that moment, the path was dropped
from watching for good and later edits never reached Studio.
"""
import os
import time

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
QtWidgets = pytest.importorskip("PySide6.QtWidgets")
from PySide6 import QtCore  # noqa: E402


@pytest.fixture(scope="module")
def app():
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


def _spin(cond, ms=3000):
    t = QtCore.QElapsedTimer()
    t.start()
    while not cond() and t.elapsed() < ms:
        QtWidgets.QApplication.processEvents()
        time.sleep(0.01)
    return cond()


@pytest.fixture
def watched(app, tmp_path):
    from core.file_watcher import FileWatcher
    src = tmp_path / "src" / "python"
    src.mkdir(parents=True)
    f = src / "talker.py"
    f.write_text("x = 1\n", encoding="utf-8")
    w = FileWatcher()
    got = []
    w.file_changed.connect(lambda p, c, ports: got.append(c))
    w.start_watching(str(tmp_path / "src"))
    return w, os.path.normpath(str(f)), got


def test_atomic_resave_keeps_watching(watched):
    w, path, got = watched
    os.remove(path)
    w.on_file_changed(path)                      # the signal came while the file is missing
    with open(path + ".tmp", "w", encoding="utf-8") as f:
        f.write("x = 2\n")
    os.replace(path + ".tmp", path)              # the editor puts the new file in place
    assert _spin(lambda: got and got[-1] == "x = 2\n")
    assert path in w.watcher.files()

    with open(path, "w", encoding="utf-8") as f:  # the next edit gets through too
        f.write("x = 3\n")
    assert _spin(lambda: got[-1] == "x = 3\n")


def test_really_deleted_file_is_dropped(watched):
    w, path, got = watched
    os.remove(path)
    w.on_file_changed(path)
    assert _spin(lambda: path not in w.watching_files)
    assert got == []


def test_empty_file_after_resave_stays_watched(watched):
    w, path, got = watched
    with open(path + ".tmp", "w", encoding="utf-8"):
        pass
    os.replace(path + ".tmp", path)              # replaced with an empty file: the OS dropped the watch
    _spin(lambda: path not in w.watcher.files(), 500)
    w.on_file_changed(path)
    assert path in w.watcher.files()
