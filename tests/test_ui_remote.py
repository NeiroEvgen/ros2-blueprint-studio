"""
UI-сценарий remote workspace на настоящем главном окне (offscreen) и
настоящем sshd: форма → Save → Connect → Sync → Build → Files: открыть,
поправить, сохранить → конфликт → Disconnect. Кнопки нажимаются как
пользователем. Пропускается без PySide6/sshd.
"""
import os
import stat
import time

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
QtWidgets = pytest.importorskip("PySide6.QtWidgets")
from PySide6 import QtCore  # noqa: E402

FAKE_COLCON = """#!/bin/bash
echo "Starting >>> $PKG"
test -f "src/$PKG/cpp/Node.cpp" || { echo "not synced" >&2; exit 9; }
echo "Finished <<< $PKG"
"""


@pytest.fixture(scope="module")
def app():
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


def wait_for(cond, timeout=15.0):
    app = QtWidgets.QApplication.instance()
    end = time.time() + timeout
    while time.time() < end:
        app.processEvents()
        if cond():
            return True
        time.sleep(0.02)
    raise AssertionError("не дождались условия")


@pytest.fixture
def window(app, sshd, tmp_path, monkeypatch):
    monkeypatch.setenv("HOME", str(tmp_path / "home"))   # known_hosts в изоляции
    import importlib
    import core.remote.profile as prof
    monkeypatch.setattr(prof, "DEFAULT_KNOWN_HOSTS", str(tmp_path / "kh"))

    from ui import main_window as mw
    cls = [c for c in vars(mw).values() if isinstance(c, type)
           and issubclass(c, QtWidgets.QMainWindow) and c.__module__ == mw.__name__][0]
    w = cls()

    project = tmp_path / "g1proj"
    (project / "src" / "cpp").mkdir(parents=True)
    (project / "src" / "cpp" / "Node.cpp").write_text("int main(){}\n")
    w.current_project_path = str(project)
    w.remote_panel.set_project(str(project))
    w.file_browser.set_sources(str(project / "src"), None)

    fakebin = tmp_path / "fakebin"; fakebin.mkdir()
    c = fakebin / "colcon"; c.write_text(FAKE_COLCON); c.chmod(c.stat().st_mode | stat.S_IEXEC)
    setup = tmp_path / "setup.bash"
    setup.write_text(f'export PATH="{fakebin}:$PATH"\nexport PKG=g1proj\n')
    ws = tmp_path / "remote_ws"; ws.mkdir()

    rp = w.remote_panel
    rp.ed_host.setText(sshd["host"]); rp.sp_port.setValue(sshd["port"])
    rp.ed_user.setText(sshd["user"]); rp.ed_key.setText(sshd["key"])
    rp.ed_ws.setText(str(ws)); rp.ed_ros.setText(str(setup))
    def _unexpected(kind):
        def f(*a, **k):
            raise AssertionError(f"неожиданный модальный QMessageBox.{kind}: {a[1:3]}")
        return staticmethod(f)
    for kind in ("question", "warning", "information", "critical"):
        monkeypatch.setattr(QtWidgets.QMessageBox, kind, _unexpected(kind))

    yield w, project, ws
    if rp.session:
        rp.session.disconnect()
    w.close()


def _log(w):
    return w.ui.console_sys.toPlainText()


def test_full_ui_flow(window, monkeypatch):
    w, project, ws = window
    rp, fb = w.remote_panel, w.file_browser

    assert w.ui.context_badge.text() == "LOCAL"
    assert not rp.btn_sync.isEnabled()

    # Save: профиль в проекте, без пароля
    rp.ed_pass.setText("secret-should-not-be-saved")
    rp.btn_save.click()
    saved = (project / ".blueprint" / "remote.yaml").read_text()
    assert "secret" not in saved and str(ws) in saved

    # Connect
    rp.btn_connect.click()
    wait_for(lambda: rp.connected)
    wait_for(lambda: rp.btn_sync.isEnabled())
    assert "SSH:" in w.ui.context_badge.text()
    assert rp.ed_pass.text() == ""          # пароль не висит в поле после подключения

    # Sync
    rp.btn_sync.click()
    wait_for(lambda: "Sync →" in _log(w))
    assert (ws / "src" / "g1proj" / "cpp" / "Node.cpp").read_text() == "int main(){}\n"

    # Build
    rp.btn_build.click()
    wait_for(lambda: "REMOTE BUILD FINISHED" in _log(w))
    assert "Finished <<< g1proj" in _log(w)
    assert "rc=0" in _log(w)

    # Files: источник «Хост», открыть файл, поправить, сохранить
    labels = [fb.cmb_source.itemText(i) for i in range(fb.cmb_source.count())]
    host_idx = next(i for i, t in enumerate(labels) if t.startswith("Хост"))
    fb.cmb_source.setCurrentIndex(host_idx)
    remote_file = str(ws / "src" / "g1proj" / "cpp" / "Node.cpp")
    fb.open_path(remote_file)
    wait_for(lambda: fb._buffer is not None and fb.editor.toPlainText() == "int main(){}\n")
    assert not fb.btn_save.isEnabled()

    fb.editor.setPlainText("int main(){ return 0; }\n")
    assert fb.lbl_file.text().startswith("● ")
    fb.btn_save.click()
    wait_for(lambda: "Сохранено" in _log(w))
    assert open(remote_file).read() == "int main(){ return 0; }\n"
    assert not fb.lbl_file.text().startswith("● ")

    # Конфликт: на «роботе» поправили после открытия → отказываемся перезаписывать
    with open(remote_file, "w") as f:
        f.write("// hotfix on robot\n")
    monkeypatch.setattr(QtWidgets.QMessageBox, "warning",
                        staticmethod(lambda *a, **k: QtWidgets.QMessageBox.No))
    fb.editor.setPlainText("// my version\n")
    fb.btn_save.click()
    time.sleep(0.5); QtWidgets.QApplication.instance().processEvents()
    assert open(remote_file).read() == "// hotfix on robot\n"

    # Disconnect с несохранёнными правками: без модального вопроса,
    # текст остаётся в редакторе с явной пометкой «НЕ сохранено»
    assert fb.editor.toPlainText() == "// my version\n"
    rp.btn_connect.click()
    wait_for(lambda: not rp.connected)
    assert fb.editor.toPlainText() == "// my version\n"
    assert "НЕ сохранено" in fb.lbl_file.text()
    assert not fb.btn_save.isEnabled()
    assert "правки" in _log(w) and "НЕ сохранены" in _log(w)
    assert w.ui.context_badge.text() == "LOCAL"
    labels = [fb.cmb_source.itemText(i) for i in range(fb.cmb_source.count())]
    assert not any(t.startswith("Хост") for t in labels)
    # у каждой строки лога есть метка контекста
    assert all(l.startswith("[") for l in _log(w).splitlines() if l.strip())


def test_invalid_profile_does_not_connect(window):
    w, *_ = window
    rp = w.remote_panel
    rp.ed_ws.setText("relative/path")
    rp.btn_connect.click()
    assert not rp.connected
    assert "абсолютным" in rp.lbl_state.text()


def test_user_says_no_combo_reverts(window, monkeypatch):
    """Отказался выбрасывать правки — комбобокс возвращается, файл остаётся открытым."""
    w, project, ws = window
    fb = w.file_browser
    (project / "src" / "extra").mkdir()
    fb.set_sources(str(project / "src"), None)
    fb.open_path(str(project / "src" / "cpp" / "Node.cpp"))
    wait_for(lambda: fb._buffer is not None)
    fb.editor.setPlainText("// dirty\n")
    # добавим второй источник, чтобы было куда переключаться
    from core.remote.providers import LocalProvider
    fb._sources.append(("Второй", LocalProvider(), str(project / "src" / "extra")))
    fb.cmb_source.blockSignals(True); fb.cmb_source.addItem("Второй"); fb.cmb_source.blockSignals(False)

    monkeypatch.setattr(QtWidgets.QMessageBox, "question",
                        staticmethod(lambda *a, **k: QtWidgets.QMessageBox.No))
    fb.cmb_source.setCurrentIndex(1)

    assert fb.cmb_source.currentIndex() == 0
    assert fb._buffer is not None and fb.editor.toPlainText() == "// dirty\n"
