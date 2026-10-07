"""
UI: вкладка Libraries (кнопка Install раньше падала: не было project_path и
класса _RebuildWorker) и кнопка «Терминал» в Remote. Qt offscreen, Docker и
ssh подменены.
"""
import os
import time

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
QtWidgets = pytest.importorskip("PySide6.QtWidgets")


@pytest.fixture(scope="module")
def app():
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


def spin(app, cond, timeout=10.0):
    end = time.time() + timeout
    while time.time() < end:
        app.processEvents()
        if cond():
            return True
        time.sleep(0.02)
    raise AssertionError("не дождались условия")


class FakeDocker:
    def __init__(self, fail=False):
        self.calls, self.fail = [], fail

    def rebuild_project_image(self, project_path, status_callback=None):
        self.calls.append(project_path)
        if status_callback:
            status_callback("building...")
        if self.fail:
            raise RuntimeError("No Dockerfile found for this project.")


@pytest.fixture
def project(tmp_path):
    from core.dockerfile_manager import DockerfileManager
    p = tmp_path / "proj"
    p.mkdir()
    # Dockerfile создаёт save_project; для теста — минимальный через менеджер
    (p / "Dockerfile").write_text("ARG BASE_IMAGE\nFROM ${BASE_IMAGE}\n", encoding="utf-8")
    return str(p)


class TestLibraryInstall:
    def test_install_without_project_does_not_crash(self, app):
        from ui.library_manager import LibraryManager
        w = LibraryManager(FakeDocker())
        w.pkg_input.setText("numpy")
        w.start_install()                      # раньше: AttributeError
        assert "проект" in w.log_output.toPlainText().lower()
        assert w.install_btn.isEnabled()

    def test_install_rebuilds_image_and_reenables_button(self, app, project):
        from ui.library_manager import LibraryManager
        d = FakeDocker()
        w = LibraryManager(d)
        w.set_project(project)
        w.pkg_input.setText("numpy")
        w.type_combo.setCurrentIndex(1)        # pip
        w.start_install()                      # раньше: NameError _RebuildWorker
        spin(app, lambda: w.install_btn.isEnabled())
        assert d.calls == [project]
        assert "numpy" in open(os.path.join(project, "Dockerfile"), encoding="utf-8").read()
        assert "Successfully" in w.log_output.toPlainText()

    def test_failed_rebuild_is_reported(self, app, project):
        from ui.library_manager import LibraryManager
        w = LibraryManager(FakeDocker(fail=True))
        w.set_project(project)
        w.pkg_input.setText("numpy")
        w.start_install()
        spin(app, lambda: w.install_btn.isEnabled())
        assert "Failed to install" in w.log_output.toPlainText()

    def test_remote_connected_warns_install_is_local(self, app, project):
        from ui.library_manager import LibraryManager
        w = LibraryManager(FakeDocker())
        w.set_project(project)
        w.set_remote_connected(True)
        w.pkg_input.setText("numpy")
        w.start_install()
        spin(app, lambda: w.install_btn.isEnabled())
        assert "ЛОКАЛЬНЫЙ" in w.log_output.toPlainText()


class TestTerminalButton:
    def test_enabled_only_with_valid_host_and_user(self, app):
        from ui.remote_panel import RemotePanel
        p = RemotePanel()
        assert not p.btn_terminal.isEnabled()
        p.ed_host.setText("192.168.123.164")
        assert not p.btn_terminal.isEnabled()          # нет пользователя
        p.ed_user.setText("unitree")
        assert p.btn_terminal.isEnabled()              # проект и подключение не нужны
        p.ed_host.setText("a&calc")
        assert not p.btn_terminal.isEnabled()

    def test_click_opens_terminal_and_logs(self, app, monkeypatch):
        import ui.remote_panel as rp
        seen = {}
        monkeypatch.setattr(rp, "open_terminal",
                            lambda prof: seen.setdefault("p", prof) and "ssh -t unitree@h")
        p = rp.RemotePanel()
        logs = []
        p.log.connect(logs.append)
        p.ed_host.setText("h"); p.ed_user.setText("unitree"); p.ed_container.setText("g1pilot_run")
        p.btn_terminal.click()
        assert seen["p"].container == "g1pilot_run"
        assert any("Открыт терминал" in m for m in logs)

    def test_error_is_shown_not_raised(self, app, monkeypatch):
        import ui.remote_panel as rp
        from core.remote.terminal import TerminalError

        def boom(prof):
            raise TerminalError("ssh.exe не найден")
        monkeypatch.setattr(rp, "open_terminal", boom)
        p = rp.RemotePanel()
        p.ed_host.setText("h"); p.ed_user.setText("u")
        p.btn_terminal.click()
        assert "ssh.exe" in p.lbl_state.text()
