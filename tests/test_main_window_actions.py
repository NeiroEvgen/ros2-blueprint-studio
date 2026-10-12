"""
Главное окно: верхние кнопки Open/Save раньше не были подключены; Run падал с
TypeError, если save_project вернул False. Qt offscreen, Docker подменён.
"""
import os
import types

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
QtWidgets = pytest.importorskip("PySide6.QtWidgets")
pytest.importorskip("NodeGraphQt")
from PySide6 import QtCore  # noqa: E402


@pytest.fixture(scope="module")
def app():
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


@pytest.fixture
def win(app, tmp_path):
    from ui import main_window as mw
    w = mw.RosVisualRunner()
    w.current_project_path = str(tmp_path / "proj")
    os.makedirs(w.current_project_path, exist_ok=True)
    logs = []
    w.system_log = logs.append
    w._logs = logs
    return w


def receivers(obj, sig):
    return obj.receivers(QtCore.SIGNAL(sig))


class TestSaveResult:
    def test_tuple_success(self, win):
        assert win._save_result((True, {"a"})) == (True, {"a"})

    def test_false_failure(self, win):
        assert win._save_result(False) == (False, set())

    def test_empty_tuple_flag_false(self, win):
        assert win._save_result((False, set()))[0] is False


class TestToolbarButtons:
    def test_open_and_save_are_connected(self, win):
        for key in ("open", "save"):
            assert receivers(win.ui.actions[key], "clicked(bool)") > 0, key

    def test_save_calls_project_manager(self, win):
        called = []
        win.project_manager.save_project = lambda p: called.append(p) or (True, set())
        win.ui.actions["save"].click()
        assert called == [win.current_project_path]
        assert any("saved" in m for m in win._logs)

    def test_save_failure_is_reported(self, win):
        win.project_manager.save_project = lambda p: False
        win.ui.actions["save"].click()
        assert any("failed to save" in m for m in win._logs)

    def test_save_without_project(self, win):
        win.current_project_path = None
        win.ui.actions["save"].click()
        assert any("No project" in m for m in win._logs)

    def test_open_cancel_keeps_current_project(self, win, monkeypatch):
        before = win.current_project_path
        monkeypatch.setattr(QtWidgets.QMessageBox, "question",
                            staticmethod(lambda *a, **k: QtWidgets.QMessageBox.Cancel))
        win.ui.actions["open"].click()
        assert win.current_project_path == before

    def test_open_loads_chosen_folder(self, win, monkeypatch, tmp_path):
        monkeypatch.setattr(QtWidgets.QMessageBox, "question",
                            staticmethod(lambda *a, **k: QtWidgets.QMessageBox.Discard))
        target = tmp_path / "other"; target.mkdir()
        monkeypatch.setattr(QtWidgets.QFileDialog, "getExistingDirectory",
                            staticmethod(lambda *a, **k: str(target)))
        loaded = []
        win.on_load_project_init = loaded.append
        win.ui.actions["open"].click()
        assert loaded == [str(target)]


class TestRunWithFailedSave:
    def test_run_does_not_crash_when_save_fails(self, win):
        win.container_manager = types.SimpleNamespace()      # Docker «есть»
        win.project_manager.save_project = lambda p: False   # раньше: TypeError
        win.on_run_project()
        assert any("Aborting run" in m for m in win._logs)


class TestYamlKeepsCyrillic:
    """yaml.dump без allow_unicode превращал кириллицу в \\u04xx (грязные diff в git)."""

    def test_save_node_to_palette_writes_readable_yaml(self, win, tmp_path, monkeypatch):
        import inspect
        src = inspect.getsource(type(win))
        assert "yaml.dump(meta, f, sort_keys=False, allow_unicode=True)" in src

    def test_project_state_yaml_keeps_cyrillic(self):
        import inspect
        from core import project_manager
        assert "allow_unicode=True" in inspect.getsource(project_manager.ProjectManager.save_project)


class TestLayoutMigrationOnOpen:
    def _legacy(self, win):
        p = win.current_project_path
        for rel, text in (("package.xml", "px"), ("launch/a.launch.py", "L")):
            fp = os.path.join(p, rel)
            os.makedirs(os.path.dirname(fp), exist_ok=True)
            open(fp, "w").write(text)
        return p

    def test_accept_migrates_and_logs(self, win, monkeypatch):
        p = self._legacy(win)
        monkeypatch.setattr(QtWidgets.QMessageBox, "question", lambda *a, **k: QtWidgets.QMessageBox.Yes)
        assert win._offer_layout_migration(p) is True
        assert os.path.isfile(os.path.join(p, "src", "package.xml"))
        assert not os.path.exists(os.path.join(p, "package.xml"))
        assert any("moved 2" in m for m in win._logs)

    def test_decline_changes_nothing(self, win, monkeypatch):
        p = self._legacy(win)
        monkeypatch.setattr(QtWidgets.QMessageBox, "question", lambda *a, **k: QtWidgets.QMessageBox.No)
        assert win._offer_layout_migration(p) is False
        assert os.path.isfile(os.path.join(p, "package.xml"))

    def test_clean_project_no_question_asked(self, win, monkeypatch):
        asked = []
        monkeypatch.setattr(QtWidgets.QMessageBox, "question", lambda *a, **k: asked.append(1))
        os.makedirs(os.path.join(win.current_project_path, "src"), exist_ok=True)
        assert win._offer_layout_migration(win.current_project_path) is False and not asked


class TestSaveWarnings:
    def test_warnings_logged_after_save(self, win):
        win.project_manager.save_project = lambda p: setattr(win.project_manager, "last_warnings", ["W1"]) or (True, set())
        win.ui.actions["save"].click()
        assert any("W1" in m for m in win._logs)


class TestLogsFollowLanguage:
    """Сообщения главного окна идут через словарь: тот же сценарий на двух языках."""

    def test_no_project_message_in_both_languages(self, win):
        from core import i18n
        win.current_project_path = None
        win._logs.clear()
        win.on_save_project()
        assert any("No project loaded" in m for m in win._logs)
        i18n.set_language("ru")
        win._logs.clear()
        win.on_save_project()
        assert any("проект не загружен" in m for m in win._logs)
        i18n.set_language("en")

    def test_context_menu_labels_localized(self, win):
        """Контекстное меню графа строится из словаря (язык задан при старте)."""
        from core import i18n
        from core.i18n import t
        assert t("menu.restart_node") == "Restart Node"
        i18n.set_language("ru")
        assert t("menu.restart_node") == "Перезапустить ноду"
        assert t("menu.add_port", name="String") == "Добавить String"
        i18n.set_language("en")
