import os
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
QtWidgets = pytest.importorskip("PySide6.QtWidgets")
from core import i18n, settings  # noqa: E402


@pytest.fixture(scope="module")
def app():
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


@pytest.fixture
def dlg(app, tmp_path, monkeypatch):
    from core.workspace_manager import WorkspaceManager
    monkeypatch.setattr(WorkspaceManager, "get_workspace_root", staticmethod(lambda: str(tmp_path)))
    from ui.launcher_dialog import LauncherDialog
    d = LauncherDialog()
    yield d
    d.deleteLater()


def pick(dlg, code):
    dlg.lang_combo.setCurrentIndex(dlg.lang_combo.findData(code))


def test_english_by_default(dlg):
    assert dlg.windowTitle() == "ROS2 Studio Launcher"
    assert dlg.btn_create.text() == "Create and Open"
    assert dlg.type_combo.itemText(2) == "Mixed (C++ + Python)"


def test_switch_to_russian_retranslates_everything(dlg):
    pick(dlg, "ru")
    assert dlg.btn_create.text() == "Создать и открыть"
    assert dlg.lbl_recent.text() == "Недавние проекты:"
    assert dlg.type_combo.itemText(1) == "C++ (требуется сборка)"
    assert dlg.name_input.placeholderText() == "Имя проекта..."
    assert dlg.windowTitle().startswith("ROS2 Studio")


def test_choice_is_saved(dlg):
    pick(dlg, "ru")
    assert settings.get("language") == "ru"


def test_saved_language_applied_on_next_start(dlg, app, tmp_path, monkeypatch):
    pick(dlg, "ru")
    i18n.set_language("en")            # "restart"
    settings.apply_language()
    from ui.launcher_dialog import LauncherDialog
    d2 = LauncherDialog()
    assert d2.btn_create.text() == "Создать и открыть"
    assert d2.lang_combo.currentData() == "ru"


def test_project_type_index_independent_of_language(dlg):
    pick(dlg, "ru")
    dlg.type_combo.setCurrentIndex(2)
    dlg.name_input.setText("mix_demo")
    dlg.create_project()
    from core.workspace_manager import WorkspaceManager
    assert WorkspaceManager.get_project_type(dlg.selected_project_path) == "mixed"


def test_invalid_name_message_localized(dlg, monkeypatch):
    shown = []
    monkeypatch.setattr(QtWidgets.QMessageBox, "warning", lambda parent, title, text, *a, **k: shown.append((title, text)))
    pick(dlg, "ru")
    dlg.name_input.setText("Bad Name")
    dlg.create_project()
    assert shown and shown[0][0] == "Недопустимое имя"


def test_language_list_comes_from_dictionaries(dlg):
    codes = [dlg.lang_combo.itemData(i) for i in range(dlg.lang_combo.count())]
    assert {"en", "ru"} <= set(codes)
    assert "Русский" in [dlg.lang_combo.itemText(i) for i in range(dlg.lang_combo.count())]
