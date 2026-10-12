"""Панель инструментов главного окна: все тексты из словаря, смена языка на лету."""
import os
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
QtWidgets = pytest.importorskip("PySide6.QtWidgets")
from core import i18n  # noqa: E402


@pytest.fixture(scope="module")
def app():
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


@pytest.fixture
def ui(app):
    from ui import main_window as mw
    cls = [c for c in vars(mw).values() if isinstance(c, type)
           and issubclass(c, QtWidgets.QMainWindow) and c.__module__ == mw.__name__][0]
    w = cls()
    yield w.ui
    w.close()
    w.deleteLater()


def test_english_texts(ui):
    assert ui.actions["open"].text().strip() == "Open"
    assert ui.actions["run"].text() == "▶ RUN"
    assert ui.palette_label.text() == "Node Palette"
    assert ui.btn_back.text() == "← Back"
    assert ui.console_tabs.tabText(0) == "System Log"
    assert ui.console_dock.windowTitle() == "Logs & Terminal"
    assert "Foxglove" in ui.actions["viz_mode"].toolTip()


def test_switch_to_russian_and_back(ui):
    assert i18n.set_language("ru")
    ui.retranslate()
    assert ui.actions["open"].text().strip() == "Открыть"
    assert ui.actions["stop"].text() == "⏹ СТОП"
    assert ui.palette_label.text() == "Палитра нод"
    idx = ui.console_tabs.indexOf(ui.console_ros)    # main_window может заменить её на Dashboard
    assert idx == -1 or ui.console_tabs.tabText(idx) == "Вывод ROS2"
    assert ui.console_tabs.tabText(ui.console_tabs.indexOf(ui.console_sys)) == "Системный лог"
    assert "реальное железо" in ui.actions["real_robot"].toolTip()
    i18n.set_language("en")
    ui.retranslate()
    assert ui.actions["open"].text().strip() == "Open"
    assert ui.palette_label.text() == "Node Palette"


def test_no_empty_button_texts(ui):
    for key, w in ui.actions.items():
        if isinstance(w, QtWidgets.QPushButton):
            assert w.text().strip(), key


def test_retranslate_does_not_touch_foreign_tabs(ui):
    """main_window вставляет свои вкладки в консоль; их заголовки смена языка менять не должна."""
    extra = QtWidgets.QWidget()
    ui.console_tabs.insertTab(1, extra, "Custom Tab")
    i18n.set_language("ru")
    ui.retranslate()
    assert ui.console_tabs.tabText(ui.console_tabs.indexOf(extra)) == "Custom Tab"
    i18n.set_language("en")
