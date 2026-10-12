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
    assert ui.actions["open"].toolTip() == "Open"
    assert ui.actions["run"].text() == "RUN"
    assert ui.palette_label.text() == "Node Palette"
    assert ui.btn_back.text() == "Back"
    assert ui.console_tabs.tabText(0) == "System Log"
    assert ui.console_dock.windowTitle() == "Logs & Terminal"
    assert "Foxglove" in ui.actions["viz_mode"].toolTip()


def test_switch_to_russian_and_back(ui):
    assert i18n.set_language("ru")
    ui.retranslate()
    assert ui.actions["open"].toolTip() == "Открыть"
    assert ui.actions["stop"].text() == "СТОП"
    assert ui.palette_label.text() == "Палитра нод"
    idx = ui.console_tabs.indexOf(ui.console_ros)    # main_window может заменить её на Dashboard
    assert idx == -1 or ui.console_tabs.tabText(idx) == "Вывод ROS2"
    assert ui.console_tabs.tabText(ui.console_tabs.indexOf(ui.console_sys)) == "Системный лог"
    assert "реальное железо" in ui.actions["real_robot"].toolTip()
    i18n.set_language("en")
    ui.retranslate()
    assert ui.actions["open"].toolTip() == "Open"
    assert ui.palette_label.text() == "Node Palette"


def test_every_button_has_label_or_tooltip(ui):
    """Кнопки без текста (только иконка) обязаны иметь подсказку, иначе их не понять."""
    for key, w in ui.actions.items():
        if isinstance(w, QtWidgets.QPushButton):
            assert w.text().strip() or w.toolTip().strip(), key


def test_retranslate_does_not_touch_foreign_tabs(ui):
    """main_window вставляет свои вкладки в консоль; их заголовки смена языка менять не должна."""
    extra = QtWidgets.QWidget()
    ui.console_tabs.insertTab(1, extra, "Custom Tab")
    i18n.set_language("ru")
    ui.retranslate()
    assert ui.console_tabs.tabText(ui.console_tabs.indexOf(extra)) == "Custom Tab"
    i18n.set_language("en")


# ---------- адаптивная верхняя панель ----------

def test_window_can_be_narrow_in_both_languages(app):
    """
    Раньше минимальная ширина окна равнялась сумме ширин ВСЕХ кнопок панели (~1430 px) —
    окно не сужалось. Теперь вторичная секция в минимум не входит.
    Проверка относительная, а не «<= 900 px»: метрики шрифтов зависят от ОС (на Windows
    Segoe UI шире, чем DejaVu в CI), абсолютное число было бы хрупким.
    """
    from ui import main_window as mw
    for lang in ("en", "ru"):
        i18n.set_language(lang)
        w = mw.RosVisualRunner()
        try:
            bar = w.ui.top_bar
            full = bar.layout().minimumSize().width()          # со всеми кнопками
            narrow = bar.minimumSizeHint().width()             # с «⋯» вместо секции
            secondary = bar.secondary.minimumSizeHint().width()
            assert narrow < full, (lang, narrow, full)
            assert full - narrow >= secondary - bar.more.sizeHint().width() - 2, (lang, narrow, full)
            assert w.minimumSizeHint().width() <= narrow + 60, (lang, w.minimumSizeHint().width(), narrow)
        finally:
            w.close(); w.deleteLater()
    i18n.set_language("en")


def test_secondary_buttons_collapse_into_menu_and_back(app):
    from ui import main_window as mw
    w = mw.RosVisualRunner()
    try:
        w.resize(1600, 500); w.show(); app.processEvents(); app.processEvents()
        bar = w.ui.top_bar
        assert bar.secondary.isVisible() and not bar.more.isVisible()
        w.resize(10, 500); app.processEvents(); app.processEvents()
        assert bar.more.isVisible() and not bar.secondary.isVisible()
        w.resize(1600, 500); app.processEvents(); app.processEvents()
        assert bar.secondary.isVisible() and not bar.more.isVisible()
    finally:
        w.close(); w.deleteLater()


def test_overflow_menu_triggers_real_actions(ui):
    clicked = []
    ui.actions["clear"].clicked.connect(lambda: clicked.append("clear"))
    acts = {a.text(): a for a in ui.more_menu.actions()}
    assert "Clear graph" in acts
    acts["Clear graph"].trigger()
    assert clicked == ["clear"]
    assert len(ui.more_menu.actions()) == 6


def test_overflow_menu_is_retranslated(ui):
    i18n.set_language("ru")
    ui.retranslate()
    assert "Очистить граф" in [a.text() for a in ui.more_menu.actions()]
    i18n.set_language("en")
    ui.retranslate()
    assert "Clear graph" in [a.text() for a in ui.more_menu.actions()]


def test_context_badge_does_not_widen_window(app):
    from core.execution_context import ExecutionContext
    from ui.context_badge import ContextBadge
    b = ContextBadge()
    b.setText("CONTAINER: a_very_long_container_name @ some_user@192.168.1.10")
    assert b.minimumWidth() <= 120


# ---------- иконки и шрифты ----------

def test_all_icons_render(app):
    from ui import icons
    for name in icons.names():
        ic = icons.icon(name)
        pm = ic.pixmap(18, 18)
        assert not pm.isNull(), name
        img = pm.toImage()
        # иконка не пустая: хотя бы один непрозрачный пиксель
        assert any(img.pixelColor(x, y).alpha() > 0 for x in range(img.width()) for y in range(img.height())), name


def test_icon_svg_is_valid_and_recolored():
    from ui import icons
    text = icons.svg("run", color="#ff0000")
    assert text.startswith("<svg") and "#ff0000" in text and "currentColor" not in text


def test_mono_font_stack_has_fallbacks():
    from ui.fonts import MONO_FAMILIES, MONO_CSS
    assert "Consolas" in MONO_FAMILIES and "DejaVu Sans Mono" in MONO_FAMILIES and "monospace" in MONO_FAMILIES
    assert "Consolas" in MONO_CSS and MONO_CSS.endswith("monospace")
