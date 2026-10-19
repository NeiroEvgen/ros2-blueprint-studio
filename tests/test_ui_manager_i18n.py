"""Main window toolbar: all texts from the dictionary, language switch on the fly."""
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
    idx = ui.console_tabs.indexOf(ui.console_ros)    # main_window may replace it with the Dashboard
    assert idx == -1 or ui.console_tabs.tabText(idx) == "Вывод ROS2"
    assert ui.console_tabs.tabText(ui.console_tabs.indexOf(ui.console_sys)) == "Системный лог"
    assert "реальное железо" in ui.actions["real_robot"].toolTip()
    i18n.set_language("en")
    ui.retranslate()
    assert ui.actions["open"].toolTip() == "Open"
    assert ui.palette_label.text() == "Node Palette"


def test_every_button_has_label_or_tooltip(ui):
    """Buttons without text (icon only) must have a tooltip, otherwise nobody understands them."""
    for key, w in ui.actions.items():
        if isinstance(w, QtWidgets.QPushButton):
            assert w.text().strip() or w.toolTip().strip(), key


def test_retranslate_does_not_touch_foreign_tabs(ui):
    """main_window inserts its own tabs into the console; a language switch must not change their titles."""
    extra = QtWidgets.QWidget()
    ui.console_tabs.insertTab(1, extra, "Custom Tab")
    i18n.set_language("ru")
    ui.retranslate()
    assert ui.console_tabs.tabText(ui.console_tabs.indexOf(extra)) == "Custom Tab"
    i18n.set_language("en")


# ---------- adaptive top bar ----------

def test_window_can_be_narrow_in_both_languages(app):
    """
    The window's minimum width used to be the sum of ALL bar button widths (~1430 px):
    the window would not shrink. The secondary section is not part of the minimum.
    The check is relative, not "<= 900 px": font metrics depend on the OS (on Windows
    Segoe UI is wider than DejaVu in CI), an absolute number would be brittle.
    """
    from ui import main_window as mw
    for lang in ("en", "ru"):
        i18n.set_language(lang)
        w = mw.RosVisualRunner()
        try:
            bar = w.ui.top_bar
            full = bar.layout().minimumSize().width()          # with all buttons
            narrow = bar.minimumSizeHint().width()             # with "⋯" instead of the section
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


# ---------- icons and fonts ----------

def test_all_icons_render(app):
    from ui import icons
    for name in icons.names():
        ic = icons.icon(name)
        pm = ic.pixmap(18, 18)
        assert not pm.isNull(), name
        img = pm.toImage()
        # the icon is not empty: at least one opaque pixel
        assert any(img.pixelColor(x, y).alpha() > 0 for x in range(img.width()) for y in range(img.height())), name


def test_icon_svg_is_valid_and_recolored():
    from ui import icons
    text = icons.svg("run", color="#ff0000")
    assert text.startswith("<svg") and "#ff0000" in text and "currentColor" not in text


def test_mono_font_stack_has_fallbacks():
    from ui.fonts import MONO_FAMILIES, MONO_CSS
    assert "Consolas" in MONO_FAMILIES and "DejaVu Sans Mono" in MONO_FAMILIES and "monospace" in MONO_FAMILIES
    assert "Consolas" in MONO_CSS and MONO_CSS.endswith("monospace")


# ---------- bottom panel: the window can be made low ----------

def test_window_can_be_short(app):
    """The Remote tab (~330 px) used to set the height: the window could not go below ~570 px."""
    from ui import main_window as mw
    w = mw.RosVisualRunner()
    try:
        assert w.minimumSizeHint().height() <= 450, w.minimumSizeHint()
    finally:
        w.close(); w.deleteLater()


def test_console_tabs_are_scrollable_and_titles_translated(app):
    from ui import main_window as mw
    w = mw.RosVisualRunner()
    try:
        tabs = w._console_tab_widget
        titles = [tabs.tabText(i) for i in range(tabs.count())]
        assert {"Docker", "Libraries", "Containers", "Remote", "Files"} <= set(titles)
        # panels stay accessible as window attributes but live in scroll areas
        assert w.remote_panel.parent() is not None
        assert isinstance(tabs.widget(tabs.indexOf(w.remote_panel.parentWidget().parentWidget())),
                          QtWidgets.QScrollArea)
        i18n.set_language("ru")
        w.retranslate_tabs()
        titles_ru = [tabs.tabText(i) for i in range(tabs.count())]
        assert "Удалённо" in titles_ru and "Файлы" in titles_ru and "Библиотеки" in titles_ru
        i18n.set_language("en")
        w.retranslate_tabs()
        assert "Remote" in [tabs.tabText(i) for i in range(tabs.count())]
    finally:
        i18n.set_language("en")
        w.close(); w.deleteLater()
