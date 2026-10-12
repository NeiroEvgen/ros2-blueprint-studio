"""Панели «Container settings» и «Remote»: подписи из словаря, на двух языках."""
import os
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
QtWidgets = pytest.importorskip("PySide6.QtWidgets")
from core import i18n  # noqa: E402


@pytest.fixture(scope="module")
def app():
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


def _form_labels(form_layout):
    out = []
    for r in range(form_layout.rowCount()):
        item = form_layout.itemAt(r, QtWidgets.QFormLayout.LabelRole)
        if item is not None and item.widget() is not None:
            out.append(item.widget().text())
    return out


def _all_texts(widget):
    texts = []
    for w in widget.findChildren(QtWidgets.QLabel):
        texts.append(w.text())
    for w in widget.findChildren(QtWidgets.QAbstractButton):
        texts.append(w.text())
    return texts


def test_container_settings_english_and_russian(app):
    from ui.container_settings import ContainerSettingsPanel
    p = ContainerSettingsPanel()
    en = _all_texts(p)
    assert "Container groups" in en and "💾 Save group config" in en
    assert "Open a project to edit container configs." in en
    i18n.set_language("ru")
    p2 = ContainerSettingsPanel()
    ru = _all_texts(p2)
    assert "Группы контейнеров" in ru and "💾 Сохранить конфиг группы" in ru
    assert p2.sp_delay.suffix() == " мс"
    assert p2.env_table.horizontalHeaderItem(0).text() == "КЛЮЧ"
    i18n.set_language("en")
    p.deleteLater(); p2.deleteLater()


def test_remote_panel_english_and_russian(app):
    from ui.remote_panel import RemotePanel
    p = RemotePanel()
    assert p.lbl_state.text() == "Not connected"
    assert p.btn_connect.text() == "🔌 Connect"
    assert p.btn_terminal.text() == "🖥 Terminal"
    i18n.set_language("ru")
    p2 = RemotePanel()
    assert p2.lbl_state.text() == "Не подключено"
    assert p2.btn_connect.text() == "🔌 Подключить"
    assert "ssh" in p2.btn_terminal.toolTip()
    i18n.set_language("en")
    p.deleteLater(); p2.deleteLater()


def test_remote_panel_has_no_robot_specific_defaults(app):
    from ui.remote_panel import RemotePanel
    p = RemotePanel()
    blob = " ".join([p.ed_host.placeholderText(), p.ed_user.placeholderText(), p.ed_ws.placeholderText()]).lower()
    assert "unitree" not in blob and "192.168.123" not in blob
    p.deleteLater()


def test_small_panels_follow_language(app):
    from ui.docker_panel import DockerPanel
    from ui.library_manager import LibraryManager

    class FakeDocker:
        def list_containers(self, *a, **k): return []
        client = None

    lib = LibraryManager(FakeDocker())
    assert lib.install_btn.text() == "Install"
    assert lib.type_combo.itemText(0).startswith("apt")
    i18n.set_language("ru")
    lib2 = LibraryManager(FakeDocker())
    assert lib2.install_btn.text() == "Установить"
    assert "Менеджер" in [w.text() for w in lib2.findChildren(QtWidgets.QLabel)][0]
    i18n.set_language("en")
    lib.deleteLater(); lib2.deleteLater()


    dp = DockerPanel(FakeDocker())
    assert dp.label.text() == "Docker Containers"
    assert dp.table.horizontalHeaderItem(2).text() == "Status"
    dp.timer.stop()
    i18n.set_language("ru")
    dp2 = DockerPanel(FakeDocker())
    assert dp2.label.text() == "Docker-контейнеры"
    assert dp2.table.horizontalHeaderItem(2).text() == "Статус"
    dp2.timer.stop()
    i18n.set_language("en")
    dp.deleteLater(); dp2.deleteLater()
