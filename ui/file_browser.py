"""
Вкладка Files: дерево файлов + редактор для любого источника.

Источники: проект (локально), удалённый хост, контейнер на удалённом хосте.
Всё удалённое читается/пишется в фоне. Save не затирает правку, сделанную
на той стороне после открытия файла (core/remote/editing.py).
"""
import os

from PySide6 import QtWidgets, QtCore, QtGui

from core.remote.providers import LocalProvider
from core.remote.editing import EditBuffer, FileChangedExternally, NotEditable
from ui.code_editor import CodeEditor, CodeHighlighter, mode_for_path
from ui.qt_worker import TaskHost

_PLACEHOLDER = "__loading__"
ROLE_PATH = QtCore.Qt.UserRole
ROLE_DIR = QtCore.Qt.UserRole + 1


class FileBrowser(QtWidgets.QWidget):
    log = QtCore.Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._tasks = TaskHost()
        self._sources = []          # [(label, provider, root)]
        self._provider = None
        self._buffer = None
        self._loading_text = False
        self._current_idx = -1
        self._orphan_path = None     # файл, чей источник пропал (связь оборвалась)
        self._build_ui()
        self.set_sources(None, None)

    # ---------- UI ----------

    def _build_ui(self):
        lay = QtWidgets.QVBoxLayout(self)
        top = QtWidgets.QHBoxLayout()
        self.cmb_source = QtWidgets.QComboBox()
        self.cmb_source.currentIndexChanged.connect(self._on_source_changed)
        btn_refresh = QtWidgets.QPushButton("⟳")
        btn_refresh.setFixedWidth(32)
        btn_refresh.clicked.connect(self.reload_tree)
        top.addWidget(QtWidgets.QLabel("Источник:"))
        top.addWidget(self.cmb_source, 1)
        top.addWidget(btn_refresh)
        lay.addLayout(top)

        split = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        self.tree = QtWidgets.QTreeWidget()
        self.tree.setHeaderLabels(["Имя", "Размер"])
        self.tree.itemExpanded.connect(self._on_expand)
        self.tree.itemDoubleClicked.connect(self._on_open)
        split.addWidget(self.tree)

        right = QtWidgets.QWidget()
        rl = QtWidgets.QVBoxLayout(right)
        rl.setContentsMargins(0, 0, 0, 0)
        bar = QtWidgets.QHBoxLayout()
        self.lbl_file = QtWidgets.QLabel("Файл не открыт")
        self.lbl_file.setStyleSheet("color: #ccc; padding: 2px;")
        self.btn_save = QtWidgets.QPushButton("💾 Save  (Ctrl+S)")
        self.btn_save.clicked.connect(self.save_current)
        bar.addWidget(self.lbl_file, 1)
        bar.addWidget(self.btn_save)
        rl.addLayout(bar)
        self.editor = CodeEditor(mode="plain")
        self.editor.textChanged.connect(self._update_title)
        rl.addWidget(self.editor)
        split.addWidget(right)
        split.setSizes([300, 700])
        lay.addWidget(split)

        QtGui.QShortcut(QtGui.QKeySequence.Save, self, activated=self.save_current)
        self._update_title()

    # ---------- источники ----------

    def set_sources(self, local_root, session):
        """
        Перестроить список источников: проект + (хост, контейнер), если подключены.
        Вызывается программно (подключение/отключение), поэтому БЕЗ модальных
        вопросов: если источник открытого файла исчез (связь оборвалась), правки
        не выбрасываются — текст остаётся в редакторе с пометкой «не сохранено».
        """
        old_provider = self._provider
        self._sources = []
        if local_root and os.path.isdir(local_root):
            self._sources.append(("Проект (локально)", LocalProvider(), local_root))
        if session is not None and session.connected:
            self._sources.append((f"Хост: {session.profile.display()}",
                                  session.host_provider(), session.remote_src))
            cp = session.container_provider()
            if cp is not None:
                root = f"{session.profile.build_dir.rstrip('/')}/src/{session.package}"
                self._sources.append((f"Контейнер: {session.profile.container}", cp, root))

        keep = next((i for i, src in enumerate(self._sources) if src[1] is old_provider), -1)
        self.cmb_source.blockSignals(True)
        self.cmb_source.clear()
        for label, _, root in self._sources:
            self.cmb_source.addItem(f"{label}   {root}")
        self.cmb_source.blockSignals(False)

        if keep >= 0:
            # источник открытого файла на месте — ничего не трогаем
            self.cmb_source.blockSignals(True)
            self.cmb_source.setCurrentIndex(keep)
            self.cmb_source.blockSignals(False)
            self._current_idx = keep
            return

        if self._buffer is not None and self.is_dirty():
            self._orphan_path = self._buffer.path
            self._buffer = None
            self.log.emit(f"⚠ Связь с источником потеряна, правки в {self._orphan_path} "
                          "НЕ сохранены — текст остался в редакторе, скопируй его.")
        else:
            self._close_file()
        self._switch_to(0 if self._sources else -1)

    def _switch_to(self, idx):
        self._current_idx = idx
        self._provider = self._sources[idx][1] if 0 <= idx < len(self._sources) else None
        self.cmb_source.blockSignals(True)
        self.cmb_source.setCurrentIndex(idx)
        self.cmb_source.blockSignals(False)
        self.reload_tree()
        self._update_title()

    def _on_source_changed(self, idx):
        """Смену источника инициировал пользователь в комбобоксе."""
        if not self._confirm_discard():
            # отказался выбрасывать правки → комбобокс обратно, интерфейс не врёт
            self.cmb_source.blockSignals(True)
            self.cmb_source.setCurrentIndex(self._current_idx)
            self.cmb_source.blockSignals(False)
            return
        self._orphan_path = None
        self._close_file()
        self._switch_to(idx)

    def _current_root(self):
        idx = self.cmb_source.currentIndex()
        return self._sources[idx][2] if 0 <= idx < len(self._sources) else None

    # ---------- дерево ----------

    def reload_tree(self):
        self.tree.clear()
        root = self._current_root()
        if not self._provider or not root:
            return
        top = QtWidgets.QTreeWidgetItem([os.path.basename(root.rstrip("/\\")) or root, ""])
        top.setData(0, ROLE_PATH, root)
        top.setData(0, ROLE_DIR, True)
        top.addChild(QtWidgets.QTreeWidgetItem([_PLACEHOLDER]))
        self.tree.addTopLevelItem(top)
        top.setExpanded(True)

    def _on_expand(self, item):
        if not (item.childCount() == 1 and item.child(0).text(0) == _PLACEHOLDER):
            return
        prov, path = self._provider, item.data(0, ROLE_PATH)
        item.child(0).setText(0, "загрузка…")

        def done(entries):
            item.takeChildren()
            for e in entries:
                child = QtWidgets.QTreeWidgetItem(
                    [("📁 " if e.is_dir else "") + e.name, "" if e.is_dir else _size(e.size)])
                child.setData(0, ROLE_PATH, e.path)
                child.setData(0, ROLE_DIR, e.is_dir)
                if e.is_dir:
                    child.addChild(QtWidgets.QTreeWidgetItem([_PLACEHOLDER]))
                item.addChild(child)

        def fail(msg):
            item.takeChildren()
            item.addChild(QtWidgets.QTreeWidgetItem([f"⚠ {msg}"]))

        self._tasks.start(lambda emit: prov.list_dir(path), done, fail, parent=self)

    # ---------- файл ----------

    def _on_open(self, item, _col):
        if item.data(0, ROLE_DIR) or not item.data(0, ROLE_PATH):
            return
        if not self._confirm_discard():
            return
        self.open_path(item.data(0, ROLE_PATH))

    def open_path(self, path):
        buf = EditBuffer(self._provider, path)

        def done(text):
            self._buffer = buf
            self._orphan_path = None
            mode = mode_for_path(path)
            self.editor.highlighter.setDocument(None)
            self._loading_text = True
            self.editor.setPlainText(text)
            self._loading_text = False
            self.editor.highlighter = CodeHighlighter(self.editor.document(), mode)
            self._update_title()

        def fail(msg):
            self.log.emit(f"Не открыть {path}: {msg}")
            QtWidgets.QMessageBox.information(self, "Файл не открыт", msg)

        self._tasks.start(lambda emit: buf.open(), done, fail, parent=self)

    def _close_file(self):
        self._buffer = None
        self._loading_text = True
        self.editor.setPlainText("")
        self._loading_text = False
        self._update_title()

    def is_dirty(self):
        return bool(self._buffer and self._buffer.is_dirty(self.editor.toPlainText()))

    def _update_title(self):
        if self._loading_text:
            return
        if not self._buffer:
            if self._orphan_path:
                self.lbl_file.setText(f"⚠ связь потеряна — НЕ сохранено: {self._orphan_path}")
                self.lbl_file.setStyleSheet("color: #ff8a65; padding: 2px; font-weight: bold;")
            else:
                self.lbl_file.setText("Файл не открыт")
                self.lbl_file.setStyleSheet("color: #ccc; padding: 2px;")
            self.btn_save.setEnabled(False)
            return
        self.lbl_file.setStyleSheet("color: #ccc; padding: 2px;")
        dirty = self.is_dirty()
        self.lbl_file.setText(("● " if dirty else "") + self._buffer.path)
        self.btn_save.setEnabled(dirty)

    def _confirm_discard(self):
        if self._orphan_path:
            what = f"{self._orphan_path} (связь потеряна, правки не сохранены)"
        elif self.is_dirty():
            what = self._buffer.path
        else:
            return True
        r = QtWidgets.QMessageBox.question(
            self, "Несохранённые изменения",
            f"В {what} есть несохранённые изменения. Выбросить их?")
        return r == QtWidgets.QMessageBox.Yes

    def save_current(self, force=False):
        if not self._buffer or not self.is_dirty():
            return
        buf, text = self._buffer, self.editor.toPlainText()

        def done(_):
            self.log.emit(f"💾 Сохранено: {buf.path}")
            self._update_title()

        def fail(msg):
            if "FileChangedExternally" in msg:
                r = QtWidgets.QMessageBox.warning(
                    self, "Файл изменён на той стороне",
                    f"{buf.path}\nизменён после того, как ты его открыл "
                    "(кто-то правил на роботе?).\n\nПерезаписать своей версией?",
                    QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No)
                if r == QtWidgets.QMessageBox.Yes:
                    self.save_current(force=True)
                return
            self.log.emit(f"❌ Не сохранено {buf.path}: {msg}")

        self._tasks.start(lambda emit: buf.save(text, force=force), done, fail, parent=self)


def _size(n):
    for unit in ("Б", "КБ", "МБ", "ГБ"):
        if n < 1024:
            return f"{n:.0f} {unit}"
        n /= 1024
    return f"{n:.1f} ТБ"
