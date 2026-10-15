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
from ui import icons
from core.i18n import t
from core import build_marks

_PLACEHOLDER = "__loading__"
ROLE_PATH = QtCore.Qt.UserRole
ROLE_DIR = QtCore.Qt.UserRole + 1
ROLE_MARK = QtCore.Qt.UserRole + 2     # (состояние, унаследовано?) — рисует MarkDelegate
COL_MARK = 2
_MARK_COLORS = {build_marks.YELLOW: "#e6c200", build_marks.GREEN: "#4caf50"}


class MarkDelegate(QtWidgets.QStyledItemDelegate):
    """Рисует кружок отметки сам: цвет не зависит от стилей приложения (текст мог выходить белым)."""

    def paint(self, painter, option, index):
        mark = index.data(ROLE_MARK)
        if mark is None:
            return super().paint(painter, option, index)
        QtWidgets.QApplication.style().drawPrimitive(
            QtWidgets.QStyle.PE_PanelItemViewItem, option, painter, option.widget)
        state, inherited = mark
        r = min(option.rect.height() - 8, 14)
        c = option.rect.center()
        rect = QtCore.QRect(c.x() - r // 2, c.y() - r // 2, r, r)
        painter.save()
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        if state is None:
            painter.setPen(QtGui.QPen(QtGui.QColor("#777"), 1.5))
            painter.setBrush(QtCore.Qt.NoBrush)
        else:
            col = QtGui.QColor(_MARK_COLORS[state])
            if inherited:
                col.setAlpha(120)
            painter.setPen(QtCore.Qt.NoPen)
            painter.setBrush(col)
        painter.drawEllipse(rect)
        painter.restore()

    def sizeHint(self, option, index):
        return QtCore.QSize(28, 22)


class FileBrowser(QtWidgets.QWidget):
    log = QtCore.Signal(str)
    marks_changed = QtCore.Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self._tasks = TaskHost()
        self._sources = []          # [(label, provider, root)]
        self._provider = None
        self._buffer = None
        self._loading_text = False
        self._current_idx = -1
        self._orphan_path = None     # файл, чей источник пропал (связь оборвалась)
        self._local_root = None      # src/ проекта; отметки сборки только для него
        self._marks = {}
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
        top.addWidget(QtWidgets.QLabel(t("fb.source")))
        top.addWidget(self.cmb_source, 1)
        top.addWidget(btn_refresh)
        lay.addLayout(top)

        split = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        self.tree = QtWidgets.QTreeWidget()
        self.tree.setHeaderLabels([t("fb.col_name"), t("fb.col_size"), t("fb.col_build")])
        self.tree.headerItem().setToolTip(COL_MARK, t("fb.build_tip"))
        self.tree.itemClicked.connect(self._on_click)
        self.tree.setItemDelegateForColumn(COL_MARK, MarkDelegate(self.tree))
        hdr = self.tree.header()
        hdr.setStretchLastSection(False)
        hdr.setSectionResizeMode(0, QtWidgets.QHeaderView.Stretch)
        hdr.setSectionResizeMode(1, QtWidgets.QHeaderView.ResizeToContents)
        hdr.setSectionResizeMode(COL_MARK, QtWidgets.QHeaderView.Fixed)
        self.tree.setColumnWidth(COL_MARK, 64)
        self.tree.itemExpanded.connect(self._on_expand)
        self.tree.itemDoubleClicked.connect(self._on_open)
        split.addWidget(self.tree)

        right = QtWidgets.QWidget()
        rl = QtWidgets.QVBoxLayout(right)
        rl.setContentsMargins(0, 0, 0, 0)
        bar = QtWidgets.QHBoxLayout()
        self.lbl_file = QtWidgets.QLabel(t("fb.no_file"))
        self.lbl_file.setStyleSheet("color: #ccc; padding: 2px;")
        self.btn_save = QtWidgets.QPushButton(t("fb.save"))
        self.btn_save.setIcon(icons.icon("save"))
        self.btn_save.clicked.connect(self.save_current)
        bar.addWidget(self.lbl_file, 1)
        bar.addWidget(self.btn_save)
        rl.addLayout(bar)
        self.editor = CodeEditor(mode="plain")
        self.editor.textChanged.connect(self._update_title)
        rl.addWidget(self.editor)
        split.addWidget(right)
        split.setSizes([380, 620])
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
        self._local_root = local_root if (local_root and os.path.isdir(local_root)) else None
        self._reload_marks()
        if local_root and os.path.isdir(local_root):
            self._sources.append((t("fb.src_project"), LocalProvider(), local_root))
        if session is not None and session.connected:
            self._sources.append((t("fb.src_host", target=session.profile.display()),
                                  session.host_provider(), session.remote_src))
            cp = session.container_provider()
            if cp is not None:
                root = f"{session.profile.build_dir.rstrip('/')}/src/{session.package}"
                self._sources.append((t("fb.src_container", name=session.profile.container), cp, root))

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
            self.log.emit(t("fb.connection_lost_log", path=self._orphan_path))
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

    # ---------- отметки сборки ----------

    def _project_dir(self):
        return os.path.dirname(os.path.normpath(self._local_root)) if self._local_root else None

    def _marks_active(self):
        """Отметки есть только у локального проекта (источник №0 — проект)."""
        return bool(self._local_root) and self._current_idx == 0 and bool(self._sources) \
            and self._sources[0][2] == self._local_root

    def _reload_marks(self):
        pd = self._project_dir()
        self._marks = build_marks.load(pd) if pd else {}

    def _rel(self, path):
        return build_marks.norm(os.path.relpath(path, self._local_root))

    def _paint_mark(self, item):
        path = item.data(0, ROLE_PATH)
        if not self._marks_active() or not path or path == self._local_root:
            item.setData(COL_MARK, ROLE_MARK, None)
            return
        rel = self._rel(path)
        own = self._marks.get(rel)
        st = own or build_marks.effective(self._marks, rel)
        item.setData(COL_MARK, ROLE_MARK, (st, own is None and st is not None))
        item.setToolTip(COL_MARK, t("fb.mark_" + st) if st else t("fb.mark_none"))

    def _on_click(self, item, col):
        if col != COL_MARK or not self._marks_active():
            return
        path = item.data(0, ROLE_PATH)
        if not path or path == self._local_root:
            return
        rel = self._rel(path)
        cur = self._marks.get(rel)
        self._marks = build_marks.set_mark(self._project_dir(), rel, build_marks.next_state(cur))
        self._repaint_marks()
        self.marks_changed.emit()

    def _repaint_marks(self):
        def walk(it):
            self._paint_mark(it)
            for i in range(it.childCount()):
                walk(it.child(i))
        for i in range(self.tree.topLevelItemCount()):
            walk(self.tree.topLevelItem(i))

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
        self.tree.setColumnHidden(COL_MARK, not self._marks_active())
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
        item.child(0).setText(0, t("fb.loading"))

        def done(entries):
            item.takeChildren()
            for e in entries:
                child = QtWidgets.QTreeWidgetItem(
                    [e.name, "" if e.is_dir else _size(e.size)])
                if e.is_dir:
                    child.setIcon(0, icons.icon("open", color="#d6b656", size=16))
                child.setData(0, ROLE_PATH, e.path)
                child.setData(0, ROLE_DIR, e.is_dir)
                if e.is_dir:
                    child.addChild(QtWidgets.QTreeWidgetItem([_PLACEHOLDER]))
                item.addChild(child)
                self._paint_mark(child)

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
            self.log.emit(t("fb.cannot_open_log", path=path, error=msg))
            QtWidgets.QMessageBox.information(self, t("fb.cannot_open_title"), msg)

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
                self.lbl_file.setText(t("fb.orphan_label", path=self._orphan_path))
                self.lbl_file.setStyleSheet("color: #ff8a65; padding: 2px; font-weight: bold;")
            else:
                self.lbl_file.setText(t("fb.no_file"))
                self.lbl_file.setStyleSheet("color: #ccc; padding: 2px;")
            self.btn_save.setEnabled(False)
            return
        self.lbl_file.setStyleSheet("color: #ccc; padding: 2px;")
        dirty = self.is_dirty()
        self.lbl_file.setText(("● " if dirty else "") + self._buffer.path)
        self.btn_save.setEnabled(dirty)

    def _confirm_discard(self):
        if self._orphan_path:
            what = t("fb.orphan_what", path=self._orphan_path)
        elif self.is_dirty():
            what = self._buffer.path
        else:
            return True
        r = QtWidgets.QMessageBox.question(
            self, t("fb.unsaved_title"),
            t("fb.unsaved_text", what=what))
        return r == QtWidgets.QMessageBox.Yes

    def save_current(self, force=False):
        if not self._buffer or not self.is_dirty():
            return
        buf, text = self._buffer, self.editor.toPlainText()

        def done(_):
            self.log.emit(t("fb.saved_log", path=buf.path))
            self._update_title()

        def fail(msg):
            if "FileChangedExternally" in msg:
                r = QtWidgets.QMessageBox.warning(
                    self, t("fb.changed_title"),
                    t("fb.changed_text", path=buf.path),
                    QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No)
                if r == QtWidgets.QMessageBox.Yes:
                    self.save_current(force=True)
                return
            self.log.emit(t("fb.not_saved_log", path=buf.path, error=msg))

        self._tasks.start(lambda emit: buf.save(text, force=force), done, fail, parent=self)


def _size(n):
    for key in ("fb.unit_b", "fb.unit_kb", "fb.unit_mb", "fb.unit_gb"):
        if n < 1024:
            return f"{n:.0f} {t(key)}"
        n /= 1024
    return f"{n:.1f} {t('fb.unit_tb')}"
