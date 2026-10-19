import sys
import shutil
import subprocess
import os
from PySide6 import QtWidgets, QtCore
from core.workspace_manager import WorkspaceManager
from core import i18n, settings
from core.i18n import t

class LauncherDialog(QtWidgets.QDialog):
    def __init__(self):
        super().__init__()
        self.resize(600, 450)
        self.selected_project_path = None
        
        layout = QtWidgets.QHBoxLayout(self)
        
        # left: project list
        left_layout = QtWidgets.QVBoxLayout()
        self.lbl_recent = QtWidgets.QLabel()
        left_layout.addWidget(self.lbl_recent)
        self.project_list = QtWidgets.QListWidget()
        self.project_list.itemDoubleClicked.connect(self.accept_selection)
        left_layout.addWidget(self.project_list)
        layout.addLayout(left_layout, stretch=2)
        
        # right: create
        right_layout = QtWidgets.QVBoxLayout()
        self.lbl_create = QtWidgets.QLabel()
        right_layout.addWidget(self.lbl_create)
        
        self.name_input = QtWidgets.QLineEdit()
        right_layout.addWidget(self.name_input)
        
        # === LANGUAGE CHOICE ===
        self.lbl_project_lang = QtWidgets.QLabel()
        right_layout.addWidget(self.lbl_project_lang)
        self.type_combo = QtWidgets.QComboBox()
        self.type_combo.addItems(["", "", ""])      # texts are set in _retranslate()
        right_layout.addWidget(self.type_combo)
        # ===================

        
        
        # right click: Open / Show in Explorer / Delete
        self.project_list.setContextMenuPolicy(QtCore.Qt.CustomContextMenu)
        self.project_list.customContextMenuRequested.connect(self.show_project_menu)
        

        # buttons under the list (duplicate the menu for visibility)
        btns_row = QtWidgets.QHBoxLayout()
        self.btn_show = btn_show = QtWidgets.QPushButton()
        btn_show.clicked.connect(self.show_selected_in_explorer)
        self.btn_del = btn_del = QtWidgets.QPushButton()
        btn_del.setStyleSheet("background-color: #7a2020; color: white;")
        btn_del.clicked.connect(self.delete_selected_project)
        btns_row.addWidget(btn_show)
        btns_row.addWidget(btn_del)
        left_layout.addLayout(btns_row)

        self.btn_create = btn_create = QtWidgets.QPushButton()
        btn_create.setStyleSheet("background-color: #2e7d32; color: white; padding: 8px;")
        btn_create.clicked.connect(self.create_project)
        right_layout.addWidget(btn_create)
        
        right_layout.addSpacing(20)
        self.btn_open_folder = btn_open_folder = QtWidgets.QPushButton()
        btn_open_folder.clicked.connect(self.browse_project)
        right_layout.addWidget(btn_open_folder)
        
        right_layout.addStretch()
        self.chk_single = QtWidgets.QCheckBox()
        self.chk_single.setChecked(bool(settings.get("single_canvas")))
        self.chk_single.toggled.connect(lambda v: settings.set("single_canvas", bool(v)))
        right_layout.addWidget(self.chk_single)
        self.lbl_ui_lang = QtWidgets.QLabel()
        right_layout.addWidget(self.lbl_ui_lang)
        self.lang_combo = QtWidgets.QComboBox()
        for lang in i18n.available_languages():
            self.lang_combo.addItem(lang["name"], lang["code"])
        idx = self.lang_combo.findData(i18n.get_language())
        self.lang_combo.setCurrentIndex(max(idx, 0))
        self.lang_combo.currentIndexChanged.connect(self._on_language_changed)
        right_layout.addWidget(self.lang_combo)
        layout.addLayout(right_layout, stretch=1)
        
        self._retranslate()
        self.refresh_list()
        self.setStyleSheet("""
            QDialog { background-color: #333; color: white; }
            QListWidget { background-color: #444; border: 1px solid #555; color: white; }
            QLineEdit, QComboBox { padding: 5px; background: #555; color: white; border: 1px solid #666; }
            QPushButton { padding: 5px; font-weight: bold; }
        """)

    def _on_language_changed(self, _index):
        code = self.lang_combo.currentData()
        if code and i18n.set_language(code):
            settings.set("language", code)
            self._retranslate()

    def _retranslate(self):
        """All window texts from the dictionary; called on creation and on language change."""
        self.setWindowTitle(t("launcher.title"))
        self.lbl_recent.setText(t("launcher.recent"))
        self.lbl_create.setText(t("launcher.create_new"))
        self.name_input.setPlaceholderText(t("launcher.name_placeholder"))
        self.lbl_project_lang.setText(t("launcher.project_language"))
        for i, key in enumerate(("type_python", "type_cpp", "type_mixed")):
            self.type_combo.setItemText(i, t(f"launcher.{key}"))
        self.btn_show.setText(t("launcher.show_explorer"))
        self.btn_del.setText(t("launcher.delete"))
        self.btn_create.setText(t("launcher.create_open"))
        self.btn_open_folder.setText(t("launcher.open_disk"))
        self.lbl_ui_lang.setText(t("launcher.interface_language"))
        self.chk_single.setText(t("launcher.single_canvas"))
        self.chk_single.setToolTip(t("launcher.single_canvas_tip"))

    def refresh_list(self):
        self.project_list.clear()
        projects = WorkspaceManager.list_projects()
        for p in projects:
            full_path = os.path.join(WorkspaceManager.get_workspace_root(), p)
            p_type = WorkspaceManager.get_project_type(full_path).upper()
            
            item = QtWidgets.QListWidgetItem(f"{p}  [{p_type}]")
            item.setData(QtCore.Qt.UserRole, full_path)
            self.project_list.addItem(item)

    def _selected_path(self):
        item = self.project_list.currentItem()
        return item.data(QtCore.Qt.UserRole) if item else None

    def show_project_menu(self, pos):
        item = self.project_list.itemAt(pos)
        if not item:
            return
        self.project_list.setCurrentItem(item)
        menu = QtWidgets.QMenu(self)
        act_open = menu.addAction(t("launcher.menu_open"))
        act_show = menu.addAction(t("launcher.show_explorer"))
        menu.addSeparator()
        act_del = menu.addAction(t("launcher.menu_delete"))
        chosen = menu.exec(self.project_list.mapToGlobal(pos))
        if chosen == act_open:
            self.accept_selection()
        elif chosen == act_show:
            self.show_selected_in_explorer()
        elif chosen == act_del:
            self.delete_selected_project()

    def show_selected_in_explorer(self):
        path = self._selected_path()
        if not path or not os.path.isdir(path):
            return
        if sys.platform == 'win32':
            os.startfile(path)
        elif sys.platform == 'darwin':
            subprocess.Popen(['open', path])
        else:
            subprocess.Popen(['xdg-open', path])

    def delete_selected_project(self):
        path = self._selected_path()
        if not path:
            return
        name = os.path.basename(path)

        # safety: delete only what lies inside the workspace root
        root = os.path.abspath(WorkspaceManager.get_workspace_root())
        target = os.path.abspath(path)
        if not target.startswith(root + os.sep):
            QtWidgets.QMessageBox.warning(
                self, t("launcher.refused_title"), t("launcher.refused_text"))
            return

        ans = QtWidgets.QMessageBox.warning(
            self, t("launcher.delete_title"),
            t("launcher.delete_text", name=name, path=path),
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No)
        if ans != QtWidgets.QMessageBox.Yes:
            return
        try:
            shutil.rmtree(target)
            self.refresh_list()
        except Exception as e:
            QtWidgets.QMessageBox.critical(
                self, t("launcher.delete_failed_title"),
                t("launcher.delete_failed_text", error=e))
            
    def create_project(self):
        name = self.name_input.text()
        if not name: return

        import re
        if not re.match(r'^[a-z][a-z0-9_]*$', name):
            QtWidgets.QMessageBox.warning(
                self, t("launcher.invalid_name_title"), t("launcher.invalid_name_text"))
            return
        
        idx = self.type_combo.currentIndex()
        p_type = ("python", "cpp", "mixed")[idx]
        
        try:
            path = WorkspaceManager.create_project(name, p_type)
            self.selected_project_path = path
            self.accept()
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, t("launcher.error_title"), str(e))

    def browse_project(self):
        root = WorkspaceManager.get_workspace_root()
        path = QtWidgets.QFileDialog.getExistingDirectory(self, t("launcher.open_folder_title"), root)
        if path:
            self.selected_project_path = path
            self.accept()

    def accept_selection(self):
        item = self.project_list.currentItem()
        if item:
            self.selected_project_path = item.data(QtCore.Qt.UserRole)
            self.accept()