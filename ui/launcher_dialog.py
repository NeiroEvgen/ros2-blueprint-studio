import sys
import shutil
import subprocess
import os
from PySide6 import QtWidgets, QtCore, QtGui
from core.workspace_manager import WorkspaceManager

class LauncherDialog(QtWidgets.QDialog):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ROS2 Studio Launcher")
        self.resize(600, 450)
        self.selected_project_path = None
        
        layout = QtWidgets.QHBoxLayout(self)
        
        # Слева: Список проектов
        left_layout = QtWidgets.QVBoxLayout()
        left_layout.addWidget(QtWidgets.QLabel("Recent Projects:"))
        self.project_list = QtWidgets.QListWidget()
        self.project_list.itemDoubleClicked.connect(self.accept_selection)
        left_layout.addWidget(self.project_list)
        layout.addLayout(left_layout, stretch=2)
        
        # Справа: Создание
        right_layout = QtWidgets.QVBoxLayout()
        right_layout.addWidget(QtWidgets.QLabel("Create New Project:"))
        
        self.name_input = QtWidgets.QLineEdit()
        self.name_input.setPlaceholderText("Project Name...")
        right_layout.addWidget(self.name_input)
        
        # === ВЫБОР ЯЗЫКА ===
        right_layout.addWidget(QtWidgets.QLabel("Project Language:"))
        self.type_combo = QtWidgets.QComboBox()
        self.type_combo.addItems(["Python (Scripts only)", "C++ (Compilation required)"])
        right_layout.addWidget(self.type_combo)
        # ===================

        
        
        # Правый клик: Open / Show in Explorer / Delete
        self.project_list.setContextMenuPolicy(QtCore.Qt.CustomContextMenu)
        self.project_list.customContextMenuRequested.connect(self.show_project_menu)
        

        # Кнопки под списком (дублируют меню для наглядности)
        btns_row = QtWidgets.QHBoxLayout()
        btn_show = QtWidgets.QPushButton(" Show in Explorer")
        btn_show.clicked.connect(self.show_selected_in_explorer)
        btn_del = QtWidgets.QPushButton(" Delete")
        btn_del.setStyleSheet("background-color: #7a2020; color: white;")
        btn_del.clicked.connect(self.delete_selected_project)
        btns_row.addWidget(btn_show)
        btns_row.addWidget(btn_del)
        left_layout.addLayout(btns_row)

        btn_create = QtWidgets.QPushButton("Create and Open")
        btn_create.setStyleSheet("background-color: #2e7d32; color: white; padding: 8px;")
        btn_create.clicked.connect(self.create_project)
        right_layout.addWidget(btn_create)
        
        right_layout.addSpacing(20)
        btn_open_folder = QtWidgets.QPushButton("Open from Disk...")
        btn_open_folder.clicked.connect(self.browse_project)
        right_layout.addWidget(btn_open_folder)
        
        right_layout.addStretch()
        layout.addLayout(right_layout, stretch=1)
        
        self.refresh_list()
        self.setStyleSheet("""
            QDialog { background-color: #333; color: white; }
            QListWidget { background-color: #444; border: 1px solid #555; color: white; }
            QLineEdit, QComboBox { padding: 5px; background: #555; color: white; border: 1px solid #666; }
            QPushButton { padding: 5px; font-weight: bold; }
        """)

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
        act_open = menu.addAction(" Open")
        act_show = menu.addAction(" Show in Explorer")
        menu.addSeparator()
        act_del = menu.addAction(" Delete project...")
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

        # Защита: удаляем только то, что лежит внутри workspace root
        root = os.path.abspath(WorkspaceManager.get_workspace_root())
        target = os.path.abspath(path)
        if not target.startswith(root + os.sep):
            QtWidgets.QMessageBox.warning(
                self, "Refused",
                "This project is outside the workspace folder — delete it manually.")
            return

        ans = QtWidgets.QMessageBox.warning(
            self, "Delete project",
            f"Permanently delete project '{name}'?\n\n{path}\n\n"
            "This removes the whole folder from disk and cannot be undone.",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No)
        if ans != QtWidgets.QMessageBox.Yes:
            return
        try:
            shutil.rmtree(target)
            self.refresh_list()
        except Exception as e:
            QtWidgets.QMessageBox.critical(
                self, "Delete failed",
                f"Could not delete project:\n{e}\n\n"
                "The folder may be in use (editor/terminal open inside it).")
            
    def create_project(self):
        name = self.name_input.text()
        if not name: return

        import re
        if not re.match(r'^[a-z][a-z0-9_]*$', name):
            QtWidgets.QMessageBox.warning(
                self, "Invalid name",
                "Project name must start with a lowercase letter and contain\n"
                "only lowercase latin letters, digits and underscores\n"
                "(it becomes a ROS 2 package name).\n\n"
                "Examples: arm_demo, my_robot2")
            return
        
        idx = self.type_combo.currentIndex()
        p_type = "python" if idx == 0 else "cpp"
        
        try:
            path = WorkspaceManager.create_project(name, p_type)
            self.selected_project_path = path
            self.accept()
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Error", str(e))

    def browse_project(self):
        root = WorkspaceManager.get_workspace_root()
        path = QtWidgets.QFileDialog.getExistingDirectory(self, "Open Project Folder", root)
        if path:
            self.selected_project_path = path
            self.accept()

    def accept_selection(self):
        item = self.project_list.currentItem()
        if item:
            self.selected_project_path = item.data(QtCore.Qt.UserRole)
            self.accept()