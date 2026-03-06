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
        
        btn_create = QtWidgets.QPushButton("Create & Open")
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

    def create_project(self):
        name = self.name_input.text()
        if not name: return
        
        # Получаем выбор пользователя
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