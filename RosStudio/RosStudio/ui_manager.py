import os
import json
from PySide6 import QtWidgets, QtCore, QtGui

# === 1. СПИСОК С DRAG & DROP И УДАЛЕНИЕМ ===
class DraggableListWidget(QtWidgets.QListWidget):
    def __init__(self, parent=None, user_lib_path=""):
        super().__init__(parent)
        self.user_lib_path = user_lib_path
        
        self.setDragEnabled(True)
        self.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.setDragDropMode(QtWidgets.QAbstractItemView.DragOnly)
        self.setDefaultDropAction(QtCore.Qt.CopyAction)
        
        # Включаем контекстное меню
        self.setContextMenuPolicy(QtCore.Qt.CustomContextMenu)
        self.customContextMenuRequested.connect(self.show_context_menu)

    def startDrag(self, supportedActions):
        item = self.currentItem()
        if not item: return
        code = item.data(QtCore.Qt.UserRole)
        if not code: return

        mime_data = QtCore.QMimeData()
        mime_data.setText(code)
        
        drag = QtGui.QDrag(self)
        drag.setMimeData(mime_data)
        
        rect = self.visualItemRect(item)
        pixmap = self.viewport().grab(rect)
        painter = QtGui.QPainter(pixmap)
        painter.setCompositionMode(QtGui.QPainter.CompositionMode_DestinationIn)
        painter.fillRect(pixmap.rect(), QtGui.QColor(0, 0, 0, 150))
        painter.end()
        drag.setPixmap(pixmap)
        drag.setHotSpot(QtCore.QPoint(pixmap.width() // 2, pixmap.height() // 2))
        drag.exec(supportedActions)

    def show_context_menu(self, pos):
        """Контекстное меню для удаления кастомных нод"""
        item = self.itemAt(pos)
        if not item: return
        
        code = item.data(QtCore.Qt.UserRole)
        
        # Разрешаем удалять только пользовательские шаблоны
        if code and str(code).startswith("USER_LIB:"):
            menu = QtWidgets.QMenu(self)
            menu.setStyleSheet("QMenu { background-color: #333; color: white; border: 1px solid #555; } QMenu::item:selected { background-color: #555; }")
            
            delete_action = menu.addAction("Delete Template")
            action = menu.exec(self.mapToGlobal(pos))
            
            if action == delete_action:
                self.delete_template(item, code)

    def delete_template(self, item, code):
        """Логика удаления файла и элемента списка"""
        filename = code.split(":", 1)[1]
        path = os.path.join(self.user_lib_path, filename)
        
        confirm = QtWidgets.QMessageBox.question(
            self, "Confirm Delete", 
            f"Are you sure you want to delete '{filename}'?",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No
        )
        
        if confirm == QtWidgets.QMessageBox.Yes:
            if os.path.exists(path):
                try:
                    os.remove(path)
                except Exception as e:
                    print(f"Error deleting file: {e}")
            self.takeItem(self.row(item))

# === 2. МЕНЕДЖЕР UI ===
class UiManager:
    def __init__(self, main_window):
        self.main_window = main_window
        self.tabs = None
        self.console_sys = None
        self.console_ros = None
        self.node_list = None
        self.actions = {}
        
        self.icons = {} 
        

        base_dir = os.path.dirname(os.path.abspath(__file__))
        self.user_lib_path = os.path.join(base_dir, "user_blueprints")
        
        self.assets_path = os.path.join(base_dir, "assets")

        if not os.path.exists(self.user_lib_path):
            os.makedirs(self.user_lib_path)
        self._load_icons()

    def _load_icons(self):
        # 1. Стандартные иконки Qt (чтобы не искать картинки для дискеты и папки)
        style = self.main_window.style()
        self.icons['save'] = style.standardIcon(QtWidgets.QStyle.SP_DialogSaveButton)
        self.icons['open'] = style.standardIcon(QtWidgets.QStyle.SP_DialogOpenButton)
        self.icons['run'] = style.standardIcon(QtWidgets.QStyle.SP_MediaPlay)
        self.icons['stop'] = style.standardIcon(QtWidgets.QStyle.SP_MediaStop)
        self.icons['clear'] = style.standardIcon(QtWidgets.QStyle.SP_TrashIcon)
        self.icons['group'] = style.standardIcon(QtWidgets.QStyle.SP_DirClosedIcon)

        # 2. Кастомная иконка Docker из папки assets
        docker_icon_path = os.path.join(self.assets_path, "docker.png")
        if os.path.exists(docker_icon_path):
            self.icons['docker'] = QtGui.QIcon(docker_icon_path)
        else:
            # Если файл не найден, ставим заглушку (иконку компьютера)
            print(f"Warning: Icon not found at {docker_icon_path}")
            self.icons['docker'] = style.standardIcon(QtWidgets.QStyle.SP_ComputerIcon)

    def setup_ui(self):
        self.main_window.setWindowTitle("ROS2 Blueprint Studio")
        self.main_window.resize(1400, 900)
        self._apply_theme()
        self._create_central_widget()
        self._create_docks()
        self._create_toolbar()
        self._create_menubar()

    def _apply_theme(self):
        pass
    def _create_menubar(self):
        menubar = self.main_window.menuBar()
        file_menu = menubar.addMenu("File")
        
        # Используем те же экшены, что и в тулбаре
        if 'save' in self.actions: file_menu.addAction(self.actions['save'])
        if 'open' in self.actions: file_menu.addAction(self.actions['open'])
        
        file_menu.addSeparator()
        
        # === EXPORT С КИТОМ ===
        # Используем загруженную иконку 'docker'
        export_action = QtGui.QAction(self.icons['docker'], "Export Portable Docker...", self.main_window)
        export_action.setToolTip("Create a portable Docker folder for deployment")
        
        self.actions['export_docker'] = export_action 
        file_menu.addAction(export_action)


    def _create_central_widget(self):
        self.tabs = QtWidgets.QTabWidget()
        self.main_window.setCentralWidget(self.tabs)

    def _create_toolbar(self):
        toolbar = self.main_window.addToolBar("Main")
        toolbar.setIconSize(QtCore.QSize(24, 24))
        toolbar.setMovable(False)
        
        # Вспомогательная функция для сокращения кода
        def add_action(key, name, tooltip, shortcut=None):
            icon = self.icons.get(key)
            action = QtGui.QAction(icon, name, self.main_window)
            action.setToolTip(tooltip) # 👇 Всплывающая подсказка
            if shortcut:
                action.setShortcut(shortcut)
            toolbar.addAction(action)
            return action

        # === КНОПКИ С ПОДПИСЯМИ ===
        self.actions['save'] = add_action('save', "Save", "Save Project (Ctrl+S)", "Ctrl+S")
        self.actions['open'] = add_action('open', "Open", "Open Project Folder (Ctrl+O)", "Ctrl+O")
        
        toolbar.addSeparator()
        self.actions['group'] = add_action('group', "Group", "Group Selected Nodes (Ctrl+G)", "Ctrl+G")
        
        toolbar.addSeparator()
        self.actions['run'] = add_action('run', "Run", "Run in Docker Container")
        self.actions['stop'] = add_action('stop', "Stop", "Stop Docker Container")
        self.actions['stop'].setEnabled(False)
        
        toolbar.addSeparator()
        self.actions['clear'] = add_action('clear', "Clear", "Clear All Nodes")

    def _create_docks(self):
        dock_sys = QtWidgets.QDockWidget("System Log", self.main_window)
        self.console_sys = QtWidgets.QTextEdit(); self.console_sys.setReadOnly(True)
        dock_sys.setWidget(self.console_sys)
        self.main_window.addDockWidget(QtCore.Qt.BottomDockWidgetArea, dock_sys)

        dock_ros = QtWidgets.QDockWidget("ROS2 Output", self.main_window)
        self.console_ros = QtWidgets.QTextEdit(); self.console_ros.setReadOnly(True)
        self.console_ros.setStyleSheet("background: #101010; color: #cccccc; font-family: Consolas;")
        dock_ros.setWidget(self.console_ros)
        self.main_window.addDockWidget(QtCore.Qt.BottomDockWidgetArea, dock_ros)
        self.main_window.tabifyDockWidget(dock_sys, dock_ros)

    def create_palette(self, callback_double_click):
        dock = QtWidgets.QDockWidget("Library", self.main_window)
        dock.setAllowedAreas(QtCore.Qt.LeftDockWidgetArea | QtCore.Qt.RightDockWidgetArea)
        
        self.node_list = DraggableListWidget(user_lib_path=self.user_lib_path)
        self.node_list.itemDoubleClicked.connect(callback_double_click)
        
        dock.setWidget(self.node_list)
        self.main_window.addDockWidget(QtCore.Qt.LeftDockWidgetArea, dock)
        self.refresh_palette()

    # === ВОТ ТУТ ГЛАВНЫЕ ИЗМЕНЕНИЯ ===
    def refresh_palette(self):
        self.node_list.clear()

        # --- COURSEWORK (НОВОЕ!) ---
        self._add_separator("Coursework Final")
        self._add_palette_item("Level Builder", "CW_BUILDER", "#ff5722")
        self._add_palette_item("Hive Mind", "CW_HIVE", "#9c27b0")
        self._add_palette_item("Smart Agent", "CW_AGENT", "#2196f3")



        # --- PYTHON NODES ---
        self._add_separator("Python Nodes")
        # Новые коды: PY_STRING_PUB, PY_TWIST_SUB и т.д.
        self._add_palette_item("Py: String Pub", "PY_STRING_PUB", "#2e7d32")
        self._add_palette_item("Py: String Sub", "PY_STRING_SUB", "#2e7d32")
        self._add_palette_item("Py: Twist Pub", "PY_TWIST_PUB", "#2e7d32")
        self._add_palette_item("Py: Twist Sub", "PY_TWIST_SUB", "#2e7d32")
        self._add_palette_item("Py: Custom Node", "PY_CUSTOM", "#1b5e20")
        
        # --- C++ NODES ---
        self._add_separator("C++ Nodes")
        self._add_palette_item("C++: String Pub", "CPP_STRING_PUB", "#1565c0")
        self._add_palette_item("C++: String Sub", "CPP_STRING_SUB", "#1565c0")
        self._add_palette_item("C++: Twist Pub", "CPP_TWIST_PUB", "#1565c0")
        self._add_palette_item("C++: Twist Sub", "CPP_TWIST_SUB", "#1565c0")
        self._add_palette_item("C++: Custom Node", "CPP_CUSTOM", "#0d47a1")
        
        # --- TOOLS ---
        self._add_separator("Tools")
        self._add_palette_item("Data Monitor", "MONITOR", "#006064")

        # --- USER LIB ---
        self._add_separator("My Blueprints")
        if os.path.exists(self.user_lib_path):
            files = [f for f in os.listdir(self.user_lib_path) if f.endswith('.json')]
            if not files:
                item = QtWidgets.QListWidgetItem("(Empty)")
                item.setForeground(QtGui.QColor("#888"))
                item.setFlags(QtCore.Qt.NoItemFlags)
                self.node_list.addItem(item)
            else:
                for f in files:
                    name = f.replace('.json', '')
                    self._add_palette_item(name, f"USER_LIB:{f}", "#e65100")

    def _add_palette_item(self, name, code, color_hex):
        item = QtWidgets.QListWidgetItem(name)
        item.setForeground(QtGui.QColor(color_hex))
        font = item.font(); font.setBold(True); font.setPointSize(10); item.setFont(font)
        item.setData(QtCore.Qt.UserRole, code)
        self.node_list.addItem(item)
    
    def _add_separator(self, text):
        item = QtWidgets.QListWidgetItem(f"--- {text} ---")
        item.setTextAlignment(QtCore.Qt.AlignCenter)
        item.setFlags(QtCore.Qt.NoItemFlags)
        item.setForeground(QtGui.QColor("#888"))
        self.node_list.addItem(item)