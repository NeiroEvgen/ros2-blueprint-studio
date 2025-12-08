import os
import json
from PySide6 import QtWidgets, QtCore, QtGui

# === 1. Drag & Drop List Widget ===
class DraggableListWidget(QtWidgets.QListWidget):
    def __init__(self, parent=None, user_lib_path=""):
        super().__init__(parent)
        self.user_lib_path = user_lib_path
        
        self.setDragEnabled(True)
        self.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.setDragDropMode(QtWidgets.QAbstractItemView.DragOnly)
        self.setDefaultDropAction(QtCore.Qt.CopyAction)
        
        # Context Menu
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
        
        # Create a visual representation for the drag
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
        """Context menu for deleting user templates."""
        item = self.itemAt(pos)
        if not item: return
        
        code = item.data(QtCore.Qt.UserRole)
        
        # Only allow deleting user library items
        if code and str(code).startswith("USER_LIB:"):
            menu = QtWidgets.QMenu(self)
            menu.setStyleSheet("QMenu { background-color: #333; color: white; border: 1px solid #555; } QMenu::item:selected { background-color: #555; }")
            
            delete_action = menu.addAction("Delete Template")
            action = menu.exec(self.mapToGlobal(pos))
            
            if action == delete_action:
                self.delete_template(item, code)

    def delete_template(self, item, code):
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

# === 2. UI Manager ===
class UiManager:
    def __init__(self, main_window):
        self.main_window = main_window
        self.tabs = None
        self.console_sys = None
        self.console_ros = None
        self.node_list = None
        self.actions = {}
        
        base_dir = os.path.dirname(os.path.abspath(__file__))
        self.user_lib_path = os.path.join(base_dir, "user_blueprints")
        if not os.path.exists(self.user_lib_path):
            os.makedirs(self.user_lib_path)

    def setup_ui(self):
        self.main_window.setWindowTitle("ROS2 Blueprint Studio")
        self.main_window.resize(1400, 900)
        self._apply_theme()
        self._create_central_widget()
        self._create_docks()
        self._create_toolbar()

    def _apply_theme(self):
        self.main_window.setStyleSheet("""
            QMainWindow { background-color: #404040; color: #000000; }
            QTabWidget::pane { border: 1px solid #555; background: #2b2b2b; }
            QTabBar::tab { background: #e0e0e0; color: #000; padding: 6px 15px; margin-right: 2px; }
            QTabBar::tab:selected { background: #fff; font-weight: bold; border-bottom: 2px solid #2196f3; }
            QDockWidget { background-color: #f0f0f0; color: #000; border: 1px solid #ccc; }
            QDockWidget::title { background: #d0d0d0; text-align: left; padding: 6px; font-weight: bold; color: #000; }
            QListWidget, DraggableListWidget { background-color: #ffffff; color: #000; border: none; font-size: 13px; }
            QListWidget::item, DraggableListWidget::item { padding: 6px; border-bottom: 1px solid #eee; }
            QListWidget::item:selected, DraggableListWidget::item:selected { background-color: #bbdefb; color: #000; }
            QTextEdit { background-color: #1e1e1e; color: #00ff00; border: none; font-family: 'Consolas', monospace; font-size: 10pt; }
        """)

    def _create_central_widget(self):
        self.tabs = QtWidgets.QTabWidget()
        self.main_window.setCentralWidget(self.tabs)

    def _create_toolbar(self):
        toolbar = self.main_window.addToolBar("Main")
        toolbar.setIconSize(QtCore.QSize(24, 24))
        toolbar.setMovable(False)
        
        def add_action(name, icon_enum, tooltip):
            icon = self.main_window.style().standardIcon(icon_enum)
            action = QtGui.QAction(icon, "", self.main_window)
            action.setToolTip(f"{name} - {tooltip}")
            toolbar.addAction(action)
            return action

        self.actions['save'] = add_action("Save", QtWidgets.QStyle.SP_DialogSaveButton, "Save Project")
        self.actions['open'] = add_action("Open", QtWidgets.QStyle.SP_DialogOpenButton, "Open Project")
        toolbar.addSeparator()
        self.actions['group'] = add_action("Group", QtWidgets.QStyle.SP_DirClosedIcon, "Group Selection (Ctrl+G)")
        toolbar.addSeparator()
        self.actions['run'] = add_action("Run", QtWidgets.QStyle.SP_MediaPlay, "Run/Deploy")
        self.actions['stop'] = add_action("Stop", QtWidgets.QStyle.SP_MediaStop, "Stop")
        self.actions['stop'].setEnabled(False)
        toolbar.addSeparator()
        self.actions['clear'] = add_action("Clear", QtWidgets.QStyle.SP_TrashIcon, "Clear Graph")

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

    def refresh_palette(self):
        self.node_list.clear()
        
        # --- PYTHON NODES ---
        self._add_separator("Python Nodes")
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