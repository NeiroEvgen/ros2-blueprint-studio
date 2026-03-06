import os
from PySide6 import QtWidgets, QtCore, QtGui

# === 1. УМНЫЙ СПИСОК (FIX DRAG & DROP) ===
class DraggableListWidget(QtWidgets.QListWidget):
    def startDrag(self, supportedActions):
        item = self.currentItem()
        if not item: return
        
        # Достаем скрытый ID ноды (например, ros.py.PyStringPubNode)
        node_id = item.data(QtCore.Qt.UserRole)
        if not node_id: return

        # Упаковываем в MIME данные как обычный текст
        mime_data = QtCore.QMimeData()
        mime_data.setText(node_id)
        
        drag = QtGui.QDrag(self)
        drag.setMimeData(mime_data)
        
        # Рисуем "призрак" иконки при перетаскивании (для красоты)
        pixmap = item.listWidget().viewport().grab(self.visualItemRect(item))
        drag.setPixmap(pixmap)
        drag.setHotSpot(QtCore.QPoint(pixmap.width() // 2, pixmap.height() // 2))
        
        drag.exec(supportedActions)

# === 2. МЕНЕДЖЕР UI ===
class UiManager:
    def __init__(self, main_window):
        self.main_window = main_window
        self.tabs = None
        self.console_sys = None
        self.console_ros = None
        self.node_palette = None
        self.actions = {}
        
        self.user_lib_path = os.path.join(os.path.dirname(__file__), "user_lib")
        if not os.path.exists(self.user_lib_path):
            os.makedirs(self.user_lib_path)

    def setup_ui(self):
        self.main_window.setWindowTitle("ROS2 Visual Studio (File-Based)")
        self.main_window.resize(1400, 900)
        
        central_widget = QtWidgets.QWidget()
        self.main_window.setCentralWidget(central_widget)
        main_layout = QtWidgets.QVBoxLayout(central_widget)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)

        # === ВЕРХНЯЯ ПАНЕЛЬ ===
        top_bar = QtWidgets.QFrame()
        top_bar.setStyleSheet("background-color: #333; border-bottom: 2px solid #222;")
        top_bar.setFixedHeight(50)
        top_layout = QtWidgets.QHBoxLayout(top_bar)
        top_layout.setContentsMargins(10, 5, 10, 5)
        top_layout.setSpacing(10)

        def add_btn(text, color="#555"):
            btn = QtWidgets.QPushButton(text)
            btn.setStyleSheet(f"""
                QPushButton {{ 
                    background-color: {color}; color: white; border: none; 
                    padding: 5px 15px; font-weight: bold; border-radius: 3px;
                }}
                QPushButton:hover {{ background-color: #666; }}
                QPushButton:pressed {{ background-color: #444; }}
                QPushButton:disabled {{ background-color: #222; color: #555; }}
            """)
            top_layout.addWidget(btn)
            return btn

        self.actions['open'] = add_btn(" Open")
        self.actions['save'] = add_btn(" Save")
        
        line = QtWidgets.QFrame(); line.setFrameShape(QtWidgets.QFrame.VLine); line.setStyleSheet("color: #444;")
        top_layout.addWidget(line)
        
        self.actions['run'] = add_btn("▶ RUN", "#2e7d32")
        self.actions['stop'] = add_btn("⏹ STOP", "#c62828")
        self.actions['stop'].setEnabled(False)
        
        top_layout.addStretch()
        
        self.actions['group'] = add_btn(" Group")
        self.actions['clear'] = add_btn(" Clear")
        self.actions['export_docker'] = add_btn(" Export")

        main_layout.addWidget(top_bar)

        # === РАБОЧАЯ ОБЛАСТЬ ===
        main_splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        main_layout.addWidget(main_splitter)

        # ПАЛИТРА
        palette_panel = QtWidgets.QWidget()
        palette_layout = QtWidgets.QVBoxLayout(palette_panel)
        palette_layout.setContentsMargins(0,0,0,0)
        lbl = QtWidgets.QLabel("Node Palette")
        lbl.setStyleSheet("background: #222; color: #ddd; padding: 5px; font-weight: bold; border-bottom: 1px solid #444;")
        palette_layout.addWidget(lbl)
        
        # ИСПОЛЬЗУЕМ НАШ НОВЫЙ КЛАСС DRAGGABLE
        self.node_palette = DraggableListWidget()
        self.node_palette.setDragEnabled(True)
        self.node_palette.setStyleSheet("border: none; background: #2b2b2b;")
        palette_layout.addWidget(self.node_palette)
        
        main_splitter.addWidget(palette_panel)

        # ГРАФЫ
        self.tabs = QtWidgets.QTabWidget()
        self.tabs.setStyleSheet("QTabWidget::pane { border: 0; }")
        main_splitter.addWidget(self.tabs)

        # КОНСОЛИ
        console_dock = QtWidgets.QDockWidget("Logs & Terminal", self.main_window)
        console_widget = QtWidgets.QWidget()
        console_layout = QtWidgets.QVBoxLayout(console_widget)
        console_layout.setContentsMargins(0,0,0,0)
        
        console_tabs = QtWidgets.QTabWidget()
        self.console_sys = QtWidgets.QTextEdit()
        self.console_sys.setReadOnly(True)
        self.console_sys.setStyleSheet("background-color: #1e1e1e; color: #00ff00; font-family: Consolas; border: none;")
        console_tabs.addTab(self.console_sys, "System Log")
        
        self.console_ros = QtWidgets.QTextEdit()
        self.console_ros.setReadOnly(True)
        self.console_ros.setStyleSheet("background-color: #1e1e1e; color: #00ffff; font-family: Consolas; border: none;")
        console_tabs.addTab(self.console_ros, "ROS2 Output")
        
        console_layout.addWidget(console_tabs)
        console_dock.setWidget(console_widget)
        self.main_window.addDockWidget(QtCore.Qt.BottomDockWidgetArea, console_dock)

        main_splitter.setStretchFactor(0, 1)
        main_splitter.setStretchFactor(1, 5)

    def set_visible_graph(self, mode="python"):
        if mode == "python":
            self.tabs.setTabVisible(0, True); self.tabs.setTabVisible(1, False); self.tabs.setCurrentIndex(0)
        else:
            self.tabs.setTabVisible(0, False); self.tabs.setTabVisible(1, True); self.tabs.setCurrentIndex(1)

    def rebuild_palette(self, node_classes, project_type="python"):
        self.node_palette.clear()
        categories = {
            "Input (Subscribers)": [], "Output (Publishers)": [],
            "Logic & Processing": [], "Tools & Utility": [], "Coursework": []
        }
        for node_cls in node_classes:
            identifier = node_cls.type_; name = node_cls.NODE_NAME
            if project_type == "python" and "ros.cpp" in identifier: continue
            if project_type == "cpp" and "ros.py" in identifier: continue
            
            item = QtWidgets.QListWidgetItem(name)
            item.setData(QtCore.Qt.UserRole, identifier)
            
            if "PubNode" in name: categories["Output (Publishers)"].append(item)
            elif "SubNode" in name: categories["Input (Subscribers)"].append(item)
            elif "Monitor" in name or "Group" in name: categories["Tools & Utility"].append(item)
            elif "Coursework" in identifier or "LevelBuilder" in name: categories["Coursework"].append(item)
            else: categories["Logic & Processing"].append(item)

        for cat_name, items in categories.items():
            if not items: continue
            header = QtWidgets.QListWidgetItem(f"--- {cat_name} ---")
            header.setFlags(QtCore.Qt.NoItemFlags)
            header.setForeground(QtGui.QColor("#aaa")); header.setBackground(QtGui.QColor("#222")); header.setTextAlignment(QtCore.Qt.AlignCenter)
            self.node_palette.addItem(header)
            for item in items: self.node_palette.addItem(item)

        if os.path.exists(self.user_lib_path):
            header = QtWidgets.QListWidgetItem("--- User Library ---")
            header.setFlags(QtCore.Qt.NoItemFlags)
            header.setForeground(QtGui.QColor("#ffaa00")); header.setBackground(QtGui.QColor("#222")); header.setTextAlignment(QtCore.Qt.AlignCenter)
            self.node_palette.addItem(header)
            for f in os.listdir(self.user_lib_path):
                if f.endswith(".json"):
                    item = QtWidgets.QListWidgetItem(f.replace(".json", ""))
                    item.setData(QtCore.Qt.UserRole, f"USER_LIB:{f}")
                    item.setForeground(QtGui.QColor("#ffaa00"))
                    self.node_palette.addItem(item)

    def create_palette(self, callback):
        self.node_palette.itemDoubleClicked.connect(callback)