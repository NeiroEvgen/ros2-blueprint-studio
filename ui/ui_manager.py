import os
from core.i18n import t
from PySide6 import QtWidgets, QtCore, QtGui

# === 1. УМНЫЙ СПИСОК  ===
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

class PaletteDropFilter(QtCore.QObject):
    node_dropped = QtCore.Signal(str)

    def eventFilter(self, obj, event):
        if event.type() == QtCore.QEvent.DragEnter:
            if event.mimeData().hasFormat("application/x-bp-node-id"):
                event.acceptProposedAction()
                return True
        elif event.type() == QtCore.QEvent.Drop:
            md = event.mimeData()
            if md.hasFormat("application/x-bp-node-id"):
                node_id = bytes(md.data("application/x-bp-node-id")).decode()
                self.node_dropped.emit(node_id)
                event.acceptProposedAction()
                return True
        return super().eventFilter(obj, event)
    
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

    def on_group_clicked(self):
        graph = self.main_window.graph_py if self.tabs.currentIndex() == 0 else self.main_window.graph_cpp
        from nodes.group_logic import create_group_from_selection
        group_node, msg = create_group_from_selection(graph)
        if group_node:
            self.main_window.system_log(t("ui.group_created", name=group_node.name()))
        else:
            self.main_window.system_log(t("ui.group_error", msg=msg))

    def update_breadcrumbs(self, stack):
        path_str = " > ".join([item["name"] for item in stack])
        self.breadcrumb_label.setText(path_str)

    def setup_ui(self):
        self.main_window.setWindowTitle(t("ui.window_title"))
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

        self.actions['open'] = add_btn("")
        self.actions['save'] = add_btn("")
        
        line = QtWidgets.QFrame(); line.setFrameShape(QtWidgets.QFrame.VLine); line.setStyleSheet("color: #444;")
        top_layout.addWidget(line)
        
        self.actions['run'] = add_btn("", "#2e7d32")
        self.actions['stop'] = add_btn("", "#c62828")
        self.actions['stop'].setEnabled(False)
        
        self.actions['viz_mode'] = QtWidgets.QComboBox()
        self.actions['viz_mode'].addItems(["Foxglove", "X11 (XLaunch)"])
        self.actions['viz_mode'].setStyleSheet(
            "QComboBox { background-color: #555; color: white; padding: 5px; border-radius: 3px; }")
        top_layout.addWidget(self.actions['viz_mode'])
        self.actions['visualize'] = add_btn("", "#1565c0")

        from ui.context_badge import ContextBadge
        self.context_badge = ContextBadge()
        top_layout.addWidget(self.context_badge)
        self.actions['real_robot'] = QtWidgets.QPushButton()
        self.actions['real_robot'].setCheckable(True)
        self.actions['real_robot'].setStyleSheet(
            "QPushButton { background-color: #555; color: white; border: none;"
            " padding: 5px 12px; border-radius: 3px; }"
            "QPushButton:checked { background-color: #b71c1c; font-weight: bold; }")
        top_layout.addWidget(self.actions['real_robot'])

        top_layout.addStretch()
        self.actions['export_palette'] = add_btn("")
        self.actions['import_palette'] = add_btn("")
        self.actions['group'] = add_btn("")
        self.actions['group'].clicked.connect(self.on_group_clicked)
        self.actions['clear'] = add_btn("")
        self.actions['export_docker'] = add_btn("")
        self.actions['import_pkg'] = add_btn("")

        main_layout.addWidget(top_bar)

        # === РАБОЧАЯ ОБЛАСТЬ ===
        main_splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        main_layout.addWidget(main_splitter)

        # ПАЛИТРА
        palette_panel = QtWidgets.QWidget()
        palette_layout = QtWidgets.QVBoxLayout(palette_panel)
        palette_layout.setContentsMargins(0,0,0,0)
        lbl = self.palette_label = QtWidgets.QLabel()
        lbl.setStyleSheet("background: #222; color: #ddd; padding: 5px; font-weight: bold; border-bottom: 1px solid #444;")
        palette_layout.addWidget(lbl)
        
        # ИСПОЛЬЗУЕМ НАШ НОВЫЙ КЛАСС DRAGGABLE
        self.node_palette = DraggableListWidget()
        self.node_palette.setDragEnabled(True)
        self.node_palette.setStyleSheet("border: none; background: #2b2b2b;")
        palette_layout.addWidget(self.node_palette)
        self.node_palette.setAcceptDrops(True)
        main_splitter.addWidget(palette_panel)

        # === ГРАФЫ ===
        graph_container = QtWidgets.QWidget()
        graph_vbox = QtWidgets.QVBoxLayout(graph_container)
        graph_vbox.setContentsMargins(0, 0, 0, 0)
        graph_vbox.setSpacing(0)

        # Breadcrumbs bar
        self.breadcrumb_bar = QtWidgets.QFrame()
        self.breadcrumb_bar.setFixedHeight(30)
        self.breadcrumb_bar.setStyleSheet("background-color: #3c3c3c; border-bottom: 1px solid #222;")
        breadcrumb_layout = QtWidgets.QHBoxLayout(self.breadcrumb_bar)
        breadcrumb_layout.setContentsMargins(10, 0, 10, 0)
        self.breadcrumb_label = QtWidgets.QLabel()
        self.breadcrumb_label.setStyleSheet("color: #bbb; font-weight: bold;")
        breadcrumb_layout.addWidget(self.breadcrumb_label)
        
        self.btn_back = QtWidgets.QPushButton()
        self.btn_back.setStyleSheet("""
            QPushButton { background: #555; color: white; border-radius: 3px; padding: 2px 10px; }
            QPushButton:hover { background: #777; }
        """)
        self.btn_back.clicked.connect(self.main_window.exit_subgraph)
        breadcrumb_layout.addWidget(self.btn_back)
        
        breadcrumb_layout.addStretch()        
        graph_vbox.addWidget(self.breadcrumb_bar)

        self.tabs = QtWidgets.QTabWidget()
        self.tabs.setStyleSheet("QTabWidget::pane { border: 0; }")
        graph_vbox.addWidget(self.tabs)
        
        main_splitter.addWidget(graph_container)

        # КОНСОЛИ
        console_dock = self.console_dock = QtWidgets.QDockWidget(self.main_window)
        console_widget = QtWidgets.QWidget()
        console_layout = QtWidgets.QVBoxLayout(console_widget)
        console_layout.setContentsMargins(0,0,0,0)
        
        console_tabs = self.console_tabs = QtWidgets.QTabWidget()
        self.console_sys = QtWidgets.QTextEdit()
        self.console_sys.setReadOnly(True)
        self.console_sys.setStyleSheet("background-color: #1e1e1e; color: #00ff00; font-family: Consolas; border: none;")
        console_tabs.addTab(self.console_sys, "")
        
        self.console_ros = QtWidgets.QTextEdit()
        self.console_ros.setReadOnly(True)
        self.console_ros.setStyleSheet("background-color: #1e1e1e; color: #00ffff; font-family: Consolas; border: none;")
        console_tabs.addTab(self.console_ros, "")
        
        console_layout.addWidget(console_tabs)
        console_dock.setWidget(console_widget)
        self.main_window.addDockWidget(QtCore.Qt.BottomDockWidgetArea, console_dock)

        main_splitter.setStretchFactor(0, 1)
        main_splitter.setStretchFactor(1, 5)
        self.retranslate()

    def retranslate(self):
        """Все тексты панели из словаря; вызывается при построении и при смене языка."""
        a = self.actions
        self.main_window.setWindowTitle(t("ui.window_title"))
        for key, text in (("open", " " + t("ui.open")), ("save", " " + t("ui.save")),
                          ("run", t("ui.run")), ("stop", t("ui.stop")),
                          ("visualize", " " + t("ui.visualize")),
                          ("export_palette", " " + t("ui.palette_export")),
                          ("import_palette", " " + t("ui.palette_import")),
                          ("group", " " + t("ui.group")), ("clear", " " + t("ui.clear")),
                          ("export_docker", " " + t("ui.export")),
                          ("import_pkg", " " + t("ui.import_pkg"))):
            a[key].setText(text)
        a['viz_mode'].setToolTip(t("ui.viz_tooltip"))
        a['real_robot'].setText(t("ui.real_robot"))
        a['real_robot'].setToolTip(t("ui.real_robot_tip"))
        self.palette_label.setText(t("ui.node_palette"))
        self.btn_back.setText(t("ui.back"))
        if not self.breadcrumb_label.text():
            self.breadcrumb_label.setText(t("ui.root"))
        self.console_dock.setWindowTitle(t("ui.logs_terminal"))
        for widget, key in ((self.console_sys, "ui.log_system"), (self.console_ros, "ui.log_ros")):
            idx = self.console_tabs.indexOf(widget)     # вкладки могут быть переставлены/заменены
            if idx >= 0:
                self.console_tabs.setTabText(idx, t(key))

    def set_visible_graph(self, mode="python"):
        if mode == "mixed":
            self.tabs.setTabVisible(0, True); self.tabs.setTabVisible(1, True); self.tabs.setCurrentIndex(0)
        elif mode == "python":
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
            # mixed: показываем всё; в одноязычных проектах — только свой язык
            if project_type == "python" and "ros.cpp" in identifier: continue
            if project_type == "cpp" and (".py" in identifier): continue
            if "internal" in identifier: continue

            item = QtWidgets.QListWidgetItem(name)
            item.setData(QtCore.Qt.UserRole, identifier)
            
            if "PubNode" in name: categories["Output (Publishers)"].append(item)
            elif "SubNode" in name: categories["Input (Subscribers)"].append(item)
            elif "Monitor" in name or "Group" in name or "Note" in name: categories["Tools & Utility"].append(item)
            
            else: categories["Logic & Processing"].append(item)

        for cat_name, items in categories.items():
            if not items: continue
            header = QtWidgets.QListWidgetItem(f"--- {cat_name} ---")
            header.setFlags(QtCore.Qt.NoItemFlags)
            header.setForeground(QtGui.QColor("#aaa")); header.setBackground(QtGui.QColor("#222")); header.setTextAlignment(QtCore.Qt.AlignCenter)
            self.node_palette.addItem(header)
            for item in items: self.node_palette.addItem(item)

        # --- USER PALETTES (Native Format) ---
        user_palettes_root = os.path.join("nodes", "user_palettes")
        if os.path.exists(user_palettes_root):
            for palette_name in os.listdir(user_palettes_root):
                palette_path = os.path.join(user_palettes_root, palette_name)
                if not os.path.isdir(palette_path): continue
                
                header = QtWidgets.QListWidgetItem(f"--- Palette: {palette_name} ---")
                header.setFlags(QtCore.Qt.NoItemFlags)
                header.setForeground(QtGui.QColor("#00ffaa")); header.setBackground(QtGui.QColor("#1a1a1a")); header.setTextAlignment(QtCore.Qt.AlignCenter)
                self.node_palette.addItem(header)
                
                for node_folder in os.listdir(palette_path):
                    node_path = os.path.join(palette_path, node_folder)
                    if os.path.isdir(node_path) and os.path.exists(os.path.join(node_path, "node.yaml")):
                        item = QtWidgets.QListWidgetItem(node_folder)
                        # Кодируем путь к папке ноды
                        item.setData(QtCore.Qt.UserRole, f"USER_NODE:{palette_name}/{node_folder}")
                        item.setForeground(QtGui.QColor("#00ffaa"))
                        self.node_palette.addItem(item)

    def create_palette(self, callback):
        self.node_palette.itemDoubleClicked.connect(callback)