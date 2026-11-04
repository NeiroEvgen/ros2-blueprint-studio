import os
from core.i18n import t
from PySide6 import QtWidgets, QtCore, QtGui
from ui.fonts import MONO_CSS
from ui import icons

# === 0. ADAPTIVE TOP BAR ===
class AdaptiveBar(QtWidgets.QFrame):
    """
    Top bar. Secondary buttons (palettes, group, clear, export, import) collapse
    into a "⋯" menu when there is not enough width. Without this, the sum of all
    button widths (especially with long Russian labels) set the window's minimum width,
    and the window could not be made narrower.
    """
    def __init__(self):
        super().__init__()
        self.secondary = None      # container with the secondary buttons
        self.more = None           # "⋯" button with the menu

    def minimumSizeHint(self):
        """The minimum does not include the secondary section: "⋯" replaces it when space runs out."""
        lay = self.layout()
        size = lay.minimumSize() if lay else QtCore.QSize(0, 0)
        w = size.width()
        if (lay is not None and self.secondary is not None and self.more is not None
                and self.secondary.isVisibleTo(self)):
            w = w - self.secondary.minimumSizeHint().width() + self.more.sizeHint().width()
        return QtCore.QSize(w, size.height())

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.update_overflow()

    def update_overflow(self):
        if self.secondary is None or self.more is None:
            return
        lay = self.layout()
        collapsed = self.more.isVisibleTo(self)
        base = lay.sizeHint().width()
        if collapsed:   # "⋯" is shown now, not the container: compute the width needed with the container
            need = base - self.more.sizeHint().width() + self.secondary.sizeHint().width()
        else:
            need = base
        if collapsed and self.width() >= need + 4:
            self.more.setVisible(False); self.secondary.setVisible(True)
        elif not collapsed and self.width() < need:
            self.secondary.setVisible(False); self.more.setVisible(True)


# === 1. DRAGGABLE PALETTE LIST ===
class DraggableListWidget(QtWidgets.QListWidget):
    def startDrag(self, supportedActions):
        item = self.currentItem()
        if not item: return
        
        # hidden node id (e.g. ros.py.PyStringPubNode)
        node_id = item.data(QtCore.Qt.UserRole)
        if not node_id: return

        # pack it into MIME data as plain text
        mime_data = QtCore.QMimeData()
        mime_data.setText(node_id)
        
        drag = QtGui.QDrag(self)
        drag.setMimeData(mime_data)
        
        # drag pixmap
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
    
# === 2. UI MANAGER ===
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
        graph = self.main_window.graphs.current(self.tabs.currentIndex())
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

        # === TOP BAR ===
        top_bar = self.top_bar = AdaptiveBar()
        top_bar.setStyleSheet("background-color: #333; border-bottom: 2px solid #222;")
        top_bar.setFixedHeight(50)
        top_layout = QtWidgets.QHBoxLayout(top_bar)
        top_layout.setContentsMargins(10, 5, 10, 5)
        top_layout.setSpacing(8)
        top_layout.setSizeConstraint(QtWidgets.QLayout.SetNoConstraint)   # AdaptiveBar sets the minimum

        def add_btn(text, color="#555", icon_name=None, compact=False, into=None):
            btn = QtWidgets.QPushButton(text)
            if icon_name:
                btn.setIcon(icons.icon(icon_name))
                btn.setIconSize(QtCore.QSize(18, 18))
            # compact: icon only (label in the tooltip) so the bar does not widen the window
            pad = "5px 8px" if compact else "5px 12px"
            btn.setStyleSheet(f"""
                QPushButton {{ 
                    background-color: {color}; color: white; border: none; 
                    padding: {pad}; font-weight: bold; border-radius: 3px;
                }}
                QPushButton:hover {{ background-color: #666; }}
                QPushButton:pressed {{ background-color: #444; }}
                QPushButton:disabled {{ background-color: #222; color: #555; }}
            """)
            (into or top_layout).addWidget(btn)
            return btn

        self.actions['open'] = add_btn("", icon_name="open", compact=True)
        self.actions['save'] = add_btn("", icon_name="save", compact=True)
        
        line = QtWidgets.QFrame(); line.setFrameShape(QtWidgets.QFrame.VLine); line.setStyleSheet("color: #444;")
        top_layout.addWidget(line)
        
        self.actions['run'] = add_btn("", "#2e7d32", icon_name="run")
        self.actions['stop'] = add_btn("", "#c62828", icon_name="stop")
        self.actions['stop'].setEnabled(False)
        
        self.actions['viz_mode'] = QtWidgets.QComboBox()
        self.actions['viz_mode'].addItems(["Foxglove", "X11 (XLaunch)"])
        self.actions['viz_mode'].setStyleSheet(
            "QComboBox { background-color: #555; color: white; padding: 5px; border-radius: 3px; }")
        top_layout.addWidget(self.actions['viz_mode'])
        self.actions['visualize'] = add_btn("", "#1565c0", icon_name="visualize")

        from ui.context_badge import ContextBadge
        self.context_badge = ContextBadge()
        top_layout.addWidget(self.context_badge)
        self.actions['real_robot'] = QtWidgets.QPushButton()
        self.actions['real_robot'].setIcon(icons.icon("robot"))
        self.actions['real_robot'].setIconSize(QtCore.QSize(18, 18))
        self.actions['real_robot'].setCheckable(True)
        self.actions['real_robot'].setStyleSheet(
            "QPushButton { background-color: #555; color: white; border: none;"
            " padding: 5px 12px; border-radius: 3px; }"
            "QPushButton:checked { background-color: #b71c1c; font-weight: bold; }")
        top_layout.addWidget(self.actions['real_robot'])

        top_layout.addStretch()
        secondary = QtWidgets.QWidget()
        sec_layout = QtWidgets.QHBoxLayout(secondary)
        sec_layout.setContentsMargins(0, 0, 0, 0)
        sec_layout.setSpacing(8)
        self.actions['export_palette'] = add_btn("", icon_name="upload", compact=True, into=sec_layout)
        self.actions['import_palette'] = add_btn("", icon_name="download", compact=True, into=sec_layout)
        self.actions['group'] = add_btn("", icon_name="group", compact=True, into=sec_layout)
        self.actions['group'].clicked.connect(self.on_group_clicked)
        self.actions['clear'] = add_btn("", icon_name="clear", compact=True, into=sec_layout)
        self.actions['export_docker'] = add_btn("", icon_name="package", compact=True, into=sec_layout)
        self.actions['import_pkg'] = add_btn("", icon_name="package_in", compact=True, into=sec_layout)
        top_layout.addWidget(secondary)

        # when space runs out, a "⋯" menu replaces the button section
        self.more_btn = QtWidgets.QToolButton()
        self.more_btn.setIcon(icons.icon("more"))
        self.more_btn.setIconSize(QtCore.QSize(18, 18))
        self.more_btn.setPopupMode(QtWidgets.QToolButton.InstantPopup)
        self.more_btn.setStyleSheet(
            "QToolButton { background-color: #555; border: none; padding: 5px 8px; border-radius: 3px; }"
            "QToolButton:hover { background-color: #666; }"
            "QToolButton::menu-indicator { image: none; }")
        self.more_menu = QtWidgets.QMenu(self.more_btn)
        self.more_btn.setMenu(self.more_menu)
        self.more_btn.setVisible(False)
        top_layout.addWidget(self.more_btn)
        top_bar.secondary = secondary
        top_bar.more = self.more_btn

        main_layout.addWidget(top_bar)

        # === WORKSPACE ===
        main_splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        main_layout.addWidget(main_splitter)

        # PALETTE
        palette_panel = QtWidgets.QWidget()
        palette_layout = QtWidgets.QVBoxLayout(palette_panel)
        palette_layout.setContentsMargins(0,0,0,0)
        lbl = self.palette_label = QtWidgets.QLabel()
        lbl.setStyleSheet("background: #222; color: #ddd; padding: 5px; font-weight: bold; border-bottom: 1px solid #444;")
        palette_layout.addWidget(lbl)
        
        # search, language filter, favorites only
        filters = QtWidgets.QHBoxLayout()
        filters.setContentsMargins(4, 4, 4, 2)
        self.palette_search = QtWidgets.QLineEdit()
        self.palette_search.setClearButtonEnabled(True)
        self.palette_search.textChanged.connect(lambda _t: self._render_palette())
        filters.addWidget(self.palette_search, 1)
        self.palette_lang = QtWidgets.QComboBox()
        self.palette_lang.addItems(["", "Python", "C++"])      # texts set in retranslate()
        self.palette_lang.currentIndexChanged.connect(lambda _i: self._render_palette())
        filters.addWidget(self.palette_lang)
        self.palette_fav_only = QtWidgets.QToolButton()
        self.palette_fav_only.setText("★")
        self.palette_fav_only.setCheckable(True)
        self.palette_fav_only.toggled.connect(lambda _c: self._render_palette())
        filters.addWidget(self.palette_fav_only)
        palette_layout.addLayout(filters)

        # draggable list
        self.node_palette = DraggableListWidget()
        self.node_palette.setContextMenuPolicy(QtCore.Qt.CustomContextMenu)
        self.node_palette.customContextMenuRequested.connect(self._palette_menu)
        self.node_palette.setDragEnabled(True)
        self.node_palette.setStyleSheet("border: none; background: #2b2b2b;")
        palette_layout.addWidget(self.node_palette)
        self.node_palette.setAcceptDrops(True)
        main_splitter.addWidget(palette_panel)

        # === GRAPHS ===
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
        self.btn_back.setIcon(icons.icon("back"))
        self.btn_back.setIconSize(QtCore.QSize(14, 14))
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

        # CONSOLES
        console_dock = self.console_dock = QtWidgets.QDockWidget(self.main_window)
        console_widget = QtWidgets.QWidget()
        console_layout = QtWidgets.QVBoxLayout(console_widget)
        console_layout.setContentsMargins(0,0,0,0)
        
        console_tabs = self.console_tabs = QtWidgets.QTabWidget()
        self.console_sys = QtWidgets.QTextEdit()
        self.console_sys.setReadOnly(True)
        self.console_sys.setStyleSheet("background-color: #1e1e1e; color: #00ff00; font-family: " + MONO_CSS + "; border: none;")
        console_tabs.addTab(self.console_sys, "")
        
        self.console_ros = QtWidgets.QTextEdit()
        self.console_ros.setReadOnly(True)
        self.console_ros.setStyleSheet("background-color: #1e1e1e; color: #00ffff; font-family: " + MONO_CSS + "; border: none;")
        console_tabs.addTab(self.console_ros, "")
        
        console_layout.addWidget(console_tabs)
        console_dock.setWidget(console_widget)
        self.main_window.addDockWidget(QtCore.Qt.BottomDockWidgetArea, console_dock)

        main_splitter.setStretchFactor(0, 1)
        main_splitter.setStretchFactor(1, 5)
        self.retranslate()

    def retranslate(self):
        """All bar texts from the dictionary; called on build and on language change."""
        a = self.actions
        self.main_window.setWindowTitle(t("ui.window_title"))
        for key, text_key in (("run", "ui.run"), ("stop", "ui.stop"), ("visualize", "ui.visualize")):
            a[key].setText(t(text_key))
        for key, text_key in (("open", "ui.open"), ("save", "ui.save"),
                              ("export_palette", "ui.palette_export"),
                              ("import_palette", "ui.palette_import"),
                              ("group", "ui.group"), ("clear", "ui.clear"),
                              ("export_docker", "ui.export"), ("import_pkg", "ui.import_pkg")):
            a[key].setText("")                       # icon only
            a[key].setToolTip(t(text_key))
            a[key].setAccessibleName(t(text_key))
        self.more_btn.setToolTip(t("ui.more"))
        self.more_menu.clear()
        for key, icon_name, text_key in (("export_palette", "upload", "ui.palette_export"),
                                         ("import_palette", "download", "ui.palette_import"),
                                         ("group", "group", "ui.group"), ("clear", "clear", "ui.clear"),
                                         ("export_docker", "package", "ui.export"),
                                         ("import_pkg", "package_in", "ui.import_pkg")):
            act = self.more_menu.addAction(icons.icon(icon_name, color="#dddddd"), t(text_key))
            act.triggered.connect(lambda _checked=False, k=key: a[k].click())
        a['viz_mode'].setToolTip(t("ui.viz_tooltip"))
        a['real_robot'].setText(t("ui.real_robot"))
        a['real_robot'].setToolTip(t("ui.real_robot_tip"))
        self.palette_label.setText(t("ui.node_palette"))
        if getattr(self, "palette_search", None) is not None:
            self.palette_search.setPlaceholderText(t("palette.search"))
            self.palette_lang.setItemText(0, t("palette.lang_all"))
            self.palette_lang.setToolTip(t("palette.lang_tip"))
            self.palette_fav_only.setToolTip(t("palette.favorites_only"))
            self._render_palette()
        self.btn_back.setText(t("ui.back"))
        if not self.breadcrumb_label.text():
            self.breadcrumb_label.setText(t("ui.root"))
        self.console_dock.setWindowTitle(t("ui.logs_terminal"))
        for widget, key in ((self.console_sys, "ui.log_system"), (self.console_ros, "ui.log_ros")):
            idx = self.console_tabs.indexOf(widget)     # tabs may be reordered/replaced
            if idx >= 0:
                self.console_tabs.setTabText(idx, t(key))

    def set_visible_graph(self, mode="python"):
        if self.tabs.count() < 2:      # single-canvas mode: one tab
            self.tabs.setCurrentIndex(0)
            return
        if mode == "mixed":
            self.tabs.setTabVisible(0, True); self.tabs.setTabVisible(1, True); self.tabs.setCurrentIndex(0)
        elif mode == "python":
            self.tabs.setTabVisible(0, True); self.tabs.setTabVisible(1, False); self.tabs.setCurrentIndex(0)
        else:
            self.tabs.setTabVisible(0, False); self.tabs.setTabVisible(1, True); self.tabs.setCurrentIndex(1)

    # ---------- node palette ----------

    _HEADER_KEYS = {"favorites": "palette.cat_favorites", "publishers": "palette.cat_publishers",
                    "subscribers": "palette.cat_subscribers", "services": "palette.cat_services",
                    "logic": "palette.cat_logic", "tools": "palette.cat_tools"}

    def rebuild_palette(self, node_classes, project_type="python"):
        """Collects the entries (built-in nodes + user palettes) and shows them with the current filters."""
        from core import palette_share
        from ui import palette_model
        self._palette_project_type = project_type
        self._palette_entries = (palette_model.builtin_entries(node_classes, project_type)
                                 + palette_model.user_entries(palette_share.PALETTES_ROOT, project_type))
        mixed = project_type == "mixed"
        self.palette_lang.setVisible(mixed)          # one-language projects show only their language
        if not mixed:
            self.palette_lang.setCurrentIndex(0)
        self._render_palette()

    def _palette_favorites(self):
        from core import settings
        return set(settings.get("palette_favorites") or [])

    def _render_palette(self):
        from ui import palette_model
        entries = getattr(self, "_palette_entries", None)
        if entries is None or self.node_palette is None:
            return
        language = ("all", "python", "cpp")[max(0, self.palette_lang.currentIndex())]
        favorites = self._palette_favorites()
        groups = palette_model.arrange(entries, self.palette_search.text(), language, favorites,
                                       self.palette_fav_only.isChecked())
        show_language = self._palette_project_type == "mixed"
        self.node_palette.clear()
        for key, items in groups:
            user = key.startswith("palette:")
            title = t("palette.cat_user", name=key[len("palette:"):]) if user else t(self._HEADER_KEYS[key])
            header = QtWidgets.QListWidgetItem(f"--- {title} ---")
            header.setFlags(QtCore.Qt.NoItemFlags)
            header.setForeground(QtGui.QColor("#00ffaa" if user else "#aaa"))
            header.setBackground(QtGui.QColor("#1a1a1a" if user else "#222"))
            header.setTextAlignment(QtCore.Qt.AlignCenter)
            self.node_palette.addItem(header)
            for e in items:
                text = e.label(show_language)
                item = QtWidgets.QListWidgetItem(("★ " if e.id in favorites else "") + text)
                item.setData(QtCore.Qt.UserRole, e.id)
                item.setToolTip(e.doc or e.id)
                if user:
                    item.setForeground(QtGui.QColor("#00ffaa"))
                self.node_palette.addItem(item)
        if not groups:
            empty = QtWidgets.QListWidgetItem(t("palette.nothing"))
            empty.setFlags(QtCore.Qt.NoItemFlags)
            empty.setForeground(QtGui.QColor("#888"))
            self.node_palette.addItem(empty)

    def toggle_palette_favorite(self, node_id):
        from core import settings
        favs = list(settings.get("palette_favorites") or [])
        if node_id in favs:
            favs.remove(node_id)
        else:
            favs.append(node_id)
        settings.set("palette_favorites", favs)
        self._render_palette()

    def _palette_menu(self, pos):
        item = self.node_palette.itemAt(pos)
        node_id = item.data(QtCore.Qt.UserRole) if item else None
        if not node_id:
            return
        menu = QtWidgets.QMenu(self.node_palette)
        starred = node_id in self._palette_favorites()
        act = menu.addAction(t("palette.unstar") if starred else t("palette.star"))
        act.triggered.connect(lambda: self.toggle_palette_favorite(node_id))
        menu.exec(self.node_palette.mapToGlobal(pos))

    def create_palette(self, callback):
        self.node_palette.itemDoubleClicked.connect(callback)