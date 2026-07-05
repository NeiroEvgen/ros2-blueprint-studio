from PySide6 import QtWidgets, QtCore, QtGui

class PropertiesWindow(QtWidgets.QWidget):
    def __init__(self, graph_py, graph_cpp, parent=None):
        super().__init__(parent)
        self.graph_py = graph_py
        self.graph_cpp = graph_cpp
        self.current_node = None
        
        self.setWindowTitle("Node Parameters")
        self.setWindowFlags(QtCore.Qt.Window | QtCore.Qt.WindowStaysOnTopHint)
        self.resize(350, 450)
        
        layout = QtWidgets.QVBoxLayout(self)
        
        # Вкладки: Параметры и Код
        self.tabs = QtWidgets.QTabWidget()
        layout.addWidget(self.tabs)
        
        # --- ВКЛАДКА ПАРАМЕТРОВ ---
        self.params_widget = QtWidgets.QWidget()
        params_vbox = QtWidgets.QVBoxLayout(self.params_widget)
        
        self.node_label = QtWidgets.QLabel("Select a node")
        self.node_label.setStyleSheet("font-weight: bold; color: #55aaff;")
        params_vbox.addWidget(self.node_label)
        
        self.scroll = QtWidgets.QScrollArea()
        self.scroll.setWidgetResizable(True)
        self.scroll_content = QtWidgets.QWidget()
        self.params_layout = QtWidgets.QFormLayout(self.scroll_content)
        self.scroll.setWidget(self.scroll_content)
        params_vbox.addWidget(self.scroll)
        
        self.tabs.addTab(self.params_widget, "Parameters")
        
        # --- ВКЛАДКА КОДА ---
        self.code_editor = QtWidgets.QTextEdit()
        self.code_editor.setStyleSheet("background-color: #1e1e1e; color: #d4d4d4; font-family: 'Consolas', monospace;")
        self.tabs.addTab(self.code_editor, "Source Code")
        
        # Кнопки
        btn_layout = QtWidgets.QHBoxLayout()
        self.add_param_btn = QtWidgets.QPushButton("Add Parameter")
        self.add_param_btn.clicked.connect(self.add_custom_parameter)
        
        self.save_btn = QtWidgets.QPushButton("Apply Changes")
        self.save_btn.setStyleSheet("background-color: #2d5a27;")
        self.save_btn.clicked.connect(self.save_parameters)
        
        btn_layout.addWidget(self.add_param_btn)
        btn_layout.addWidget(self.save_btn)
        layout.addLayout(btn_layout)
        
        # Подписка на события обоих графов
        self.graph_py.node_selected.connect(self.on_node_selected)
        self.graph_cpp.node_selected.connect(self.on_node_selected)

    def on_node_selected(self, node):
        self.current_node = node
        self.node_label.setText(f"Node: {node.name()}")
        self.refresh_params()

    def set_node(self, node):
        """Метод для явной установки ноды (например, при двойном клике)"""
        self.on_node_selected(node)
    def refresh_params(self):
        while self.params_layout.count():
            child = self.params_layout.takeAt(0)
            if child.widget(): child.widget().deleteLater()
            
        if not self.current_node: return
        props = self.current_node.model.custom_properties
        
        # Секция: Основные настройки
        self._add_section_label("--- General Settings ---")
        self._add_row("node_name", props.get("node_name", ""))
        
        # Секция: Порты и Связи
        self._add_section_label("--- Ports & Topics ---")
        if "topic_name" in props:
            self._add_row("topic_name", props.get("topic_name", ""))
        
        # Секция: Пользовательские переменные
        self._add_section_label("--- Custom Variables ---")
        for key, val in props.items():
            if key in ["node_name", "topic_name", "source_file", "code_content", "type", "group_uid", "saved_ports_config", "template_key"]: 
                continue
            self._add_row(key, val)

    def _add_section_label(self, text):
        label = QtWidgets.QLabel(text)
        label.setStyleSheet("color: #888; font-weight: bold; margin-top: 10px;")
        self.params_layout.addRow(label)

    def _add_row(self, key, val):
        line_edit = QtWidgets.QLineEdit(str(val))
        line_edit.setObjectName(key)
        self.params_layout.addRow(key, line_edit)

    def add_custom_parameter(self):
        if not self.current_node: return
        name, ok = QtWidgets.QInputDialog.getText(self, "New Parameter", "Parameter Name:")
        if ok and name:
            # Создаём свойство на ноде сразу, чтобы save_parameters не упал
            if not self.current_node.has_property(name):
                try:
                    self.current_node.create_property(name, "")
                except Exception:
                    pass
            self._add_row(name, "")

    def save_parameters(self):
        if not self.current_node: return

        # 1. Сохраняем параметры из формы
        for i in range(self.params_layout.count()):
            label_item = self.params_layout.itemAt(i, QtWidgets.QFormLayout.LabelRole)
            field_item = self.params_layout.itemAt(i, QtWidgets.QFormLayout.FieldRole)

            if label_item and field_item:
                key = label_item.widget().text()
                val = field_item.widget().text()
                self._safe_set(key, val)

        # 2. Сохраняем код из редактора
        new_code = self.code_editor.toPlainText()
        self._safe_set("code_content", new_code)

        parent = self.parent()
        if parent is not None and hasattr(parent, "statusBar"):
            parent.statusBar().showMessage("Parameters and Code saved", 2000)
        elif parent is not None and hasattr(parent, "system_log"):
            parent.system_log("Parameters and Code saved")

    def _safe_set(self, key, val):
        """set_property с гарантией, что свойство существует (NodeGraphQt требует)."""
        if not self.current_node.has_property(key):
            try:
                self.current_node.create_property(key, val)
                return
            except Exception:
                pass
        self.current_node.set_property(key, val)
