import sys
import logging
import traceback
import re
import json
import os
from functools import partial
from PySide6 import QtWidgets, QtCore, QtGui, QtSvg, QtXml
from NodeGraphQt import NodeGraph
#from launcher import LauncherDialog

try:
    from docker_manager import RosContainerManager
    from ui_manager import UiManager
    from project_manager import ProjectManager
    from graph_compiler import GraphCompiler
    from code_editor import AdvancedCodeDialog
    #from launcher_dialog import LauncherDialog 
    from export_manager import ExportManager   
    from workspace_manager import WorkspaceManager
    # 1. ОБЫЧНЫЕ НОДЫ И ПАЛИТРА
    from nodes.custom_nodes import (
        LevelBuilderNode, HiveMindNode, SmartAgentNode,
        PyStringPubNode, PyStringSubNode, PyTwistPubNode, PyTwistSubNode, PyCustomNode,
        CppStringPubNode, CppStringSubNode, CppTwistPubNode, CppTwistSubNode, CppCustomNode,
        MSG_COLORS
    )
    
    # 2. ГРУППА И МОНИТОР
    from nodes.group_logic import RosGroupNode, create_group_from_selection, toggle_group_visibility, fix_group_after_load
    from nodes.monitor_node import MonitorNode


except ImportError as e:
    print(f"IMPORT ERROR: {e}")
    sys.exit(1)

logging.basicConfig(level=logging.INFO)

# --- ПАЛИТРА ЦВЕТОВ (Для монитора) ---
TYPE_COLORS = {
    "std_msgs/String": (255, 235, 59),
    "geometry_msgs/Twist": (255, 152, 0),
    "DEFAULT": (100, 100, 100)
}

class LauncherDialog(QtWidgets.QDialog):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ROS2 Studio Launcher")
        self.resize(600, 400)
        self.selected_project_path = None
        
        # UI Setup
        layout = QtWidgets.QHBoxLayout(self)
        
        # Left Side: Project List
        left_layout = QtWidgets.QVBoxLayout()
        left_layout.addWidget(QtWidgets.QLabel("Recent Projects:"))
        
        self.project_list = QtWidgets.QListWidget()
        self.project_list.itemDoubleClicked.connect(self.accept_selection)
        left_layout.addWidget(self.project_list)
        
        layout.addLayout(left_layout, stretch=2)
        
        # Right Side: Actions
        right_layout = QtWidgets.QVBoxLayout()
        
        lbl_new = QtWidgets.QLabel("Create New:")
        right_layout.addWidget(lbl_new)
        
        self.name_input = QtWidgets.QLineEdit()
        self.name_input.setPlaceholderText("Project Name...")
        right_layout.addWidget(self.name_input)
        
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
            QListWidget { background-color: #444; border: 1px solid #555; color: white; font-size: 14px; }
            QLineEdit { padding: 5px; background: #555; color: white; border: 1px solid #666; }
            QPushButton { padding: 5px; }
        """)

    def refresh_list(self):
        self.project_list.clear()
        projects = WorkspaceManager.list_projects()
        for p in projects:
            item = QtWidgets.QListWidgetItem(p)
            item.setData(QtCore.Qt.UserRole, os.path.join(WorkspaceManager.get_workspace_root(), p))
            self.project_list.addItem(item)

    def create_project(self):
        name = self.name_input.text()
        if not name: return
        try:
            path = WorkspaceManager.create_project(name)
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


# --- WORKERS ---
class LogMonitorWorker(QtCore.QThread):
    new_log_line = QtCore.Signal(str)
    def __init__(self, manager):
        super().__init__(); self.manager = manager; self.is_running = True
    def run(self):
        if not self.manager or not self.manager.container: return
        try:
            buffer = ""
            for chunk in self.manager.container.logs(stream=True, follow=True):
                if not self.is_running: break
                text_chunk = chunk.decode('utf-8', errors='replace')
                buffer += text_chunk
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    self.new_log_line.emit(line)
            if buffer.strip(): self.new_log_line.emit(buffer)
        except Exception as e: pass
    def stop(self): self.is_running = False

class DeployWorker(QtCore.QThread):
    log_signal = QtCore.Signal(str); finished_signal = QtCore.Signal()
    def __init__(self, scripts, manager):
        super().__init__(); self.scripts = scripts; self.manager = manager
    def run(self):
        try:
            self.log_signal.emit("--- DEPLOY ---")
            self.manager.ensure_image(lambda m: self.log_signal.emit(m))
            self.manager.start_session(lambda m: self.log_signal.emit(m))
            py_s = [s for s in self.scripts if s['language'] == 'python']
            cpp_s = [s for s in self.scripts if s['language'] == 'cpp']
            for s in py_s: self.manager.inject_and_run_script(s['filename'], s['code'], lambda m: self.log_signal.emit(m))
            if cpp_s: self.manager.build_and_run_cpp_nodes(cpp_s, lambda m: self.log_signal.emit(m))
            self.finished_signal.emit()
        except Exception as e:
            traceback.print_exc()
            self.log_signal.emit(f"ERR: {e}")

# --- EDITOR CLASSES ---
class CodeEditorDialog(QtWidgets.QDialog):
    def __init__(self, code, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Редактор кода"); self.resize(900, 700)
        layout = QtWidgets.QVBoxLayout(self)
        self.editor = QtWidgets.QPlainTextEdit()
        self.editor.setPlainText(code)
        self.editor.setStyleSheet("background-color: #1e1e1e; color: #d4d4d4; font-family: Consolas; font-size: 11pt;")
        layout.addWidget(self.editor)
        btn_box = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel)
        btn_box.accepted.connect(self.accept); btn_box.rejected.connect(self.reject)
        layout.addWidget(btn_box)
    def get_code(self): return self.editor.toPlainText()

class GroupSettingsDialog(QtWidgets.QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Group Settings"); self.resize(400, 300)
        layout = QtWidgets.QVBoxLayout(self)
        layout.addWidget(QtWidgets.QLabel("Group Name:"))
        self.name_edit = QtWidgets.QLineEdit("MySubGraph")
        layout.addWidget(self.name_edit)
        layout.addWidget(QtWidgets.QLabel("Color:"))
        self.color_btn = QtWidgets.QPushButton("Select Color")
        self.color_btn.clicked.connect(self.choose_color)
        self.color_btn.setStyleSheet("background-color: #555; color: white; padding: 10px;")
        layout.addWidget(self.color_btn)
        self.selected_color = (50, 50, 50) 
        layout.addWidget(QtWidgets.QLabel(""))
        self.save_chk = QtWidgets.QCheckBox("Save to Library")
        layout.addWidget(self.save_chk)
        btn_box = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel)
        btn_box.accepted.connect(self.accept); btn_box.rejected.connect(self.reject)
        layout.addWidget(btn_box)
    def choose_color(self):
        c = QtWidgets.QColorDialog.getColor()
        if c.isValid():
            self.selected_color = (c.red(), c.green(), c.blue())
            self.color_btn.setStyleSheet(f"background-color: rgb({c.red()},{c.green()},{c.blue()}); color: white; padding: 10px;")
    def get_data(self): return {"name": self.name_edit.text(), "color": self.selected_color, "save": self.save_chk.isChecked()}

class GraphDropFilter(QtCore.QObject):
    def __init__(self, graph, node_map_callback):
        super().__init__(); self.graph = graph; self.get_node_type = node_map_callback
    def eventFilter(self, watched, event):
        if event.type() in [QtCore.QEvent.DragEnter, QtCore.QEvent.DragMove]:
            if event.mimeData().hasText(): event.acceptProposedAction(); return True
        elif event.type() == QtCore.QEvent.Drop:
            code = event.mimeData().text()
            node_type = self.get_node_type(code)
            if node_type:
                pos = event.position().toPoint()
                scene_pos = self.graph.viewer().mapToScene(pos)
                try: self.graph.create_node(node_type, pos=[scene_pos.x(), scene_pos.y()])
                except Exception as e: print(f"Drop Error: {e}")
                event.acceptProposedAction(); return True
        return super().eventFilter(watched, event)

# --- MAIN APP ---
class RosOrchestratorApp(QtWidgets.QMainWindow):
    def __init__(self, project_path=None):
        super().__init__()
        self.current_project_path = project_path
        self.ui = UiManager(self)
        self.ui.setup_ui()
        
        self.graph_py = NodeGraph()
        self.graph_cpp = NodeGraph()
        
        if 'export_docker' in self.ui.actions:
            self.ui.actions['export_docker'].triggered.connect(self.on_export_docker)

        # === ДОБАВЬТЕ ЭТИ СТРОКИ СЮДА ===
        # Разрешаем циклы (соединение выхода обратно на вход)
        self.graph_py.set_acyclic(False)
        self.graph_cpp.set_acyclic(False)
        # =================================

        # 1. СПИСОК ВСЕХ НОД
        node_classes = [
            PyStringPubNode, PyStringSubNode, PyTwistPubNode, PyTwistSubNode, PyCustomNode,
            CppStringPubNode, CppStringSubNode, CppTwistPubNode, CppTwistSubNode, CppCustomNode,
            MonitorNode, 
            RosGroupNode,
            LevelBuilderNode, HiveMindNode, SmartAgentNode 
        ]
        
        # 2. РЕГИСТРАЦИЯ
        self.graph_py.register_nodes(node_classes)
        self.graph_cpp.register_nodes(node_classes)
        
        # (Удалили set_context_menu_from_file, чтобы не было ошибки NoneType)
        
        print("Final Registered Nodes:", self.graph_py.registered_nodes())
        
        self.graph_py.node_double_clicked.connect(self.on_node_double_click)
        self.graph_cpp.node_double_clicked.connect(self.on_node_double_click)
        
        self.ui.tabs.addTab(self.graph_py.widget, "Python Blueprints")
        self.ui.tabs.addTab(self.graph_cpp.widget, "C++ Blueprints")

        # DnD
        self.graph_py.widget.setAcceptDrops(True); self.graph_py.viewer().setAcceptDrops(True); self.graph_py.viewer().viewport().setAcceptDrops(True)
        self.graph_cpp.widget.setAcceptDrops(True); self.graph_cpp.viewer().setAcceptDrops(True); self.graph_cpp.viewer().viewport().setAcceptDrops(True)

        def resolve_node_type(code):
            if code.startswith("USER_LIB:"):
                self.add_from_palette_logic(code, use_mouse_pos=True); return None
            return self._get_node_identifier(code)

        self.filter_py = GraphDropFilter(self.graph_py, resolve_node_type)
        self.graph_py.viewer().viewport().installEventFilter(self.filter_py)
        self.filter_cpp = GraphDropFilter(self.graph_cpp, resolve_node_type)
        self.graph_cpp.viewer().viewport().installEventFilter(self.filter_cpp)

        self.project_manager = ProjectManager(self.graph_py, self.graph_cpp)
        
        if self.current_project_path:
            import os
            auto_load_file = os.path.join(self.current_project_path, "project.json")
            if os.path.exists(auto_load_file):
                # Небольшая задержка, чтобы GUI успел прорисоваться
                QtCore.QTimer.singleShot(100, lambda: self.project_manager.load_project(auto_load_file))
                print(f"Auto-loading project from: {self.current_project_path}")
        
        self.container_manager = None
        try: 
            self.container_manager = RosContainerManager()
            print("SUCCESS: Docker Connected!")
        except Exception as e:
            print(f"WARNING: Docker connection failed! {e}")

        self.ui.create_palette(self.add_from_palette)
        self.ui.actions['save'].triggered.connect(self.on_save_project)
        self.ui.actions['open'].triggered.connect(self.on_load_project)
        self.ui.actions['run'].triggered.connect(self.on_deploy_run)
        self.ui.actions['stop'].triggered.connect(self.on_stop)
        self.ui.actions['clear'].triggered.connect(self.on_clear)
        self.ui.actions['group'].triggered.connect(self.on_group_nodes)

        self.log_worker = None
        self.setup_connections()
        
        # --- ИНИЦИАЛИЗАЦИЯ МЕНЮ ДЛЯ КАСТОМНЫХ НОД ---
        self.setup_custom_context_menu(self.graph_py)
        self.setup_custom_context_menu(self.graph_cpp)

    # --- ИСПРАВЛЕННАЯ ЛОГИКА МЕНЮ ---
    def setup_custom_context_menu(self, graph):
        """Добавляем в контекстное меню графа пункт 'Add Output Port...'"""
        # !!! ФИКС: В твоей версии это функция, надо вызывать через () !!!
        try:
            root_menu = graph.context_menu() 
        except TypeError:
            root_menu = graph.context_menu

        root_menu.add_separator()
        
        # === 1. СУЩЕСТВУЮЩЕЕ МЕНЮ (OUTPUTS) ===
        out_menu = root_menu.add_menu("Add Custom Output (Publisher)")
        for name, color in MSG_COLORS.items():
            short_name = name.split('/')[-1]
            out_menu.add_command(
                f"Add {short_name}", 
                partial(self.on_add_port_clicked, graph, name, color)
            )

        # === 2. НОВОЕ МЕНЮ (INPUTS) ===
        in_menu = root_menu.add_menu("Add Custom Input (Subscriber)")
        for name, color in MSG_COLORS.items():
            short_name = name.split('/')[-1]
            # Вызываем новую функцию on_add_input_clicked
            in_menu.add_command(
                f"Add {short_name}", 
                partial(self.on_add_input_clicked, graph, name, color)
            )

    def on_add_port_clicked(self, graph, port_type, port_color, *args):
        """Добавляет порт И КОД к выделенной ноде"""
        selected_nodes = graph.selected_nodes()
        if not selected_nodes:
            self.system_log("⚠️ Выделите Custom ноду (ЛКМ), затем добавляйте порт.")
            return

        for node in selected_nodes:
            try:
                # 1. Генерация имени порта
                short_type = port_type.split('/')[-1].lower() # twist
                base_name = f"out_{short_type}"
                count = 1
                
                existing_ports = node.outputs().keys()
                # Ищем свободное имя (out_twist_1, out_twist_2)
                name = base_name
                while f"{base_name}_{count}" in existing_ports:
                    count += 1
                name = f"{base_name}_{count}"
                
                # 2. Визуальное добавление
                node.add_output(name, multi_output=True, display_name=True, color=port_color)
                
                # 3. АВТО-ДОБАВЛЕНИЕ КОДА
                self._inject_code_for_port(node, name, port_type)
                
                self.system_log(f"✅ Port & Code added: '{name}' to '{node.name()}'")
                
            except Exception as e:
                self.system_log(f"❌ Error adding port: {e}")
                traceback.print_exc()

    def _inject_code_for_port(self, node, port_name, port_type_ros):
        """Вставляет код инициализации паблишера прямо в редактор ноды"""
        if not node.has_property('code_content'): return
        
        current_code = node.get_property('code_content')
        is_cpp = 'cpp' in node.type_
        
        # Определяем типы данных для кода
        # port_type_ros пример: "geometry_msgs/Twist"
        pkg, cls = port_type_ros.split('/') 
        
        new_code = current_code
        
        if not is_cpp:
            # --- PYTHON GENERATION ---
            # Ищем маркер: # [AUTO-GEN-PUBS]
            marker = "# [AUTO-GEN-PUBS]"
            
            # Строка кода: self.pub_out_twist_1 = self.create_publisher(Twist, 'out_twist_1_topic', 10)
            # В Python нам нужен только класс (Twist), предполагаем что импорты уже есть в шаблоне Custom
            code_line = f"        self.pub_{port_name} = self.create_publisher({cls}, '{{topic_name}}_{port_name}', 10)"
            
            if marker in current_code:
                # Вставляем строку ПОСЛЕ маркера
                new_code = current_code.replace(marker, f"{marker}\n{code_line}")
                
        else:
            # --- C++ GENERATION ---
            # Нам нужно вставить в ДВА места: в конструктор и в private переменные
            
            # 1. Переменная (в private)
            # std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Twist>> pub_out_twist_1_;
            cpp_type = f"{pkg}::msg::{cls}"
            var_name = f"pub_{port_name}_"
            
            marker_vars = "// [AUTO-GEN-VARS]"
            var_line = f"  std::shared_ptr<rclcpp::Publisher<{cpp_type}>> {var_name};"
            
            if marker_vars in new_code:
                new_code = new_code.replace(marker_vars, f"{marker_vars}\n{var_line}")
            
            # 2. Инициализация (в public конструктор)
            # pub_out_twist_1_ = this->create_publisher<geometry_msgs::msg::Twist>("{topic_name}_out_twist_1", 10);
            marker_pubs = "// [AUTO-GEN-PUBS]"
            init_line = f"    {var_name} = this->create_publisher<{cpp_type}>(\"{{topic_name}}_{port_name}\", 10);"
            
            if marker_pubs in new_code:
                new_code = new_code.replace(marker_pubs, f"{marker_pubs}\n{init_line}")

        # Сохраняем обновленный код обратно в ноду
        if new_code != current_code:
            node.set_property('code_content', new_code)

        # === НОВЫЙ МЕТОД: Обработка клика по меню добавления входа ===
    def on_add_input_clicked(self, graph, port_type, port_color, *args):
        selected_nodes = graph.selected_nodes()
        if not selected_nodes:
            self.system_log("⚠️ Выделите Custom ноду, затем добавляйте вход.")
            return

        for node in selected_nodes:
            try:
                # 1. Генерация имени порта (in_twist_1, in_string_2 и т.д.)
                short_type = port_type.split('/')[-1].lower()
                base_name = f"in_{short_type}"
                count = 1
                existing_ports = node.inputs().keys()
                
                name = base_name
                while f"{base_name}_{count}" in existing_ports:
                    count += 1
                name = f"{base_name}_{count}"
                
                # 2. Визуальное добавление порта
                # multi_input=True важно для смешивания данных, если нужно
                node.add_input(name, multi_input=True, display_name=True, color=port_color)
                
                # 3. Инъекция кода ПОДПИСЧИКА
                self._inject_code_for_input(node, name, port_type)
                
                self.system_log(f"✅ Input & Code added: '{name}' to '{node.name()}'")
                
            except Exception as e:
                self.system_log(f"❌ Error adding input: {e}")
                traceback.print_exc()

    # === НОВЫЙ МЕТОД: Генерация кода для подписчика ===
    def _inject_code_for_input(self, node, port_name, port_type_ros):
        if not node.has_property('code_content'): return
        
        current_code = node.get_property('code_content')
        is_cpp = 'cpp' in node.type_
        pkg, cls = port_type_ros.split('/') # например: geometry_msgs, Twist
        
        new_code = current_code
        
        if not is_cpp:
            # --- PYTHON (Генерируем callback и подписку) ---
            marker = "# [AUTO-GEN-SUBS]"
            
            # 1. Создаем функцию Callback
            callback_name = f"callback_{port_name}"
            callback_code = (
                f"\n    def {callback_name}(self, msg):\n"
                f"        self.get_logger().info(f'Got on {port_name}: {{msg.data}}')"
            )
            
            # 2. Создаем подписку в __init__
            sub_line = (
                f"        self.sub_{port_name} = self.create_subscription("
                f"{cls}, '{{topic_name}}_{port_name}', self.{callback_name}, 10)"
            )
            
            # Вставляем всё вместе после маркера
            if marker in current_code:
                new_code = current_code.replace(marker, f"{marker}{callback_code}\n{sub_line}")

        else:
            # --- C++ (Сложнее: private field, callback declaration, public init) ---
            cpp_type = f"{pkg}::msg::{cls}"
            var_name = f"sub_{port_name}_"
            cb_name = f"on_{port_name}"
            
            # 1. Private переменные и метод (Callback)
            # Ищем маркер переменных (если его нет в шаблоне, добавьте его в custom_nodes.py!)
            # В вашем шаблоне cpp_custom есть "// [AUTO-GEN-SUBS]", будем использовать его для всего,
            # но по-хорошему в C++ классе лучше иметь разделение private/public.
            # Для простоты вставим всё в конструктор или используем тот же маркер аккуратно.
            
            # СТРАТЕГИЯ ДЛЯ C++: 
            # В вашем текущем шаблоне cpp_custom (custom_nodes.py) структура простая.
            # Мы добавим подписку в конструктор, а callback и переменную придется впихнуть хитро.
            
            # Лучше всего обновить шаблон `cpp_custom` в custom_nodes.py, добавив маркер `private: // [AUTO-GEN-VARS]`
            # Но если работать с тем что есть, вставим в тело класса.
            
            marker_main = "// [AUTO-GEN-SUBS]"
            
            # Код для вставки:
            code_chunk = f"""
    // Generated Input: {port_name}
    rclcpp::Subscription<{cpp_type}>::SharedPtr {var_name};
    void {cb_name}(const {cpp_type} & msg) const {{
        RCLCPP_INFO(this->get_logger(), "Got {port_name}");
    }}
    // Init inside constructor (hacky insertion, requires careful template setup)
    // NOTE: For proper C++ generation, ensure your template has specific sections.
            """
            # ПРИМЕЧАНИЕ: Автоматическая генерация C++ кода сложнее из-за разделения .hpp/.cpp 
            # или структуры класса. Самый простой способ для вашего проекта:
            # Вставлять `create_subscription` в конструктор, а callback определять как лямбду.
            
            lambda_sub = (
                f"        {var_name} = this->create_subscription<{cpp_type}>(\n"
                f"            \"{{topic_name}}_{port_name}\", 10, \n"
                f"            [this](const {cpp_type} & msg){{ RCLCPP_INFO(this->get_logger(), \"Got {port_name}\"); }});"
            )
            
            # Добавляем объявление переменной перед конструктором (если возможно) или просто используем auto внутри (но тогда переменная умрет).
            # Чтобы переменная жила, она должна быть полем класса.
            
            # Рекомендую добавить маркер `private:` в шаблон `cpp_custom` в файле custom_nodes.py
            # Если маркера нет, вставим лямбду с сохранением в `auto` (опасно, сборщик мусора удалит).
            # Поэтому добавим переменную в public для надежности (раз уж мы генерируем код).
            
            injection = (
                f"\n    rclcpp::Subscription<{cpp_type}>::SharedPtr {var_name};" # Поле
                f"\n    // Init in constructor:" # Просто комментарий, инициализация ниже
            )
            
            if marker_main in new_code:
                 # Вставляем поле
                 new_code = new_code.replace("public:", f"public:{injection}")
                 # Вставляем инициализацию в маркер SUBS
                 new_code = new_code.replace(marker_main, f"{marker_main}\n{lambda_sub}")

        if new_code != current_code:
            node.set_property('code_content', new_code) 

    def on_export_docker(self):
        # 1. Скомпилировать граф
        graph = self.graph_py if self.ui.tabs.currentIndex() == 0 else self.graph_cpp
        scripts = GraphCompiler(graph).compile()
        
        if not scripts:
            QtWidgets.QMessageBox.warning(self, "Export", "Graph is empty or failed to compile!")
            return

        if not self.current_project_path:
             QtWidgets.QMessageBox.warning(self, "Export", "Please save the project first (Workspace needed)!")
             return

        # 2. Спросить про GUI
        include_gui = QtWidgets.QMessageBox.question(
            self, "Docker Export", 
            "Do you want to include GUI support (RViz/QT)?\n(Select 'No' for headless/server mode)",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No
        ) == QtWidgets.QMessageBox.Yes

        # 3. Экспорт
        try:
            out_path = ExportManager.export_to_portable_package(
                self.current_project_path, 
                scripts, 
                include_gui
            )
            QtWidgets.QMessageBox.information(self, "Success", f"Portable package created at:\n{out_path}")
            # Можно открыть папку в проводнике
            QtGui.QDesktopServices.openUrl(QtCore.QUrl.fromLocalFile(out_path))
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Error", f"Export failed: {e}")

    def _get_node_identifier(self, code):
        """Маппинг кнопок палитры на реальные классы (СИНХРОНИЗИРОВАНО С ЛОГОМ)"""
        mapping = {
            "CW_BUILDER": 'ros.coursework.LevelBuilderNode',
            "CW_HIVE":    'ros.coursework.HiveMindNode',
            "CW_AGENT":   'ros.coursework.SmartAgentNode',
            # Python (Имена строго из лога Final Registered Nodes)
            "PY_STRING_PUB": 'ros.py.PyStringPubNode',
            "PY_STRING_SUB": 'ros.py.PyStringSubNode',
            "PY_TWIST_PUB":  'ros.py.PyTwistPubNode',
            "PY_TWIST_SUB":  'ros.py.PyTwistSubNode',
            "PY_CUSTOM":     'ros.py.PyCustomNode',
            
            # C++ (Имена строго из лога Final Registered Nodes)
            "CPP_STRING_PUB": 'ros.cpp.CppStringPubNode',
            "CPP_STRING_SUB": 'ros.cpp.CppStringSubNode',
            "CPP_TWIST_PUB":  'ros.cpp.CppTwistPubNode',
            "CPP_TWIST_SUB":  'ros.cpp.CppTwistSubNode',
            "CPP_CUSTOM":     'ros.cpp.CppCustomNode',
            
            # Tools
            "MONITOR": 'nodes.utility.MonitorNode',
            "GROUP": 'ros.nodes.RosGroupNode'
        }
        return mapping.get(code)

    def system_log(self, msg): 
        if self.ui and self.ui.console_sys: self.ui.console_sys.append(f"> {msg}")
        else: print(f"> {msg}")

    def on_node_double_click(self, node):
        # 1. Если это группа - заходим внутрь
        if node.type_ == 'ros.nodes.RosGroupNode':
            toggle_group_visibility(self.graph_py if self.ui.tabs.currentIndex()==0 else self.graph_cpp, node)
            return
        
        # 2. Если у ноды есть код - открываем КРАСИВЫЙ редактор
        if hasattr(node, 'get_property') and node.has_property('code_content'):
            # Определяем язык для подсветки
            lang = 'cpp' if 'cpp' in node.type_ else 'python'
            
            # Создаем новое окно
            dlg = AdvancedCodeDialog(node.get_property('code_content'), language=lang, parent=self)
            
            if dlg.exec():
                # Сохраняем код обратно в ноду
                node.set_property('code_content', dlg.get_code())
                self.system_log(f"Code for '{node.name()}' updated.")

    def add_from_palette_logic(self, code, use_mouse_pos=False):
        graph = self.graph_py if self.ui.tabs.currentIndex() == 0 else self.graph_cpp
        
        if code.startswith("USER_LIB:"):
            filename = code.split(":", 1)[1]
            path = os.path.join(self.ui.user_lib_path, filename)
            if os.path.exists(path):
                try:
                    with open(path, 'r', encoding='utf-8') as f: 
                        data = json.load(f)
                    graph.paste_nodes(data)
                    pasted_nodes = graph.selected_nodes()
                    fix_group_after_load(graph, pasted_nodes)
                    self.system_log(f"Template '{filename}' added.")
                except Exception as e: 
                    self.system_log(f"Error loading template: {e}")
                    traceback.print_exc()
            return
        
        node_type = self._get_node_identifier(code)
        if node_type:
             graph.create_node(node_type, pos=[0, 0])

    def add_from_palette(self, item):
        code = item.data(QtCore.Qt.UserRole)
        if code: self.add_from_palette_logic(code)

    def on_group_nodes(self):
        graph = self.graph_py if self.ui.tabs.currentIndex() == 0 else self.graph_cpp
        try:
            dlg = GroupSettingsDialog(self)
            if dlg.exec() != QtWidgets.QDialog.Accepted: return
            data = dlg.get_data()
            
            group, msg = create_group_from_selection(graph, name=data['name'], color=data['color'])
            
            if group:
                if data['save']: self._save_group_to_library(graph, group, data['name'])
                self.system_log(f"Group created.")
            else:
                self.system_log(f"Group Error: {msg}")
        except Exception as e:
            self.system_log(f"Critical Group Error: {e}")
            traceback.print_exc()

    def _save_group_to_library(self, graph, group_node, name):
        try:
            internal_ids = group_node.get_property('internal_nodes')
            all_nodes = {n.id: n for n in graph.all_nodes()}
            nodes_to_save = [group_node]
            hidden_temp = []
            for nid in internal_ids:
                if nid in all_nodes:
                    n = all_nodes[nid]
                    nodes_to_save.append(n)
                    if hasattr(n, 'view') and not n.view.isVisible():
                        n.view.setVisible(True)
                        hidden_temp.append(n)
            clipboard_data = graph.copy_nodes(nodes_to_save)
            for n in hidden_temp: n.view.setVisible(False)
            filename = "".join([c for c in f"{name}.json" if c.isalpha() or c.isdigit() or c in (' ', '.', '_')]).strip()
            path = os.path.join(self.ui.user_lib_path, filename)
            with open(path, 'w', encoding='utf-8') as f: json.dump(clipboard_data, f, indent=4)
            self.system_log(f"Saved: {filename}")
            self.ui.refresh_palette()
        except Exception as e: 
            self.system_log(f"Save error: {e}")
            traceback.print_exc()

    def on_deploy_run(self):
        self.ui.console_sys.clear(); self.ui.console_ros.clear()
        graph = self.graph_py if self.ui.tabs.currentIndex() == 0 else self.graph_cpp
        has_nodes = False
        warnings = []
        for node in graph.all_nodes():
            if node.type_ == 'nodeGraphQt.nodes.BackdropNode': continue
            has_nodes = True
            
            connected = any([p.connected_ports() for p in node.input_ports() + node.output_ports()])
            if not connected and node.type_ != 'ros.nodes.RosGroup':
                warnings.append(f"Warning: Node '{node.name()}' disconnected.")
        
        if not has_nodes: return self.system_log("Error: Empty graph.")
        if warnings: 
            for w in warnings: self.system_log(w)

        scripts = GraphCompiler(graph).compile()
        if not scripts and not warnings: return self.system_log("Warning: No code generated.")

        self.ui.actions['run'].setEnabled(False); self.ui.actions['stop'].setEnabled(True)
        self.deploy_worker = DeployWorker(scripts, self.container_manager)
        self.deploy_worker.log_signal.connect(self.system_log)
        self.deploy_worker.finished_signal.connect(self.start_monitoring)
        self.deploy_worker.start()

    def start_monitoring(self):
        self.log_worker = LogMonitorWorker(self.container_manager)
        self.log_worker.new_log_line.connect(self.ros_log_line); self.log_worker.start()

    def ros_log_line(self, line):
        clean = re.sub(r'\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])', '', line).strip()
        self.ui.console_ros.moveCursor(QtGui.QTextCursor.End)
        self.ui.console_ros.insertPlainText(clean + "\n")
        self.ui.console_ros.verticalScrollBar().setValue(self.ui.console_ros.verticalScrollBar().maximum())
        
        match = re.search(r'\[([a-zA-Z0-9_\-\.]+)\]:', clean)
        if match:
            all_matches = re.findall(r'\[([a-zA-Z0-9_\-\.]+)\]', clean.split(':')[0])
            if all_matches:
                name = all_matches[-1]
                for g in [self.graph_py, self.graph_cpp]:
                    for node in g.all_nodes():
                        if node.get_property('node_name') == name:
                            # Проверяем все выходы
                            for out in node.output_ports():
                                for p in out.connected_ports():
                                    if p.node().type_ == 'nodes.utility.MonitorNode': 
                                        p.node().update_data(clean)

    def on_save_project(self):
        # Если проект открыт через Лаунчер, у нас есть путь к папке
        if self.current_project_path:
            # Сохраняем строго в project.json внутри папки
            save_path = os.path.join(self.current_project_path, "project.json")
            success = self.project_manager.save_project(save_path)
            
            if success:
                self.system_log(f"✅ Project saved to: {save_path}")
                # Визуально мигнем в статусбаре (если есть) или просто лог
            else:
                QtWidgets.QMessageBox.critical(self, "Save Error", "Could not save project file!")
        else:
            # Если вдруг (маловероятно) путь не задан — спрашиваем как раньше
            f, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Save As", "", "JSON (*.json)")
            if f: self.project_manager.save_project(f)
    def on_load_project(self):
        f, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Open", "", "JSON (*.json)")
        if f: self.project_manager.load_project(f)
    def on_stop(self):
        if self.log_worker: self.log_worker.stop()
        if self.container_manager: 
            try: self.container_manager.container.stop()
            except: pass
        self.ui.actions['run'].setEnabled(True); self.ui.actions['stop'].setEnabled(False); self.system_log("Stopped.")
    def on_clear(self):
        graph = self.graph_py if self.ui.tabs.currentIndex() == 0 else self.graph_cpp
        graph.clear_session()
    def keyPressEvent(self, event):
        if event.modifiers() & QtCore.Qt.ControlModifier and event.key() == QtCore.Qt.Key_G: self.on_group_nodes()
        elif event.key() in [QtCore.Qt.Key_Delete, QtCore.Qt.Key_Backspace]:
            g = self.graph_py if self.ui.tabs.currentIndex() == 0 else self.graph_cpp
            g.delete_nodes(g.selected_nodes())
        else: super().keyPressEvent(event)

    def setup_connections(self):
        self.graph_py.port_connected.connect(self.on_port_connected)
        self.graph_py.port_disconnected.connect(self.on_port_disconnected)
        self.graph_cpp.port_connected.connect(self.on_port_connected)
        self.graph_cpp.port_disconnected.connect(self.on_port_disconnected)

    def on_port_connected(self, port_in, port_out):
        node = port_in.node()
        if node.type_ == 'nodes.utility.MonitorNode':
            port_name = port_out.name().lower()
            color = TYPE_COLORS['DEFAULT']
            label = "Unknown"

            if "twist" in port_name: 
                color = TYPE_COLORS['geometry_msgs/Twist']
                label = "Twist"
            elif "string" in port_name: 
                color = TYPE_COLORS['std_msgs/String']
                label = "String"
            else:
                source = port_out.node()
                if "Twist" in source.name(): color = TYPE_COLORS['geometry_msgs/Twist']; label="Twist"
                elif "String" in source.name(): color = TYPE_COLORS['std_msgs/String']; label="String"

            node.set_color(*color)
            node.set_property('name', f"Monitor ({label})")

    def on_port_disconnected(self, port_in, port_out):
        node = port_in.node()
        if node.type_ == 'nodes.utility.MonitorNode':
            if not port_in.connected_ports():
                node.set_color(*TYPE_COLORS["DEFAULT"])
                node.set_property('name', "Monitor (No Data)")

if __name__ == '__main__':
    from qt_material import apply_stylesheet
    app = QtWidgets.QApplication(sys.argv)

    extra = {
        # Делаем шрифт чуть больше и жирнее
        'font_family': 'Segoe UI', # Или Roboto, Arial
        'font_size': '14px',
        'line_height': '14px',
        
        # Принудительно красим важные элементы в яркий белый
        'density_scale': '-1', # Сделает интерфейс чуть компактнее (по желанию)
    }
    
    apply_stylesheet(app, theme='dark_teal.xml', extra=extra)

    app.setStyleSheet(app.styleSheet() + """
        QLabel { color: #ffffff; font-weight: bold; }
        QCheckBox { color: #ffffff; }
        QGroupBox { color: #ffffff; font-weight: bold; }
        QPushButton { font-weight: bold; }
        /* Делаем заголовки доков (Library, Logs) контрастными */
        QDockWidget { color: white; font-weight: bold; }
        QDockWidget::title { background: #2c3e50; text-align: center; padding: 5px; }
    """)

    # 1. Показываем Лаунчер
    launcher = LauncherDialog()
    if launcher.exec() == QtWidgets.QDialog.Accepted:
        # 2. Если выбрали проект - запускаем Основное Окно
        project_path = launcher.selected_project_path
        window = RosOrchestratorApp(project_path) # Передаем путь
        window.show()
        sys.exit(app.exec())
    else:
        # Если закрыли лаунчер - выходим
        sys.exit(0)