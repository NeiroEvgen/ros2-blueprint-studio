import os
import sys
import re
import subprocess
import traceback
from functools import partial
from PySide6 import QtWidgets, QtCore, QtGui

# === CORE MODULES ===
from core.docker_manager import RosContainerManager
from core.project_manager import ProjectManager
from core.file_watcher import FileWatcher
from core.export_manager import ExportManager 
from core.workspace_manager import WorkspaceManager
from core.ros_process import DeployWorker, LogMonitorWorker


from compilers.graph_compiler import GraphCompiler

# === UI MODULES ===
from ui.ui_manager import UiManager
from ui.dashboard import ConsoleDashboard  # <-- NEW DASHBOARD
from ui.graph_setup import setup_graphs, TYPE_COLORS, ALL_NODE_CLASSES

# === NODES ===
from nodes.base import MSG_COLORS
from nodes.group_logic import toggle_group_visibility

class RosVisualRunner(QtWidgets.QMainWindow):
    def __init__(self, project_path=None):
        super().__init__()
        self.current_project_path = project_path
        
        # 1. UI Setup
        self.ui = UiManager(self)
        self.ui.setup_ui()
        
        self.dashboard = ConsoleDashboard()
        
        if hasattr(self.ui, 'console_ros'):
            # Ищем QTabWidget, в котором лежит наша консоль (он может быть не прямым родителем)
            tab_widget = self.ui.console_ros.parent()
            while tab_widget and not isinstance(tab_widget, QtWidgets.QTabWidget):
                tab_widget = tab_widget.parent()
                
            if tab_widget:
                idx = tab_widget.indexOf(self.ui.console_ros)
                if idx != -1:
                    # Убираем старую текстовую консоль и вставляем таблицу
                    tab_widget.removeTab(idx)
                    tab_widget.insertTab(idx, self.dashboard, "ROS2 Dashboard")
                

        # 2. Graph Setup (delegated to ui/graph_setup.py)
        self.graph_py, self.graph_cpp, self.f_py, self.f_cpp = setup_graphs(
            self.ui.tabs, self.resolve_node_type
        )

        self.setup_graph_signals()

        # 3. Core Managers
        self.project_manager = ProjectManager(self.graph_py, self.graph_cpp)
        self.watcher = FileWatcher()
        self.watcher.file_changed.connect(self.on_file_changed_externally)

        # 4. Docker Connect
        self.container_manager = None
        try:
            self.container_manager = RosContainerManager()
        except Exception: 
            self.system_log("WARNING: Docker not found or not running.")

        # 5. Connect Actions
        self.connect_actions()

        # 6. Init Logic
        if self.current_project_path:
            # Delay slightly to ensure UI is ready
            QtCore.QTimer.singleShot(100, lambda: self.on_load_project_init(self.current_project_path))
        else:
            self.ui.rebuild_palette(ALL_NODE_CLASSES, "python")
            self.ui.set_visible_graph("python")

    # === CORE LOGIC ===

    def resolve_node_type(self, code):
        """Determines node type from dropped mime data"""
        if code.startswith("USER_LIB:"):
            self.add_from_palette_logic(code); return None
        if code and "." in code: return code # Direct class path
        return None

    def on_load_project_init(self, path):
        self.project_manager.load_project(path)
        self._start_watcher_safe(path)
        p_type = WorkspaceManager.get_project_type(path)
        self.ui.rebuild_palette(ALL_NODE_CLASSES, p_type)
        self.ui.set_visible_graph(p_type)
        self.system_log(f"Project loaded: {p_type.upper()}")

    def _start_watcher_safe(self, project_path):
        src_path = os.path.join(project_path, "src")
        if os.path.exists(src_path):
            self.watcher.start_watching(src_path)
            self.system_log(f"👀 Watcher started on: {src_path}")

    def connect_actions(self):
        self.ui.create_palette(self.add_from_palette)
        
        # Standard Actions
        self.ui.actions['save'].clicked.connect(self.on_save_project)
        self.ui.actions['open'].clicked.connect(self.on_load_project)
        self.ui.actions['run'].clicked.connect(self.on_deploy_run)
        self.ui.actions['stop'].clicked.connect(self.on_stop)
        self.ui.actions['clear'].clicked.connect(self.on_clear)
        self.ui.actions['group'].clicked.connect(self.on_group_nodes)
        
        # Shortcuts
        self.del_shortcut = QtGui.QShortcut(QtGui.QKeySequence(QtCore.Qt.Key_Delete), self)
        self.del_shortcut.activated.connect(self.on_delete_selection)
        
        # --CTRL+S ---
        self.save_shortcut = QtGui.QShortcut(QtGui.QKeySequence("Ctrl+S"), self)
        self.save_shortcut.activated.connect(self.on_save_project)
        # Context Menus
        self.setup_custom_context_menu(self.graph_py)
        self.setup_custom_context_menu(self.graph_cpp)

        if 'export_docker' in self.ui.actions:
            self.ui.actions['export_docker'].clicked.connect(self.on_export_docker)

    # === DEPLOYMENT ===

    def on_deploy_run(self):
        if not self.current_project_path: 
            self.system_log("Error: No project loaded.")
            return

        self.dashboard.clear()
        self.project_manager.save_project(self.current_project_path)
        
        # Clear logs
        self.ui.console_sys.clear()

        # UI State
        self.ui.actions['run'].setEnabled(False)
        self.ui.actions['stop'].setEnabled(True)

        # Worker Start
        self.deploy_worker = DeployWorker(self.container_manager, self.current_project_path)
        self.deploy_worker.sys_signal.connect(self.system_log)
        
        # === CONNECT TO DASHBOARD ===
        self.deploy_worker.ros_signal.connect(self.on_dashboard_log)
        
        self.deploy_worker.finished_signal.connect(self.on_deploy_finished)
        self.deploy_worker.start()

    def on_dashboard_log(self, text):
        """Redirects logs to the smart dashboard"""
        self.dashboard.process_log(text)
        # Duplicate critical errors to system log
        if "process has died" in text or "ERROR" in text:
             self.system_log(f"ROS ALERT: {text}")

    def on_stop(self):
        if hasattr(self, 'deploy_worker') and self.deploy_worker:
            self.deploy_worker.stop()
            self.system_log("Stopping container...")
        
        if self.container_manager:
            try: self.container_manager.container.stop()
            except: pass
            
        self.ui.actions['run'].setEnabled(True)
        self.ui.actions['stop'].setEnabled(False)

    def on_deploy_finished(self):
        self.on_stop()
        self.system_log("--- SESSION ENDED ---")

    # === FILE WATCHING & SYNC ===
    
    @QtCore.Slot(str, str, list)
    def on_file_changed_externally(self, filename, content, ports):
        
        graphs = [self.graph_py, self.graph_cpp]
        self.project_manager.sync_node_from_file(filename, content, ports, graphs, self.system_log)
        
        # --- AutoRout ---
        self.auto_route_hardcoded_topics(self.graph_py)
        self.auto_route_hardcoded_topics(self.graph_cpp)
                            #file_changed // sync_node_from_file
    # === UTILS ===

    def system_log(self, msg):
        if hasattr(self.ui, 'console_sys'):
            self.ui.console_sys.append(f"> {msg}")
        else:
            print(f"> {msg}")

    def add_from_palette(self, item):
        code = item.data(QtCore.Qt.UserRole)
        if code: 
             graph = self.graph_py if self.ui.tabs.currentIndex() == 0 else self.graph_cpp
             graph.create_node(code, pos=[0, 0])

    def on_export_docker(self):
        if not self.current_project_path:
            self.system_log(" Error: No project loaded. Please save or open a project first.")
            return
            
        # 1. Принудительно сохраняем актуальное состояние графа на диск
        self.project_manager.save_project(self.current_project_path)
        
        # 2. Архивируем проект
        try:
            path = ExportManager.export_to_portable_package(self.current_project_path, include_gui=False)
            QtWidgets.QMessageBox.information(self, "Success", f"Project successfully exported to:\n{path}")
            self.system_log(f" Exported successfully to: {path}")
        except Exception as e:
            self.system_log(f" Export Failed: {e}")
            import traceback
            traceback.print_exc()

    def on_save_project(self):
        if self.current_project_path:
            self.project_manager.save_project(self.current_project_path)
            self._start_watcher_safe(self.current_project_path)
            self.system_log(" Project saved (Ctrl+S)") # <--- ВОТ ЭТА СТРОЧКА
        else:
            root = WorkspaceManager.get_workspace_root()
            path = QtWidgets.QFileDialog.getExistingDirectory(self, "Save Project", root)
            if path:
                self.current_project_path = path
                self.project_manager.save_project(path)
                self._start_watcher_safe(path)

    def on_load_project(self):
        root = WorkspaceManager.get_workspace_root()
        path = QtWidgets.QFileDialog.getExistingDirectory(self, "Open Project", root)
        if path:
            self.current_project_path = path
            self.on_load_project_init(path)

    def on_clear(self):
        graph = self.graph_py if self.ui.tabs.currentIndex() == 0 else self.graph_cpp
        graph.clear_session()

    def on_delete_selection(self):
        graph = self.graph_py if self.ui.tabs.currentIndex() == 0 else self.graph_cpp
        nodes = graph.selected_nodes()

        # === 1. УДАЛЯЕМ ФАЙЛЫ С ДИСКА ===
    
        if self.current_project_path and nodes:
            src_dir = os.path.join(self.current_project_path, "src")
            
            for node in nodes:
                if node.has_property('source_file'):
                    filename = node.get_property('source_file')
                    if filename:
                        # Ищем файл в возможных директориях
                        possible_paths = [
                            os.path.join(src_dir, "cpp", filename),
                            os.path.join(src_dir, "python", filename),
                            os.path.join(src_dir, filename)
                        ]
                        
                        for p in possible_paths:
                            if os.path.exists(p):
                                try:
                                    os.remove(p)
                                    self.system_log(f" Deleted source file: {filename}")
                                except Exception as e:
                                    self.system_log(f" Failed to delete {filename}: {e}")
                                break 

        # === 2. УДАЛЯЕМ ВИЗУАЛ ===
    
        if nodes:
            graph.delete_nodes(nodes)

    def on_group_nodes(self):
        # Future implementation
        pass 

    # === SIGNALS & PORTS ===
    
    def setup_graph_signals(self):
        self.graph_py.node_double_clicked.connect(self.on_node_double_click)
        self.graph_cpp.node_double_clicked.connect(self.on_node_double_click)
        
        for g in [self.graph_py, self.graph_cpp]:
            g.port_connected.connect(self.on_port_connected)
            g.port_disconnected.connect(self.on_port_disconnected)

    def on_node_double_click(self, node):
        if node.type_ == 'ros.nodes.RosGroupNode':
            toggle_group_visibility(node.graph, node)
            return
        
        # Open file in external editor
        if hasattr(self.project_manager, 'open_node_in_editor'):
            self.project_manager.open_node_in_editor(node, self.current_project_path)
        else:
            # Fallback if method is missing
            self.system_log(f"Opening node: {node.name()}")

    def on_port_connected(self, port_in, port_out):
        node = port_in.node()
        if node.type_ == 'nodes.utility.MonitorNode':
            port_name = port_out.name().lower()
            color = TYPE_COLORS.get("DEFAULT")
            if "twist" in port_name: color = TYPE_COLORS.get("geometry_msgs/Twist", (255,152,0))
            elif "string" in port_name: color = TYPE_COLORS.get("std_msgs/String", (255,235,59))
            node.set_color(*color)
            node.set_property('name', f"Monitor ({port_name})")

    def on_port_disconnected(self, port_in, port_out):
        node = port_in.node()
        if node.type_ == 'nodes.utility.MonitorNode' and not port_in.connected_ports():
            node.set_color(*TYPE_COLORS["DEFAULT"])
            node.set_property('name', "Monitor")
    
    def auto_route_hardcoded_topics(self, graph):
        """Автоматически соединяет паблишеры и сабскрайберы с одинаковыми именами топиков"""
        pubs_by_topic = {}
        subs_by_topic = {}

        # 1. Собираем все порты по именам топиков
        for node in graph.all_nodes():
            for port in node.output_ports():
                topic_name = port.name()
                if topic_name not in pubs_by_topic:
                    pubs_by_topic[topic_name] = []
                pubs_by_topic[topic_name].append(port)

            for port in node.input_ports():
                topic_name = port.name()
                if topic_name not in subs_by_topic:
                    subs_by_topic[topic_name] = []
                subs_by_topic[topic_name].append(port)

        # 2. Ищем совпадения и натягиваем провода
        routed_count = 0
        for topic, pub_ports in pubs_by_topic.items():
            if topic in subs_by_topic:
                sub_ports = subs_by_topic[topic]
                for pub_port in pub_ports:
                    for sub_port in sub_ports:
                        # Проверяем, чтобы не дублировать уже существующую связь
                        if sub_port not in pub_port.connected_ports():
                            try:
                                pub_port.connect_to(sub_port)
                                
                                # В NodeGraphQt это свойства, а не методы!
                                pub_port.color = (50, 205, 50, 255)
                                sub_port.color = (50, 205, 50, 255)
                                
                                pub_port.border_color = (0, 255, 0, 255)
                                sub_port.border_color = (0, 255, 0, 255)

                                # Блокируем намертво
                                pub_port.locked = True
                                sub_port.locked = True
                                
                                # Принудительно заставляем ноду перерисовать себя в UI
                                pub_port.node().update()
                                sub_port.node().update()
                                
                                routed_count += 1
                            except Exception as e:
                                # Теперь, если что-то пойдет не так, мы увидим это в терминале!
                                print(f" Error auto-rout: {e}")
                                
        if routed_count > 0:
            self.system_log(f" Auto-Routed {routed_count} hardcoded connections!")

    # === CONTEXT MENU ===
    def setup_custom_context_menu(self, graph):
        """Adds custom actions (add port, rescan) to right-click menu"""
        try: 
            root_menu = graph.context_menu() 
        except TypeError: 
            root_menu = graph.context_menu
            
        root_menu.add_separator()
        
        # Manual Rescan Action
        root_menu.add_command(" Rescan Code for Ports", partial(self.on_manual_rescan, graph))
        root_menu.add_separator()
        
        # Add Output Ports
        out_menu = root_menu.add_menu("Add Custom Output (Publisher)")
        for name, color in MSG_COLORS.items():
            short_name = name.split('/')[-1]
            out_menu.add_command(f"Add {short_name}", partial(self.on_add_port_clicked, graph, name, color))
            
        # Add Input Ports
        in_menu = root_menu.add_menu("Add Custom Input (Subscriber)")
        for name, color in MSG_COLORS.items():
            short_name = name.split('/')[-1]
            in_menu.add_command(f"Add {short_name}", partial(self.on_add_input_clicked, graph, name, color))

    def on_manual_rescan(self, graph, *args, **kwargs):
        selected = graph.selected_nodes()
        if not selected: return
        for node in selected:
            if not node.has_property('source_file'): continue
            filename = node.get_property('source_file')
            if self.current_project_path:
                full_path = os.path.join(self.current_project_path, "src", filename)
                if os.path.exists(full_path):
                    with open(full_path, 'r', encoding='utf-8') as f: content = f.read()
                    ports = self.watcher._scan_ports(content, full_path)
                    self.on_file_changed_externally(filename, content, ports)
                    self.system_log(f" Manually rescanned: {filename}")

    def on_add_port_clicked(self, graph, port_type, port_color):
        selected_nodes = graph.selected_nodes()
        for node in selected_nodes:
            # Logic to generate unique port name
            short_type = port_type.split('/')[-1].lower()
            base_name = f"out_{short_type}"
            count = 1
            while f"{base_name}_{count}" in node.outputs().keys(): count += 1
            name = f"{base_name}_{count}"
            
            node.add_output(name, multi_output=True, display_name=True, color=port_color)
            
            # Helper to inject code (delegated to project manager logic or inline)
            self.inject_code_port(node, name, port_type, "pub")

    def on_add_input_clicked(self, graph, port_type, port_color):
        selected_nodes = graph.selected_nodes()
        for node in selected_nodes:
            short_type = port_type.split('/')[-1].lower()
            base_name = f"in_{short_type}"
            count = 1
            while f"{base_name}_{count}" in node.inputs().keys(): count += 1
            name = f"{base_name}_{count}"
            
            node.add_input(name, multi_input=True, display_name=True, color=port_color)
            self.inject_code_port(node, name, port_type, "sub")

    def inject_code_port(self, node, port_name, port_type_ros, mode):
        """Внедряет код паблишера/сабскрайбера прямо в текст ноды по маркерам"""
        if not node.has_property('code_content'): return
        current = node.get_property('code_content')
        pkg, cls = port_type_ros.split('/')
        new_code = current
        
        if 'cpp' in node.type_:
            cpp_type = f"{pkg}::msg::{cls}"
            if mode == "pub":
                var_name = f"pub_{port_name}_"
                if "// [AUTO-GEN-VARS]" in new_code: 
                    new_code = new_code.replace("// [AUTO-GEN-VARS]", f"// [AUTO-GEN-VARS]\n    std::shared_ptr<rclcpp::Publisher<{cpp_type}>> {var_name};")
                if "// [AUTO-GEN-PUBS]" in new_code: 
                    new_code = new_code.replace("// [AUTO-GEN-PUBS]", f"// [AUTO-GEN-PUBS]\n    {var_name} = this->create_publisher<{cpp_type}>(\"{{topic_name}}_{port_name}\", 10);")
            else:
                var_name = f"sub_{port_name}_"
                if "// [AUTO-GEN-SUBS]" in new_code:
                    line = (f"        {var_name} = this->create_subscription<{cpp_type}>(\n"
                            f"            \"{{topic_name}}_{port_name}\", 10, \n"
                            f"            [this](const {cpp_type} & msg){{ RCLCPP_INFO(this->get_logger(), \"Got {port_name}\"); }});")
                    new_code = new_code.replace("public:", f"public:\n    rclcpp::Subscription<{cpp_type}>::SharedPtr {var_name};")
                    new_code = new_code.replace("// [AUTO-GEN-SUBS]", f"// [AUTO-GEN-SUBS]\n{line}")
        else:
            if mode == "pub":
                if "# [AUTO-GEN-PUBS]" in new_code: 
                    new_code = new_code.replace("# [AUTO-GEN-PUBS]", f"# [AUTO-GEN-PUBS]\n        self.pub_{port_name} = self.create_publisher({cls}, '{{topic_name}}_{port_name}', 10)")
            else:
                if "# [AUTO-GEN-SUBS]" in new_code:
                    cb = f"\n    def callback_{port_name}(self, msg):\n        self.get_logger().info(f'Got {port_name}')"
                    sub = f"        self.sub_{port_name} = self.create_subscription({cls}, '{{topic_name}}_{port_name}', self.callback_{port_name}, 10)"
                    new_code = current.replace("# [AUTO-GEN-SUBS]", f"# [AUTO-GEN-SUBS]{cb}\n{sub}")
                    
        if new_code != current: 
            node.set_property('code_content', new_code)