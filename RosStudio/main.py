import sys
import logging
import traceback
import re
import json
import os
from functools import partial
from PySide6 import QtWidgets, QtCore, QtGui
from NodeGraphQt import NodeGraph

try:
    from docker_manager import RosContainerManager
    from ui_manager import UiManager
    from project_manager import ProjectManager
    from graph_compiler import GraphCompiler
    from code_editor import AdvancedCodeDialog
    
    # Standard Nodes & Palette
    from nodes.custom_nodes import (
        PyStringPubNode, PyStringSubNode, PyTwistPubNode, PyTwistSubNode, PyCustomNode,
        CppStringPubNode, CppStringSubNode, CppTwistPubNode, CppTwistSubNode, CppCustomNode,
        MSG_COLORS
    )
    
    # Groups & Utility
    from nodes.group_logic import RosGroupNode, create_group_from_selection, toggle_group_visibility, fix_group_after_load
    from nodes.monitor_node import MonitorNode
    
except ImportError as e:
    print(f"IMPORT ERROR: {e}")
    sys.exit(1)

logging.basicConfig(level=logging.INFO)

# --- Palette Colors (For Monitor) ---
TYPE_COLORS = {
    "std_msgs/String": (255, 235, 59),
    "geometry_msgs/Twist": (255, 152, 0),
    "DEFAULT": (100, 100, 100)
}

# --- Workers ---
class LogMonitorWorker(QtCore.QThread):
    new_log_line = QtCore.Signal(str)
    
    def __init__(self, manager):
        super().__init__()
        self.manager = manager
        self.is_running = True
        
    def run(self):
        if not self.manager or not self.manager.container: 
            return
        try:
            buffer = ""
            for chunk in self.manager.container.logs(stream=True, follow=True):
                if not self.is_running: break
                text_chunk = chunk.decode('utf-8', errors='replace')
                buffer += text_chunk
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    self.new_log_line.emit(line)
            if buffer.strip(): 
                self.new_log_line.emit(buffer)
        except Exception:
            pass
            
    def stop(self): 
        self.is_running = False

class DeployWorker(QtCore.QThread):
    log_signal = QtCore.Signal(str)
    finished_signal = QtCore.Signal()
    
    def __init__(self, scripts, manager):
        super().__init__()
        self.scripts = scripts
        self.manager = manager
        
    def run(self):
        try:
            self.log_signal.emit("--- DEPLOY START ---")
            self.manager.ensure_image(lambda m: self.log_signal.emit(m))
            self.manager.start_session(lambda m: self.log_signal.emit(m))
            
            py_s = [s for s in self.scripts if s['language'] == 'python']
            cpp_s = [s for s in self.scripts if s['language'] == 'cpp']
            
            for s in py_s: 
                self.manager.inject_and_run_script(s['filename'], s['code'], lambda m: self.log_signal.emit(m))
            
            if cpp_s: 
                self.manager.build_and_run_cpp_nodes(cpp_s, lambda m: self.log_signal.emit(m))
                
            self.finished_signal.emit()
        except Exception as e:
            traceback.print_exc()
            self.log_signal.emit(f"ERR: {e}")

# --- Helper Dialogs ---
class GroupSettingsDialog(QtWidgets.QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Group Settings")
        self.resize(400, 300)
        
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
        btn_box.accepted.connect(self.accept)
        btn_box.rejected.connect(self.reject)
        layout.addWidget(btn_box)
        
    def choose_color(self):
        c = QtWidgets.QColorDialog.getColor()
        if c.isValid():
            self.selected_color = (c.red(), c.green(), c.blue())
            self.color_btn.setStyleSheet(f"background-color: rgb({c.red()},{c.green()},{c.blue()}); color: white; padding: 10px;")
            
    def get_data(self): 
        return {"name": self.name_edit.text(), "color": self.selected_color, "save": self.save_chk.isChecked()}

class GraphDropFilter(QtCore.QObject):
    def __init__(self, graph, node_map_callback):
        super().__init__()
        self.graph = graph
        self.get_node_type = node_map_callback
        
    def eventFilter(self, watched, event):
        if event.type() in [QtCore.QEvent.DragEnter, QtCore.QEvent.DragMove]:
            if event.mimeData().hasText(): 
                event.acceptProposedAction()
                return True
        elif event.type() == QtCore.QEvent.Drop:
            code = event.mimeData().text()
            node_type = self.get_node_type(code)
            if node_type:
                pos = event.position().toPoint()
                scene_pos = self.graph.viewer().mapToScene(pos)
                try: 
                    self.graph.create_node(node_type, pos=[scene_pos.x(), scene_pos.y()])
                except Exception as e: 
                    print(f"Drop Error: {e}")
                event.acceptProposedAction()
                return True
        return super().eventFilter(watched, event)

# --- MAIN APP ---
class RosOrchestratorApp(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.ui = UiManager(self)
        self.ui.setup_ui()
        
        self.graph_py = NodeGraph()
        self.graph_cpp = NodeGraph()
        
        # 1. Register Nodes
        node_classes = [
            PyStringPubNode, PyStringSubNode, PyTwistPubNode, PyTwistSubNode, PyCustomNode,
            CppStringPubNode, CppStringSubNode, CppTwistPubNode, CppTwistSubNode, CppCustomNode,
            MonitorNode, 
            RosGroupNode 
        ]
        
        self.graph_py.register_nodes(node_classes)
        self.graph_cpp.register_nodes(node_classes)
        
        print("Registered Nodes:", self.graph_py.registered_nodes())
        
        self.graph_py.node_double_clicked.connect(self.on_node_double_click)
        self.graph_cpp.node_double_clicked.connect(self.on_node_double_click)
        
        self.ui.tabs.addTab(self.graph_py.widget, "Python Blueprints")
        self.ui.tabs.addTab(self.graph_cpp.widget, "C++ Blueprints")

        # Setup Drag and Drop
        self.graph_py.widget.setAcceptDrops(True)
        self.graph_py.viewer().setAcceptDrops(True) 
        self.graph_py.viewer().viewport().setAcceptDrops(True)
        
        self.graph_cpp.widget.setAcceptDrops(True)
        self.graph_cpp.viewer().setAcceptDrops(True)
        self.graph_cpp.viewer().viewport().setAcceptDrops(True)

        def resolve_node_type(code):
            if code.startswith("USER_LIB:"):
                self.add_from_palette_logic(code, use_mouse_pos=True)
                return None
            return self._get_node_identifier(code)

        self.filter_py = GraphDropFilter(self.graph_py, resolve_node_type)
        self.graph_py.viewer().viewport().installEventFilter(self.filter_py)
        self.filter_cpp = GraphDropFilter(self.graph_cpp, resolve_node_type)
        self.graph_cpp.viewer().viewport().installEventFilter(self.filter_cpp)

        self.project_manager = ProjectManager(self.graph_py, self.graph_cpp)
        
        self.container_manager = None
        try: 
            self.container_manager = RosContainerManager()
            self.system_log("SUCCESS: Docker Connected!")
        except Exception as e:
            self.system_log(f"WARNING: Docker connection failed! {e}")

        self.ui.create_palette(self.add_from_palette)
        self.ui.actions['save'].triggered.connect(self.on_save_project)
        self.ui.actions['open'].triggered.connect(self.on_load_project)
        self.ui.actions['run'].triggered.connect(self.on_deploy_run)
        self.ui.actions['stop'].triggered.connect(self.on_stop)
        self.ui.actions['clear'].triggered.connect(self.on_clear)
        self.ui.actions['group'].triggered.connect(self.on_group_nodes)

        self.log_worker = None
        self.setup_connections()
        
        # Initialize Context Menu
        self.setup_custom_context_menu(self.graph_py)
        self.setup_custom_context_menu(self.graph_cpp)

    def setup_custom_context_menu(self, graph):
        """Adds 'Add Custom Output' submenu to the graph context menu."""
        try:
            root_menu = graph.context_menu() 
        except TypeError:
            root_menu = graph.context_menu

        root_menu.add_separator()
        
        port_menu = root_menu.add_menu("Add Custom Output")
        
        for name, color in MSG_COLORS.items():
            short_name = name.split('/')[-1]
            port_menu.add_command(
                f"Add {short_name}", 
                partial(self.on_add_port_clicked, graph, name, color)
            )

    def on_add_port_clicked(self, graph, port_type, port_color, *args):
        """Adds a port AND corresponding code to the selected node."""
        selected_nodes = graph.selected_nodes()
        if not selected_nodes:
            self.system_log("⚠️ Select a custom node first.")
            return

        for node in selected_nodes:
            try:
                # 1. Generate port name
                short_type = port_type.split('/')[-1].lower() # twist
                base_name = f"out_{short_type}"
                count = 1
                
                existing_ports = node.outputs().keys()
                name = base_name
                while f"{base_name}_{count}" in existing_ports:
                    count += 1
                name = f"{base_name}_{count}"
                
                # 2. Visual addition
                node.add_output(name, multi_output=True, display_name=True, color=port_color)
                
                # 3. Code Injection
                self._inject_code_for_port(node, name, port_type)
                
                self.system_log(f"✅ Port & Code added: '{name}' to '{node.name()}'")
                
            except Exception as e:
                self.system_log(f"❌ Error adding port: {e}")
                traceback.print_exc()

    def _inject_code_for_port(self, node, port_name, port_type_ros):
        """Injects publisher initialization code directly into the node editor."""
        if not node.has_property('code_content'): return
        
        current_code = node.get_property('code_content')
        is_cpp = 'cpp' in node.type_
        
        # port_type_ros example: "geometry_msgs/Twist"
        pkg, cls = port_type_ros.split('/') 
        
        new_code = current_code
        
        if not is_cpp:
            # --- PYTHON GENERATION ---
            marker = "# [AUTO-GEN-PUBS]"
            
            # self.pub_out_twist_1 = self.create_publisher(Twist, 'out_twist_1_topic', 10)
            code_line = f"        self.pub_{port_name} = self.create_publisher({cls}, '{{topic_name}}_{port_name}', 10)"
            
            if marker in current_code:
                new_code = current_code.replace(marker, f"{marker}\n{code_line}")
                
        else:
            # --- C++ GENERATION ---
            # 1. Variable (in private)
            cpp_type = f"{pkg}::msg::{cls}"
            var_name = f"pub_{port_name}_"
            
            marker_vars = "// [AUTO-GEN-VARS]"
            var_line = f"  std::shared_ptr<rclcpp::Publisher<{cpp_type}>> {var_name};"
            
            if marker_vars in new_code:
                new_code = new_code.replace(marker_vars, f"{marker_vars}\n{var_line}")
            
            # 2. Initialization (in public constructor)
            marker_pubs = "// [AUTO-GEN-PUBS]"
            init_line = f"    {var_name} = this->create_publisher<{cpp_type}>(\"{{topic_name}}_{port_name}\", 10);"
            
            if marker_pubs in new_code:
                new_code = new_code.replace(marker_pubs, f"{marker_pubs}\n{init_line}")

        if new_code != current_code:
            node.set_property('code_content', new_code)

    def _get_node_identifier(self, code):
        """Mapping palette buttons to real node classes."""
        mapping = {
            # Python
            "PY_STRING_PUB": 'ros.py.PyStringPubNode',
            "PY_STRING_SUB": 'ros.py.PyStringSubNode',
            "PY_TWIST_PUB":  'ros.py.PyTwistPubNode',
            "PY_TWIST_SUB":  'ros.py.PyTwistSubNode',
            "PY_CUSTOM":     'ros.py.PyCustomNode',
            
            # C++
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
        if self.ui and self.ui.console_sys: 
            self.ui.console_sys.append(f"> {msg}")
        else: 
            print(f"> {msg}")

    def on_node_double_click(self, node):
        # 1. If Group -> Enter/Expand
        if node.type_ == 'ros.nodes.RosGroupNode':
            toggle_group_visibility(self.graph_py if self.ui.tabs.currentIndex()==0 else self.graph_cpp, node)
            return
        
        # 2. If Code Node -> Open Editor
        if hasattr(node, 'get_property') and node.has_property('code_content'):
            lang = 'cpp' if 'cpp' in node.type_ else 'python'
            dlg = AdvancedCodeDialog(node.get_property('code_content'), language=lang, parent=self)
            
            if dlg.exec():
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
                if data['save']: 
                    self._save_group_to_library(graph, group, data['name'])
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
            
            # Temporarily show hidden nodes to capture them
            for nid in internal_ids:
                if nid in all_nodes:
                    n = all_nodes[nid]
                    nodes_to_save.append(n)
                    if hasattr(n, 'view') and not n.view.isVisible():
                        n.view.setVisible(True)
                        hidden_temp.append(n)
                        
            clipboard_data = graph.copy_nodes(nodes_to_save)
            
            # Hide them again
            for n in hidden_temp: n.view.setVisible(False)
            
            filename = "".join([c for c in f"{name}.json" if c.isalpha() or c.isdigit() or c in (' ', '.', '_')]).strip()
            path = os.path.join(self.ui.user_lib_path, filename)
            
            with open(path, 'w', encoding='utf-8') as f: 
                json.dump(clipboard_data, f, indent=4)
                
            self.system_log(f"Saved: {filename}")
            self.ui.refresh_palette()
        except Exception as e: 
            self.system_log(f"Save error: {e}")
            traceback.print_exc()

    def on_deploy_run(self):
        self.ui.console_sys.clear()
        self.ui.console_ros.clear()
        graph = self.graph_py if self.ui.tabs.currentIndex() == 0 else self.graph_cpp
        has_nodes = False
        warnings = []
        
        for node in graph.all_nodes():
            if node.type_ == 'nodeGraphQt.nodes.BackdropNode': continue
            has_nodes = True
            
            connected = any([p.connected_ports() for p in node.input_ports() + node.output_ports()])
            if not connected and node.type_ != 'ros.nodes.RosGroup':
                warnings.append(f"Warning: Node '{node.name()}' disconnected.")
        
        if not has_nodes: 
            return self.system_log("Error: Empty graph.")
        if warnings: 
            for w in warnings: self.system_log(w)

        scripts = GraphCompiler(graph).compile()
        if not scripts and not warnings: 
            return self.system_log("Warning: No code generated.")

        self.ui.actions['run'].setEnabled(False)
        self.ui.actions['stop'].setEnabled(True)
        
        self.deploy_worker = DeployWorker(scripts, self.container_manager)
        self.deploy_worker.log_signal.connect(self.system_log)
        self.deploy_worker.finished_signal.connect(self.start_monitoring)
        self.deploy_worker.start()

    def start_monitoring(self):
        self.log_worker = LogMonitorWorker(self.container_manager)
        self.log_worker.new_log_line.connect(self.ros_log_line)
        self.log_worker.start()

    def ros_log_line(self, line):
        clean = re.sub(r'\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])', '', line).strip()
        self.ui.console_ros.moveCursor(QtGui.QTextCursor.End)
        self.ui.console_ros.insertPlainText(clean + "\n")
        self.ui.console_ros.verticalScrollBar().setValue(self.ui.console_ros.verticalScrollBar().maximum())
        
        # Route data to monitor nodes
        match = re.search(r'\[([a-zA-Z0-9_\-\.]+)\]:', clean)
        if match:
            all_matches = re.findall(r'\[([a-zA-Z0-9_\-\.]+)\]', clean.split(':')[0])
            if all_matches:
                name = all_matches[-1]
                for g in [self.graph_py, self.graph_cpp]:
                    for node in g.all_nodes():
                        if node.get_property('node_name') == name:
                            for out in node.output_ports():
                                for p in out.connected_ports():
                                    if p.node().type_ == 'nodes.utility.MonitorNode': 
                                        p.node().update_data(clean)

    def on_save_project(self):
        f, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Save", "", "JSON (*.json)")
        if f: self.project_manager.save_project(f)
        
    def on_load_project(self):
        f, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Open", "", "JSON (*.json)")
        if f: self.project_manager.load_project(f)
        
    def on_stop(self):
        if self.log_worker: self.log_worker.stop()
        if self.container_manager: 
            try: self.container_manager.container.stop()
            except: pass
        self.ui.actions['run'].setEnabled(True)
        self.ui.actions['stop'].setEnabled(False)
        self.system_log("Stopped.")
        
    def on_clear(self):
        graph = self.graph_py if self.ui.tabs.currentIndex() == 0 else self.graph_cpp
        graph.clear_session()
        
    def keyPressEvent(self, event):
        if event.modifiers() & QtCore.Qt.ControlModifier and event.key() == QtCore.Qt.Key_G: 
            self.on_group_nodes()
        elif event.key() in [QtCore.Qt.Key_Delete, QtCore.Qt.Key_Backspace]:
            g = self.graph_py if self.ui.tabs.currentIndex() == 0 else self.graph_cpp
            g.delete_nodes(g.selected_nodes())
        else: 
            super().keyPressEvent(event)

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
                if "Twist" in source.name(): 
                    color = TYPE_COLORS['geometry_msgs/Twist']
                    label="Twist"
                elif "String" in source.name(): 
                    color = TYPE_COLORS['std_msgs/String']
                    label="String"

            node.set_color(*color)
            node.set_property('name', f"Monitor ({label})")

    def on_port_disconnected(self, port_in, port_out):
        node = port_in.node()
        if node.type_ == 'nodes.utility.MonitorNode':
            if not port_in.connected_ports():
                node.set_color(*TYPE_COLORS["DEFAULT"])
                node.set_property('name', "Monitor (No Data)")

if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    window = RosOrchestratorApp()
    window.show()
    sys.exit(app.exec())