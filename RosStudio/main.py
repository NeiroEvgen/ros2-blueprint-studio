import sys
import logging
import traceback
import re
import json
import os
from PySide6 import QtWidgets, QtCore, QtGui
from NodeGraphQt import NodeGraph

try:
    from docker_manager import RosContainerManager
    from ui_manager import UiManager
    from project_manager import ProjectManager
    from graph_compiler import GraphCompiler
    
    from nodes.custom_nodes import (
        RosPyPublisherNode, RosPySubscriberNode, RosPyCustomNode,
        RosCppPublisherNode, RosCppSubscriberNode, RosCppCustomNode,
        RosGroupNode
    )
    from nodes.monitor_node import MonitorNode

except ImportError as e:
    print(f"ОШИБКА ИМПОРТА: {e}")
    sys.exit(1)

logging.basicConfig(level=logging.INFO)

# --- WORKERS (Без изменений) ---
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
        except Exception as e: self.log_signal.emit(f"ERR: {e}"); traceback.print_exc()

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
        self.setWindowTitle("Настройки Группы"); self.resize(400, 300)
        layout = QtWidgets.QVBoxLayout(self)
        layout.addWidget(QtWidgets.QLabel("Имя группы:"))
        self.name_edit = QtWidgets.QLineEdit("MySubGraph")
        layout.addWidget(self.name_edit)
        layout.addWidget(QtWidgets.QLabel("Цвет:"))
        self.color_btn = QtWidgets.QPushButton("Выбрать цвет")
        self.color_btn.clicked.connect(self.choose_color)
        self.color_btn.setStyleSheet("background-color: #555; color: white; padding: 10px;")
        layout.addWidget(self.color_btn)
        self.selected_color = (50, 50, 50) 
        layout.addWidget(QtWidgets.QLabel(""))
        self.save_chk = QtWidgets.QCheckBox("Сохранить в библиотеку (User Blueprints)")
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

# --- ГЛАВНОЕ ПРИЛОЖЕНИЕ ---
class RosOrchestratorApp(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.ui = UiManager(self)
        self.ui.setup_ui()
        
        # Создаем графы
        self.graph_py = NodeGraph()
        self.graph_cpp = NodeGraph()
        
        node_classes = [RosPyPublisherNode, RosPySubscriberNode, RosPyCustomNode, 
                        RosCppPublisherNode, RosCppSubscriberNode, RosCppCustomNode, MonitorNode, RosGroupNode]
        
        self.graph_py.register_nodes(node_classes)
        self.graph_cpp.register_nodes(node_classes)
        
        self.graph_py.node_double_clicked.connect(self.on_node_double_click)
        self.graph_cpp.node_double_clicked.connect(self.on_node_double_click)
        
        self.ui.tabs.addTab(self.graph_py.widget, "Python Blueprints")
        self.ui.tabs.addTab(self.graph_cpp.widget, "C++ Blueprints")

        # === НАСТРОЙКА DRAG & DROP ДЛЯ ГРАФОВ ===
        self._setup_drag_and_drop(self.graph_py)
        self._setup_drag_and_drop(self.graph_cpp)

        self.project_manager = ProjectManager(self.graph_py, self.graph_cpp)
        self.container_manager = None
        try: self.container_manager = RosContainerManager()
        except Exception as e: print(f"Docker Error: {e}")

        self.ui.create_palette(self.add_from_palette)

        self.ui.actions['save'].triggered.connect(self.on_save_project)
        self.ui.actions['open'].triggered.connect(self.on_load_project)
        self.ui.actions['run'].triggered.connect(self.on_deploy_run)
        self.ui.actions['stop'].triggered.connect(self.on_stop)
        self.ui.actions['clear'].triggered.connect(self.on_clear)
        self.ui.actions['group'].triggered.connect(self.on_group_nodes)

    def system_log(self, msg): 
        if hasattr(self, 'ui') and self.ui and self.ui.console_sys:
            self.ui.console_sys.append(f"> {msg}")
        else: print(f"> {msg}")

    # === DRAG AND DROP ЛОГИКА ===
    def _setup_drag_and_drop(self, graph):
        """
        Переопределяем события виджета графа, чтобы он принимал данные из списка.
        """
        viewer = graph.widget
        viewer.setAcceptDrops(True)

        original_drag_enter = viewer.dragEnterEvent
        original_drag_move = viewer.dragMoveEvent
        original_drop = viewer.dropEvent

        def dragEnterEvent(event):
            if event.mimeData().hasText(): event.acceptProposedAction()
            else: original_drag_enter(event)

        def dragMoveEvent(event):
            if event.mimeData().hasText(): event.acceptProposedAction()
            else: original_drag_move(event)

        def dropEvent(event):
            if event.mimeData().hasText():
                code = event.mimeData().text()
                mouse_pos = event.position().toPoint()
                scene_pos = viewer.mapToScene(mouse_pos)
                
                # Спавним ноду с небольшим смещением, чтобы курсор был в центре (примерно)
                final_x = scene_pos.x() - 100
                final_y = scene_pos.y() - 30
                self.add_from_palette(None, code=code, pos=[final_x, final_y])
                
                event.acceptProposedAction()
            else:
                original_drop(event)

        viewer.dragEnterEvent = dragEnterEvent
        viewer.dragMoveEvent = dragMoveEvent
        viewer.dropEvent = dropEvent

    # === ИЗМЕНЕННЫЙ МЕТОД ДОБАВЛЕНИЯ ===
    def add_from_palette(self, item, code=None, pos=[0,0]):
        if item: code = item.data(QtCore.Qt.UserRole)
        if not code: return
        
        curr_idx = self.ui.tabs.currentIndex()
        graph = self.graph_py if curr_idx == 0 else self.graph_cpp

        if code.startswith("USER_LIB:"):
            filename = code.split(":", 1)[1]
            path = os.path.join(self.ui.user_lib_path, filename)
            if os.path.exists(path):
                try:
                    with open(path, 'r', encoding='utf-8') as f: data = json.load(f)
                    graph.paste_nodes(data)
                    self.system_log(f"Шаблон '{filename}' добавлен.")
                except Exception as e: self.system_log(f"Ошибка: {e}")
            return

        if curr_idx == 0 and "CPP" in str(code): return self.system_log("Ошибка: Вы на вкладке Python!")
        if curr_idx == 1 and "PY" in str(code): return self.system_log("Ошибка: Вы на вкладке C++!")
        
        node_map = {
            "PY_PUB": 'ros.nodes.python.RosPyPublisherNode',
            "PY_SUB": 'ros.nodes.python.RosPySubscriberNode',
            "PY_CUSTOM": 'ros.nodes.python.RosPyCustomNode',
            "CPP_PUB": 'ros.nodes.cpp.RosCppPublisherNode',
            "CPP_SUB": 'ros.nodes.cpp.RosCppSubscriberNode',
            "CPP_CUSTOM": 'ros.nodes.cpp.RosCppCustomNode',
            "MONITOR": 'nodes.utility.MonitorNode'
        }
        if code in node_map: 
            graph.create_node(node_map[code], pos=pos)

    # === ЛОГИКА ГРУППИРОВКИ ===
    def on_group_nodes(self):
        graph = self.graph_py if self.ui.tabs.currentIndex() == 0 else self.graph_cpp
        selected = graph.selected_nodes()
        if not selected: return self.system_log("Выберите ноды для группировки!")
        try:
            dlg = GroupSettingsDialog(self)
            if dlg.exec() != QtWidgets.QDialog.Accepted: return
            data = dlg.get_data()
            center_x = sum([n.pos()[0] for n in selected]) / len(selected)
            center_y = sum([n.pos()[1] for n in selected]) / len(selected)
            group_type = RosGroupNode.__identifier__ + '.' + RosGroupNode.NODE_NAME
            group_node = graph.create_node(group_type, pos=[center_x, center_y])
            if not group_node: return self.system_log(f"Ошибка создания группы")
            for node in selected: 
                group_node.add_node(node)
                # Корректируем позицию внутри группы (делаем локальной)
                node.set_pos(node.pos()[0] - center_x, node.pos()[1] - center_y)
            group_node.set_name(data['name'])
            group_node.set_color(data['color'][0], data['color'][1], data['color'][2])
            graph.clear_selection(); group_node.set_selected(True)
            if data['save']: self._save_group_to_library(graph, group_node, data['name'])
            self.system_log(f"Группа '{data['name']}' создана.")
        except Exception as e: self.system_log(f"Ошибка группировки: {e}"); traceback.print_exc()

    def _save_group_to_library(self, graph, group_node, name):
        try:
            clipboard_data = graph.copy_nodes([group_node])
            filename = "".join([c for c in f"{name}.json" if c.isalpha() or c.isdigit() or c in (' ', '.', '_')]).strip()
            path = os.path.join(self.ui.user_lib_path, filename)
            with open(path, 'w', encoding='utf-8') as f: json.dump(clipboard_data, f, indent=4)
            self.system_log(f"Сохранено: {filename}"); self.ui.refresh_palette()
        except Exception as e: self.system_log(f"Ошибка сохранения: {e}")

    def ros_log_line(self, line):
        if not line.strip(): return
        match = re.search(r'\[([a-zA-Z0-9_]+)\]:', line)
        if match:
            name = match.group(1)
            for g in [self.graph_py, self.graph_cpp]:
                for node in g.all_nodes():
                    if hasattr(node, 'get_property') and node.get_property('node_name') == name:
                        out = node.get_output('out')
                        if out:
                            for p in out.connected_ports():
                                if p.node().type_ == 'nodes.utility.MonitorNode': p.node().update_data(line)
        self.ui.console_ros.moveCursor(QtGui.QTextCursor.End); self.ui.console_ros.insertPlainText(line + "\n")
        self.ui.console_ros.verticalScrollBar().setValue(self.ui.console_ros.verticalScrollBar().maximum())

    def on_node_double_click(self, node):
        if hasattr(node, 'get_property') and node.get_property('code_content'):
            code = node.get_property('code_content')
            dlg = CodeEditorDialog(code, self)
            if dlg.exec():
                node.set_property('code_content', dlg.get_code())
                self.system_log(f"Код ноды {node.name()} обновлен.")

    def on_save_project(self):
        f, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Save", "", "JSON (*.json)"); 
        if f: self.project_manager.save_project(f)
    def on_load_project(self):
        f, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Open", "", "JSON (*.json)"); 
        if f: self.project_manager.load_project(f)
    def on_deploy_run(self):
        self.ui.console_sys.clear(); self.ui.console_ros.clear()
        graph = self.graph_py if self.ui.tabs.currentIndex() == 0 else self.graph_cpp
        scripts = GraphCompiler(graph).compile()
        if not scripts: return self.system_log("Пусто.")
        self.ui.actions['run'].setEnabled(False); self.ui.actions['stop'].setEnabled(True)
        self.deploy_worker = DeployWorker(scripts, self.container_manager)
        self.deploy_worker.log_signal.connect(self.system_log)
        self.deploy_worker.finished_signal.connect(self.start_monitoring)
        self.deploy_worker.start()
    def start_monitoring(self):
        self.log_worker = LogMonitorWorker(self.container_manager)
        self.log_worker.new_log_line.connect(self.ros_log_line); self.log_worker.start()
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
            graph = self.graph_py if self.ui.tabs.currentIndex() == 0 else self.graph_cpp
            graph.delete_nodes(graph.selected_nodes())
        else: super().keyPressEvent(event)

if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    window = RosOrchestratorApp()
    window.show()
    sys.exit(app.exec())