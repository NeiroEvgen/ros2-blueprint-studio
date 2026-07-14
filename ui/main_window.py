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
from core.container_config import ContainerConfigStore, safe_group_name, resolved_env
from core.project_importer import import_ros2_package
from ui.container_settings import ContainerSettingsPanel

from compilers.graph_compiler import GraphCompiler

# === UI MODULES ===
from ui.ui_manager import UiManager
from ui.dashboard import ConsoleDashboard
from ui.docker_panel import DockerPanel
from ui.library_manager import LibraryManager
from ui.graph_setup import setup_graphs, TYPE_COLORS, ALL_NODE_CLASSES
from ui.properties_window import PropertiesWindow
# === NODES ===
from nodes.base import MSG_COLORS
from nodes.group_logic import toggle_group_visibility

class RosVisualRunner(QtWidgets.QMainWindow):
    def __init__(self, project_path=None):
        super().__init__()
        self.current_project_path = project_path
        self.auto_route_enabled = False  # По умолчанию выключено
        self.route_settings = {
            "color": "#ffffff",
            "msg_type": "std_msgs/msg/String",
            "protocol": "RMW_IMPLEMENTATION_DEFAULT"
        }
        
        # 1. UI Setup
        self.ui = UiManager(self)
        self.ui.setup_ui()
        self._palette_drop = PaletteDropFilter()
        self.ui.node_palette.installEventFilter(self._palette_drop)
        self._palette_drop.node_dropped.connect(self._on_node_dropped_to_palette)
        self.dashboard = ConsoleDashboard()
        
        # 4. Docker Connect
        self.container_manager = None
        try:
            self.container_manager = RosContainerManager()
        except Exception: 
            self.system_log("WARNING: Docker not found or not running.")

        self.docker_panel = DockerPanel(self.container_manager) if self.container_manager else QtWidgets.QLabel("Docker not available")
        self.library_manager = LibraryManager(self.container_manager) if self.container_manager else QtWidgets.QLabel("Docker not available")
        
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
                    tab_widget.addTab(self.docker_panel, "Docker")
                    tab_widget.addTab(self.library_manager, "Libraries")
                    self.container_settings = ContainerSettingsPanel(
                        get_docker_manager=lambda: self.container_manager)
                    tab_widget.addTab(self.container_settings, "Containers")
        # 2. Graph Setup (delegated to ui/graph_setup.py)
        self.graph_py, self.graph_cpp, self.f_py, self.f_cpp = setup_graphs(
            self.ui.tabs, self.resolve_node_type, self.on_graph_delete
        )

        # 2.1 Properties Window
        self.properties_win = PropertiesWindow(self.graph_py, self.graph_cpp, self)
        self.properties_win.hide()

        self.setup_graph_signals()

        # 3. Core Managers
        self.project_manager = ProjectManager(self.graph_py, self.graph_cpp)
        self.watcher = FileWatcher()
        self.watcher.file_changed.connect(self.on_file_changed_externally)

        # 5. Connect Actions
        self.connect_actions()       # Навигация
        self.current_subgraph = None
        self.navigation_stack = [{"name": "Root", "node": None}]

        # 6. Init Logic
        if self.current_project_path:
            # Delay slightly to ensure UI is ready
            QtCore.QTimer.singleShot(100, lambda: self.on_load_project_init(self.current_project_path))
        else:
            self.ui.rebuild_palette(ALL_NODE_CLASSES, "python")
            self.ui.set_visible_graph("python")

    def _refresh_pipe_visibility(self, graph):
        return
        """Прячет пайпы, у которых хотя бы один конец на невидимой ноде."""
        try:
            scene = graph.scene()
            from NodeGraphQt.qgraphics.pipe import PipeItem
        except Exception:
            return
        for item in scene.items():
            if not isinstance(item, PipeItem):
                continue
            src = getattr(item, 'input_port', None)
            dst = getattr(item, 'output_port', None)
            visible = True
            for port_item in (src, dst):
                if port_item is None:
                    continue
                # у порта есть ссылка на ноду-владельца
                node_item = getattr(port_item, 'node', None)
                if node_item is not None and not node_item.isVisible():
                    visible = False
                    break
            item.setVisible(visible)

    def on_graph_delete(self, graph, with_contents):
        """Delete = распустить группу (дети наружу). Ctrl+Delete = удалить с содержимым.
        Для обычных нод — просто удаляем."""
        selected = graph.selected_nodes()
        if not selected:
            return

        from nodes.group_logic import dissolve_group
        for node in list(selected):
            is_group = node.type_ in ('ros.nodes.RosGroup', 'ros.nodes.RosGroupNode')
            try:
                if is_group:
                    if with_contents:
                        # Ctrl+Del: удаляем детей и терминалы вместе с группой
                        gid = node.get_property('group_uid')
                        for child in list(graph.all_nodes()):
                            if child == node:
                                continue
                            if (child.has_property('parent_group_id')
                                    and child.get_property('parent_group_id') == gid):
                                graph.delete_node(child)
                        graph.delete_node(node)
                        self.system_log(f"Group + contents deleted: {node.name()}")
                    else:
                        # Del: распускаем — дети наружу, терминалы удаляются
                        dissolve_group(graph, node)
                        graph.delete_node(node)
                        self.system_log(f"Group dissolved: {node.name()}")
                else:
                    # обычная нода
                    graph.delete_node(node)
            except Exception as e:
                self.system_log(f"Delete error: {e}")

        self._refresh_container_counts() if hasattr(self, '_refresh_container_counts') else None

    def enter_subgraph(self, group_node):
        self.current_subgraph = group_node
        name = group_node.get_property('node_name')
        self.navigation_stack.append({"name": name, "node": group_node})
        self.ui.update_breadcrumbs(self.navigation_stack)
        
        # Скрываем всё, кроме детей этой группы
        gid = group_node.get_property('group_uid')
        graph = group_node.graph
        for node in graph.all_nodes():
            if node == group_node:
                self._set_node_visible(node, False)
            elif node.has_property('parent_group_id') and node.get_property('parent_group_id') == gid:
                self._set_node_visible(node, True)
            else:
                self._set_node_visible(node, False)
        
        #self._refresh_pipe_visibility(graph)  # см. ниже про граф
        self.system_log(f"Entered subgraph: {name}")

    def exit_subgraph(self):
        if len(self.navigation_stack) <= 1:
            return
            
        self.navigation_stack.pop()
        self.ui.update_breadcrumbs(self.navigation_stack)
        
        target = self.navigation_stack[-1]
        self.current_subgraph = target["node"]
        
        graph = self.graph_py if self.ui.tabs.currentIndex() == 0 else self.graph_cpp
        
        if self.current_subgraph is None:
            # Мы в Root: показываем только то, что НЕ вложено в группу.
            for node in graph.all_nodes():
                # Ребёнок группы определяется по parent_group_id, а не по group_uid
                # (у самой группы group_uid тоже задан — её прятать нельзя).
                parent = node.get_property('parent_group_id') \
                    if node.has_property('parent_group_id') else ""
                is_child = bool(parent)
                node.view.setVisible(not is_child)
        else:
            # Мы вышли в родительский сабграф
            parent_gid = self.current_subgraph.get_property('group_uid')
            for node in graph.all_nodes():
                if node == self.current_subgraph:
                    node.view.setVisible(False)
                elif node.has_property('parent_group_id') and node.get_property('parent_group_id') == parent_gid:
                    node.view.setVisible(True)
                else:
                    node.view.setVisible(False)
            
        self.system_log("Exited to " + target["name"])

    def setup_graph_signals(self):
        """Подключает сигналы графа для открытия редактора и настроек"""
        for graph in [self.graph_py, self.graph_cpp]:
            graph.node_double_clicked.connect(self.on_node_double_clicked)
            self.setup_graph_context_menus(graph)

    def _set_node_visible(self, node, visible):
        """Прячет/показывает ноду вместе с её связями."""
        if hasattr(node, 'view'):
            node.view.setVisible(visible)
        try:
            for port in node.input_ports() + node.output_ports():
                pv = port.view  # PortItem (QGraphicsItem)
                # connected_pipes — dict или list в зависимости от версии
                pipes = getattr(pv, 'connected_pipes', None)
                if pipes is None:
                    continue
                iterable = pipes.values() if isinstance(pipes, dict) else pipes
                for pipe in iterable:
                    pipe.setVisible(visible)
        except Exception as e:
            print(f"pipe visibility skip: {e}")

    def on_node_double_clicked(self, node):
        """
        Двойной клик: группа -> внутрь; обычная нода -> код.
        Ctrl + двойной клик -> окно свойств.
        """
        if not node:
            return

        modifiers = QtWidgets.QApplication.keyboardModifiers()
        ctrl_held = bool(modifiers & QtCore.Qt.ControlModifier)

        is_group = node.type_ in ('ros.nodes.RosGroup', 'ros.nodes.RosGroupNode') \
            or node.has_property('internal_nodes')

        # Ctrl + двойной клик -> только свойства
        if ctrl_held:
            self.properties_win.set_node(node)
            self.properties_win.show()
            self.system_log(f"Opened properties for: {node.name()}")
            return

        # Обычный двойной клик по группе -> заходим внутрь
        if is_group:
            self.enter_subgraph(node)
            return

        # Обычный двойной клик по обычной ноде -> только код
        if node.has_property('code_content'):
            if not self.current_project_path:
                self.system_log("Открой/сохрани проект перед редактированием кода ноды.")
                return
            
            self.project_manager.open_node_in_editor(node, self.current_project_path)
            self.watcher.start_watching(os.path.join(self.current_project_path, "src"))
            self.system_log(f"Opened code for: {node.name()}")
    def connect_actions(self):
        """Восстанавливает связи кнопок управления"""
        self.ui.actions['run'].clicked.connect(self.on_run_project)
        self.ui.actions['stop'].clicked.connect(self.on_stop_project)
        self.ui.actions['clear'].clicked.connect(self.on_clear_graph)
        self.ui.actions['export_docker'].clicked.connect(self.on_export_docker)
        if 'import_pkg' in self.ui.actions:
            self.ui.actions['import_pkg'].clicked.connect(self.on_import_foreign_project)
        self.ui.actions['visualize'].clicked.connect(self.on_visualize)
        self.ui.actions['export_palette'].clicked.connect(self.on_export_palette)
        self.ui.actions['import_palette'].clicked.connect(self.on_import_palette)



    def on_visualize(self):
        mode = self.ui.actions['viz_mode'].currentText()

        if mode.startswith("X11"):
            # Классика: просто напоминание, RViz-нода сама откроет окно при Run
            self.system_log("X11 mode: убедись, что XLaunch запущен "
                            "(Multiple windows, Disable access control).")
            return

        # === Foxglove ===
        if not self.container_manager or not self.container_manager.container:
            self.system_log("Error: сессия не запущена. Нажми Run, потом Visualize.")
            return
        try:
            self.container_manager.start_foxglove_bridge(self.system_log)
        except Exception as e:
            self.system_log(f"Foxglove bridge error: {e}")
            return

        # Открываем Foxglove в браузере с автоподключением к ws://localhost:8765
        import webbrowser
        url = ("https://app.foxglove.dev/?ds=foxglove-websocket"
               "&ds.url=ws%3A%2F%2Flocalhost%3A8765")
        webbrowser.open(url)
        self.system_log("Foxglove открыт в браузере (подключение к ws://localhost:8765)")

    def on_clear_graph(self):
        graph = self.graph_py if self.ui.tabs.currentIndex() == 0 else self.graph_cpp
        graph.clear_session()
        self.system_log("Graph cleared.")

    def on_run_project(self):
        if not self.current_project_path:
            self.system_log("Error: No project loaded.")
            return
        if not self.container_manager:
            self.system_log("Error: Docker is not available. Cannot run.")
            return

        # 1. Сначала компилируем/сохраняем граф в файлы проекта
        self.system_log("Saving & generating project files...")
        ok = self.project_manager.save_project(self.current_project_path)
        if not ok:
            self.system_log("Error: Failed to generate project files. Aborting run.")
            return
        self.watcher.start_watching(os.path.join(self.current_project_path, "src"))

        self.ui.actions['run'].setEnabled(False)
        self.ui.actions['stop'].setEnabled(True)
        self.system_log("Starting project...")

        
        try:
            store = ContainerConfigStore(self.current_project_path)
            self.container_manager.extra_env = resolved_env(store.get("main"))
        except Exception as e:
            self.system_log(f"WARN: env config skipped: {e}")

        # 2. Запускаем деплой в отдельном потоке
        self.deploy_worker = DeployWorker(self.container_manager, self.current_project_path)
        self.deploy_worker.sys_signal.connect(self.system_log)
        self.deploy_worker.ros_signal.connect(self.ros_log)
        self.deploy_worker.finished_signal.connect(self.on_deploy_finished)
        self.deploy_worker.start()

    def on_deploy_finished(self):
        """Вызывается, когда DeployWorker завершил работу."""
        self.ui.actions['run'].setEnabled(True)
        self.ui.actions['stop'].setEnabled(False)
        self.system_log("--- DEPLOY FINISHED ---")

    def on_stop_project(self):
        # Останавливаем worker и контейнер, если они запущены
        if hasattr(self, 'deploy_worker') and self.deploy_worker is not None:
            self.deploy_worker.stop()
        self.ui.actions['run'].setEnabled(True)
        self.ui.actions['stop'].setEnabled(False)
        self.system_log("Project stopped.")

    def on_export_docker(self):
        if not self.current_project_path:
            self.system_log("Error: No project loaded.")
            return
        # Сначала сгенерируем актуальные файлы из графа
        self.system_log("Generating fresh project files before export...")
        if not self.project_manager.save_project(self.current_project_path):
            self.system_log("Error: project generation failed. Export aborted.")
            return
        try:
            self.system_log("Exporting portable Docker package...")
            zip_path = ExportManager.export_to_portable_package(self.current_project_path)
            self.system_log(f"Export complete: {zip_path}")
            QtWidgets.QMessageBox.information(
                self, "Export Complete",
                f"Portable Docker package created:\n{zip_path}"
            )
        except Exception as e:
            self.system_log(f"Export failed: {e}")
            traceback.print_exc()

    def on_load_project_init(self, path):
        """Логика загрузки проекта с адаптацией интерфейса"""
        self.current_project_path = path
        p_type = WorkspaceManager.get_project_type(path)
        self.ui.set_visible_graph(p_type)
        self.ui.rebuild_palette(ALL_NODE_CLASSES, p_type)
        self.project_manager.load_project(path)
        self.watcher.start_watching(os.path.join(path, "src"))
        if hasattr(self, 'container_settings'):
            self.container_settings.set_project(path)
            self._refresh_container_counts()
        self.system_log(f"Project loaded: {path} ({p_type.upper()})")

    def system_log(self, text):
        self.ui.console_sys.append(text)

    def ros_log(self, text):
        self.dashboard.process_log(text)

    def on_file_changed_externally(self, path, content, ports):
        """Файл ноды изменён снаружи (редактор) — синкаем код и порты в ноду."""
        try:
            self.project_manager.sync_node_from_file(
                path, content, ports,
                graphs=[self.graph_py, self.graph_cpp],
                log_callback=self.system_log)
        except Exception as e:
            self.system_log(f"Sync error: {e}")
    # === CORE LOGIC ===

    def resolve_node_type(self, code):
        """Determines node type from dropped mime data"""
        if code.startswith("USER_NODE:"):
            self.add_user_node_logic(code)
            return None
        if code and "." in code: return code # Direct class path
        return None

    def add_user_node_logic(self, code):
        """Логика восстановления кастомной ноды из палитры"""
        try:
            path_parts = code.replace("USER_NODE:", "").split("/")
            palette_name = path_parts[0]
            node_folder = path_parts[1]

            base_dir = os.path.join("nodes", "user_palettes", palette_name, node_folder)
            meta_path = os.path.join(base_dir, "node.yaml")

            if not os.path.exists(meta_path): return

            import yaml
            with open(meta_path, 'r', encoding='utf-8') as f:
                meta = yaml.safe_load(f)

            graph = self.graph_py if self.ui.tabs.currentIndex() == 0 else self.graph_cpp

            # === ФИКС 2: уникальное имя (Name, Name_1, Name_2...) ===
            base_name = meta['name']
            existing = {n.name() for n in graph.all_nodes()}
            new_name = base_name
            i = 1
            while new_name in existing:
                new_name = f"{base_name}_{i}"
                i += 1

            node = graph.create_node(meta['type'], name=new_name)

            # Восстанавливаем свойства
            for key, val in meta.get('properties', {}).items():
                try:
                    node.set_property(key, val)
                except Exception:
                    # свойства, которых нет у базового класса, создаём
                    try:
                        node.create_property(key, val)
                    except Exception:
                        pass

            # === ФИКС 1: восстанавливаем кастомные порты ===
            have_in = set(node.inputs().keys())
            have_out = set(node.outputs().keys())
            for p in meta.get('ports', []):
                pname, pdir = p.get('name'), p.get('type')
                if not pname:
                    continue
                try:
                    if pdir == 'in' and pname not in have_in:
                        node.add_input(pname, multi_input=True, display_name=True)
                    elif pdir == 'out' and pname not in have_out:
                        node.add_output(pname, multi_output=True, display_name=True)
                except Exception as pe:
                    self.system_log(f"WARN: port '{pname}' skipped: {pe}")

            # Восстанавливаем код из файла (Закон удобства)
            ext = ".cpp" if "cpp" in meta['type'].lower() else ".py"
            code_path = os.path.join(base_dir, f"logic{ext}")
            if os.path.exists(code_path):
                with open(code_path, 'r', encoding='utf-8') as f:
                    node.set_property("code_content", f.read())

            self.system_log(f"Successfully imported user node: {new_name}")
        except Exception as e:
            self.system_log(f"Error importing user node: {e}")
            
    def on_save_node_to_palette(self, node):
        palette_name, ok = QtWidgets.QInputDialog.getText(self, "Save to Palette", "Enter Palette Name:")
        if not (ok and palette_name): return

        node_name = node.name()
        # Путь: nodes/user_palettes/ИмяПалитры/ИмяНоды/
        base_dir = os.path.join("nodes", "user_palettes", palette_name, node_name)
        os.makedirs(base_dir, exist_ok=True)

        # 1. Сохраняем метаданные в YAML (свойства, порты)
        meta = {
            "name": node_name,
            "type": node.type_,
            "properties": node.model.custom_properties,
            "ports": ([{"name": p.name(), "type": "in"} for p in node.input_ports()]
                      + [{"name": p.name(), "type": "out"} for p in node.output_ports()])
        }
        
        import yaml
        with open(os.path.join(base_dir, "node.yaml"), 'w', encoding='utf-8') as f:
            yaml.dump(meta, f, sort_keys=False)

        # 2. Сохраняем код в его родном расширении (Закон удобства)
        code = node.get_property("code_content")
        ext = ".cpp" if "cpp" in node.type_.lower() else ".py"
        with open(os.path.join(base_dir, f"logic{ext}"), 'w', encoding='utf-8') as f:
            f.write(code if code else "")

        self.system_log(f" Node '{node_name}' saved to palette '{palette_name}' as native files.")
        # Обновляем UI палитры
        p_type = WorkspaceManager.get_project_type(self.current_project_path) if self.current_project_path else "python"
        self.ui.rebuild_palette(ALL_NODE_CLASSES, p_type)

    def _on_node_dropped_to_palette(self, node_id):
        """Нода перетащена Alt+drag'ом на палитру — сохраняем её как кастомную."""
        for g in (self.graph_py, self.graph_cpp):
            for n in g.all_nodes():
                if n.id == node_id:
                    self.on_save_node_to_palette(n)
                    return
                
    def on_export_palette(self):
        from core.palette_share import export_palette, PALETTES_ROOT
        # Какие палитры есть
        palettes = []
        if os.path.isdir(PALETTES_ROOT):
            palettes = [d for d in os.listdir(PALETTES_ROOT)
                        if os.path.isdir(os.path.join(PALETTES_ROOT, d))]
        if not palettes:
            self.system_log("No user palettes to export. Save a node to a palette first.")
            return
        name, ok = QtWidgets.QInputDialog.getItem(
            self, "Export Palette", "Choose palette:", palettes, 0, False)
        if not (ok and name):
            return
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Export palette as", f"{name}.bppalette",
            "Blueprint Palette (*.bppalette)")
        if not path:
            return
        try:
            out = export_palette(name, path)
            self.system_log(f"📤 Palette exported: {out}")
        except Exception as e:
            self.system_log(f"Export failed: {e}")

    def on_import_palette(self):
        from core.palette_share import import_palette
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Import palette", "", "Blueprint Palette (*.bppalette *.zip)")
        if not path:
            return
        try:
            pname, nodes = import_palette(path)
        except FileExistsError as e:
            ans = QtWidgets.QMessageBox.question(
                self, "Palette exists",
                f"Palette '{e}' already exists. Overwrite?")
            if ans != QtWidgets.QMessageBox.Yes:
                return
            try:
                pname, nodes = import_palette(path, overwrite=True)
            except Exception as e2:
                self.system_log(f"Import failed: {e2}")
                return
        except Exception as e:
            self.system_log(f"Import failed: {e}")
            return

        # Обновляем палитру в UI
        p_type = WorkspaceManager.get_project_type(self.current_project_path) \
            if self.current_project_path else "python"
        self.ui.rebuild_palette(ALL_NODE_CLASSES, p_type)
        self.system_log(f"📥 Imported palette '{pname}': {len(nodes)} node(s): {', '.join(nodes)}")

    def on_assign_container(self, graph, *args, **kwargs):
        """Назначает выбранным нодам деплой-группу (свойство container_group)."""
        selected = graph.selected_nodes()
        if not selected:
            self.system_log("Select node(s) first.")
            return

        existing = ["main"]
        if hasattr(self, 'container_settings') and self.container_settings.store:
            existing = self.container_settings.group_names()

        name, ok = QtWidgets.QInputDialog.getItem(
            self, "Assign to container",
            "Container group (введи новое имя или выбери):",
            existing, 0, True)
        if not (ok and name):
            return
        gname = safe_group_name(name)

        for node in selected:
            if not node.has_property('container_group'):
                try:
                    node.create_property('container_group', gname)
                except Exception:
                    pass
            node.set_property('container_group', gname)

        # Если группа новая — создаём ей дефолтный конфиг
        if hasattr(self, 'container_settings') and self.container_settings.store \
                and gname not in self.container_settings.group_names():
            from core.container_config import make_config
            self.container_settings.store.save(make_config(gname))
            self.container_settings.refresh_groups()

        self._refresh_container_counts()
        self.system_log(f"Assigned {len(selected)} node(s) -> container '{gname}'")

    def _refresh_container_counts(self):
        """Счётчик нод по группам в UI (roadmap 2.4)."""
        if not hasattr(self, 'container_settings') or not self.container_settings.store:
            return
        counts = {}
        for graph in [self.graph_py, self.graph_cpp]:
            for node in graph.all_nodes():
                g = "main"
                if node.has_property('container_group'):
                    g = node.get_property('container_group') or "main"
                counts[g] = counts.get(g, 0) + 1
        self.container_settings.refresh_groups(node_counts=counts)

    # ================= v0.6.0: FOREIGN PROJECT IMPORT =================

    # Какие типы нод использовать для импортированных файлов.
    # ⚠ Для python нет generic-ноды — используется StringPub как контейнер кода.
    #   Если заведёшь PyCustomNode — поменяй идентификатор здесь.
    IMPORT_NODE_TYPES = {
        'cpp': 'ros.cpp.CppCustomNode',
        'python': 'ros.py.PyStringPubNode',
    }

    def on_import_foreign_project(self):
        path = QtWidgets.QFileDialog.getExistingDirectory(
            self, "Import existing ROS 2 package (folder)")
        if not path:
            return
        try:
            result = import_ros2_package(path)
        except Exception as e:
            self.system_log(f"Import failed: {e}")
            return

        graph = self.graph_py if self.ui.tabs.currentIndex() == 0 else self.graph_cpp

        # Зависимости — в лог (панель зависимостей можно навесить позже)
        if result['deps']:
            self.system_log(f"📦 {result['package_name']} depends: "
                            + ", ".join(result['deps']))
        for w in result['warnings']:
            self.system_log(f"WARN: {w}")

        # Раскидываем ноды сеткой
        cols = 4
        spacing_x, spacing_y = 320, 220
        created = 0
        for i, entry in enumerate(result['files']):
            node_type = self.IMPORT_NODE_TYPES.get(entry['language'])
            if not node_type:
                continue
            try:
                pos = [(i % cols) * spacing_x, (i // cols) * spacing_y]
                title = entry['filename']
                if entry['classes']:
                    title = f"{entry['classes'][0]} ({entry['filename']})"
                node = graph.create_node(node_type, name=title, pos=pos)

                for prop, val in (('is_imported', True),
                                  ('source_file', entry['relpath']),
                                  ('code_content', entry['code'])):
                    if not node.has_property(prop):
                        try:
                            node.create_property(prop, val)
                        except Exception:
                            pass
                    node.set_property(prop, val)
                created += 1
            except Exception as e:
                self.system_log(f"WARN: node for {entry['filename']} failed: {e}")

        self._refresh_container_counts()
        self.system_log(
            f"✅ Imported '{result['package_name']}': {created} file-node(s). "
            "Связи не восстанавливаются (Level 1) — ноды помечены is_imported.")

    def setup_graph_context_menus(self, graph):
        """Adds custom actions (add port, rescan) to right-click menu"""
        try: 
            root_menu = graph.context_menu() 
        except TypeError: 
            root_menu = graph.context_menu
            
        root_menu.add_separator()

        # v0.6.0: деплой-группы
        root_menu.add_command(" Assign to container...", partial(self.on_assign_container, graph))
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
        parts = port_type_ros.split('/')
        pkg = parts[0]
        cls = parts[-1]
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
class PaletteDropFilter(QtCore.QObject):
    node_dropped = QtCore.Signal(str)  # node_id

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