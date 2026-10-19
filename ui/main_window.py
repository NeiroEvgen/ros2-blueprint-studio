import logging
import os
import sys
import traceback
from functools import partial
from PySide6 import QtWidgets, QtCore

# === CORE MODULES ===
from core.docker_manager import RosContainerManager
from core.execution_context import resolve_context, ExecutionContext
from core.project_manager import ProjectManager
from core.file_watcher import FileWatcher
from core.export_manager import ExportManager 
from core.workspace_manager import WorkspaceManager
from core.i18n import t
from core import settings
from ui.scroll import scrollable
from ui.graph_set import GraphSet
from core import layout as project_layout
from core.ros_process import DeployWorker
from core.container_config import ContainerConfigStore, safe_group_name, resolved_env
from core.project_importer import import_ros2_package
from ui.container_settings import ContainerSettingsPanel
from core.process_manager import ProcessManager

# === UI MODULES ===
from ui.ui_manager import UiManager
from ui.dashboard import ConsoleDashboard
from ui.docker_panel import DockerPanel
from ui.library_manager import LibraryManager
from ui.graph_setup import setup_graphs, ALL_NODE_CLASSES
from ui.properties_window import PropertiesWindow
# === NODES ===
from nodes.base import MSG_COLORS

class RosVisualRunner(QtWidgets.QMainWindow):
    def __init__(self, project_path=None):
        super().__init__()
        self.current_project_path = project_path
        self.auto_route_enabled = False  # off by default
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
        self._exec_ctx = ExecutionContext()
        self._remote = (False, "", "")   # connected, user@host, container
        self._real_robot = False
    

        # 4. Docker Connect
        
        self.container_manager = None
        try:
            self.container_manager = RosContainerManager()
        except Exception: 
            self.system_log(t("log.docker_missing"))
        self.process_manager = ProcessManager(self.container_manager)
        self.docker_panel = DockerPanel(self.container_manager) if self.container_manager else QtWidgets.QLabel(t("dock.unavailable"))
        self.library_manager = LibraryManager(self.container_manager) if self.container_manager else QtWidgets.QLabel(t("dock.unavailable"))
        
        if hasattr(self.ui, 'console_ros'):
            # find the QTabWidget that holds our console (it may not be the direct parent)
            tab_widget = self.ui.console_ros.parent()
            while tab_widget and not isinstance(tab_widget, QtWidgets.QTabWidget):
                tab_widget = tab_widget.parent()
                
            if tab_widget:
                idx = tab_widget.indexOf(self.ui.console_ros)
                if idx != -1:
                    # replace the old text console with the table
                    tab_widget.removeTab(idx)
                    self._console_tab_widget = tab_widget
                    self._console_tab_keys = {}          # tab page -> title key

                    def add_tab(widget, key, at=None):
                        page = widget if key == "tab.dashboard" else scrollable(widget)
                        if at is None:
                            tab_widget.addTab(page, t(key))
                        else:
                            tab_widget.insertTab(at, page, t(key))
                        self._console_tab_keys[page] = key
                        return page

                    add_tab(self.dashboard, "tab.dashboard", at=idx)
                    add_tab(self.docker_panel, "tab.docker")
                    add_tab(self.library_manager, "tab.libraries")
                    self.container_settings = ContainerSettingsPanel(
                        get_docker_manager=lambda: self.container_manager)
                    add_tab(self.container_settings, "tab.containers")
                    from ui.remote_panel import RemotePanel
                    from ui.file_browser import FileBrowser
                    self.remote_panel = RemotePanel()
                    self.remote_panel.log.connect(self.system_log)
                    self.remote_panel.connection_changed.connect(self._on_remote_changed)
                    add_tab(self.remote_panel, "tab.remote")
                    self.file_browser = FileBrowser()
                    self.file_browser.log.connect(self.system_log)
                    self.file_browser.marks_changed.connect(self.sync_green_nodes)
                    add_tab(self.file_browser, "tab.files")
        # 2. Graph Setup (delegated to ui/graph_setup.py)
        self.graph_py, self.graph_cpp, self.f_py, self.f_cpp = setup_graphs(
            self.ui.tabs, self.resolve_node_type, self.on_graph_delete,
            single=bool(settings.get("single_canvas")),
        )

        # 2.1 Properties Window
        self.graphs = GraphSet(self.graph_py, self.graph_cpp)
        self.properties_win = PropertiesWindow(self.graph_py, self.graph_cpp, self)
        self.properties_win.hide()

        self.setup_graph_signals()

        # 3. Core Managers
        self.project_manager = ProjectManager(self.graph_py, self.graph_cpp)
        self.watcher = FileWatcher()
        self.watcher.file_changed.connect(self.on_file_changed_externally)

        # 5. Connect Actions
        self.connect_actions()
        self.ui.actions['real_robot'].toggled.connect(self.on_real_robot_toggled)
        self._refresh_context()
        self.current_subgraph = None
        self.navigation_stack = [{"name": t("ui.root"), "node": None}]

        # 6. Init Logic
        if self.current_project_path:
            # Delay slightly to ensure UI is ready
            QtCore.QTimer.singleShot(100, lambda: self.on_load_project_init(self.current_project_path))
        else:
            self.ui.rebuild_palette(ALL_NODE_CLASSES, "python")
            self.ui.set_visible_graph("python")

    def on_graph_delete(self, graph, with_contents):
        """Delete = dissolve the group (children move out). Ctrl+Delete = delete with contents.
        Plain nodes are simply deleted."""
        selected = graph.selected_nodes()
        if not selected:
            return

        from nodes.group_logic import dissolve_group
        for node in list(selected):
            is_group = node.type_ in ('ros.nodes.RosGroup', 'ros.nodes.RosGroupNode')
            try:
                if is_group:
                    if with_contents:
                        # Ctrl+Del: delete children and terminals together with the group
                        gid = node.get_property('group_uid')
                        for child in list(graph.all_nodes()):
                            if child == node:
                                continue
                            if (child.has_property('parent_group_id')
                                    and child.get_property('parent_group_id') == gid):
                                graph.delete_node(child)
                        graph.delete_node(node)
                        self.system_log(t("log.group_deleted", name=node.name()))
                    else:
                        # Del: dissolve; children move out, terminals are deleted
                        dissolve_group(graph, node)
                        graph.delete_node(node)
                        self.system_log(t("log.group_dissolved", name=node.name()))
                else:
                    # plain node
                    graph.delete_node(node)
            except Exception as e:
                self.system_log(t("log.delete_error", error=e))

        self._refresh_container_counts() if hasattr(self, '_refresh_container_counts') else None

    def enter_subgraph(self, group_node):
        self.current_subgraph = group_node
        name = group_node.get_property('node_name')
        self.navigation_stack.append({"name": name, "node": group_node})
        self.ui.update_breadcrumbs(self.navigation_stack)
        
        # hide everything except this group's children
        gid = group_node.get_property('group_uid')
        graph = group_node.graph
        for node in graph.all_nodes():
            if node == group_node:
                self._set_node_visible(node, False)
            elif node.has_property('parent_group_id') and node.get_property('parent_group_id') == gid:
                self._set_node_visible(node, True)
            else:
                self._set_node_visible(node, False)
        
        self.system_log(t("log.entered_subgraph", name=name))

    def exit_subgraph(self):
        if len(self.navigation_stack) <= 1:
            return
            
        self.navigation_stack.pop()
        self.ui.update_breadcrumbs(self.navigation_stack)
        
        target = self.navigation_stack[-1]
        self.current_subgraph = target["node"]
        
        graph = self.graphs.current(self.ui.tabs.currentIndex())
        
        if self.current_subgraph is None:
            # at Root: show only what is NOT nested in a group.
            for node in graph.all_nodes():
                # a group child is recognised by parent_group_id, not by group_uid
                # (the group itself has group_uid too, and it must not be hidden).
                parent = node.get_property('parent_group_id') \
                    if node.has_property('parent_group_id') else ""
                is_child = bool(parent)
                node.view.setVisible(not is_child)
        else:
            # exited to the parent subgraph
            parent_gid = self.current_subgraph.get_property('group_uid')
            for node in graph.all_nodes():
                if node == self.current_subgraph:
                    node.view.setVisible(False)
                elif node.has_property('parent_group_id') and node.get_property('parent_group_id') == parent_gid:
                    node.view.setVisible(True)
                else:
                    node.view.setVisible(False)
            
        self.system_log(t("log.exited_to", name=target["name"]))

    def setup_graph_signals(self):
        """Connects graph signals for opening the editor and the settings."""
        for graph in self.graphs.all():
            graph.node_double_clicked.connect(self.on_node_double_clicked)
            self.setup_graph_context_menus(graph)

    def _set_node_visible(self, node, visible):
        """Hides/shows a node together with its connections."""
        if hasattr(node, 'view'):
            node.view.setVisible(visible)
        try:
            for port in node.input_ports() + node.output_ports():
                pv = port.view  # PortItem (QGraphicsItem)
                # connected_pipes is a dict or a list depending on the version
                pipes = getattr(pv, 'connected_pipes', None)
                if pipes is None:
                    continue
                iterable = pipes.values() if isinstance(pipes, dict) else pipes
                for pipe in iterable:
                    pipe.setVisible(visible)
        except Exception as e:
            logging.debug(f"pipe visibility skip: {e}")

    def on_node_double_clicked(self, node):
        """
        Double click: group -> enter it; plain node -> code.
        Ctrl + double click -> properties window.
        """
        if not node:
            return

        modifiers = QtWidgets.QApplication.keyboardModifiers()
        ctrl_held = bool(modifiers & QtCore.Qt.ControlModifier)

        is_group = node.type_ in ('ros.nodes.RosGroup', 'ros.nodes.RosGroupNode') \
            or node.has_property('internal_nodes')

        # Ctrl + double click -> properties only
        if ctrl_held:
            self.properties_win.set_node(node)
            self.properties_win.show()
            self.system_log(t("log.opened_properties", name=node.name()))
            return

        # plain double click on a group -> enter it
        if is_group:
            self.enter_subgraph(node)
            return

        # plain double click on a plain node -> code only
        if node.has_property('code_content'):
            if not self.current_project_path:
                self.system_log(t("log.open_project_first"))
                return
            
            self.project_manager.open_node_in_editor(node, self.current_project_path)
            self.watcher.start_watching(os.path.join(self.current_project_path, "src"))
            self.system_log(t("log.opened_code", name=node.name()))
    def connect_actions(self):
        """Connects the toolbar buttons."""
        self.ui.actions['run'].clicked.connect(self.on_run_project)
        self.ui.actions['stop'].clicked.connect(self.on_stop_project)
        self.ui.actions['clear'].clicked.connect(self.on_clear_graph)
        self.ui.actions['export_docker'].clicked.connect(self.on_export_docker)
        if 'import_pkg' in self.ui.actions:
            self.ui.actions['import_pkg'].clicked.connect(self.on_import_foreign_project)
        self.ui.actions['visualize'].clicked.connect(self.on_visualize)
        self.ui.actions['export_palette'].clicked.connect(self.on_export_palette)
        self.ui.actions['import_palette'].clicked.connect(self.on_import_palette)
        self.ui.actions['save'].clicked.connect(self.on_save_project)
        self.ui.actions['open'].clicked.connect(self.on_open_project)

    @staticmethod
    def _save_result(res):
        """
        save_project returns (True, apt_deps) on success and False on failure.
        Normalize to (ok, apt_deps) so callers do not fail on unpacking.
        """
        if isinstance(res, tuple):
            return bool(res[0]), (res[1] if len(res) > 1 else set())
        return bool(res), set()

    def _save_now(self):
        """Save the current project -> (ok, apt_deps); a successful Save clears "unsaved changes"."""
        ok, deps = self._save_result(self.project_manager.save_project(self.current_project_path))
        if ok:
            self._mark_clean()
        return ok, deps

    def _mark_clean(self):
        for g in self.graphs.all():
            try:
                g.undo_stack().setClean()
            except Exception:
                pass

    def _has_unsaved_changes(self):
        if not self.current_project_path:
            return False
        for g in self.graphs.all():
            try:
                if not g.undo_stack().isClean():
                    return True
            except Exception:
                pass
        return False

    def closeEvent(self, event):
        """
        Close: ask about unsaved changes, stop the run and background tasks and wait for
        their threads; destroying a live QThread aborts the process.
        """
        fb = getattr(self, "file_browser", None)
        if fb is not None and not fb._confirm_discard():
            event.ignore()
            return
        if self._has_unsaved_changes():
            ans = QtWidgets.QMessageBox.question(
                self, t("dlg.close_title"), t("dlg.save_before_close"),
                QtWidgets.QMessageBox.Save | QtWidgets.QMessageBox.Discard
                | QtWidgets.QMessageBox.Cancel)
            if ans == QtWidgets.QMessageBox.Cancel:
                event.ignore()
                return
            if ans == QtWidgets.QMessageBox.Save and not self._save_now()[0]:
                self.system_log(t("log.save_failed_close_cancelled"))
                event.ignore()
                return
        QtWidgets.QApplication.setOverrideCursor(QtCore.Qt.WaitCursor)
        try:
            leftovers = self.shutdown_background()
        finally:
            QtWidgets.QApplication.restoreOverrideCursor()
        event.accept()
        if leftovers:
            # A thread missed the timeout (e.g. docker pull). A live QThread cannot be destroyed
            # (abort), and waiting longer hangs the window. The process is exiting anyway.
            sys.stderr.write(f"[close] background threads still running: {leftovers}; exiting\n")
            sys.stderr.flush()
            os._exit(0)

    def shutdown_background(self, timeout_ms=15000):
        """Stops run/build/sync and waits for the threads. Returns the names of those that did not stop."""
        leftovers = []
        worker = getattr(self, "deploy_worker", None)
        if worker is not None and worker.isRunning():
            worker.stop()
            if not worker.wait(timeout_ms):
                leftovers.append("deploy")
        bg = getattr(self, "_bg_tasks", None)
        if bg is not None:
            bg.wait_all(timeout_ms)
            if bg.busy:
                leftovers.append("node commands")
        for name in ("remote_panel", "file_browser"):
            panel = getattr(self, name, None)
            if panel is not None and hasattr(panel, "shutdown") and not panel.shutdown(timeout_ms):
                leftovers.append(name)
        return leftovers

    def on_save_project(self):
        if not self.current_project_path:
            self.system_log(t("log.no_project"))
            return
        ok, _ = self._save_now()
        self.system_log(t("log.project_saved") if ok else t("log.save_failed"))
        self._log_save_warnings()

    def _log_save_warnings(self):
        """Generator/CMake linter warnings after Save -> system log."""
        for text in getattr(self.project_manager, "last_warnings", []) or []:
            self.system_log(t("save.warning", text=text))

    def _offer_layout_migration(self, path):
        """Old project with ROS files outside src/: offer to move them (with a backup)."""
        lay = project_layout.ProjectLayout(path)
        legacy = lay.legacy_items()
        if not legacy:
            return False
        ans = QtWidgets.QMessageBox.question(
            self, t("layout.legacy_title"), t("layout.legacy_question", count=len(legacy)),
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No, QtWidgets.QMessageBox.No)
        if ans != QtWidgets.QMessageBox.Yes:
            return False
        report = project_layout.migrate(path)
        self.system_log(t("layout.migration_done", summary=report.summary()))
        return True

    def on_open_project(self):
        """Open another project from disk (like "Open from Disk..." in the launcher)."""
        if self.current_project_path:
            ans = QtWidgets.QMessageBox.question(
                self, t("dlg.open_project_title"),
                t("dlg.save_before_open"),
                QtWidgets.QMessageBox.Save | QtWidgets.QMessageBox.Discard
                | QtWidgets.QMessageBox.Cancel)
            if ans == QtWidgets.QMessageBox.Cancel:
                return
            if ans == QtWidgets.QMessageBox.Save:
                ok, _ = self._save_now()
                if not ok:
                    self.system_log(t("log.save_failed_open_cancelled"))
                    return
        root = WorkspaceManager.get_workspace_root()
        path = QtWidgets.QFileDialog.getExistingDirectory(self, t("dlg.open_folder_title"), root)
        if path:
            self.on_load_project_init(path)



    def on_visualize(self):
        mode = self.ui.actions['viz_mode'].currentText()

        if mode.startswith("X11"):
            # classic mode: just a reminder, the RViz node opens its window on Run
            self.system_log(t("log.x11_hint"))
            return

        # === Foxglove ===
        if not self.container_manager or not self.container_manager.container:
            self.system_log(t("log.session_not_running"))
            return
        try:
            self.container_manager.start_foxglove_bridge(self.system_log)
        except Exception as e:
            self.system_log(t("log.foxglove_error", error=e))
            return

        # open Foxglove in the browser, auto-connecting to ws://localhost:8765
        import webbrowser
        url = ("https://app.foxglove.dev/?ds=foxglove-websocket"
               "&ds.url=ws%3A%2F%2Flocalhost%3A8765")
        webbrowser.open(url)
        self.system_log(t("log.foxglove_opened"))

    def on_clear_graph(self):
        graph = self.graphs.current(self.ui.tabs.currentIndex())
        graph.clear_session()
        self.system_log(t("log.graph_cleared"))

    def on_run_project(self):
        if not self.current_project_path:
            self.system_log(t("log.no_project"))
            return
        if not self.container_manager:
            self.system_log(t("log.docker_unavailable"))
            return
        if self._deploy_running():
            # the previous run is still stopping: a new worker would drop the reference to a live QThread -> abort
            self.system_log(t("log.deploy_still_stopping"))
            return

        # 1. Generate/save the graph into the project files first
        self.system_log(t("log.generating"))
        ok, apt_packages = self._save_now()
        if not ok:
            self.system_log(t("log.generate_failed_abort"))
            return
        self.watcher.start_watching(os.path.join(self.current_project_path, "src"))

        self.ui.actions['run'].setEnabled(False)
        self.ui.actions['stop'].setEnabled(True)
        self.system_log(t("log.starting"))

        
        try:
            store = ContainerConfigStore(self.current_project_path)
            self.container_manager.extra_env = resolved_env(store.get("main"))
        except Exception as e:
            self.system_log(t("log.env_skipped", error=e))

        # 2. Deploy in a separate thread
        self.deploy_worker = DeployWorker(self.container_manager, self.current_project_path, extra_apt_packages=apt_packages)
        self.deploy_worker.sys_signal.connect(self.system_log)
        self.deploy_worker.ros_signal.connect(self.ros_log)
        self.deploy_worker.finished_signal.connect(self.on_deploy_finished)
        self.deploy_worker.start()
        QtCore.QTimer.singleShot(1500, self._refresh_context)

    def _deploy_running(self):
        w = getattr(self, "deploy_worker", None)
        return bool(w is not None and w.isRunning())

    def on_deploy_finished(self):
        """Called when DeployWorker has finished."""
        self.ui.actions['run'].setEnabled(True)
        self.ui.actions['stop'].setEnabled(False)
        self.system_log(t("log.deploy_finished"))
        self._refresh_context()

    def on_stop_project(self):
        # stop the worker and the container if they are running
        if self._deploy_running():
            self.deploy_worker.stop()
            # on_deploy_finished re-enables Run once the worker has really finished
            self.ui.actions['run'].setEnabled(False)
        else:
            self.ui.actions['run'].setEnabled(True)
        self.ui.actions['stop'].setEnabled(False)
        self.system_log(t("log.stopped"))
        self._refresh_context()

    def on_export_docker(self):
        if not self.current_project_path:
            self.system_log(t("log.no_project"))
            return
        # generate up-to-date files from the graph first
        self.system_log(t("log.export_generating"))
        if not self._save_now()[0]:
            self.system_log(t("log.export_generate_failed"))
            return
        try:
            self.system_log(t("log.export_running"))
            zip_path = ExportManager.export_to_portable_package(self.current_project_path)
            self.system_log(t("log.export_complete", path=zip_path))
            QtWidgets.QMessageBox.information(
                self, t("dlg.export_complete_title"),
                t("dlg.export_complete_text", path=zip_path)
            )
        except Exception as e:
            self.system_log(t("log.export_failed", error=e))
            traceback.print_exc()

    def on_load_project_init(self, path):
        """Load a project and adapt the UI to it."""
        self.current_project_path = path
        self._offer_layout_migration(path)
        p_type = WorkspaceManager.get_project_type(path)
        self.ui.set_visible_graph(p_type)
        self.ui.rebuild_palette(ALL_NODE_CLASSES, p_type)
        self.project_manager.load_project(path)
        if self.project_manager.load_failed_path:
            self.system_log(t("log.state_unreadable"))
        self.watcher.start_watching(os.path.join(path, "src"))
        if hasattr(self, 'container_settings'):
            self.container_settings.set_project(path)
        if hasattr(self, "remote_panel"):
            self.remote_panel.set_project(path)
        if hasattr(self.library_manager, "set_project"):
            self.library_manager.set_project(path)
        if hasattr(self, "file_browser"):
            self.file_browser.set_sources(os.path.join(path, "src"), None)
            self._refresh_container_counts()
        self.sync_green_nodes()
        self._load_real_robot_flag()
        self._refresh_context()
        self._mark_clean()                 # loading and green nodes are not user edits
        self.system_log(t("log.project_loaded", path=path, type=p_type.upper()))

    def retranslate_tabs(self):
        """Titles of the bottom panel tabs (Dashboard/Docker/Libraries/Containers/Remote/Files)."""
        tabs = getattr(self, "_console_tab_widget", None)
        for page, key in getattr(self, "_console_tab_keys", {}).items():
            idx = tabs.indexOf(page) if tabs is not None else -1
            if idx >= 0:
                tabs.setTabText(idx, t(key))
        if hasattr(self.ui, "retranslate"):
            self.ui.retranslate()

    def system_log(self, text):
        ctx = getattr(self, "_exec_ctx", None)
        prefix = ctx.short() + " " if ctx else ""
        self.ui.console_sys.append(f"{prefix}{text}")

    # ---------- execution context (where commands run) ----------

    def _project_target(self):
        if not self.current_project_path:
            return None
        try:
            return ContainerConfigStore(self.current_project_path).get("main").get("target")
        except Exception:
            return None

    def _refresh_context(self):
        connected, rhost, rcontainer = getattr(self, "_remote", (False, "", ""))
        if connected:
            # connected to a remote target: it takes priority over the local container,
            # Sync / Build go there.
            self._exec_ctx = resolve_context({"type": "ssh", "host": rhost},
                                             bool(rcontainer), rcontainer, self._real_robot)
        else:
            cm = self.container_manager
            active = bool(cm and getattr(cm, "container", None) is not None)
            name = getattr(cm, "container_name", "") if cm else ""
            self._exec_ctx = resolve_context(self._project_target(), active, name,
                                             self._real_robot)
        if hasattr(self.ui, "context_badge"):
            self.ui.context_badge.set_context(self._exec_ctx)
        proj = os.path.basename(self.current_project_path) if self.current_project_path else "no project"
        self.setWindowTitle(f"ROS2 Blueprint Studio — {proj}  ·  [{self._exec_ctx.label()}]")

    def _load_real_robot_flag(self):
        target = self._project_target() or {}
        self._real_robot = bool(target.get("real_robot", False))
        btn = self.ui.actions.get("real_robot")
        if btn:
            btn.blockSignals(True); btn.setChecked(self._real_robot); btn.blockSignals(False)

    def _on_remote_changed(self, connected, host, container):
        self._remote = (connected, host, container)
        if hasattr(self.library_manager, "set_remote_connected"):
            self.library_manager.set_remote_connected(connected)
        self._refresh_context()
        if hasattr(self, "file_browser") and self.current_project_path:
            session = self.remote_panel.session if connected else None
            self.file_browser.set_sources(os.path.join(self.current_project_path, "src"), session)

    def on_real_robot_toggled(self, checked):
        self._real_robot = bool(checked)
        if self.current_project_path:
            try:
                store = ContainerConfigStore(self.current_project_path)
                cfg = store.get("main")
                cfg.setdefault("target", {})["real_robot"] = self._real_robot
                store.save(cfg)
            except Exception as e:
                self.ui.console_sys.append(t("log.real_robot_not_saved", error=e))
        self._refresh_context()
        self.system_log(t("log.target_robot") if self._real_robot else t("log.target_sim"))

    def ros_log(self, text):
        self.dashboard.process_log(text)

    def on_file_changed_externally(self, path, content, ports):
        """A node file was changed outside (editor): sync code and ports into the node."""
        try:
            self.project_manager.sync_node_from_file(
                path, content, ports,
                graphs=self.graphs.all(),
                log_callback=self.system_log)
        except Exception as e:
            self.system_log(t("log.sync_error", error=e))
    # === CORE LOGIC ===

    def resolve_node_type(self, code):
        """Determines node type from dropped mime data"""
        if code.startswith("USER_NODE:"):
            self.add_user_node_logic(code)
            return None
        if code and "." in code: return code # Direct class path
        return None

    def add_user_node_logic(self, code):
        """Recreates a custom node from a palette."""
        try:
            path_parts = code.replace("USER_NODE:", "").split("/")
            palette_name = path_parts[0]
            node_folder = path_parts[1]

            from core import palette_share
            base_dir = os.path.join(palette_share.PALETTES_ROOT, palette_name, node_folder)
            meta_path = os.path.join(base_dir, "node.yaml")

            if not os.path.exists(meta_path): return

            import yaml
            with open(meta_path, 'r', encoding='utf-8') as f:
                meta = yaml.safe_load(f)

            graph = self.graphs.current(self.ui.tabs.currentIndex())

            # unique name: Name, Name_1, Name_2...
            base_name = meta['name']
            existing = {n.name() for n in graph.all_nodes()}
            new_name = base_name
            i = 1
            while new_name in existing:
                new_name = f"{base_name}_{i}"
                i += 1

            node = graph.create_node(meta['type'], name=new_name)

            # properties
            for key, val in meta.get('properties', {}).items():
                try:
                    node.set_property(key, val)
                except Exception:
                    # properties the base class does not have are created
                    try:
                        node.create_property(key, val)
                    except Exception:
                        pass

            # custom ports
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
                    self.system_log(t("log.port_skipped", port=pname, error=pe))

            # code from logic.cpp / logic.py
            ext = ".cpp" if "cpp" in meta['type'].lower() else ".py"
            code_path = os.path.join(base_dir, f"logic{ext}")
            if os.path.exists(code_path):
                with open(code_path, 'r', encoding='utf-8') as f:
                    node.set_property("code_content", f.read())

            self.system_log(t("log.user_node_imported", name=new_name))
        except Exception as e:
            self.system_log(t("log.user_node_import_error", error=e))

    def _node_run_spec(self, node):
        """How the node is started in the session (same rule as the launch generator)."""
        from core.process_manager import node_spec
        from core.remote.session import package_name_for
        if not self.current_project_path or not node.has_property('source_file'):
            return None
        bound = node.get_property('bound_file') if node.has_property('bound_file') else None
        return node_spec(node.type_, node.get_property('source_file'),
                         package_name_for(self.current_project_path), bound)

    def _ros_node_name(self, node):
        """Node name in ROS: launch names a C++ node after its file."""
        spec = self._node_run_spec(node)
        return spec["entry"] if spec and spec["language"] == "cpp" else node.name()

    def _background(self, fn):
        """Docker commands (build, lifecycle) off the UI thread."""
        if not hasattr(self, "_bg_tasks"):
            from ui.qt_worker import TaskHost
            self._bg_tasks = TaskHost()
        self._bg_tasks.start(fn, on_fail=self.system_log, on_line=self.system_log, parent=self)

    def on_restart_selected(self, graph, *args, **kwargs):
        jobs = []
        for node in graph.selected_nodes():
            spec = self._node_run_spec(node)
            if spec is None:
                self.system_log(t("log.restart_no_file", name=node.name()))
                continue
            jobs.append((node.name(), spec))
        if not jobs:
            return
        pm = self.process_manager

        def work(emit):
            for name, spec in jobs:
                pm.restart_node(name, emit, rebuild_cpp=spec["language"] == "cpp", spec=spec)
        self._background(work)

    def on_freeze_selected(self, graph, target_state, *args, **kwargs):
        names = []
        for node in graph.selected_nodes():
            if self._is_lifecycle_node(node):
                names.append(self._ros_node_name(node))
            else:
                self.system_log(t("log.not_lifecycle", name=node.name()))
        if not names:
            return
        pm = self.process_manager

        def work(emit):
            for name in names:
                pm.set_lifecycle_state(name, target_state, emit)
        self._background(work)

    def _is_lifecycle_node(self, node):
        return 'lifecycle' in getattr(node, 'NODE_NAME', '').lower() or \
               'lifecycle' in node.type_.lower()
    
    def on_save_node_to_palette(self, node):
        palette_name, ok = QtWidgets.QInputDialog.getText(self, t("dlg.save_palette_title"), t("dlg.save_palette_prompt"))
        if not (ok and palette_name): return

        node_name = node.name()
        # Path: <app>/nodes/user_palettes/PaletteName/NodeName/
        from core import palette_share
        base_dir = os.path.join(palette_share.PALETTES_ROOT, palette_name, node_name)
        os.makedirs(base_dir, exist_ok=True)

        # 1. Metadata to YAML (properties, ports)
        meta = {
            "name": node_name,
            "type": node.type_,
            "properties": node.model.custom_properties,
            "ports": ([{"name": p.name(), "type": "in"} for p in node.input_ports()]
                      + [{"name": p.name(), "type": "out"} for p in node.output_ports()])
        }
        
        import yaml
        with open(os.path.join(base_dir, "node.yaml"), 'w', encoding='utf-8') as f:
            yaml.dump(meta, f, sort_keys=False, allow_unicode=True)

        # 2. Code into a file with its native extension so it can be opened in an editor
        code = node.get_property("code_content")
        ext = ".cpp" if "cpp" in node.type_.lower() else ".py"
        with open(os.path.join(base_dir, f"logic{ext}"), 'w', encoding='utf-8') as f:
            f.write(code if code else "")

        self.system_log(t("log.node_saved_palette", node=node_name, palette=palette_name))
        # refresh the palette UI
        p_type = WorkspaceManager.get_project_type(self.current_project_path) if self.current_project_path else "python"
        self.ui.rebuild_palette(ALL_NODE_CLASSES, p_type)

    def _on_node_dropped_to_palette(self, node_id):
        """A node was Alt+dragged onto the palette: save it as a custom node."""
        n = self.graphs.find_node(node_id)
        if n is not None:
            self.on_save_node_to_palette(n)
                
    def on_export_palette(self):
        from core.palette_share import export_palette, PALETTES_ROOT
        # available palettes
        palettes = []
        if os.path.isdir(PALETTES_ROOT):
            palettes = [d for d in os.listdir(PALETTES_ROOT)
                        if os.path.isdir(os.path.join(PALETTES_ROOT, d))]
        if not palettes:
            self.system_log(t("log.no_palettes"))
            return
        name, ok = QtWidgets.QInputDialog.getItem(
            self, t("dlg.export_palette_title"), t("dlg.export_palette_choose"), palettes, 0, False)
        if not (ok and name):
            return
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, t("dlg.export_palette_as"), f"{name}.bppalette",
            "Blueprint Palette (*.bppalette)")
        if not path:
            return
        try:
            out = export_palette(name, path)
            self.system_log(t("log.palette_exported", path=out))
        except Exception as e:
            self.system_log(t("log.export_failed", error=e))

    def on_import_palette(self):
        from core.palette_share import import_palette
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, t("dlg.import_palette_title"), "", "Blueprint Palette (*.bppalette *.zip)")
        if not path:
            return
        try:
            pname, nodes = import_palette(path)
        except FileExistsError as e:
            ans = QtWidgets.QMessageBox.question(
                self, t("dlg.palette_exists_title"),
                t("dlg.palette_exists_text", name=e))
            if ans != QtWidgets.QMessageBox.Yes:
                return
            try:
                pname, nodes = import_palette(path, overwrite=True)
            except Exception as e2:
                self.system_log(t("log.import_failed", error=e2))
                return
        except Exception as e:
            self.system_log(t("log.import_failed", error=e))
            return

        # refresh the palette in the UI
        p_type = WorkspaceManager.get_project_type(self.current_project_path) \
            if self.current_project_path else "python"
        self.ui.rebuild_palette(ALL_NODE_CLASSES, p_type)
        self.system_log(t("log.palette_imported", name=pname, count=len(nodes), nodes=", ".join(nodes)))

    def on_assign_container(self, graph, *args, **kwargs):
        """Assigns a deploy group (container_group property) to the selected nodes."""
        selected = graph.selected_nodes()
        if not selected:
            self.system_log(t("log.select_nodes_first"))
            return

        existing = ["main"]
        if hasattr(self, 'container_settings') and self.container_settings.store:
            existing = self.container_settings.group_names()

        name, ok = QtWidgets.QInputDialog.getItem(
            self, t("dlg.assign_title"),
            t("dlg.assign_prompt"),
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

        # new group: create a default config for it
        if hasattr(self, 'container_settings') and self.container_settings.store \
                and gname not in self.container_settings.group_names():
            from core.container_config import make_config
            self.container_settings.store.save(make_config(gname))
            self.container_settings.refresh_groups()

        self._refresh_container_counts()
        self.system_log(t("log.assigned", count=len(selected), group=gname))

    def _refresh_container_counts(self):
        """Node count per group in the UI."""
        if not hasattr(self, 'container_settings') or not self.container_settings.store:
            return
        counts = {}
        for graph in self.graphs.all():
            for node in graph.all_nodes():
                g = "main"
                if node.has_property('container_group'):
                    g = node.get_property('container_group') or "main"
                counts[g] = counts.get(g, 0) + 1
        self.container_settings.refresh_groups(node_counts=counts)

    # ================= v0.6.0: FOREIGN PROJECT IMPORT =================

    # Node types used for imported files.
    # ⚠ There is no generic Python node: StringPub is used as a code container.
    #   When PyCustomNode exists, change the identifier here.
    IMPORT_NODE_TYPES = {
        'cpp': 'ros.cpp.CppCustomNode',
        'python': 'ros.py.PyStringPubNode',
    }

    @staticmethod
    def _node_binds(node, lang, filename, rel):
        """The node is already bound to this file (new: by bound_file, old: by file name)."""
        if (('cpp' in node.type_) != (lang == 'cpp')):
            return False
        bound = node.get_property('bound_file')
        if bound:
            return bound == rel
        return node.get_property('source_file') == filename and "/" not in rel.split("/", 1)[-1]

    def _ensure_file_node(self, path, langs, rel):
        """File node for a green non-code file (CMakeLists.txt, package.xml, header...). Returns 1 if created."""
        from core import build_marks
        from core.node_sources import code_hash
        if any(n.has_property('bound_file') and n.get_property('bound_file') == rel
               and n.type_ == self.project_manager.FILE_NODE_TYPE
               for g in self.graphs.all() for n in g.all_nodes()):
            return 0
        lang = build_marks.node_language(rel)
        if lang not in langs:
            lang = "cpp" if "cpp" in langs else "python"
        graph = self.graphs.for_language(lang)
        try:
            with open(os.path.join(path, "src", rel), 'r', encoding='utf-8') as f:
                code = f.read()
        except (OSError, UnicodeDecodeError) as e:
            self.system_log(t("log.node_failed", file=rel, error=e))
            return 0
        try:
            taken = len(graph.all_nodes())
            node = graph.create_node(self.project_manager.FILE_NODE_TYPE, name=os.path.basename(rel),
                                     pos=[(taken % 4) * 320, (taken // 4) * 220])
            for prop, val in (('bound_file', rel), ('code_content', code), ('code_sync_hash', code_hash(code))):
                if not node.has_property(prop):
                    node.create_property(prop, val)
                node.set_property(prop, val)
            return 1
        except Exception as e:
            self.system_log(t("log.node_failed", file=rel, error=e))
            return 0

    def sync_green_nodes(self):
        """Green-marked files (Files tab) -> canvas nodes bound to the file on disk.
        Removing the mark does NOT delete the node: it may have wires and settings."""
        path = getattr(self, "current_project_path", None)
        if not path:
            return 0
        from core import build_marks, graph_ir
        from core.node_sources import code_hash
        from core.workspace_manager import WorkspaceManager
        langs = graph_ir.project_languages(WorkspaceManager.get_project_type(path))
        self.project_manager.sync_file_nodes(os.path.join(path, "src"))
        nodes, plain = build_marks.green_node_files(path, langs)
        created = 0
        for lang, filename, rel in nodes:
            graph = self.graphs.for_language(lang)
            if any(n.has_property('bound_file') and self._node_binds(n, lang, filename, rel)
                   for g in self.graphs.all() for n in g.all_nodes()):
                continue
            try:
                full = os.path.join(path, "src", rel)
                with open(full, 'r', encoding='utf-8') as f:
                    code = f.read()
                stem = os.path.splitext(filename)[0]
                taken = len(graph.all_nodes())
                node = graph.create_node(self.IMPORT_NODE_TYPES[lang], name=stem,
                                         pos=[(taken % 4) * 320, (taken // 4) * 220])
                for prop, val in (('source_file', filename), ('bound_file', rel),
                                  ('code_content', code), ('code_sync_hash', code_hash(code))):
                    if not node.has_property(prop):
                        node.create_property(prop, val)
                    node.set_property(prop, val)
                created += 1
            except Exception as e:
                self.system_log(t("log.node_failed", file=filename, error=e))
        for rel in plain:
            created += self._ensure_file_node(path, langs, rel)
        if created:
            self._refresh_container_counts()
            self.system_log(t("log.green_nodes", count=created))
        return created

    def on_import_foreign_project(self):
        path = QtWidgets.QFileDialog.getExistingDirectory(
            self, t("dlg.import_pkg_title"))
        if not path:
            return
        try:
            result = import_ros2_package(path)
        except Exception as e:
            self.system_log(t("log.import_failed", error=e))
            return

        graph = self.graphs.current(self.ui.tabs.currentIndex())

        # dependencies go to the log
        if result['deps']:
            self.system_log(t("log.pkg_depends", name=result['package_name'],
                              deps=", ".join(result['deps'])))
        for w in result['warnings']:
            self.system_log(t("log.warn", text=w))

        # lay the nodes out on a grid
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
                self.system_log(t("log.node_failed", file=entry['filename'], error=e))

        self._refresh_container_counts()
        self.system_log(t("log.pkg_imported", name=result['package_name'], count=created))

    def setup_graph_context_menus(self, graph):
        """Adds custom actions (add port, rescan) to right-click menu"""
        try: 
            root_menu = graph.context_menu() 
        except TypeError: 
            root_menu = graph.context_menu
            
        root_menu.add_separator()

        # v0.6.0: deploy groups
        root_menu.add_command(" " + t("menu.assign_container"), partial(self.on_assign_container, graph))
        root_menu.add_command(" " + t("menu.restart_node"), partial(self.on_restart_selected, graph))
        root_menu.add_command(" " + t("menu.freeze"), partial(self.on_freeze_selected, graph, 'deactivate'))
        root_menu.add_command(" " + t("menu.unfreeze"), partial(self.on_freeze_selected, graph, 'activate'))
        root_menu.add_separator()

        # Manual Rescan Action
        root_menu.add_command(" " + t("menu.rescan"), partial(self.on_manual_rescan, graph))
        root_menu.add_separator()
        
        # Add Output Ports
        out_menu = root_menu.add_menu(t("menu.add_output"))
        for name, color in MSG_COLORS.items():
            short_name = name.split('/')[-1]
            out_menu.add_command(t("menu.add_port", name=short_name), partial(self.on_add_port_clicked, graph, name, color))
            
        # Add Input Ports
        in_menu = root_menu.add_menu(t("menu.add_input"))
        for name, color in MSG_COLORS.items():
            short_name = name.split('/')[-1]
            in_menu.add_command(t("menu.add_port", name=short_name), partial(self.on_add_input_clicked, graph, name, color))

    def on_manual_rescan(self, graph, *args, **kwargs):
        selected = graph.selected_nodes()
        if not selected: return
        for node in selected:
            if not node.has_property('source_file') or not self.current_project_path:
                continue
            filename = node.get_property('source_file')
            full_path = self._node_source_path(node)
            if not full_path:
                self.system_log(t("log.rescan_not_found", file=filename))
                continue
            with open(full_path, 'r', encoding='utf-8') as f: content = f.read()
            ports = self.watcher._scan_ports(content, full_path)
            self.on_file_changed_externally(filename, content, ports)
            self.system_log(t("log.rescanned", file=filename))

    def _node_source_path(self, node):
        """Node file on disk: bound_file (green mark) or src/cpp|python/<source_file>."""
        src = os.path.join(self.current_project_path, "src")
        bound = node.get_property('bound_file') if node.has_property('bound_file') else None
        filename = node.get_property('source_file') or ""
        lang = "cpp" if "cpp" in node.type_ else "python"
        candidates = [os.path.join(src, bound)] if bound else []
        candidates += [os.path.join(project_layout.ProjectLayout(self.current_project_path).code_dir(lang), filename),
                       os.path.join(src, filename)]          # old layout: code directly in src/
        return next((p for p in candidates if filename and os.path.isfile(p)), None)

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
        """Injects publisher/subscriber code into the node text at the markers."""
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