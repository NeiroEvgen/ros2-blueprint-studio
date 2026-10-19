"""
Main window: the Open/Save toolbar buttons used not to be connected; Run failed with
TypeError if save_project returned False. Qt offscreen, Docker faked.
"""
import os
import types

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
QtWidgets = pytest.importorskip("PySide6.QtWidgets")
pytest.importorskip("NodeGraphQt")
from PySide6 import QtCore  # noqa: E402


@pytest.fixture(scope="module")
def app():
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


@pytest.fixture
def win(app, tmp_path):
    from ui import main_window as mw
    w = mw.RosVisualRunner()
    w.current_project_path = str(tmp_path / "proj")
    os.makedirs(w.current_project_path, exist_ok=True)
    logs = []
    w.system_log = logs.append
    w._logs = logs
    return w


def receivers(obj, sig):
    return obj.receivers(QtCore.SIGNAL(sig))


class TestSaveResult:
    def test_tuple_success(self, win):
        assert win._save_result((True, {"a"})) == (True, {"a"})

    def test_false_failure(self, win):
        assert win._save_result(False) == (False, set())

    def test_empty_tuple_flag_false(self, win):
        assert win._save_result((False, set()))[0] is False


class TestToolbarButtons:
    def test_open_and_save_are_connected(self, win):
        for key in ("open", "save"):
            assert receivers(win.ui.actions[key], "clicked(bool)") > 0, key

    def test_save_calls_project_manager(self, win):
        called = []
        win.project_manager.save_project = lambda p: called.append(p) or (True, set())
        win.ui.actions["save"].click()
        assert called == [win.current_project_path]
        assert any("saved" in m for m in win._logs)

    def test_save_failure_is_reported(self, win):
        win.project_manager.save_project = lambda p: False
        win.ui.actions["save"].click()
        assert any("failed to save" in m for m in win._logs)

    def test_save_without_project(self, win):
        win.current_project_path = None
        win.ui.actions["save"].click()
        assert any("No project" in m for m in win._logs)

    def test_open_cancel_keeps_current_project(self, win, monkeypatch):
        before = win.current_project_path
        monkeypatch.setattr(QtWidgets.QMessageBox, "question",
                            staticmethod(lambda *a, **k: QtWidgets.QMessageBox.Cancel))
        win.ui.actions["open"].click()
        assert win.current_project_path == before

    def test_open_loads_chosen_folder(self, win, monkeypatch, tmp_path):
        monkeypatch.setattr(QtWidgets.QMessageBox, "question",
                            staticmethod(lambda *a, **k: QtWidgets.QMessageBox.Discard))
        target = tmp_path / "other"; target.mkdir()
        monkeypatch.setattr(QtWidgets.QFileDialog, "getExistingDirectory",
                            staticmethod(lambda *a, **k: str(target)))
        loaded = []
        win.on_load_project_init = loaded.append
        win.ui.actions["open"].click()
        assert loaded == [str(target)]


class TestRunWithFailedSave:
    def test_run_does_not_crash_when_save_fails(self, win):
        win.container_manager = types.SimpleNamespace()      # Docker "is there"
        win.project_manager.save_project = lambda p: False   # used to be a TypeError
        win.on_run_project()
        assert any("Aborting run" in m for m in win._logs)


class TestYamlKeepsCyrillic:
    """yaml.dump without allow_unicode turned Cyrillic into \\u04xx (noisy git diffs)."""

    def test_save_node_to_palette_writes_readable_yaml(self, win, tmp_path, monkeypatch):
        import inspect
        src = inspect.getsource(type(win))
        assert "yaml.dump(meta, f, sort_keys=False, allow_unicode=True)" in src

    def test_project_state_yaml_keeps_cyrillic(self):
        import inspect
        from core import project_manager
        assert "allow_unicode=True" in inspect.getsource(project_manager.ProjectManager.save_project)


class TestLayoutMigrationOnOpen:
    def _legacy(self, win):
        p = win.current_project_path
        for rel, text in (("package.xml", "px"), ("launch/a.launch.py", "L")):
            fp = os.path.join(p, rel)
            os.makedirs(os.path.dirname(fp), exist_ok=True)
            open(fp, "w").write(text)
        return p

    def test_accept_migrates_and_logs(self, win, monkeypatch):
        p = self._legacy(win)
        monkeypatch.setattr(QtWidgets.QMessageBox, "question", lambda *a, **k: QtWidgets.QMessageBox.Yes)
        assert win._offer_layout_migration(p) is True
        assert os.path.isfile(os.path.join(p, "src", "package.xml"))
        assert not os.path.exists(os.path.join(p, "package.xml"))
        assert any("moved 2" in m for m in win._logs)

    def test_decline_changes_nothing(self, win, monkeypatch):
        p = self._legacy(win)
        monkeypatch.setattr(QtWidgets.QMessageBox, "question", lambda *a, **k: QtWidgets.QMessageBox.No)
        assert win._offer_layout_migration(p) is False
        assert os.path.isfile(os.path.join(p, "package.xml"))

    def test_clean_project_no_question_asked(self, win, monkeypatch):
        asked = []
        monkeypatch.setattr(QtWidgets.QMessageBox, "question", lambda *a, **k: asked.append(1))
        os.makedirs(os.path.join(win.current_project_path, "src"), exist_ok=True)
        assert win._offer_layout_migration(win.current_project_path) is False and not asked


class TestSaveWarnings:
    def test_warnings_logged_after_save(self, win):
        win.project_manager.save_project = lambda p: setattr(win.project_manager, "last_warnings", ["W1"]) or (True, set())
        win.ui.actions["save"].click()
        assert any("W1" in m for m in win._logs)


class TestLogsFollowLanguage:
    """Main window messages go through the dictionary: the same scenario in two languages."""

    def test_no_project_message_in_both_languages(self, win):
        from core import i18n
        win.current_project_path = None
        win._logs.clear()
        win.on_save_project()
        assert any("No project loaded" in m for m in win._logs)
        i18n.set_language("ru")
        win._logs.clear()
        win.on_save_project()
        assert any("проект не загружен" in m for m in win._logs)
        i18n.set_language("en")

    def test_context_menu_labels_localized(self, win):
        """The graph context menu is built from the dictionary (language set at startup)."""
        from core import i18n
        from core.i18n import t
        assert t("menu.restart_node") == "Restart Node"
        i18n.set_language("ru")
        assert t("menu.restart_node") == "Перезапустить ноду"
        assert t("menu.add_port", name="String") == "Добавить String"
        i18n.set_language("en")


class TestManualRescan:
    """Rescan looked for the file in src/<file>, but it lives in src/python|cpp/, and silently did nothing."""

    @pytest.fixture
    def loaded(self, app, tmp_path):
        import shutil
        from ui import main_window as mw
        root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        proj = tmp_path / "py_pubsub"
        shutil.copytree(os.path.join(root, "examples", "py_pubsub"), proj)
        w = mw.RosVisualRunner()
        w.on_load_project_init(str(proj))
        logs = []
        w.system_log = logs.append
        w._logs = logs
        yield w, proj
        w._mark_clean()
        w.close()

    def _talker(self, w):
        return next(n for g in w.graphs.all() for n in g.all_nodes()
                    if (n.get_property('source_file') if n.has_property('source_file') else "") == "talker.py")

    def test_rescan_reads_file_from_code_dir(self, loaded):
        w, proj = loaded
        node = self._talker(w)
        path = proj / "src" / "python" / "talker.py"
        code = path.read_text(encoding="utf-8").replace(
            "create_publisher(String, '/chatter', 10)",
            "create_publisher(String, '/chatter', 10)\n        self.p2 = self.create_publisher(String, 'status', 10)")
        assert "'status'" in code
        path.write_text(code, encoding="utf-8")
        node.set_selected(True)
        w.on_manual_rescan(node.graph)
        assert "status" in [p.name() for p in node.output_ports()]
        assert any("talker.py" in m for m in w._logs)

    def test_rescan_reports_missing_file(self, loaded):
        w, proj = loaded
        node = self._talker(w)
        os.remove(proj / "src" / "python" / "talker.py")
        node.set_selected(True)
        w.on_manual_rescan(node.graph)
        assert any("not found" in m for m in w._logs)


class TestNodeMenuCommands:
    """Restart/Freeze from the menu: the right spec, off the UI thread, no crash without Docker."""

    def _node(self, win, type_="ros.cpp.CppCustomNode", source="talker.cpp"):
        g = win.graphs.all()[0]
        n = g.create_node(type_, name="Talker")
        if not n.has_property("source_file"):
            n.create_property("source_file", source)
        n.set_property("source_file", source)
        n.set_selected(True)
        return g, n

    def test_restart_passes_launch_spec_in_background(self, win):
        calls = []
        win.process_manager = types.SimpleNamespace(
            restart_node=lambda name, log, rebuild_cpp=False, spec=None: calls.append((name, rebuild_cpp, spec)))
        g, n = self._node(win)
        win.on_restart_selected(g)
        win._bg_tasks.wait_all(5000)
        pkg = os.path.basename(win.current_project_path)
        assert calls == [("Talker", True, {"language": "cpp", "package_name": pkg, "entry": "talker"})]

    def test_freeze_without_docker_reports(self, win):
        from core.process_manager import ProcessManager
        win.process_manager = ProcessManager(None)
        g, n = self._node(win, "ros.cpp.CppCustomNode", "cam.cpp")
        win._is_lifecycle_node = lambda node: True
        win.on_freeze_selected(g, "deactivate")
        win._bg_tasks.wait_all(5000)
        QtWidgets.QApplication.processEvents()
        assert any("Run" in m for m in win._logs)
