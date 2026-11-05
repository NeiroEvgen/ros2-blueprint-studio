"""
apt_safe integration in docker_manager, without real Docker.

FakeContainer records every exec_run and answers with canned replies. The generated
commands are then really run in bash with fake apt-get/rosdep:
we check that the apt -> rosdep composition works, not just that it is assembled.
"""
import os
import stat
import subprocess
import types
import pytest

from tests.conftest import requires_posix_shell
from core.docker_manager import RosContainerManager


class FakeContainer:
    def __init__(self, missing=b""):
        self.calls = []
        self.missing = missing

    def exec_run(self, cmd, **kw):
        self.calls.append((cmd, kw))
        if isinstance(cmd, list) and "dpkg -s" in cmd[-1]:
            return types.SimpleNamespace(output=self.missing, exit_code=0)
        return types.SimpleNamespace(output=iter([b"line\n"]) if kw.get("stream")
                                     else b"", exit_code=0)


def _manager(container):
    m = RosContainerManager.__new__(RosContainerManager)   # no Docker connection
    m.container = container
    return m


def _cmd_text(cmd):
    return cmd[-1] if isinstance(cmd, list) else cmd


class TestSessionPackages:

    def test_nothing_missing_means_no_apt(self):
        c = FakeContainer(missing=b"")
        installed = _manager(c)._install_session_packages()
        assert installed == []
        assert not any("apt-get" in _cmd_text(cmd) for cmd, _ in c.calls)
        assert any("session_setup_done" in _cmd_text(cmd) for cmd, _ in c.calls)

    def test_only_missing_packages_installed(self):
        c = FakeContainer(missing=b"ros-humble-foxglove-bridge\n")
        installed = _manager(c)._install_session_packages()
        assert installed == ["ros-humble-foxglove-bridge"]
        apt_calls = [(cmd, kw) for cmd, kw in c.calls if "apt-get install" in _cmd_text(cmd)]
        assert len(apt_calls) == 1
        text = _cmd_text(apt_calls[0][0])
        assert "ros-humble-foxglove-bridge" in text
        assert "ros-humble-turtlesim" not in text
        assert apt_calls[0][1].get("detach") is True
        assert "session_setup_done" in text

    def test_garbage_from_dpkg_output_ignored(self):
        """Nothing but the known session packages may get into apt."""
        c = FakeContainer(missing=b"ros-humble-turtlesim\n; rm -rf /\n")
        installed = _manager(c)._install_session_packages()
        assert installed == ["ros-humble-turtlesim"]


class TestInstallPackage:

    def test_rejects_injection(self):
        c = FakeContainer()
        msgs = []
        ok = _manager(c).install_package("foo; rm -rf /", "apt", msgs.append)
        assert ok is False
        assert c.calls == []
        assert any("invalid" in m for m in msgs)

    def test_valid_name_goes_through_apt_safe(self):
        c = FakeContainer()
        ok = _manager(c).install_package("libyaml-cpp-dev", "apt", lambda m: None)
        assert ok is True
        cmd, _ = c.calls[0]
        assert isinstance(cmd, list)
        assert "apt-get install -y libyaml-cpp-dev" in cmd[-1]
        assert "lock busy" in cmd[-1]


class TestRunProjectLaunchDeps:

    def _dep_cmd(self, extra):
        c = FakeContainer()
        m = _manager(c)
        log = []
        m.run_project_launch(log.append, lambda t: None, extra_apt_packages=extra)
        for cmd, _ in c.calls:
            if "rosdep install" in _cmd_text(cmd):
                return cmd, log
        raise AssertionError("dep_cmd not found")

    def test_dep_cmd_is_argv_list_with_apt_then_rosdep(self):
        cmd, _ = self._dep_cmd(["ros-humble-moveit"])
        assert isinstance(cmd, list)
        text = cmd[-1]
        assert text.index("apt-get install -y ros-humble-moveit") < text.index("rosdep install")

    def test_invalid_registry_name_skipped_not_crash(self):
        cmd, log = self._dep_cmd(["good-pkg", "bad pkg"])
        assert any("Skipping invalid" in l for l in log)

    @requires_posix_shell
    def test_composed_script_really_runs(self, tmp_path):
        """Run the generated script in bash with fake apt-get and rosdep."""
        cmd, _ = self._dep_cmd(["ros-humble-moveit"])
        bindir = tmp_path / "bin"; bindir.mkdir()
        for name, body in {
            "apt-get": 'echo "APT $@" >> "$TRACE"; exit 0\n',
            "rosdep": 'echo "ROSDEP $@" >> "$TRACE"; exit 0\n',
        }.items():
            f = bindir / name
            f.write_text("#!/bin/bash\n" + body)
            f.chmod(f.stat().st_mode | stat.S_IEXEC)
        trace = tmp_path / "trace"
        script = cmd[-1].replace("/tmp/rosdep_out.log", str(tmp_path / "rosdep.log"))
        env = dict(os.environ, PATH=f"{bindir}:{os.environ['PATH']}", TRACE=str(trace))
        subprocess.run(["bash", "-c", script], env=env, timeout=30)
        lines = trace.read_text().splitlines()
        assert lines[0] == "APT update"
        assert lines[1] == "APT install -y ros-humble-moveit"
        assert lines[2].startswith("ROSDEP install")


class LaunchChunks(FakeContainer):
    """ros2 launch output cut the way Docker delivers it: lines glued, split, CRLF from the tty."""
    CHUNKS = [b"[INFO] [launch]: Default logging verbosity is set to INFO\r\n[INFO] [python3-1]: process",
              b" started with pid [35]\r\n",
              b"[python3-1] [INFO] [123.4] [add_two_ints_server]: Incoming request: a: 2 b: 3\r\n[python3-2] [INF",
              b"O] [123.5] [add_two_ints_client]: Result of add_two_ints: for 2 + 3 = 5"]

    def exec_run(self, cmd, **kw):
        if kw.get("stream") and "ros2 launch" in _cmd_text(cmd):
            self.calls.append((cmd, kw))
            return types.SimpleNamespace(output=iter(self.CHUNKS), exit_code=0)
        return super().exec_run(cmd, **kw)


class TestLaunchOutput:
    def test_each_line_reaches_the_dashboard_whole(self):
        lines = []
        _manager(LaunchChunks()).run_project_launch(lambda m: None, lines.append)
        assert lines == [
            "[INFO] [launch]: Default logging verbosity is set to INFO\n",
            "[INFO] [python3-1]: process started with pid [35]\n",
            "[python3-1] [INFO] [123.4] [add_two_ints_server]: Incoming request: a: 2 b: 3\n",
            "[python3-2] [INFO] [123.5] [add_two_ints_client]: Result of add_two_ints: for 2 + 3 = 5\n",
        ]

    def test_dashboard_shows_one_row_per_node(self):
        os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
        qtw = pytest.importorskip("PySide6.QtWidgets")
        qtw.QApplication.instance() or qtw.QApplication([])
        from ui.dashboard import ConsoleDashboard
        dash = ConsoleDashboard()
        _manager(LaunchChunks()).run_project_launch(lambda m: None, dash.process_log)
        assert "Result of add_two_ints" in "".join(dash.history["python3-2"])
        assert "Incoming request" in "".join(dash.history["python3-1"])
        assert not any("set to INFO[INFO]" in "".join(h) for h in dash.history.values())   # nothing glued
