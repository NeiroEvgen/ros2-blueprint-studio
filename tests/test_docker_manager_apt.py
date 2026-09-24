"""
1.3 — интеграция apt_safe в docker_manager, без настоящего Docker.

FakeContainer записывает все exec_run и отвечает заготовками. Сгенерированные
команды затем реально гоняются в bash с поддельными apt-get/rosdep —
проверяем, что композиция apt → rosdep работает, а не только собирается.
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
    m = RosContainerManager.__new__(RosContainerManager)   # без подключения к Docker
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
        """В apt не должно попасть ничего, кроме известных пакетов сессии."""
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
        assert any("недопустимые" in m for m in msgs)

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
        raise AssertionError("dep_cmd не найден")

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
        """Гоняем сгенерированный скрипт в bash с поддельными apt-get и rosdep."""
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
