"""
2.3 — удалённая сборка и запуск.

Критерии приёмки:
- ошибка компиляции видна в System Log ПОЛНОСТЬЮ (stderr не теряется,
  порядок сохранён)
- прерывание из UI реально останавливает процесс на удалённой машине —
  всё дерево, а не только верхний shell
- запущенная нода видна в списке процессов студии

Гоняется против НАСТОЯЩЕГО sshd.
"""
import shlex
import subprocess
import threading
import time
import types
import uuid

import pytest

from core.remote.profile import RemoteProfile
from core.remote.runner import RemoteRunner, PreflightError


@pytest.fixture
def client(sshd):
    import paramiko
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(sshd["host"], sshd["port"], sshd["user"], key_filename=sshd["key"],
              look_for_keys=False, allow_agent=False)
    yield c
    c.close()


def _runner(client, tmp_path, **kw):
    ws = kw.pop("ws", str(tmp_path))
    return RemoteRunner(client, RemoteProfile(host="h", user="u", remote_workspace=ws, **kw),
                        package_name="mpd")


def _pgrep(pattern):
    r = subprocess.run(["pgrep", "-f", pattern], capture_output=True, text=True)
    return [p for p in r.stdout.split() if p]


class TestStreaming:

    def test_lines_streamed_in_order(self, client, tmp_path):
        lines = []
        rc = _runner(client, tmp_path).run_streaming(
            "echo one; echo two; echo three", lines.append)
        assert rc == 0
        assert lines == ["one", "two", "three"]

    def test_stderr_merged_in_order(self, client, tmp_path):
        """Ошибка компиляции идёт в stderr — она не должна потеряться."""
        lines = []
        _runner(client, tmp_path).run_streaming(
            "echo 'compiling Node.cpp'; "
            "echo \"Node.cpp:12:5: error: 'foo' was not declared\" >&2; "
            "echo 'make: *** Error 1'", lines.append)
        assert lines == ["compiling Node.cpp",
                         "Node.cpp:12:5: error: 'foo' was not declared",
                         "make: *** Error 1"]

    def test_exit_code_propagated(self, client, tmp_path):
        assert _runner(client, tmp_path).run_streaming("exit 7", lambda l: None) == 7

    def test_long_output_not_truncated(self, client, tmp_path):
        lines = []
        _runner(client, tmp_path).run_streaming("seq 1 20000", lines.append)
        assert len(lines) == 20000 and lines[-1] == "20000"

    def test_cyrillic_output(self, client, tmp_path):
        lines = []
        _runner(client, tmp_path).run_streaming(
            "echo 'ошибка: не найден заголовок'", lines.append)
        assert lines == ["ошибка: не найден заголовок"]

    def test_last_line_without_newline(self, client, tmp_path):
        lines = []
        _runner(client, tmp_path).run_streaming("printf 'a\\nb'", lines.append)
        assert lines == ["a", "b"]

    def test_pgid_marker_not_leaked(self, client, tmp_path):
        lines = []
        _runner(client, tmp_path).run_streaming("echo hi", lines.append)
        assert not any("__BP_PGID__" in l for l in lines)


class TestCancel:

    def test_cancel_kills_whole_process_tree(self, client, tmp_path):
        """Критерий: не только верхний shell, а colcon→make→gcc целиком."""
        tag = f"{uuid.uuid4().int % 1000}.{uuid.uuid4().int % 900 + 100}"
        r = _runner(client, tmp_path)
        lines, result = [], {}
        script = (f"sleep 3{tag} & sleep 4{tag} & echo started; sleep 5{tag}; wait")
        t = threading.Thread(target=lambda: result.update(
            rc=r.run_streaming(script, lines.append)))
        t.start()
        for _ in range(100):
            if "started" in lines:
                break
            time.sleep(0.05)
        assert _pgrep(f"sleep [345]{tag}"), "процессы должны были стартовать"

        started = time.time()
        assert r.cancel() is True
        t.join(timeout=10)

        assert not t.is_alive(), "run_streaming не вернулся после отмены"
        assert time.time() - started < 8
        assert _pgrep(f"sleep [345]{tag}") == [], "в дереве остались живые процессы"

    def test_cancel_without_running_is_noop(self, client, tmp_path):
        assert _runner(client, tmp_path).cancel() is False


class TestBackgroundNodes:

    def test_named_process_listed_and_stoppable(self, client, tmp_path):
        """Критерий: запущенная нода видна в списке процессов."""
        tag = f"6{uuid.uuid4().int % 1000}.25"
        r = _runner(client, tmp_path)
        t = threading.Thread(target=lambda: r.run_streaming(
            f"echo up; sleep {tag}", lambda l: None, name="TrajectoryPlayer"))
        t.start()
        for _ in range(100):
            if r.list_running():
                break
            time.sleep(0.05)
        assert list(r.list_running()) == ["TrajectoryPlayer"]

        assert r.stop("TrajectoryPlayer") is True
        t.join(timeout=10)
        assert r.list_running() == {}
        assert _pgrep(f"sleep {tag}") == []


class TestPreflight:

    def test_missing_workspace(self, client, tmp_path):
        r = _runner(client, tmp_path, ws=f"/tmp/nope-{uuid.uuid4().hex}")
        with pytest.raises(PreflightError, match="нет каталога"):
            r.preflight()

    def test_missing_src_says_sync_first(self, client, tmp_path):
        with pytest.raises(PreflightError, match="Sync"):
            _runner(client, tmp_path).preflight()

    def test_missing_ros(self, client, tmp_path):
        (tmp_path / "src").mkdir()
        r = _runner(client, tmp_path, ros_setup="/opt/ros/nonexistent/setup.bash")
        with pytest.raises(PreflightError, match="не найден"):
            r.preflight()

    def test_missing_colcon(self, client, tmp_path):
        (tmp_path / "src").mkdir()
        setup = tmp_path / "setup.bash"
        setup.write_text("# fake ros\n")
        r = _runner(client, tmp_path, ros_setup=str(setup))
        if subprocess.run(["bash", "-c", "command -v colcon"]).returncode == 0:
            pytest.skip("colcon установлен в тестовой среде")
        with pytest.raises(PreflightError, match="colcon"):
            r.preflight()

    def test_build_refuses_without_preflight_pass(self, client, tmp_path):
        with pytest.raises(PreflightError):
            _runner(client, tmp_path).build(lambda l: None)


class FakeSSH:
    def __init__(self):
        self.commands = []

    def exec_command(self, command, timeout=None):
        self.commands.append(command)
        ch = types.SimpleNamespace(recv_exit_status=lambda: 1)
        return (types.SimpleNamespace(), types.SimpleNamespace(read=lambda: b"", channel=ch),
                types.SimpleNamespace(read=lambda: b""))


class TestContainerWrapping:

    def _r(self):
        ssh = FakeSSH()
        prof = RemoteProfile(host="h", user="u", remote_workspace="/home/u/ws",
                             container="g1pilot_run", container_workspace="/ros2_ws")
        return RemoteRunner(ssh, prof, "mpd"), ssh

    def test_kill_goes_through_same_container(self):
        """docker exec без -t не убивает процесс внутри — kill обязан идти туда же."""
        r, ssh = self._r()
        r._current_pgid = 4242
        r._kill_group(4242, grace=0)
        assert ssh.commands
        for cmd in ssh.commands:
            argv = shlex.split(cmd)
            assert argv[:4] == ["docker", "exec", "-i", "g1pilot_run"]
            assert "-- -4242" in argv[-1]

    def test_build_runs_in_container_workspace(self):
        r, _ = self._r()
        assert "cd /ros2_ws" in r.build_script()
        assert "/home/u/ws" not in r.build_script()

    def test_build_packages_select(self):
        r, _ = self._r()
        assert r.build_script(["mpd"]).endswith("--packages-select mpd")

    def test_run_script_quotes_args(self):
        r, _ = self._r()
        s = r.run_script("TrajectoryPlayerNode", ["--ros-args", "-p", "x:=a b"])
        assert "ros2 run mpd TrajectoryPlayerNode --ros-args -p 'x:=a b'" in s
