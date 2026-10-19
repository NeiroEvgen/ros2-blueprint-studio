"""
Restart/Freeze from the context menu.

Restart used to always answer "not tracked": nodes start from the launch file, and
ProcessManager only knew the ones it started itself. Freeze without Docker or without a session
failed with AttributeError on None. Docker faked.
"""
import re
import types

import pytest

from core.process_manager import ProcessManager, node_spec, _kill_pattern


class FakeContainer:
    def __init__(self, build_rc=0):
        self.calls = []
        self.build_rc = build_rc

    def exec_run(self, cmd, detach=False):
        script = cmd[-1] if isinstance(cmd, list) else cmd
        self.calls.append((script, detach))
        rc = self.build_rc if "colcon build" in script else 0
        return types.SimpleNamespace(exit_code=rc, output=b"ok")


def pm_with(container):
    return ProcessManager(types.SimpleNamespace(container=container))


class TestSpec:
    def test_cpp_runs_by_file_stem(self):
        assert node_spec("ros.cpp.CppCustomNode", "talker.cpp", "demo") == \
            {"language": "cpp", "package_name": "demo", "entry": "talker"}

    def test_python_runs_script_in_container(self):
        assert node_spec("ros.py.PyStringPubNode", "talker.py", "demo")["entry"] == \
            "/root/ros2_ws/src/user_project/python/talker.py"

    def test_python_bound_file_uses_its_path(self):
        assert node_spec("ros.py.PyStringPubNode", "tracker.py", "demo", "lib/vision/tracker.py")["entry"] == \
            "/root/ros2_ws/src/user_project/lib/vision/tracker.py"

    def test_no_source_file(self):
        assert node_spec("ros.cpp.CppCustomNode", "", "demo") is None


class TestKillPattern:
    def test_cpp_does_not_hit_similar_names(self):
        pat = _kill_pattern(node_spec("ros.cpp.X", "talker.cpp", "demo"))
        assert re.search(pat, "/root/ros2_ws/install/demo/lib/demo/talker --ros-args -r __node:=talker")
        assert not re.search(pat, "/root/ros2_ws/install/demo/lib/demo/talker_2 --ros-args")
        assert not re.search(pat, "ros2 launch /root/ros2_ws/src/user_project/launch/project_launch.py")

    def test_python_matches_launch_and_restart_forms(self):
        pat = _kill_pattern(node_spec("ros.py.X", "talker.py", "demo"))
        assert re.search(pat, "python3 -u /root/ros2_ws/src/user_project/python/talker.py")
        assert not re.search(pat, "python3 -u /root/ros2_ws/src/user_project/python/talker.py.bak")


class TestRestart:
    def test_launch_started_node_restarts_with_spec(self):
        c = FakeContainer()
        logs = []
        spec = node_spec("ros.cpp.X", "talker.cpp", "demo")
        assert pm_with(c).restart_node("talker", logs.append, rebuild_cpp=True, spec=spec) is True
        scripts = [s for s, _ in c.calls]
        assert "colcon build" in scripts[0] and "--packages-select demo" in scripts[0]
        assert scripts[1].startswith("pkill -f")
        assert "ros2 run demo talker" in scripts[2] and c.calls[2][1] is True   # detach

    def test_failed_build_does_not_restart(self):
        c = FakeContainer(build_rc=2)
        spec = node_spec("ros.cpp.X", "talker.cpp", "demo")
        assert pm_with(c).restart_node("talker", lambda m: None, rebuild_cpp=True, spec=spec) is False
        assert len(c.calls) == 1

    def test_without_spec_and_not_tracked(self):
        logs = []
        assert pm_with(FakeContainer()).restart_node("x", logs.append) is False
        assert "not tracked" in logs[0]


@pytest.mark.parametrize("docker", [None, types.SimpleNamespace(container=None)])
def test_no_docker_or_no_session_is_reported_not_crashed(docker):
    pm = ProcessManager(docker)
    logs = []
    spec = node_spec("ros.py.X", "talker.py", "demo")
    assert pm.restart_node("talker", logs.append, spec=spec) is False
    assert pm.set_lifecycle_state("talker", "deactivate", logs.append) is False
    assert pm.get_lifecycle_state("talker") == ""
    assert len(logs) == 2 and all("Run" in m for m in logs)


def test_lifecycle_name_is_quoted():
    c = FakeContainer()
    assert pm_with(c).set_lifecycle_state("cam node", "activate", lambda m: None) is True
    assert "ros2 lifecycle set '/cam node' activate" in c.calls[0][0]
