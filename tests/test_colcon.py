"""colcon: команда сборки и разбор вывода (фикстуры — по реальным логам Studio)."""
import re
import pytest
from core import colcon

OK = """Starting >>> demo_stage
Finished <<< demo_stage [1min 2s]

Summary: 1 package finished [1min 3s]
""".splitlines()

MISSING_HEADER = """Starting >>> demo_stage
/root/ros2_ws/src/user_project/cpp/task_executor_node.cpp:12:10: fatal error: trajectory_msgs/msg/joint_trajectory.hpp: No such file or directory
   12 | #include <trajectory_msgs/msg/joint_trajectory.hpp>
compilation terminated.
make[2]: *** [CMakeFiles/x.dir/build.make:76: x.o] Error 1
Failed   <<< demo_stage [5.2s, exited with code 2]

Summary: 0 packages finished [5.3s]
  1 package failed: demo_stage
""".splitlines()

PLAIN_SIG = """Starting >>> demo_stage
CMake Error at CMakeLists.txt:88 (target_link_libraries):
  The plain signature for target_link_libraries has already been used with the
  target "task_core".  All uses of target_link_libraries with a target must
  be either all-keyword or all-plain.
Failed   <<< demo_stage [1.0s, exited with code 1]
""".splitlines()

NO_SOURCE = """Starting >>> demo_stage
CMake Error at CMakeLists.txt:40 (add_executable):
  Cannot find source file:

    src/cpp/missing.cpp
CMake Error: Cannot find source file: missing.cpp
Failed   <<< demo_stage [0.9s, exited with code 1]
""".splitlines()


def test_ok():
    r = colcon.parse_output(OK)
    assert r.ok and r.built == ["demo_stage"] and not r.failed
    assert "OK" in r.summary()


def test_missing_header():
    r = colcon.parse_output(MISSING_HEADER)
    assert not r.ok and r.failed == ["demo_stage"]
    e = r.errors[0]
    assert e.file.endswith("task_executor_node.cpp") and e.line == 12
    assert "trajectory_msgs" in e.message
    assert "FAILED" in r.summary() and "task_executor_node.cpp:12" in r.summary()


def test_plain_signature():
    r = colcon.parse_output(PLAIN_SIG)
    assert not r.ok
    e = r.errors[0]
    assert e.kind == "cmake" and e.file == "CMakeLists.txt" and e.line == 88
    assert "plain signature" in e.message


def test_cmake_no_source():
    r = colcon.parse_output(NO_SOURCE)
    assert not r.ok
    assert any("Cannot find source file" in e.message for e in r.errors)


def test_ansi_stripped():
    r = colcon.parse_output(["\x1b[31mFailed   <<< pkg [1s]\x1b[0m"])
    assert r.failed == ["pkg"] and not r.ok


def test_empty_is_ok():
    assert colcon.parse_output([]).ok


def test_build_command_shape():
    c = colcon.build_command()
    assert "colcon build" in c and "--symlink-install" in c and "source /opt/ros/humble" in c
    assert "/root/ros2_ws" in c


def test_summary_limit():
    lines = [f"/a/b{i}.cpp:{i+1}:1: error: bad" for i in range(9)] + ["Failed <<< p [1s]"]
    s = colcon.parse_output(lines).summary(limit=3)
    assert "and 6 more" in s


# ── run_project_launch: упавшая сборка не должна приводить к запуску ──
class _Res:
    def __init__(self, code=0, out=()):
        self.exit_code, self.output = code, iter(out)


class FakeContainer:
    def __init__(self, build_lines, exit_code):
        self.cmds, self.build_lines, self.exit_code = [], build_lines, exit_code

    def exec_run(self, cmd, stream=False, tty=False, **kw):
        joined = cmd if isinstance(cmd, str) else " ".join(cmd)
        self.cmds.append(joined)
        if joined.startswith("test -f"):
            return _Res(0)
        if "colcon build" in joined:
            lines = [(l + "\n").encode() for l in self.build_lines] + [f"__COLCON_EXIT={self.exit_code}\n".encode()]
            return _Res(0, lines)
        if "ros2 launch" in joined:
            return _Res(0, [b"launched"])
        return _Res(0, [])


def _manager(container):
    from core.docker_manager import RosContainerManager as DockerManager
    m = DockerManager.__new__(DockerManager)
    m.container = container
    return m


def test_failed_build_aborts_launch():
    c = FakeContainer(MISSING_HEADER, 2)
    sys_log, ros_log = [], []
    _manager(c).run_project_launch(sys_log.append, ros_log.append)
    assert not any("ros2 launch" in x for x in c.cmds)
    assert ros_log == []
    assert any("launch aborted" in m for m in sys_log)
    assert any("trajectory_msgs" in m for m in sys_log)


def test_ok_build_launches():
    c = FakeContainer(OK, 0)
    sys_log, ros_log = [], []
    _manager(c).run_project_launch(sys_log.append, ros_log.append)
    assert any("ros2 launch" in x for x in c.cmds)
    assert "launched" in ros_log
    assert any("Build phase finished" in m for m in sys_log)


def test_nonzero_exit_without_parsable_errors_aborts():
    c = FakeContainer(["something odd"], 1)
    sys_log, ros_log = [], []
    _manager(c).run_project_launch(sys_log.append, ros_log.append)
    assert not any("ros2 launch" in x for x in c.cmds)
