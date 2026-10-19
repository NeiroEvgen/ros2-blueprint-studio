"""Skip apt/rosdep if package.xml and the extra packages have not changed (saves ~25 s per run)."""
import os
import stat
import subprocess
import types

import pytest

from tests.conftest import requires_posix_shell
from core.docker_manager import RosContainerManager, DEPS_STAMP_PREFIX

DIGEST = b"a" * 64 + b"  -\n"


class Fake:
    """exec_run double: remembers commands; the stamp "exists" by a flag."""
    def __init__(self, stamped=False, digest=DIGEST):
        self.stamped, self.digest, self.calls = stamped, digest, []

    def exec_run(self, cmd, **kw):
        self.calls.append(cmd)
        text = cmd if isinstance(cmd, str) else " ".join(cmd)
        if "sha256sum" in text:
            return types.SimpleNamespace(output=self.digest, exit_code=0 if self.digest else 1)
        if cmd == ["test", "-f"] or (isinstance(cmd, list) and cmd[:2] == ["test", "-f"]):
            return types.SimpleNamespace(output=b"", exit_code=0 if self.stamped else 1)
        if "CMakeLists.txt" in text:
            return types.SimpleNamespace(output=b"", exit_code=0)
        out = iter([b"__COLCON_EXIT=0\n"]) if kw.get("stream") else b""
        return types.SimpleNamespace(output=out, exit_code=0)


def run(fake, extra=None):
    m = RosContainerManager.__new__(RosContainerManager)
    m.container = fake
    logs = []
    m.run_project_launch(logs.append, lambda s: None, extra_apt_packages=extra)
    return logs


def dep_calls(fake):
    return [c for c in fake.calls if isinstance(c, list) and c[:2] == ["bash", "-c"] and "rosdep" in c[2]]


def test_unchanged_deps_are_skipped():
    f = Fake(stamped=True)
    logs = run(f)
    assert dep_calls(f) == []
    assert any("skipping" in l for l in logs)


def test_first_run_installs_and_marks():
    f = Fake(stamped=False)
    run(f)
    (cmd,) = dep_calls(f)
    assert "touch " + DEPS_STAMP_PREFIX in cmd[2] and "rosdep install" in cmd[2]


def test_fingerprint_depends_on_extra_packages_and_package_xml():
    m = RosContainerManager.__new__(RosContainerManager)
    m.container = Fake()
    a, b = m._deps_stamp([]), m._deps_stamp(["htop"])
    m.container = Fake(digest=b"b" * 64 + b"  -\n")
    c = m._deps_stamp([])
    assert len({a, b, c}) == 3


def test_no_fingerprint_means_always_install():
    f = Fake(stamped=True, digest=b"")
    run(f)
    (cmd,) = dep_calls(f)
    assert "touch" not in cmd[2]


def _fake_bin(tmp_path, apt_rc, rosdep_rc):
    d = tmp_path / "bin"; d.mkdir()
    for name, rc in (("apt-get", apt_rc), ("rosdep", rosdep_rc), ("sleep", 0)):
        p = d / name
        p.write_text(f"#!/bin/sh\nexit {rc}\n")
        p.chmod(p.stat().st_mode | stat.S_IEXEC)
    return d


@requires_posix_shell
@pytest.mark.parametrize("apt_rc,rosdep_rc,marked", [(0, 0, True), (0, 1, False), (100, 0, False)])
def test_stamp_only_written_when_installs_succeed(tmp_path, apt_rc, rosdep_rc, marked):
    f = Fake(stamped=False)
    run(f)
    script = dep_calls(f)[0][2].replace("/root/ros2_ws/.studio_deps_", str(tmp_path / "stamp_"))
    script = script.replace("/root/ros2_ws/src", str(tmp_path)).replace("/tmp/rosdep_out.log", str(tmp_path / "log"))
    bindir = _fake_bin(tmp_path, apt_rc, rosdep_rc)
    subprocess.run(["bash", "-c", script], env={**os.environ, "PATH": f"{bindir}:{os.environ['PATH']}"},
                   capture_output=True, timeout=60)
    assert any(p.name.startswith("stamp_") for p in tmp_path.iterdir()) is marked
