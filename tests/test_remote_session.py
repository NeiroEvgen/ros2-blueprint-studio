"""
Сквозной сценарий remote workspace на НАСТОЯЩЕМ sshd:
connect → sync → build (с ошибкой компиляции) → правка → sync → build ок.

colcon подменяется фейком через ros_setup: этот скрипт source-ится перед
сборкой и кладёт фейковый colcon в PATH. Фейк проверяет, что запущен в
workspace и что исходники реально доехали в src/<pkg>.
"""
import os
import stat

import pytest

from core.remote.profile import RemoteProfile
from core.remote.session import RemoteSession, package_name_for, NotConnected

FAKE_COLCON = r"""#!/bin/bash
echo "Starting >>> $PKG"
if [ ! -f "src/$PKG/cpp/Node.cpp" ]; then
  echo "fake-colcon: sources not synced into $(pwd)/src/$PKG" >&2; exit 9
fi
if grep -q BROKEN "src/$PKG/cpp/Node.cpp"; then
  echo "src/$PKG/cpp/Node.cpp:3:1: error: 'BROKEN' was not declared in this scope" >&2
  echo "Failed   <<< $PKG"
  exit 1
fi
echo "Finished <<< $PKG"
"""


@pytest.fixture
def remote_env(sshd, tmp_path):
    ws = tmp_path / "remote_ws"
    ws.mkdir()
    fakebin = tmp_path / "fakebin"
    fakebin.mkdir()
    colcon = fakebin / "colcon"
    colcon.write_text(FAKE_COLCON)
    colcon.chmod(colcon.stat().st_mode | stat.S_IEXEC)

    project = tmp_path / "My G1 Proj"
    (project / "src" / "cpp").mkdir(parents=True)
    (project / "src" / "cpp" / "Node.cpp").write_text("int main(){\n  return 0;\n}\n")
    pkg = package_name_for(str(project))

    setup = tmp_path / "setup.bash"
    setup.write_text(f'export PATH="{fakebin}:$PATH"\nexport PKG="{pkg}"\n')

    profile = RemoteProfile(host=sshd["host"], port=sshd["port"], user=sshd["user"],
                            key_path=sshd["key"], remote_workspace=str(ws),
                            ros_setup=str(setup))
    s = RemoteSession(str(project), profile, known_hosts=str(tmp_path / "kh"))
    yield s, project, ws, pkg
    s.disconnect()


def test_package_name_matches_generator_rule():
    assert package_name_for("/x/My G1 Proj") == "my_g1_proj"
    assert package_name_for("C:\\Users\\u\\mpd\\") in ("mpd", "c__users_u_mpd_")


def test_operations_require_connection(tmp_path):
    s = RemoteSession(str(tmp_path), RemoteProfile())
    with pytest.raises(NotConnected):
        s.sync()


def test_full_cycle(remote_env):
    s, project, ws, pkg = remote_env
    s.connect()
    assert s.connected

    r = s.sync()
    assert r.uploaded == ["cpp/Node.cpp"] and r.ok
    assert (ws / "src" / pkg / "cpp" / "Node.cpp").exists()

    lines = []
    assert s.build(lines.append) == 0
    assert lines[-1] == f"Finished <<< {pkg}"

    # ломаем код → ошибка компиляции должна прийти в лог целиком
    (project / "src" / "cpp" / "Node.cpp").write_text("int main(){\n  BROKEN;\n}\n")
    assert s.sync().uploaded == ["cpp/Node.cpp"]
    lines = []
    rc = s.build(lines.append)
    assert rc == 1
    assert any("error: 'BROKEN' was not declared" in l for l in lines)
    assert lines[-1] == f"Failed   <<< {pkg}"

    # чиним → зелёная сборка
    (project / "src" / "cpp" / "Node.cpp").write_text("int main(){\n  return 0;\n}\n")
    s.sync()
    assert s.build(lambda l: None) == 0


def test_build_before_sync_explains_what_to_do(remote_env):
    s, *_ = remote_env
    s.connect()
    from core.remote.runner import PreflightError
    with pytest.raises(PreflightError, match="Sync"):
        s.build(lambda l: None)


def test_sync_reports_to_log(remote_env):
    s, *_ = remote_env
    s.connect()
    log = []
    s.sync(on_line=log.append)
    assert any(l.startswith("Sync →") for l in log)


def test_disconnect(remote_env):
    s, *_ = remote_env
    s.connect()
    s.disconnect()
    assert not s.connected
