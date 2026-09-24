"""
1.3 — apt_safe.

Сниппет исполняется в НАСТОЯЩЕМ bash с поддельным apt-get на PATH:
проверяем поведение (ретраи, коды выхода), а не текст команды.

Плюс критерий приёмки: в docker_manager.py не осталось прямых вызовов
apt-get — все рантайм-вызовы идут через core/apt_safe.
"""
import os
import re
import stat
import subprocess
import pytest

from tests.conftest import requires_posix_shell
from core.apt_safe import (
    apt_snippet, apt_cmd, validate_packages, missing_check_snippet,
    InvalidPackageName,
)

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _fake_apt(tmp_path, script_body):
    bindir = tmp_path / "bin"
    bindir.mkdir()
    fake = bindir / "apt-get"
    fake.write_text("#!/bin/bash\n" + script_body, encoding="utf-8")
    fake.chmod(fake.stat().st_mode | stat.S_IEXEC)
    return bindir


def _run(snippet, bindir, tmp_path):
    env = dict(os.environ, PATH=f"{bindir}:{os.environ['PATH']}",
               COUNTER=str(tmp_path / "counter"))
    return subprocess.run(["bash", "-c", snippet], env=env,
                          capture_output=True, text=True, timeout=30)


LOCK_THEN_OK = """
n=$(cat "$COUNTER" 2>/dev/null || echo 0); n=$((n+1)); echo $n > "$COUNTER"
if [ $n -le 2 ]; then
  echo "E: Could not get lock /var/lib/dpkg/lock-frontend" >&2; exit 100
fi
echo "ok: $@"; exit 0
"""

ALWAYS_LOCKED = """
echo "E: Could not get lock /var/lib/dpkg/lock-frontend" >&2; exit 100
"""

REAL_ERROR = """
n=$(cat "$COUNTER" 2>/dev/null || echo 0); n=$((n+1)); echo $n > "$COUNTER"
echo "E: Unable to locate package $3" >&2; exit 100
"""


@requires_posix_shell
class TestBehaviour:

    def test_retries_on_lock_then_succeeds(self, tmp_path):
        bindir = _fake_apt(tmp_path, LOCK_THEN_OK)
        r = _run(apt_snippet(update=True, delay=0), bindir, tmp_path)
        assert r.returncode == 0
        assert "lock busy, retry" in r.stdout
        assert (tmp_path / "counter").read_text().strip() == "3"

    def test_real_error_fails_fast_without_retry(self, tmp_path):
        """Опечатка в имени пакета не должна крутиться минуту."""
        bindir = _fake_apt(tmp_path, REAL_ERROR)
        r = _run(apt_snippet(["ros-humble-typo"], update=False, delay=0),
                 bindir, tmp_path)
        assert r.returncode != 0
        assert (tmp_path / "counter").read_text().strip() == "1"
        assert "retry" not in r.stdout

    def test_gives_up_after_retries(self, tmp_path):
        bindir = _fake_apt(tmp_path, ALWAYS_LOCKED)
        r = _run(apt_snippet(update=True, retries=3, delay=0), bindir, tmp_path)
        assert r.returncode != 0
        assert r.stdout.count("lock busy") == 3

    def test_install_skipped_if_update_failed(self, tmp_path):
        bindir = _fake_apt(tmp_path, 'echo "E: $1 broken" >&2; exit 1\n')
        r = _run(apt_snippet(["foo"], update=True, delay=0), bindir, tmp_path)
        assert r.returncode != 0
        assert "install" not in r.stdout

    def test_success_path_runs_update_and_install(self, tmp_path):
        bindir = _fake_apt(tmp_path, 'echo "ran: $@"; exit 0\n')
        r = _run(apt_snippet(["pkg-a", "pkg-b"], update=True, delay=0), bindir, tmp_path)
        assert r.returncode == 0
        assert "ran: update" in r.stdout
        assert "ran: install -y pkg-a pkg-b" in r.stdout

    def test_noninteractive(self):
        assert "DEBIAN_FRONTEND=noninteractive" in apt_snippet(["x1"])

    def test_cmd_is_argv_list(self):
        cmd = apt_cmd(["pkg-a"])
        assert cmd[:2] == ["bash", "-c"] and len(cmd) == 3

    def test_snippet_has_no_single_quotes(self):
        """Можно вкладывать в bash -c '...' при композиции с rosdep."""
        assert "'" not in apt_snippet(["pkg-a"], update=True)


class TestValidation:

    @pytest.mark.parametrize("name", [
        "ros-humble-foxglove-bridge", "libopencv-dev", "g++", "python3.10",
        "libstdc++6", "foo=1.2.3-1ubuntu2",
    ])
    def test_valid_names(self, name):
        assert validate_packages([name]) == [name]

    @pytest.mark.parametrize("name", [
        "foo; rm -rf /", "foo'bar", 'foo"bar', "$(whoami)", "foo bar",
        "Foo", "-y", "foo`id`",
    ])
    def test_rejects_injection_and_garbage(self, name):
        with pytest.raises(InvalidPackageName):
            validate_packages([name])

    def test_dedup_and_sort(self):
        assert validate_packages(["b-pkg", "a-pkg", "b-pkg", " "]) == ["a-pkg", "b-pkg"]


@requires_posix_shell
class TestMissingCheck:

    def test_reports_only_missing(self, tmp_path):
        bindir = tmp_path / "bin"; bindir.mkdir()
        dpkg = bindir / "dpkg"
        dpkg.write_text('#!/bin/bash\n[ "$2" = "have-it" ] && exit 0; exit 1\n')
        dpkg.chmod(dpkg.stat().st_mode | stat.S_IEXEC)
        env = dict(os.environ, PATH=f"{bindir}:{os.environ['PATH']}")
        r = subprocess.run(["bash", "-c", missing_check_snippet(["have-it", "need-it"])],
                           env=env, capture_output=True, text=True)
        assert r.stdout.split() == ["need-it"]


class TestAcceptance:

    def test_no_direct_apt_get_in_docker_manager(self):
        """Критерий приёмки 1.3."""
        src = open(os.path.join(ROOT, "core", "docker_manager.py"), encoding="utf-8").read()
        hits = [l.strip() for l in src.splitlines() if "apt-get" in l]
        assert hits == [], f"прямые вызовы apt-get: {hits}"

    def test_no_fuser_lock_polling_left(self):
        src = open(os.path.join(ROOT, "core", "docker_manager.py"), encoding="utf-8").read()
        assert "fuser" not in src
