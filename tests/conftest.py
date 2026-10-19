"""
Shared test infrastructure.

The tests run PURE generator functions: no Qt, no Docker, no ROS.
ProjectManager is created with graph_py=None, graph_cpp=None: the methods
_generate_cpp_build_files / _flatten_nodes / _process_and_save_files
do not touch the graphs, they only need session data and the file system.
"""
import os
import sys
import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)


# Some tests execute generated bash scripts with a fake apt-get.
# Windows has neither bash nor chmod +x, so those tests are skipped there.
requires_posix_shell = pytest.mark.skipif(
    os.name == "nt", reason="needs a POSIX shell (bash); skipped on Windows")


@pytest.fixture
def pm():
    from core.project_manager import ProjectManager
    return ProjectManager(None, None)


@pytest.fixture
def cpp_project(tmp_path):
    """
    Minimal C++ project structure of Studio:
        <tmp>/my_proj/src/cpp/
    Returns the function add(filename, code) and the path to src.
    """
    src = tmp_path / "my_proj" / "src"
    (src / "cpp").mkdir(parents=True)
    (src / "launch").mkdir()

    def add(filename, code):
        (src / "cpp" / filename).write_text(code, encoding="utf-8")

    return src, add


MAIN_CPP = """#include "rclcpp/rclcpp.hpp"
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::shutdown();
  return 0;
}
"""

HELPER_CPP = """#include "rclcpp/rclcpp.hpp"
// helper without main: called from another node
double helper(double x) { return x * 2.0; }
"""


# ------------------------------------------------------------------
# A real SSH server for the remote workspace tests.
# Without sshd on the system (a typical Windows machine) the tests using this
# fixture are skipped, not failed.
# ------------------------------------------------------------------
import shutil as _shutil
import socket as _socket
import subprocess as _subprocess
import time as _time


def _free_port():
    s = _socket.socket(); s.bind(("127.0.0.1", 0)); p = s.getsockname()[1]; s.close()
    return p


@pytest.fixture(scope="session")
def sshd(tmp_path_factory):
    sshd_bin = _shutil.which("sshd") or ("/usr/sbin/sshd" if os.path.exists("/usr/sbin/sshd") else None)
    if not sshd_bin or os.name == "nt":
        pytest.skip("sshd not available: tests with real SSH are skipped")
    try:
        import paramiko
    except ImportError:
        pytest.skip("paramiko is not installed")

    try:
        os.makedirs("/run/sshd", exist_ok=True)   # privsep dir, otherwise sshd silently does not start
    except OSError:
        pass
    d = tmp_path_factory.mktemp("sshd")
    host_key = d / "host_key"
    _subprocess.run(["ssh-keygen", "-q", "-t", "ed25519", "-N", "", "-f", str(host_key)], check=True)
    client_key = d / "client_key"
    _subprocess.run(["ssh-keygen", "-q", "-t", "ed25519", "-N", "", "-f", str(client_key)], check=True)
    auth = d / "authorized_keys"
    auth.write_text((d / "client_key.pub").read_text())
    os.chmod(auth, 0o600)

    port = _free_port()
    user = os.environ.get("USER") or __import__("getpass").getuser()
    cfg = d / "sshd_config"
    cfg.write_text(
        f"Port {port}\nListenAddress 127.0.0.1\nHostKey {host_key}\n"
        f"AuthorizedKeysFile {auth}\nPasswordAuthentication no\n"
        f"PermitRootLogin prohibit-password\nStrictModes no\nUsePAM no\n"
        f"PidFile {d}/sshd.pid\nSubsystem sftp internal-sftp\n")
    proc = _subprocess.Popen([sshd_bin, "-D", "-e", "-f", str(cfg)],
                             stdout=_subprocess.PIPE, stderr=_subprocess.STDOUT)
    for _ in range(50):
        try:
            _socket.create_connection(("127.0.0.1", port), timeout=0.2).close()
            break
        except OSError:
            _time.sleep(0.1)
    else:
        proc.kill()
        pytest.skip("sshd did not start")

    info = {"host": "127.0.0.1", "port": port, "user": user,
            "key": str(client_key), "host_key_pub": (d / "host_key.pub").read_text(),
            "dir": d}
    yield info
    proc.terminate()
    try:
        proc.wait(timeout=5)
    except Exception:
        proc.kill()


@pytest.fixture(autouse=True)
def _english_ui():
    """Tests check English texts; the developer's system locale must not matter."""
    from core import i18n
    i18n.set_language("en")
    yield
    i18n.set_language("en")


@pytest.fixture(autouse=True)
def _isolated_settings(tmp_path, monkeypatch):
    """Tests never read or write the real ~/.ros_blueprint_studio."""
    monkeypatch.setenv("STUDIO_SETTINGS_DIR", str(tmp_path / "_settings"))
    # Most tests describe the two-tab mode; the single canvas is enabled explicitly.
    from core import settings
    settings.set("single_canvas", False)
