"""
Общая инфраструктура тестов.

Тесты гоняют ЧИСТЫЕ функции генераторов — без Qt, без Docker, без ROS.
ProjectManager создаётся с graph_py=None, graph_cpp=None: методы
_generate_cpp_build_files / _flatten_nodes / _process_and_save_files
графы не трогают, им нужны только данные сессии и файловая система.
"""
import os
import sys
import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)


# Часть тестов исполняет сгенерированные bash-скрипты с поддельным apt-get.
# На Windows нет ни bash, ни chmod +x — такие тесты там пропускаются.
requires_posix_shell = pytest.mark.skipif(
    os.name == "nt", reason="нужен POSIX-shell (bash); на Windows пропускается")


@pytest.fixture
def pm():
    from core.project_manager import ProjectManager
    return ProjectManager(None, None)


@pytest.fixture
def cpp_project(tmp_path):
    """
    Минимальная структура C++ проекта студии:
        <tmp>/my_proj/src/cpp/
    Возвращает функцию add(filename, code) и путь к src.
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
// хелпер без main: вызывается из другой ноды
double helper(double x) { return x * 2.0; }
"""


# ------------------------------------------------------------------
# Настоящий SSH-сервер для тестов remote workspace.
# Если sshd в системе нет (обычная Windows-машина) — тесты с этой
# фикстурой пропускаются, а не падают.
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
        pytest.skip("sshd недоступен — тесты с реальным SSH пропущены")
    try:
        import paramiko
    except ImportError:
        pytest.skip("paramiko не установлен")

    try:
        os.makedirs("/run/sshd", exist_ok=True)   # privsep dir, иначе sshd молча не стартует
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
        pytest.skip("sshd не поднялся")

    info = {"host": "127.0.0.1", "port": port, "user": user,
            "key": str(client_key), "host_key_pub": (d / "host_key.pub").read_text(),
            "dir": d}
    yield info
    proc.terminate()
    try:
        proc.wait(timeout=5)
    except Exception:
        proc.kill()
