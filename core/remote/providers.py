"""
FileProvider — единый интерфейс к файлам, где бы они ни лежали.

Реализации:
  LocalProvider         — файлы на машине со студией
  SSHProvider           — файлы на удалённом хосте (SFTP)
  SSHContainerProvider  — файлы внутри контейнера на удалённом хосте
                          (docker exec через SSH-канал; ровно случай
                          g1pilot_run на Jetson)

Отдельного провайдера для ЛОКАЛЬНОГО контейнера нет намеренно: контейнер
сессии студии монтирует src проекта (bind mount), его файлы — это и есть
локальные файлы, LocalProvider их покрывает.

Запись везде атомарная: сначала во временный файл рядом, потом rename.
Оборвалась связь посреди сохранения — на роботе останется старая версия
файла, а не половина новой.
"""
import hashlib
import os
import posixpath
import shlex
import stat as _stat
import threading
import uuid
from dataclasses import dataclass


@dataclass(frozen=True)
class Entry:
    name: str
    path: str
    is_dir: bool
    size: int = 0
    mtime: float = 0.0


def _sort(entries):
    return sorted(entries, key=lambda e: (not e.is_dir, e.name.lower()))


class FileProvider:
    kind = "abstract"
    sep = "/"

    def label(self):
        return self.kind

    def join(self, *parts):
        return posixpath.join(*parts)

    def dirname(self, path):
        return posixpath.dirname(path)

    def list_dir(self, path):
        raise NotImplementedError

    def read_bytes(self, path):
        raise NotImplementedError

    def write_bytes(self, path, data):
        raise NotImplementedError

    def exists(self, path):
        raise NotImplementedError

    def makedirs(self, path):
        raise NotImplementedError

    def remove(self, path):
        raise NotImplementedError

    def sha1(self, path):
        return hashlib.sha1(self.read_bytes(path)).hexdigest()

    def read_text(self, path):
        return self.read_bytes(path).decode("utf-8")

    def write_text(self, path, text):
        self.write_bytes(path, text.encode("utf-8"))


# ======================================================================
#  Local
# ======================================================================

class LocalProvider(FileProvider):
    kind = "local"
    sep = os.sep

    def join(self, *parts):
        return os.path.join(*parts)

    def dirname(self, path):
        return os.path.dirname(path)

    def list_dir(self, path):
        out = []
        with os.scandir(path) as it:
            for de in it:
                try:
                    st = de.stat()
                    out.append(Entry(de.name, de.path, de.is_dir(), st.st_size, st.st_mtime))
                except OSError:
                    continue
        return _sort(out)

    def read_bytes(self, path):
        with open(path, "rb") as f:
            return f.read()

    def write_bytes(self, path, data):
        parent = os.path.dirname(path)
        if parent:
            os.makedirs(parent, exist_ok=True)
        tmp = f"{path}.tmp-{uuid.uuid4().hex[:8]}"
        with open(tmp, "wb") as f:
            f.write(data)
        os.replace(tmp, path)

    def exists(self, path):
        return os.path.exists(path)

    def makedirs(self, path):
        os.makedirs(path, exist_ok=True)

    def remove(self, path):
        if os.path.exists(path):
            os.remove(path)


# ======================================================================
#  SSH (SFTP до хоста)
# ======================================================================

class SSHProvider(FileProvider):
    """
    SFTP-канал paramiko НЕ потокобезопасен: UI одновременно гоняет дерево
    файлов, открытие файла и Sync в разных потоках — без блокировки сессия
    виснет или падает с 'Server connection dropped'. Поэтому все SFTP-операции
    сериализуются. sha1 идёт через exec_command (свой канал), блокировка не нужна.
    """
    kind = "ssh"

    def __init__(self, ssh_client, label=""):
        self.ssh = ssh_client
        self.sftp = ssh_client.open_sftp()
        self._label = label
        self._lock = threading.RLock()

    def label(self):
        return self._label or "ssh"

    def list_dir(self, path):
        with self._lock:
            attrs = self.sftp.listdir_attr(path)
        out = [Entry(a.filename, posixpath.join(path, a.filename),
                     _stat.S_ISDIR(a.st_mode or 0), a.st_size or 0, a.st_mtime or 0)
               for a in attrs]
        return _sort(out)

    def read_bytes(self, path):
        with self._lock:
            with self.sftp.open(path, "rb") as f:
                return f.read()

    def write_bytes(self, path, data):
        with self._lock:
            self.makedirs(posixpath.dirname(path))
            tmp = f"{path}.tmp-{uuid.uuid4().hex[:8]}"
            with self.sftp.open(tmp, "wb") as f:
                f.write(data)
            try:
                self.sftp.posix_rename(tmp, path)
            except IOError:
                # сервер без posix-rename@openssh.com: удаляем цель и переименовываем
                try:
                    self.sftp.remove(path)
                except IOError:
                    pass
                self.sftp.rename(tmp, path)

    def exists(self, path):
        with self._lock:
            try:
                self.sftp.stat(path)
                return True
            except IOError:
                return False

    def makedirs(self, path):
        if not path or path == "/":
            return
        with self._lock:
            parts = []
            cur = path
            while cur and cur != "/" and not self.exists(cur):
                parts.append(cur)
                cur = posixpath.dirname(cur)
            for p in reversed(parts):
                self.sftp.mkdir(p)

    def remove(self, path):
        with self._lock:
            try:
                self.sftp.remove(path)
            except IOError:
                pass

    def sha1(self, path):
        # считаем на удалённой стороне: не гоняем файл по сети ради хэша
        rc, out, _ = exec_remote(self.ssh, f"sha1sum {shlex.quote(path)}")
        if rc == 0 and out:
            return out.split()[0].decode()
        return super().sha1(path)

    def close(self):
        with self._lock:
            try:
                self.sftp.close()
            except Exception:
                pass


# ======================================================================
#  Контейнер на удалённом хосте (docker exec через SSH)
# ======================================================================

def exec_remote(ssh, command, stdin_data=None, timeout=60):
    """Выполнить команду на удалённой стороне: (rc, stdout, stderr)."""
    stdin, stdout, stderr = ssh.exec_command(command, timeout=timeout)
    if stdin_data is not None:
        stdin.write(stdin_data)
        stdin.channel.shutdown_write()
    out = stdout.read()
    err = stderr.read()
    rc = stdout.channel.recv_exit_status()
    return rc, out, err


class RemoteCommandError(RuntimeError):
    pass


class SSHExecProvider(FileProvider):
    """
    Файлы через shell-команды на удалённой стороне. prefix — чем обернуть
    команду: [] = прямо на хосте, ["docker","exec","-i",name] = в контейнере.
    """
    kind = "exec"

    def __init__(self, ssh_client, prefix=(), label=""):
        self.ssh = ssh_client
        self.prefix = list(prefix)
        self._label = label

    def label(self):
        return self._label or self.kind

    def _cmd(self, script):
        argv = self.prefix + ["sh", "-c", script]
        return " ".join(shlex.quote(a) for a in argv)

    def _run(self, script, stdin_data=None):
        rc, out, err = exec_remote(self.ssh, self._cmd(script), stdin_data)
        return rc, out, err

    def _check(self, script, stdin_data=None):
        rc, out, err = self._run(script, stdin_data)
        if rc != 0:
            raise RemoteCommandError(err.decode(errors="replace").strip() or f"rc={rc}")
        return out

    def list_dir(self, path):
        q = shlex.quote(path)
        # Разделитель — NUL в конце записи и TAB внутри: имена с пробелами
        # и даже с переводом строки не ломают разбор.
        out = self._check(
            f"find {q} -mindepth 1 -maxdepth 1 -printf '%y\\t%s\\t%T@\\t%f\\0'")
        entries = []
        for rec in out.split(b"\0"):
            if not rec:
                continue
            kind, size, mtime, name = rec.decode("utf-8", errors="replace").split("\t", 3)
            entries.append(Entry(name, posixpath.join(path, name), kind == "d",
                                 int(size or 0), float(mtime or 0)))
        return _sort(entries)

    def read_bytes(self, path):
        return self._check(f"cat {shlex.quote(path)}")

    def write_bytes(self, path, data):
        q = shlex.quote(path)
        tmp = shlex.quote(f"{path}.tmp-{uuid.uuid4().hex[:8]}")
        parent = shlex.quote(posixpath.dirname(path) or "/")
        self._check(f"mkdir -p {parent} && cat > {tmp} && mv -f {tmp} {q}", stdin_data=data)

    def exists(self, path):
        rc, _, _ = self._run(f"test -e {shlex.quote(path)}")
        return rc == 0

    def makedirs(self, path):
        self._check(f"mkdir -p {shlex.quote(path)}")

    def remove(self, path):
        self._run(f"rm -f {shlex.quote(path)}")

    def sha1(self, path):
        out = self._check(f"sha1sum {shlex.quote(path)}")
        return out.split()[0].decode()


class SSHContainerProvider(SSHExecProvider):
    kind = "container"

    def __init__(self, ssh_client, container, label=""):
        if not container:
            raise ValueError("не указан контейнер")
        super().__init__(ssh_client, ["docker", "exec", "-i", container],
                         label or f"{container}")
        self.container = container
