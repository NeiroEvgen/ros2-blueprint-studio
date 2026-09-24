"""
2.4 (backend) — контрактные тесты FileProvider.

Один и тот же набор гоняется против каждой реализации:
  local — LocalProvider на tmp-папке
  ssh   — SSHProvider через НАСТОЯЩИЙ sshd (SFTP)
  exec  — SSHExecProvider через настоящий sshd: тот же код, что у
          SSHContainerProvider, только без префикса docker exec
          (Docker-демона в тестовой среде нет; сам префикс проверен
          отдельным юнит-тестом ниже)

Если sshd недоступен (обычная Windows-машина) — ssh/exec пропускаются.
"""
import hashlib
import os
import posixpath
import types
import uuid

import pytest

from core.remote.providers import (
    LocalProvider, SSHProvider, SSHExecProvider, SSHContainerProvider,
    RemoteCommandError,
)


@pytest.fixture
def ssh_client(sshd):
    import paramiko
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(sshd["host"], sshd["port"], sshd["user"], key_filename=sshd["key"],
              look_for_keys=False, allow_agent=False)
    yield c
    c.close()


@pytest.fixture(params=["local", "ssh", "exec"])
def prov(request, tmp_path):
    """(provider, root) — root уже существует и пуст."""
    if request.param == "local":
        root = str(tmp_path / "root")
        os.makedirs(root)
        yield LocalProvider(), root
        return
    client = request.getfixturevalue("ssh_client")
    root = f"/tmp/bp-test-{uuid.uuid4().hex[:10]}"
    os.makedirs(root)
    p = SSHProvider(client) if request.param == "ssh" else SSHExecProvider(client)
    yield p, root
    import shutil
    shutil.rmtree(root, ignore_errors=True)


class TestContract:

    def test_text_roundtrip_with_cyrillic_content(self, prov):
        p, root = prov
        path = p.join(root, "note.txt")
        p.write_text(path, "Привет, робот G1 — 🦾\n")
        assert p.read_text(path) == "Привет, робот G1 — 🦾\n"

    def test_cyrillic_and_space_in_names(self, prov):
        """Именно на таких путях ломался scp весь прошлую неделю."""
        p, root = prov
        path = p.join(root, "Евгений проект", "мой файл.cpp")
        p.write_text(path, "int main(){}\n")
        assert p.read_text(path) == "int main(){}\n"
        names = [e.name for e in p.list_dir(p.join(root, "Евгений проект"))]
        assert names == ["мой файл.cpp"]

    def test_write_creates_parent_dirs(self, prov):
        p, root = prov
        path = p.join(root, "a", "b", "c", "deep.txt")
        p.write_text(path, "x")
        assert p.exists(path)

    def test_overwrite(self, prov):
        p, root = prov
        path = p.join(root, "f.txt")
        p.write_text(path, "first")
        p.write_text(path, "second")
        assert p.read_text(path) == "second"

    def test_binary_roundtrip(self, prov):
        p, root = prov
        data = bytes(range(256)) * 10
        path = p.join(root, "blob.bin")
        p.write_bytes(path, data)
        assert p.read_bytes(path) == data

    def test_no_tmp_files_left_after_write(self, prov):
        p, root = prov
        p.write_text(p.join(root, "f.txt"), "data")
        p.write_text(p.join(root, "f.txt"), "data2")
        assert [e.name for e in p.list_dir(root)] == ["f.txt"]

    def test_list_dir_dirs_first_and_types(self, prov):
        p, root = prov
        p.write_text(p.join(root, "zeta.txt"), "12345")
        p.write_text(p.join(root, "alpha.txt"), "1")
        p.makedirs(p.join(root, "src"))
        entries = p.list_dir(root)
        assert [e.name for e in entries] == ["src", "alpha.txt", "zeta.txt"]
        assert entries[0].is_dir and not entries[1].is_dir
        assert {e.name: e.size for e in entries if not e.is_dir} == {"alpha.txt": 1,
                                                                      "zeta.txt": 5}

    def test_exists_and_remove(self, prov):
        p, root = prov
        path = p.join(root, "gone.txt")
        assert not p.exists(path)
        p.write_text(path, "x")
        assert p.exists(path)
        p.remove(path)
        assert not p.exists(path)

    def test_remove_missing_is_noop(self, prov):
        p, root = prov
        p.remove(p.join(root, "never-existed.txt"))

    def test_sha1_matches(self, prov):
        p, root = prov
        path = p.join(root, "h.txt")
        p.write_text(path, "hash me")
        assert p.sha1(path) == hashlib.sha1(b"hash me").hexdigest()

    def test_read_missing_raises(self, prov):
        p, root = prov
        with pytest.raises((IOError, OSError, RemoteCommandError)):
            p.read_bytes(p.join(root, "missing.txt"))


class FakeSSH:
    """Записывает команды, которые ушли бы на удалённую сторону."""
    def __init__(self):
        self.commands = []

    def exec_command(self, command, timeout=None):
        self.commands.append(command)
        ch = types.SimpleNamespace(recv_exit_status=lambda: 0, shutdown_write=lambda: None)
        out = types.SimpleNamespace(read=lambda: b"", channel=ch)
        err = types.SimpleNamespace(read=lambda: b"")
        inp = types.SimpleNamespace(write=lambda d: None, channel=ch)
        return inp, out, err


class TestContainerProvider:

    def test_commands_wrapped_in_docker_exec(self):
        ssh = FakeSSH()
        SSHContainerProvider(ssh, "g1pilot_run").exists("/ros2_ws/src/x.cpp")
        assert ssh.commands[0].startswith("docker exec -i g1pilot_run sh -c ")

    def test_path_with_quote_is_escaped(self):
        ssh = FakeSSH()
        SSHContainerProvider(ssh, "c").exists("/tmp/it's here.cpp")
        cmd = ssh.commands[0]
        # команда должна пережить путь с апострофом, не разорвав кавычки
        import shlex
        argv = shlex.split(cmd)
        assert argv[:5] == ["docker", "exec", "-i", "c", "sh"]
        # второй уровень: sh -c внутри контейнера распакует путь дословно
        assert shlex.split(argv[-1]) == ["test", "-e", "/tmp/it's here.cpp"]

    def test_container_required(self):
        with pytest.raises(ValueError):
            SSHContainerProvider(FakeSSH(), "")


class TestConcurrency:
    """
    UI гоняет дерево файлов, открытие файла и Sync в разных потоках через
    ОДИН SSH-провайдер. SFTP-канал paramiko не потокобезопасен — без
    сериализации сессия падает с 'Server connection dropped'.
    Нашёл это UI-тест, однопоточные тесты выше поймать не могли.
    """

    def test_parallel_ops_on_one_sftp_provider(self, ssh_client):
        import threading
        root = f"/tmp/bp-conc-{uuid.uuid4().hex[:8]}"
        os.makedirs(root)
        p = SSHProvider(ssh_client)
        for i in range(10):
            p.write_text(p.join(root, f"f{i}.txt"), f"data {i}\n" * 200)

        errors = []

        def worker(n):
            try:
                for _ in range(15):
                    p.list_dir(root)
                    assert p.read_text(p.join(root, f"f{n % 10}.txt")).startswith("data")
                    p.write_text(p.join(root, f"w{n}.txt"), "x" * 5000)
                    p.exists(p.join(root, "nope"))
            except Exception as e:
                errors.append(repr(e))

        threads = [threading.Thread(target=worker, args=(n,)) for n in range(8)]
        for t in threads:
            t.start()
        for t in threads:
            t.join(timeout=60)

        import shutil
        shutil.rmtree(root, ignore_errors=True)
        assert errors == []
