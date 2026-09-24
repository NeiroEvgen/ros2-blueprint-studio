"""
2.2 — профиль удалённой цели.

Критерии приёмки:
- профиль сохраняется в проекте, пароль — нигде
- ключ хоста запоминается при первом подключении (TOFU)
- при смене ключа хоста подключение ОТКАЗЫВАЕТ (защита от подмены машины)
"""
import os
import subprocess

import pytest
import yaml

from core.remote.profile import (
    RemoteProfile, ProfileStore, connect, HostKeyChanged, PROFILE_REL,
)


def _valid(**kw):
    base = dict(host="192.168.123.164", user="unitree",
                remote_workspace="/home/unitree/blueprint_ws/mpd")
    base.update(kw)
    return RemoteProfile(**base)


class TestProfileStore:

    def test_roundtrip(self, tmp_path):
        store = ProfileStore(str(tmp_path))
        p = _valid(container="g1pilot_run", container_workspace="/host_src/mpd", port=2222)
        store.save(p)
        assert store.load() == p

    def test_saved_inside_project(self, tmp_path):
        ProfileStore(str(tmp_path)).save(_valid())
        assert (tmp_path / PROFILE_REL).exists()

    def test_no_secrets_on_disk(self, tmp_path):
        ProfileStore(str(tmp_path)).save(_valid(key_path="~/.ssh/id_ed25519"))
        data = yaml.safe_load((tmp_path / PROFILE_REL).read_text(encoding="utf-8"))
        assert "password" not in data
        assert not any("pass" in k.lower() for k in data)

    def test_password_in_yaml_is_ignored(self, tmp_path):
        """Даже если кто-то вписал пароль руками — профиль его не подхватит."""
        path = tmp_path / PROFILE_REL
        path.parent.mkdir(parents=True)
        path.write_text(yaml.safe_dump({"host": "h", "user": "u",
                                        "remote_workspace": "/w", "password": "123"}))
        p = ProfileStore(str(tmp_path)).load()
        assert not hasattr(p, "password")

    def test_missing_file_gives_empty_profile(self, tmp_path):
        p = ProfileStore(str(tmp_path)).load()
        assert not p.enabled

    @pytest.mark.parametrize("kw,fragment", [
        ({"host": ""}, "хост"),
        ({"user": ""}, "пользователь"),
        ({"remote_workspace": ""}, "remote_workspace"),
        ({"remote_workspace": "blueprint_ws"}, "абсолютным"),
        ({"port": 0}, "порт"),
        ({"container": "c", "container_workspace": "rel/path"}, "абсолютным"),
    ])
    def test_validation(self, tmp_path, kw, fragment):
        with pytest.raises(ValueError, match=fragment):
            ProfileStore(str(tmp_path)).save(_valid(**kw))

    def test_build_dir(self):
        assert _valid().build_dir == "/home/unitree/blueprint_ws/mpd"
        assert _valid(container="c", container_workspace="/host_src/mpd").build_dir == "/host_src/mpd"
        # контейнер без отдельного пути: считаем, что смонтирован тем же путём
        assert _valid(container="c").build_dir == "/home/unitree/blueprint_ws/mpd"


class TestConnectTOFU:

    def _profile(self, sshd):
        return RemoteProfile(host=sshd["host"], port=sshd["port"], user=sshd["user"],
                             key_path=sshd["key"], remote_workspace="/tmp/ws")

    def test_first_connect_remembers_host(self, sshd, tmp_path):
        kh = str(tmp_path / "known_hosts")
        c = connect(self._profile(sshd), known_hosts=kh)
        c.close()
        text = open(kh).read()
        assert f"[{sshd['host']}]:{sshd['port']}" in text

    def test_second_connect_uses_remembered_key(self, sshd, tmp_path):
        kh = str(tmp_path / "known_hosts")
        connect(self._profile(sshd), known_hosts=kh).close()
        before = open(kh).read()
        connect(self._profile(sshd), known_hosts=kh).close()
        assert open(kh).read() == before

    def test_changed_host_key_is_rejected(self, sshd, tmp_path):
        """Критерий безопасности: подменённая машина не проходит."""
        other = tmp_path / "impostor"
        subprocess.run(["ssh-keygen", "-q", "-t", "ed25519", "-N", "", "-f", str(other)],
                       check=True)
        alg, key = (tmp_path / "impostor.pub").read_text().split()[:2]
        kh = tmp_path / "known_hosts"
        kh.write_text(f"[{sshd['host']}]:{sshd['port']} {alg} {key}\n")

        with pytest.raises(HostKeyChanged, match="изменился"):
            connect(self._profile(sshd), known_hosts=str(kh))

    def test_invalid_profile_does_not_connect(self, tmp_path):
        with pytest.raises(ValueError):
            connect(RemoteProfile(), known_hosts=str(tmp_path / "kh"))
