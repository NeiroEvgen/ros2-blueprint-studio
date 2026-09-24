"""
2.2 — синхронизация исходников.

Критерии приёмки из плана:
- изменённый файл доезжает одной командой
- неизменённые файлы не пересылаются повторно
Плюс защита, которой не было в плане, но которая следует из работы на G1
(файлы правились прямо на роботе через ssh/sed):
- правки на удалённой стороне не затираются молча (конфликт)
- чужие файлы (build/, install/, созданное руками) никогда не трогаются

Удалённая сторона — локальная папка и НАСТОЯЩИЙ sshd.
"""
import json
import os
import shutil
import uuid

import pytest

from core.remote.providers import LocalProvider, SSHProvider
from core.remote.sync import sync_tree, scan_local, MANIFEST_NAME


@pytest.fixture(params=["local", "ssh"])
def remote(request, tmp_path):
    if request.param == "local":
        root = str(tmp_path / "remote_ws")
        yield LocalProvider(), root
        return
    import paramiko
    info = request.getfixturevalue("sshd")
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(info["host"], info["port"], info["user"], key_filename=info["key"],
              look_for_keys=False, allow_agent=False)
    root = f"/tmp/bp-sync-{uuid.uuid4().hex[:10]}"
    yield SSHProvider(c), root
    shutil.rmtree(root, ignore_errors=True)
    c.close()


@pytest.fixture
def project(tmp_path):
    root = tmp_path / "proj" / "src"
    (root / "cpp").mkdir(parents=True)
    _w(str(root / "cpp" / "Node.cpp"), "int main(){}\n")
    _w(str(root / "cpp" / "Helper.cpp"), "double h(){return 1;}\n")
    (root / "data").mkdir()
    _w(str(root / "data" / "mission.yaml"), "goal: 1\n")
    return root


def _w(path, text):
    """Пишем БАЙТ В БАЙТ: в текстовом режиме Windows превратил бы \n в \r\n,
    а провайдер работает с байтами — тесты сравнивали бы разное."""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "wb") as f:
        f.write(text.encode("utf-8"))


class TestBasics:

    def test_first_sync_uploads_everything(self, project, remote):
        prov, root = remote
        r = sync_tree(str(project), prov, root)
        assert sorted(r.uploaded) == ["cpp/Helper.cpp", "cpp/Node.cpp", "data/mission.yaml"]
        assert prov.read_text(prov.join(root, "cpp", "Node.cpp")) == "int main(){}\n"
        assert r.ok

    def test_second_sync_sends_nothing(self, project, remote):
        """Критерий: неизменённое не пересылается."""
        prov, root = remote
        sync_tree(str(project), prov, root)
        r = sync_tree(str(project), prov, root)
        assert r.uploaded == [] and r.deleted == []
        assert r.unchanged == 3

    def test_only_changed_file_is_sent(self, project, remote):
        """Критерий: изменённый файл доезжает одной командой."""
        prov, root = remote
        sync_tree(str(project), prov, root)
        _w(str(project / "cpp" / "Node.cpp"), "int main(){return 1;}\n")
        r = sync_tree(str(project), prov, root)
        assert r.uploaded == ["cpp/Node.cpp"]
        assert prov.read_text(prov.join(root, "cpp", "Node.cpp")) == "int main(){return 1;}\n"

    def test_new_local_file_uploaded(self, project, remote):
        prov, root = remote
        sync_tree(str(project), prov, root)
        _w(str(project / "cpp" / "New.cpp"), "// new\n")
        assert sync_tree(str(project), prov, root).uploaded == ["cpp/New.cpp"]

    def test_deleted_locally_deleted_remotely(self, project, remote):
        prov, root = remote
        sync_tree(str(project), prov, root)
        (project / "cpp" / "Helper.cpp").unlink()
        r = sync_tree(str(project), prov, root)
        assert r.deleted == ["cpp/Helper.cpp"]
        assert not prov.exists(prov.join(root, "cpp", "Helper.cpp"))

    def test_manifest_written(self, project, remote):
        prov, root = remote
        sync_tree(str(project), prov, root)
        data = json.loads(prov.read_text(prov.join(root, MANIFEST_NAME)))
        assert set(data["files"]) == {"cpp/Helper.cpp", "cpp/Node.cpp", "data/mission.yaml"}

    def test_cyrillic_file_name(self, project, remote):
        prov, root = remote
        _w(str(project / "data" / "заметка.txt"), "привет\n")
        r = sync_tree(str(project), prov, root)
        assert "data/заметка.txt" in r.uploaded
        assert prov.read_text(prov.join(root, "data", "заметка.txt")) == "привет\n"


class TestExcludes:

    def test_build_artifacts_not_uploaded(self, project, remote):
        prov, root = remote
        for d in ("build", "install", "log", ".git", "cpp/__pycache__", ".blueprint"):
            _w(str(project / d / "junk.txt"), "x")
        _w(str(project / "cpp" / "x.pyc"), "x")
        _w(str(project / "CMakeLists.txt.bak"), "x")
        r = sync_tree(str(project), prov, root)
        assert sorted(r.uploaded) == ["cpp/Helper.cpp", "cpp/Node.cpp", "data/mission.yaml"]

    def test_scan_local_uses_posix_paths(self, project):
        assert all("\\" not in rel for rel in scan_local(str(project)))


class TestSafety:

    def test_remote_only_files_never_touched(self, project, remote):
        """build/, install/ и созданное на роботе руками — не наше, не трогаем."""
        prov, root = remote
        sync_tree(str(project), prov, root)
        colcon_artifact = prov.join(root, "build", "Node", "CMakeCache.txt")
        hand_made = prov.join(root, "data", "calibration.yaml")
        prov.write_text(colcon_artifact, "cache")
        prov.write_text(hand_made, "offsets: [0.1]")

        for _ in range(2):
            sync_tree(str(project), prov, root)

        assert prov.read_text(colcon_artifact) == "cache"
        assert prov.read_text(hand_made) == "offsets: [0.1]"

    def test_remote_edit_kept_when_local_unchanged(self, project, remote):
        """Правил на роботе, локально не трогал → правка остаётся."""
        prov, root = remote
        sync_tree(str(project), prov, root)
        target = prov.join(root, "data", "mission.yaml")
        prov.write_text(target, "goal: 2  # fixed on robot\n")

        r = sync_tree(str(project), prov, root)

        assert prov.read_text(target) == "goal: 2  # fixed on robot\n"
        assert r.uploaded == []

    def test_conflict_both_sides_changed_not_overwritten(self, project, remote):
        """Главная защита: ровно сценарий «правил через sed на G1»."""
        prov, root = remote
        sync_tree(str(project), prov, root)
        target = prov.join(root, "cpp", "Node.cpp")
        prov.write_text(target, "// hotfix on robot\n")
        _w(str(project / "cpp" / "Node.cpp"), "// local change\n")

        r = sync_tree(str(project), prov, root)

        assert prov.read_text(target) == "// hotfix on robot\n"
        assert [c[0] for c in r.conflicts] == ["cpp/Node.cpp"]
        assert not r.ok

    def test_conflict_does_not_block_other_files(self, project, remote):
        prov, root = remote
        sync_tree(str(project), prov, root)
        prov.write_text(prov.join(root, "cpp", "Node.cpp"), "// robot\n")
        _w(str(project / "cpp" / "Node.cpp"), "// local\n")
        _w(str(project / "cpp" / "Helper.cpp"), "// helper v2\n")

        r = sync_tree(str(project), prov, root)

        assert r.uploaded == ["cpp/Helper.cpp"]
        assert [c[0] for c in r.conflicts] == ["cpp/Node.cpp"]

    def test_conflict_persists_until_resolved(self, project, remote):
        prov, root = remote
        sync_tree(str(project), prov, root)
        prov.write_text(prov.join(root, "cpp", "Node.cpp"), "// robot\n")
        _w(str(project / "cpp" / "Node.cpp"), "// local\n")
        sync_tree(str(project), prov, root)
        r = sync_tree(str(project), prov, root)
        assert [c[0] for c in r.conflicts] == ["cpp/Node.cpp"]

    def test_delete_conflict_keeps_remote_edit(self, project, remote):
        prov, root = remote
        sync_tree(str(project), prov, root)
        target = prov.join(root, "cpp", "Helper.cpp")
        prov.write_text(target, "// edited on robot\n")
        (project / "cpp" / "Helper.cpp").unlink()

        r = sync_tree(str(project), prov, root)

        assert prov.read_text(target) == "// edited on robot\n"
        assert [c[0] for c in r.conflicts] == ["cpp/Helper.cpp"]

    def test_first_sync_into_dir_with_foreign_file(self, project, remote):
        """Папка на роботе уже есть (как deploy_Iurlasov): чужое не затираем."""
        prov, root = remote
        prov.write_text(prov.join(root, "cpp", "Node.cpp"), "// copied by hand earlier\n")

        r = sync_tree(str(project), prov, root)

        assert prov.read_text(prov.join(root, "cpp", "Node.cpp")) == "// copied by hand earlier\n"
        assert [c[0] for c in r.conflicts] == ["cpp/Node.cpp"]

    def test_first_sync_adopts_identical_file(self, project, remote):
        prov, root = remote
        prov.write_text(prov.join(root, "cpp", "Node.cpp"), "int main(){}\n")
        r = sync_tree(str(project), prov, root)
        assert "cpp/Node.cpp" not in r.uploaded
        assert r.ok
        # теперь файл наш: локальная правка поедет штатно
        _w(str(project / "cpp" / "Node.cpp"), "int main(){return 2;}\n")
        assert sync_tree(str(project), prov, root).uploaded == ["cpp/Node.cpp"]

    def test_dry_run_changes_nothing(self, project, remote):
        prov, root = remote
        r = sync_tree(str(project), prov, root, dry_run=True)
        assert len(r.uploaded) == 3
        assert not prov.exists(prov.join(root, MANIFEST_NAME))
        assert not prov.exists(prov.join(root, "cpp", "Node.cpp"))
