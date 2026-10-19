"""
Saving from the editor never overwrites someone else's edits.
Runs on a local folder and through a real sshd.
"""
import os
import shutil
import uuid
import pytest

from core.remote.providers import LocalProvider, SSHProvider
from core.remote.editing import EditBuffer, FileChangedExternally, NotEditable


@pytest.fixture(params=["local", "ssh"])
def prov(request, tmp_path):
    if request.param == "local":
        root = str(tmp_path / "r"); os.makedirs(root)
        yield LocalProvider(), root
        return
    import paramiko
    i = request.getfixturevalue("sshd")
    c = paramiko.SSHClient(); c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(i["host"], i["port"], i["user"], key_filename=i["key"],
              look_for_keys=False, allow_agent=False)
    root = f"/tmp/bp-edit-{uuid.uuid4().hex[:8]}"; os.makedirs(root)
    yield SSHProvider(c), root
    shutil.rmtree(root, ignore_errors=True); c.close()


def test_open_edit_save(prov):
    p, root = prov
    path = p.join(root, "Node.cpp"); p.write_text(path, "int a;\n")
    b = EditBuffer(p, path)
    assert b.open() == "int a;\n"
    b.save("int b;\n")
    assert p.read_text(path) == "int b;\n"


def test_dirty_tracking(prov):
    p, root = prov
    path = p.join(root, "f.txt"); p.write_text(path, "x")
    b = EditBuffer(p, path); b.open()
    assert not b.is_dirty("x")
    assert b.is_dirty("y")
    b.save("y")
    assert not b.is_dirty("y")


def test_external_change_blocks_save(prov):
    """Opened in the browser, and meanwhile someone edited it on the robot via ssh."""
    p, root = prov
    path = p.join(root, "mission.yaml"); p.write_text(path, "goal: 1\n")
    b = EditBuffer(p, path); b.open()
    p.write_text(path, "goal: 2  # fixed on robot\n")

    with pytest.raises(FileChangedExternally):
        b.save("goal: 3\n")
    assert p.read_text(path) == "goal: 2  # fixed on robot\n"


def test_force_overwrites(prov):
    p, root = prov
    path = p.join(root, "f.txt"); p.write_text(path, "a")
    b = EditBuffer(p, path); b.open()
    p.write_text(path, "b")
    b.save("c", force=True)
    assert p.read_text(path) == "c"


def test_consecutive_saves_ok(prov):
    """After our own save, the next one must not count as a conflict."""
    p, root = prov
    path = p.join(root, "f.txt"); p.write_text(path, "1")
    b = EditBuffer(p, path); b.open()
    b.save("2"); b.save("3"); b.save("4")
    assert p.read_text(path) == "4"


def test_binary_refused(prov):
    p, root = prov
    path = p.join(root, "blob"); p.write_bytes(path, b"ab\0cd")
    with pytest.raises(NotEditable, match="binary"):
        EditBuffer(p, path).open()


def test_non_utf8_refused(prov):
    p, root = prov
    path = p.join(root, "cp1251.txt"); p.write_bytes(path, "привет".encode("cp1251"))
    with pytest.raises(NotEditable, match="UTF-8"):
        EditBuffer(p, path).open()


def test_cyrillic_content_roundtrip(prov):
    p, root = prov
    path = p.join(root, "заметка.md"); p.write_text(path, "# Робот\n")
    b = EditBuffer(p, path); b.open()
    b.save("# Робот\n")
    assert p.read_text(path) == "# Робот\n"
