import ast
import os
from core.generators.launch_generator import (
    build_launch_files, group_files, write_launch_files)

CF = [
    {"filename": "a.cpp", "language": "cpp", "group": "robot"},
    {"filename": "b.cpp", "language": "cpp", "group": "robot"},
    {"filename": "c.py", "language": "python", "group": "pc"},
    {"filename": "d.cpp", "language": "cpp"},
]


def test_group_files_default_main_and_order():
    g = group_files(CF)
    assert list(g) == ["robot", "pc", "main"]
    assert [f["filename"] for f in g["robot"]] == ["a.cpp", "b.cpp"]


def test_build_names_and_validity():
    out = build_launch_files(CF, "pkg")
    assert set(out) == {"project_launch.py", "robot_launch.py", "pc_launch.py", "main_launch.py"}
    for src in out.values():
        ast.parse(src)


def test_project_launch_contains_all_group_launch_only_own():
    out = build_launch_files(CF, "pkg")
    for n in ("a", "b", "d"):
        assert f"'{n}'" in out["project_launch.py"]
    assert "'a'" in out["robot_launch.py"] and "'d'" not in out["robot_launch.py"]
    assert "c.py" in out["pc_launch.py"] and "'a'" not in out["pc_launch.py"]


def test_empty_gives_only_project_launch():
    out = build_launch_files([], "pkg")
    assert list(out) == ["project_launch.py"]
    ast.parse(out["project_launch.py"])


def test_write_creates_dir_cleans_pycache_returns_groups(tmp_path):
    ld = tmp_path / "src" / "launch"
    (ld / "__pycache__").mkdir(parents=True)
    (ld / "__pycache__" / "x.pyc").write_bytes(b"0")
    groups = write_launch_files(str(ld), CF, "pkg")
    assert not (ld / "__pycache__").exists()
    assert (ld / "project_launch.py").is_file() and (ld / "robot_launch.py").is_file()
    assert set(groups) == {"robot", "pc", "main"}


def test_write_keeps_foreign_launch_files(tmp_path):
    ld = tmp_path / "launch"
    ld.mkdir()
    (ld / "petri.launch.py").write_text("# user", encoding="utf-8")
    write_launch_files(str(ld), CF, "pkg")
    assert (ld / "petri.launch.py").read_text(encoding="utf-8") == "# user"


def test_cyrillic_package_and_group(tmp_path):
    out = build_launch_files([{"filename": "x.py", "language": "python", "group": "группа"}], "pkg")
    assert "группа_launch.py" in out
    ast.parse(out["группа_launch.py"])
