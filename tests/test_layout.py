import os
import pytest
from core.layout import ProjectLayout, migrate


def w(root, rel, text="x"):
    p = os.path.join(root, rel)
    os.makedirs(os.path.dirname(p), exist_ok=True)
    with open(p, "w", encoding="utf-8") as f:
        f.write(text)
    return p


def r(root, rel):
    with open(os.path.join(root, rel), encoding="utf-8") as f:
        return f.read()


def tree(root):
    out = set()
    for dp, dns, fns in os.walk(root):
        for fn in fns:
            out.add(os.path.relpath(os.path.join(dp, fn), root).replace("\\", "/"))
    return out


class TestPaths:
    def test_paths(self, tmp_path):
        L = ProjectLayout(str(tmp_path))
        assert L.cmake == os.path.join(L.src, "CMakeLists.txt")
        assert L.package_xml == os.path.join(L.src, "package.xml")
        assert L.launch == os.path.join(L.src, "launch")
        assert L.code_dir("cpp").endswith(os.path.join("src", "cpp"))
        assert L.code_dir("python").endswith(os.path.join("src", "python"))
        assert L.state_file.endswith(os.path.join(".blueprint", "state.yaml"))

    def test_ensure(self, tmp_path):
        L = ProjectLayout(str(tmp_path))
        L.ensure("cpp")
        for d in (L.src, L.launch, L.blueprint, L.code_dir("cpp")):
            assert os.path.isdir(d)

    def test_new_project_not_legacy(self, tmp_path):
        w(str(tmp_path), "src/CMakeLists.txt")
        w(str(tmp_path), "docker/docker-compose.yaml")
        w(str(tmp_path), "Dockerfile")
        w(str(tmp_path), "README_NODES.md")
        assert not ProjectLayout(str(tmp_path)).is_legacy()


class TestMigrate:
    def test_noop_creates_nothing(self, tmp_path):
        root = str(tmp_path)
        w(root, "src/CMakeLists.txt")
        before = tree(root)
        rep = migrate(root)
        assert not rep.changed and tree(root) == before
        assert not os.path.exists(os.path.join(root, ".blueprint"))

    def test_root_files_moved_into_src(self, tmp_path):
        root = str(tmp_path)
        w(root, "CMakeLists.txt", "cm")
        w(root, "package.xml", "px")
        w(root, "launch/a.launch.py", "L")
        rep = migrate(root)
        assert r(root, "src/CMakeLists.txt") == "cm"
        assert r(root, "src/package.xml") == "px"
        assert r(root, "src/launch/a.launch.py") == "L"
        assert not os.path.exists(os.path.join(root, "launch"))
        assert not os.path.exists(os.path.join(root, "CMakeLists.txt"))
        assert set(rep.moved) == {"CMakeLists.txt", "package.xml", "launch/a.launch.py"}

    def test_backup_contains_everything_touched(self, tmp_path):
        root = str(tmp_path)
        w(root, "cpp/n.cpp", "N")
        rep = migrate(root)
        assert r(rep.backup_dir, "cpp/n.cpp") == "N"
        assert rep.backup_dir.startswith(ProjectLayout(root).backup_root)

    def test_identical_duplicate_removed(self, tmp_path):
        root = str(tmp_path)
        w(root, "executor_core.cpp", "same")
        w(root, "src/executor_core.cpp", "same")
        rep = migrate(root)
        assert rep.identical == ["executor_core.cpp"]
        assert not os.path.exists(os.path.join(root, "executor_core.cpp"))
        assert r(root, "src/executor_core.cpp") == "same"

    def test_conflict_src_wins_root_backed_up(self, tmp_path):
        root = str(tmp_path)
        w(root, "task_core.cpp", "OLD")
        w(root, "src/task_core.cpp", "NEW")
        rep = migrate(root)
        assert rep.conflicts == ["task_core.cpp"]
        assert r(root, "src/task_core.cpp") == "NEW"
        assert r(rep.backup_dir, "task_core.cpp") == "OLD"
        assert not os.path.exists(os.path.join(root, "task_core.cpp"))

    def test_idempotent(self, tmp_path):
        root = str(tmp_path)
        w(root, "package.xml", "px")
        w(root, "include/robot/a.hpp", "h")
        migrate(root)
        snap = tree(root)
        rep2 = migrate(root)
        assert not rep2.changed and tree(root) == snap

    def test_dry_run_changes_nothing(self, tmp_path):
        root = str(tmp_path)
        w(root, "package.xml", "px")
        before = tree(root)
        rep = migrate(root, dry_run=True)
        assert rep.moved == ["package.xml"] and tree(root) == before

    def test_untouched_things_stay(self, tmp_path):
        root = str(tmp_path)
        for rel in ("src/x.cpp", "docker/docker-compose.yaml", ".blueprint/state.yaml",
                    "Dockerfile", "docker-compose.yml", "project_config.json",
                    "README_NODES.md", "venv/lib/a.py"):
            w(root, rel, rel)
        w(root, "package.xml", "px")
        migrate(root)
        for rel in ("src/x.cpp", "docker/docker-compose.yaml", ".blueprint/state.yaml",
                    "Dockerfile", "docker-compose.yml", "project_config.json",
                    "README_NODES.md", "venv/lib/a.py"):
            assert r(root, rel) == rel

    def test_cyrillic_names(self, tmp_path):
        root = str(tmp_path)
        w(root, "launch/запуск.launch.py", "ок")
        migrate(root)
        assert r(root, "src/launch/запуск.launch.py") == "ок"

    def test_real_world_project(self, tmp_path):
        """Раскладка из реального проекта demo_stage (корень + src/ с дублями)."""
        root = str(tmp_path)
        for rel in ("CMakeLists.txt", "package.xml", "executor_core.cpp", "task_core.cpp",
                    "cpp/task_executor_node.cpp", "include/demo_stage/types.hpp",
                    "launch/sim.launch.py"):
            w(root, rel, "A")
        for rel in ("src/CMakeLists.txt", "src/package.xml", "src/executor_core.cpp",
                    "src/task_core.cpp", "src/cpp/task_executor_node.cpp",
                    "src/include/demo_stage/types.hpp", "src/launch/sim.launch.py"):
            w(root, rel, "A")
        w(root, "cpp/arm_driver.cpp", "drv")      # есть только в корне
        w(root, ".blueprint/state.yaml", "s")
        rep = migrate(root)
        assert r(root, "src/cpp/arm_driver.cpp") == "drv"
        assert len(rep.identical) == 7 and rep.moved == ["cpp/arm_driver.cpp"]
        assert not ProjectLayout(root).is_legacy()
        assert r(root, ".blueprint/state.yaml") == "s"


def test_cli_dry_run_then_real(tmp_path):
    import subprocess, sys
    root = str(tmp_path)
    w(root, "package.xml", "px")
    repo = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    env = dict(os.environ, PYTHONPATH=repo, PYTHONIOENCODING="utf-8")
    out = subprocess.run([sys.executable, "-m", "core.layout", root, "--dry-run"],
                         capture_output=True, text=True, env=env, encoding="utf-8")
    assert out.returncode == 0 and "move" in out.stdout
    assert os.path.exists(os.path.join(root, "package.xml"))
    subprocess.run([sys.executable, "-m", "core.layout", root], check=True, env=env,
                   capture_output=True)
    assert r(root, "src/package.xml") == "px"
