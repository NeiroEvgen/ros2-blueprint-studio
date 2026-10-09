"""
Раскладка папок проекта — единственный источник путей + миграция старых проектов.

    <project>/
      src/              ← всё, с чем работает ROS 2 (код, launch, CMakeLists, package.xml)
        cpp|python/  launch/  include/ ...
      docker/           ← docker-compose.yaml шаблон
      .blueprint/       ← состояние редактора (state.yaml), legacy_backup/
      Dockerfile, docker-compose.yml   ← пока в корне (отдельное решение)

Чистая логика: без Qt и docker.
"""
import filecmp
import os
import shutil
import time
from dataclasses import dataclass, field
from typing import List, Optional

# Что считается «ROS-мусором в корне» у старых проектов.
LEGACY_ROOT_FILES = ("CMakeLists.txt", "package.xml")
LEGACY_ROOT_DIRS = ("launch", "cpp", "python", "include", "scripts", "test",
                    "config", "msg", "srv", "urdf", "lib", "kinematics_lib")
LEGACY_ROOT_EXTS = (".cpp", ".hpp", ".h", ".cc", ".py")


class ProjectLayout:
    def __init__(self, root: str):
        self.root = os.path.abspath(root)

    src = property(lambda s: os.path.join(s.root, "src"))
    launch = property(lambda s: os.path.join(s.src, "launch"))
    cmake = property(lambda s: os.path.join(s.src, "CMakeLists.txt"))
    package_xml = property(lambda s: os.path.join(s.src, "package.xml"))
    docker = property(lambda s: os.path.join(s.root, "docker"))
    blueprint = property(lambda s: os.path.join(s.root, ".blueprint"))
    state_file = property(lambda s: os.path.join(s.blueprint, "state.yaml"))
    config_file = property(lambda s: os.path.join(s.root, "project_config.json"))
    backup_root = property(lambda s: os.path.join(s.blueprint, "legacy_backup"))

    def code_dir(self, language: str) -> str:
        """Папка исходников языка: 'cpp' → src/cpp, остальное → src/python."""
        return os.path.join(self.src, "cpp" if language == "cpp" else "python")

    def code_dirs(self, project_type: str):
        """Папки кода проекта: 'mixed' → обе."""
        langs = ("cpp", "python") if project_type == "mixed" else (("cpp",) if project_type == "cpp" else ("python",))
        return [self.code_dir(l) for l in langs]

    def ensure(self, project_type: str) -> None:
        for d in (self.blueprint, self.src, self.launch, *self.code_dirs(project_type)):
            os.makedirs(d, exist_ok=True)

    def legacy_items(self) -> List[str]:
        """Относительные пути (от корня, всегда через «/») старых ROS-файлов вне src/."""
        found = []
        if not os.path.isdir(self.root):
            return found
        for name in sorted(os.listdir(self.root)):
            p = os.path.join(self.root, name)
            if os.path.isfile(p) and (name in LEGACY_ROOT_FILES
                                      or name.lower().endswith(LEGACY_ROOT_EXTS)):
                found.append(name)
            elif os.path.isdir(p) and name in LEGACY_ROOT_DIRS:
                for dp, dns, fns in os.walk(p):
                    dns[:] = [d for d in dns if d != "__pycache__"]
                    for fn in fns:
                        found.append(os.path.relpath(os.path.join(dp, fn), self.root).replace(os.sep, "/"))
        return found

    def is_legacy(self) -> bool:
        return bool(self.legacy_items())


@dataclass
class MigrationReport:
    moved: List[str] = field(default_factory=list)       # перенесено в src/
    identical: List[str] = field(default_factory=list)   # дубль src/, убрано из корня
    conflicts: List[str] = field(default_factory=list)   # отличается от src/: src оставлен, корневая копия в backup
    backup_dir: Optional[str] = None
    dry_run: bool = False

    @property
    def changed(self) -> bool:
        return bool(self.moved or self.identical or self.conflicts)

    def summary(self) -> str:
        if not self.changed:
            return "Layout: already up to date"
        s = (f"Layout migrated: moved {len(self.moved)}, duplicates removed "
             f"{len(self.identical)}, conflicts {len(self.conflicts)}")
        if self.backup_dir:
            s += f"; backup: {self.backup_dir}"
        return s


def _backup(layout: ProjectLayout, rel: str, stamp_dir: str) -> None:
    dst = os.path.join(stamp_dir, rel)
    os.makedirs(os.path.dirname(dst), exist_ok=True)
    shutil.copy2(os.path.join(layout.root, rel), dst)


def _prune_empty(path: str, stop: str) -> None:
    while path != stop and os.path.isdir(path):
        entries = [e for e in os.listdir(path) if e != "__pycache__"]
        if entries:
            return
        shutil.rmtree(path, ignore_errors=True)
        path = os.path.dirname(path)


def migrate(root: str, dry_run: bool = False) -> MigrationReport:
    """
    Приводит старый проект к новой раскладке. Безопасно и идемпотентно:
      * каждый затрагиваемый файл сначала копируется в .blueprint/legacy_backup/<stamp>/;
      * файл переносится в src/ только если там нет такого же пути;
      * идентичный дубль — удаляется из корня;
      * отличающийся — src/ побеждает, корневая версия остаётся только в backup.
    Если нечего мигрировать — ничего не создаётся.
    """
    lay = ProjectLayout(root)
    rep = MigrationReport(dry_run=dry_run)
    items = lay.legacy_items()
    if not items:
        return rep

    stamp = time.strftime("%Y%m%d-%H%M%S")
    stamp_dir = os.path.join(lay.backup_root, stamp)
    n = 1
    while os.path.exists(stamp_dir):
        n += 1
        stamp_dir = os.path.join(lay.backup_root, f"{stamp}-{n}")
    rep.backup_dir = stamp_dir

    for rel in items:
        old = os.path.join(lay.root, rel)
        new = os.path.join(lay.src, rel)
        if os.path.exists(new):
            same = filecmp.cmp(old, new, shallow=False)
            (rep.identical if same else rep.conflicts).append(rel)
        else:
            rep.moved.append(rel)
        if dry_run:
            continue
        _backup(lay, rel, stamp_dir)
        if rel in rep.moved:
            os.makedirs(os.path.dirname(new), exist_ok=True)
            shutil.move(old, new)
        else:
            os.remove(old)
        _prune_empty(os.path.dirname(old), lay.root)
    return rep


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(description="Миграция проекта Studio на раскладку src/")
    ap.add_argument("project")
    ap.add_argument("--dry-run", action="store_true", help="только показать, что будет сделано")
    a = ap.parse_args()
    rep = migrate(a.project, dry_run=a.dry_run)
    for tag, lst in (("move", rep.moved), ("dup-removed", rep.identical), ("conflict(src kept)", rep.conflicts)):
        for rel in lst:
            print(f"{tag:20s} {rel}")
    print(("[dry-run] " if a.dry_run else "") + rep.summary())
