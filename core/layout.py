"""
Project folder layout: the single source of paths + migration of old projects.

    <project>/
      src/              <- everything ROS 2 works with (code, launch, CMakeLists, package.xml)
        cpp|python/  launch/  include/ ...
      docker/           <- docker-compose.yaml template
      .blueprint/       <- editor state (state.yaml), legacy_backup/
      Dockerfile, docker-compose.yml   <- in the root for now (separate decision)

Pure logic: no Qt, no docker.
"""
import filecmp
import os
import shutil
import time
from dataclasses import dataclass, field
from typing import List, Optional

from core.i18n import t

# What counts as "ROS clutter in the root" in old projects.
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
        """Source folder of a language: 'cpp' -> src/cpp, anything else -> src/python."""
        return os.path.join(self.src, "cpp" if language == "cpp" else "python")

    def code_dirs(self, project_type: str):
        """Code folders of the project: 'mixed' -> both."""
        langs = ("cpp", "python") if project_type == "mixed" else (("cpp",) if project_type == "cpp" else ("python",))
        return [self.code_dir(l) for l in langs]

    def ensure(self, project_type: str) -> None:
        for d in (self.blueprint, self.src, self.launch, *self.code_dirs(project_type)):
            os.makedirs(d, exist_ok=True)

    def legacy_items(self) -> List[str]:
        """Relative paths (from the root, always with "/") of old ROS files outside src/."""
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
    moved: List[str] = field(default_factory=list)       # moved into src/
    identical: List[str] = field(default_factory=list)   # duplicate of src/, removed from the root
    conflicts: List[str] = field(default_factory=list)   # differs from src/: src kept, root copy in backup
    backup_dir: Optional[str] = None
    dry_run: bool = False

    @property
    def changed(self) -> bool:
        return bool(self.moved or self.identical or self.conflicts)

    def summary(self) -> str:
        if not self.changed:
            return t("layout.up_to_date")
        s = t("layout.migrated", moved=len(self.moved), dups=len(self.identical),
              conflicts=len(self.conflicts))
        if self.backup_dir:
            s += "; " + t("layout.backup", path=self.backup_dir)
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
    Brings an old project to the new layout. Safe and idempotent:
      * every affected file is first copied to .blueprint/legacy_backup/<stamp>/;
      * a file moves into src/ only if that path does not exist there;
      * an identical duplicate is removed from the root;
      * a differing one: src/ wins, the root version stays only in the backup.
    If there is nothing to migrate, nothing is created.
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
    ap = argparse.ArgumentParser(description="Migrate a Studio project to the src/ layout")
    ap.add_argument("project")
    ap.add_argument("--dry-run", action="store_true", help="only show what would be done")
    a = ap.parse_args()
    rep = migrate(a.project, dry_run=a.dry_run)
    for tag, lst in (("move", rep.moved), ("dup-removed", rep.identical), ("conflict(src kept)", rep.conflicts)):
        for rel in lst:
            print(f"{tag:20s} {rel}")
    print(("[dry-run] " if a.dry_run else "") + rep.summary())
