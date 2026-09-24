"""
Инкрементальная синхронизация исходников: локальный проект → удалённая цель.

На удалённой стороне лежит манифест <remote_root>/.blueprint_sync.json:
{относительный_путь: sha1} — что именно мы туда положили в прошлый раз.

Правила (все проверены тестами):
1. Неизменённые файлы не пересылаются.
2. Файлы, которых нет в манифесте, НИКОГДА не трогаются: build/, install/,
   log/ и всё, что создано на роботе руками, в безопасности.
3. Конфликт: файл изменён И локально, И на роботе (кто-то правил прямо там
   через ssh/sed, как было на G1) → НЕ перезаписываем, сообщаем.
4. Удаление: файл пропал локально → удаляем на роботе, только если там он
   не менялся с прошлой синхронизации. Иначе — конфликт, оставляем.
5. dry_run показывает план, ничего не меняя.
"""
import fnmatch
import hashlib
import json
import os
from dataclasses import dataclass, field

MANIFEST_NAME = ".blueprint_sync.json"

DEFAULT_EXCLUDES = (
    "build", "install", "log", ".git", "__pycache__", ".blueprint",
    "*.pyc", "*.pyo", "*.swp", "*~", ".DS_Store", "*.tmp-*", "CMakeLists.txt.bak*",
)


@dataclass
class SyncReport:
    uploaded: list = field(default_factory=list)
    deleted: list = field(default_factory=list)
    unchanged: int = 0
    conflicts: list = field(default_factory=list)   # (rel, причина)
    dry_run: bool = False

    @property
    def ok(self):
        return not self.conflicts

    def summary(self):
        verb = "будет" if self.dry_run else ""
        parts = [f"↑ {len(self.uploaded)}", f"✕ {len(self.deleted)}",
                 f"= {self.unchanged}"]
        if self.conflicts:
            parts.append(f"⚠ конфликтов: {len(self.conflicts)}")
        return (f"[dry-run] " if self.dry_run else "") + "  ".join(parts)


def _excluded(rel, patterns):
    for part in rel.split("/"):
        for pat in patterns:
            if fnmatch.fnmatch(part, pat):
                return True
    return False


def scan_local(root, excludes=DEFAULT_EXCLUDES):
    """{posix_rel: sha1} для всех файлов проекта, кроме исключённых."""
    result = {}
    for dirpath, dirnames, filenames in os.walk(root):
        rel_dir = os.path.relpath(dirpath, root).replace(os.sep, "/")
        rel_dir = "" if rel_dir == "." else rel_dir
        dirnames[:] = [d for d in dirnames
                       if not _excluded((rel_dir + "/" + d).lstrip("/"), excludes)]
        for fn in filenames:
            rel = (rel_dir + "/" + fn).lstrip("/")
            if _excluded(rel, excludes):
                continue
            with open(os.path.join(dirpath, fn), "rb") as f:
                result[rel] = hashlib.sha1(f.read()).hexdigest()
    return result


def _load_manifest(remote, remote_root):
    path = remote.join(remote_root, MANIFEST_NAME)
    if not remote.exists(path):
        return {}
    try:
        data = json.loads(remote.read_text(path))
        return data.get("files", {}) if isinstance(data, dict) else {}
    except Exception:
        return {}


def _remote_sha(remote, path):
    try:
        return remote.sha1(path) if remote.exists(path) else None
    except Exception:
        return None


def sync_tree(local_root, remote, remote_root, excludes=DEFAULT_EXCLUDES,
              delete=True, dry_run=False, on_progress=None):
    def say(msg):
        if on_progress:
            on_progress(msg)

    report = SyncReport(dry_run=dry_run)
    local = scan_local(local_root, excludes)
    manifest = _load_manifest(remote, remote_root)
    new_manifest = dict(manifest)

    if not dry_run:
        remote.makedirs(remote_root)

    # --- загрузка изменённого ---
    for rel in sorted(local):
        sha = local[rel]
        if manifest.get(rel) == sha:
            report.unchanged += 1
            continue
        rpath = remote.join(remote_root, *rel.split("/"))
        if rel in manifest:
            current = _remote_sha(remote, rpath)
            if current is not None and current != manifest[rel] and current != sha:
                report.conflicts.append((rel, "изменён и локально, и на удалённой стороне"))
                continue
        else:
            current = _remote_sha(remote, rpath)
            if current is not None and current != sha:
                report.conflicts.append(
                    (rel, "на удалённой стороне уже есть другой файл с этим именем"))
                continue
            if current == sha:
                new_manifest[rel] = sha
                report.unchanged += 1
                continue
        report.uploaded.append(rel)
        say(f"↑ {rel}")
        if not dry_run:
            with open(os.path.join(local_root, *rel.split("/")), "rb") as f:
                remote.write_bytes(rpath, f.read())
            new_manifest[rel] = sha

    # --- удаление пропавшего локально ---
    if delete:
        for rel in sorted(set(manifest) - set(local)):
            rpath = remote.join(remote_root, *rel.split("/"))
            current = _remote_sha(remote, rpath)
            if current is None:
                new_manifest.pop(rel, None)
                continue
            if current != manifest[rel]:
                report.conflicts.append((rel, "удалён локально, но изменён на удалённой стороне"))
                continue
            report.deleted.append(rel)
            say(f"✕ {rel}")
            if not dry_run:
                remote.remove(rpath)
                new_manifest.pop(rel, None)

    if not dry_run:
        remote.write_text(remote.join(remote_root, MANIFEST_NAME),
                          json.dumps({"version": 1, "files": new_manifest},
                                     ensure_ascii=False, indent=1, sort_keys=True))
    return report
