"""
Incremental source sync: local project -> remote target.

The remote side keeps a manifest <remote_root>/.blueprint_sync.json:
{relative_path: sha1}: exactly what we put there last time.

Rules (all covered by tests):
1. Unchanged files are not sent.
2. Files that are not in the manifest are NEVER touched: build/, install/,
   log/ and anything created on the robot by hand are safe.
3. Conflict: a file changed BOTH locally AND on the robot (someone edited it
   right there via ssh/sed) -> NOT overwritten, reported.
4. Deletion: a file gone locally is deleted on the robot only if it has not
   changed there since the last sync. Otherwise it is a conflict and stays.
5. dry_run shows the plan without changing anything.
"""
import fnmatch
import hashlib
import json
import os
from dataclasses import dataclass, field

from core.i18n import t

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
    conflicts: list = field(default_factory=list)   # (rel, reason)
    dry_run: bool = False

    @property
    def ok(self):
        return not self.conflicts

    def summary(self):
        parts = [f"↑ {len(self.uploaded)}", f"✕ {len(self.deleted)}",
                 f"= {self.unchanged}"]
        if self.conflicts:
            parts.append(t("sync.conflicts", count=len(self.conflicts)))
        return ("[dry-run] " if self.dry_run else "") + "  ".join(parts)


def _excluded(rel, patterns):
    for part in rel.split("/"):
        for pat in patterns:
            if fnmatch.fnmatch(part, pat):
                return True
    return False


def scan_local(root, excludes=DEFAULT_EXCLUDES):
    """{posix_rel: sha1} for all project files except the excluded ones."""
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

    # --- upload what changed ---
    for rel in sorted(local):
        sha = local[rel]
        if manifest.get(rel) == sha:
            report.unchanged += 1
            continue
        rpath = remote.join(remote_root, *rel.split("/"))
        if rel in manifest:
            current = _remote_sha(remote, rpath)
            if current is not None and current != manifest[rel] and current != sha:
                report.conflicts.append((rel, t("sync.both_changed")))
                continue
        else:
            current = _remote_sha(remote, rpath)
            if current is not None and current != sha:
                report.conflicts.append(
                    (rel, t("sync.remote_exists")))
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

    # --- delete what disappeared locally ---
    if delete:
        for rel in sorted(set(manifest) - set(local)):
            rpath = remote.join(remote_root, *rel.split("/"))
            current = _remote_sha(remote, rpath)
            if current is None:
                new_manifest.pop(rel, None)
                continue
            if current != manifest[rel]:
                report.conflicts.append((rel, t("sync.deleted_but_changed")))
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
