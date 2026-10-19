"""
Which code files Studio generated itself, so that ghost cleanup on Save
touches only them and never code a person put into src/cpp|python.

    <project>/.blueprint/generated.json   {"version": 1, "files": ["cpp/talker.cpp", ...]}
    <project>/.blueprint/trash/<time>/   stale files go here instead of being deleted

Paths are relative to src/, always with "/". No Qt.
"""
import json
import os
import shutil
import time

MANIFEST_REL = os.path.join(".blueprint", "generated.json")
TRASH_REL = os.path.join(".blueprint", "trash")

_CODE_DIR_BY_EXT = {".cpp": "cpp", ".py": "python"}


def manifest_path(project_dir):
    return os.path.join(project_dir, MANIFEST_REL)


def load(project_dir):
    """Set of paths from the manifest, or None if there is no manifest yet (project before 0.6.0)."""
    path = manifest_path(project_dir)
    if not os.path.exists(path):
        return None
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        return {str(p) for p in data.get("files", [])}
    except (OSError, ValueError, AttributeError):
        return set()          # broken manifest: delete nothing, Save rewrites it


def seed_from_state(project_dir):
    """
    No manifest -> take the files recorded in the previous state.yaml (source_file of nodes):
    Studio definitely created those. No state.yaml -> empty, i.e. nothing to delete.
    """
    from core import state_store
    try:
        st = state_store.read_state(project_dir)
    except Exception:
        return set()
    if st is None:
        return set()
    nodes = (st.merged or {}).get("nodes") or {}
    items = nodes.values() if isinstance(nodes, dict) else nodes
    out = set()
    for n in items:
        fname = ((n or {}).get("custom") or {}).get("source_file")
        if not fname or "/" in fname.replace("\\", "/"):
            continue          # files in subfolders (green marks) are not ours
        sub = _CODE_DIR_BY_EXT.get(os.path.splitext(fname)[1])
        if sub:
            out.add(f"{sub}/{fname}")
    return out


def previous(project_dir):
    """What Studio generated last time (the manifest or, for old projects, state.yaml)."""
    known = load(project_dir)
    return seed_from_state(project_dir) if known is None else known


def save(project_dir, files):
    path = manifest_path(project_dir)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump({"version": 1, "files": sorted(files)}, f, ensure_ascii=False, indent=1)
    os.replace(tmp, path)


def new_trash_dir(project_dir):
    return os.path.join(project_dir, TRASH_REL, time.strftime("%Y%m%d-%H%M%S"))


def move_to_trash(src_dir, rel, trash_dir):
    """src/<rel> -> <trash_dir>/<rel>. Returns the new path."""
    dst = os.path.join(trash_dir, *rel.split("/"))
    os.makedirs(os.path.dirname(dst), exist_ok=True)
    shutil.move(os.path.join(src_dir, *rel.split("/")), dst)
    return dst
