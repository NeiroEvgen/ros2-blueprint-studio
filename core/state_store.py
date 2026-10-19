"""
Project state.yaml: reading any version, writing v4.

  v3.6: {"meta": {"version": "3.6"}, "graphs": {"python": <session>, "cpp": <session>}}
  v4.0: {"meta": {"version": "4.0", "merge": <meta for splitting back>},
         "graph": <one shared session>}

Reading v3.6 does not change the file. Writing v4 over v3.6 first keeps a copy
state.v3.6.yaml.bak (once), so going back to an older Studio is possible.
No Qt.
"""
import os
import shutil
from dataclasses import dataclass
from typing import Optional

import yaml

from core import graph_ir

V36, V4 = "3.6", "4.0"


# Tuples (colours, positions) are written as plain lists and read back as tuples:
# the same rules as in ProjectManager.
def _tuple_representer(dumper, data):
    return dumper.represent_sequence('tag:yaml.org,2002:seq', data)


def _tuple_constructor(loader, node):
    return tuple(loader.construct_sequence(node))


yaml.add_representer(tuple, _tuple_representer)
yaml.add_constructor('tag:yaml.org,2002:python/tuple', _tuple_constructor, Loader=yaml.SafeLoader)


@dataclass
class ProjectState:
    version: str
    py: dict            # session of the "python" part (for the old two-tab UI)
    cpp: dict           # session of the "cpp" part
    merged: dict        # shared graph (v4) for the single canvas
    merge_meta: dict


def _version(doc: dict) -> str:
    return str((doc.get("meta") or {}).get("version", V36))


def parse_state(doc: dict) -> ProjectState:
    """dict from state.yaml -> ProjectState (any supported version)."""
    if not isinstance(doc, dict):
        raise ValueError("state.yaml: a mapping was expected")
    ver = _version(doc)
    if "graph" in doc:                                   # v4+
        merged = doc["graph"] or {"nodes": {}, "connections": []}
        meta = (doc.get("meta") or {}).get("merge") or {"cpp_ids": [], "cpp_dy": 0.0, "remap": {}}
        py, cpp = graph_ir.split_session(merged, meta)
        return ProjectState(ver, py, cpp, merged, meta)
    graphs = doc.get("graphs") or {}
    py, cpp = graphs.get("python") or {}, graphs.get("cpp") or {}
    merged, meta = graph_ir.merge_sessions(py, cpp)
    return ProjectState(ver, py, cpp, merged, meta)


def read_state(project_dir: str) -> Optional[ProjectState]:
    path = os.path.join(project_dir, ".blueprint", "state.yaml")
    if not os.path.exists(path):
        return None
    with open(path, "r", encoding="utf-8") as f:
        return parse_state(yaml.safe_load(f))


def build_v4(merged: dict, merge_meta: dict) -> dict:
    return {"meta": {"version": V4, "merge": merge_meta}, "graph": merged}


def write_state_v4(project_dir: str, merged: dict, merge_meta: dict) -> str:
    bp = os.path.join(project_dir, ".blueprint")
    os.makedirs(bp, exist_ok=True)
    path = os.path.join(bp, "state.yaml")
    if os.path.exists(path):
        try:
            with open(path, "r", encoding="utf-8") as f:
                old_ver = _version(yaml.safe_load(f) or {})
        except Exception:
            old_ver = None
        bak = os.path.join(bp, "state.v3.6.yaml.bak")
        if old_ver == V36 and not os.path.exists(bak):
            shutil.copy2(path, bak)
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        yaml.dump(build_v4(merged, merge_meta), f, sort_keys=False, allow_unicode=True)
    os.replace(tmp, path)
    return path
