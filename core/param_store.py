"""
Parameter values saved into the project: <project>/src/config/params.yaml.

Standard ROS 2 params file, so the generated launch passes it to every node
(C++: Node(parameters=...), Python: --ros-args --params-file) and it is synced to the robot
together with src/:

    /pid_demo:
      ros__parameters:
        kp: 1.5

The Parameters window adds or replaces values; everything else in the file is kept.
A file that does not parse is never overwritten. No Qt.
"""
import os

import yaml

from core.i18n import t
from core.layout import ProjectLayout

HEADER = ("# ROS 2 parameters saved by ROS 2 Blueprint Studio (Parameters window).\n"
          "# Passed to the nodes by the generated launch files. Can be edited by hand.\n")


class ParamFileError(ValueError):
    pass


def path_for(project_path):
    return ProjectLayout(project_path).params_file


def _fqn(node):
    return node if node.startswith("/") else "/" + node


def load(project_path):
    """{"/node": {name: value}}; empty if there is no file. ParamFileError if it does not parse."""
    out = {}
    for node, body in _read_raw(path_for(project_path)).items():
        values = (body or {}).get("ros__parameters") if isinstance(body, dict) else None
        if isinstance(values, dict):
            out[_fqn(str(node))] = dict(values)
    return out


def _read_raw(p):
    if not os.path.exists(p):
        return {}
    try:
        with open(p, encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
    except (OSError, yaml.YAMLError) as e:
        raise ParamFileError(t("params.file_broken", path=p, error=e)) from e
    if not isinstance(data, dict):
        raise ParamFileError(t("params.file_broken", path=p, error="not a mapping"))
    return data


def save_values(project_path, node, values):
    """
    Merges {name: value} into the node's section and writes the file atomically.
    Other nodes, other parameters of this node and unknown keys stay as they were.
    """
    p = path_for(project_path)
    data = _read_raw(p)
    node = _fqn(node)
    key = next((k for k in data if _fqn(str(k)) == node), node)    # "pid" and "/pid" are the same node
    section = data.get(key)
    if not isinstance(section, dict):
        section = {}
    params = section.get("ros__parameters")
    if not isinstance(params, dict):
        params = {}
    params.update(values)
    section["ros__parameters"] = params
    data[key] = section
    os.makedirs(os.path.dirname(p), exist_ok=True)
    tmp = p + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        f.write(HEADER)
        yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False, default_flow_style=False)
    os.replace(tmp, p)
    return p
