# core/project_importer.py
# v0.6.0: minimal import of someone else's ROS 2 package.
# What it does: finds package.xml -> name and dependencies; collects .py/.cpp
# files -> one node per file (connections are not reverse-engineered). For Python
# it extracts class names via ast (reliable, stdlib). No connections yet.
# Imported nodes must be marked is_imported=True so Studio does not
# try to regenerate them with its markers.

import os
import ast
import xml.etree.ElementTree as ET

from core.i18n import t

MAX_FILE_BYTES = 200 * 1024  # do not pull huge files into code_content
SKIP_PY = {"setup.py", "__init__.py", "conf.py"}


def import_ros2_package(root_path):
    """
    Returns:
    {
      'package_name': str,
      'deps': [str, ...],
      'files': [ {'filename','relpath','language','code','classes':[...] }, ... ],
      'warnings': [str, ...],
    }
    Raises ValueError if this does not look like a ROS 2 package.
    """
    result = {"package_name": "", "deps": [], "files": [], "warnings": []}

    pkg_xml = _find_package_xml(root_path)
    if pkg_xml:
        name, deps = _parse_package_xml(pkg_xml)
        result["package_name"] = name
        result["deps"] = deps
    else:
        result["warnings"].append(t("import.no_package_xml"))
        result["package_name"] = os.path.basename(os.path.normpath(root_path))

    for dirpath, dirnames, filenames in os.walk(root_path):
        # skip junk / service folders
        dirnames[:] = [d for d in dirnames if d not in
                       {".git", "build", "install", "log", "__pycache__", ".blueprint", "node_modules"}]
        for fn in filenames:
            full = os.path.join(dirpath, fn)
            rel = os.path.relpath(full, root_path)
            if fn.endswith(".py") and fn not in SKIP_PY:
                entry = _read_entry(full, rel, "python", result["warnings"])
                if entry:
                    entry["classes"] = _py_classes(entry["code"])
                    result["files"].append(entry)
            elif fn.endswith((".cpp", ".cc")):
                entry = _read_entry(full, rel, "cpp", result["warnings"])
                if entry:
                    entry["classes"] = []
                    result["files"].append(entry)

    if not result["files"]:
        raise ValueError(t("import.no_sources"))
    return result


def _find_package_xml(root_path):
    direct = os.path.join(root_path, "package.xml")
    if os.path.exists(direct):
        return direct
    for dirpath, dirnames, filenames in os.walk(root_path):
        dirnames[:] = [d for d in dirnames if d not in {".git", "build", "install", "log"}]
        if "package.xml" in filenames:
            return os.path.join(dirpath, "package.xml")
    return None


def _parse_package_xml(path):
    name, deps = "", []
    try:
        tree = ET.parse(path)
        root = tree.getroot()
        el = root.find("name")
        if el is not None and el.text:
            name = el.text.strip()
        for tag in ("depend", "build_depend", "exec_depend", "run_depend"):
            for d in root.findall(tag):
                if d.text and d.text.strip() not in deps:
                    deps.append(d.text.strip())
    except Exception:
        pass
    return name, deps


def _read_entry(full, rel, language, warnings):
    try:
        if os.path.getsize(full) > MAX_FILE_BYTES:
            warnings.append(t("import.too_big", file=rel))
            return None
        with open(full, "r", encoding="utf-8", errors="replace") as f:
            code = f.read()
        return {"filename": os.path.basename(full), "relpath": rel,
                "language": language, "code": code}
    except Exception as e:
        warnings.append(t("import.read_failed", file=rel, error=e))
        return None


def _py_classes(code):
    """Top-level class names via ast. Must never fail."""
    try:
        tree = ast.parse(code)
        return [n.name for n in tree.body if isinstance(n, ast.ClassDef)]
    except Exception:
        return []
