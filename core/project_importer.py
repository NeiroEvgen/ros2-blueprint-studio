# core/project_importer.py
# v0.6.0 — Минимальный импорт чужого ROS 2 пакета (Уровень 1 из roadmap).
# Что делает: находит package.xml -> имя и зависимости; собирает .py/.cpp
# файлы -> по ноде на файл (без реверса связей). Для Python через ast
# вытаскивает имена классов (надёжно, stdlib). Связи НЕ строим (Уровень 2).
# Импортированные ноды должны помечаться is_imported=True, чтобы студия
# не пыталась их регенерить своими маркерами.

import os
import ast
import xml.etree.ElementTree as ET

MAX_FILE_BYTES = 200 * 1024  # не тащим гигантские файлы в code_content
SKIP_PY = {"setup.py", "__init__.py", "conf.py"}


def import_ros2_package(root_path):
    """
    Возвращает:
    {
      'package_name': str,
      'deps': [str, ...],
      'files': [ {'filename','relpath','language','code','classes':[...] }, ... ],
      'warnings': [str, ...],
    }
    Бросает ValueError, если это не похоже на ROS 2 пакет.
    """
    result = {"package_name": "", "deps": [], "files": [], "warnings": []}

    pkg_xml = _find_package_xml(root_path)
    if pkg_xml:
        name, deps = _parse_package_xml(pkg_xml)
        result["package_name"] = name
        result["deps"] = deps
    else:
        result["warnings"].append("package.xml не найден — импортирую как набор файлов.")
        result["package_name"] = os.path.basename(os.path.normpath(root_path))

    for dirpath, dirnames, filenames in os.walk(root_path):
        # не лезем в мусорные/служебные папки
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
        raise ValueError("В выбранной папке не найдено ни одного .py/.cpp файла.")
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
            warnings.append(f"Пропущен (слишком большой): {rel}")
            return None
        with open(full, "r", encoding="utf-8", errors="replace") as f:
            code = f.read()
        return {"filename": os.path.basename(full), "relpath": rel,
                "language": language, "code": code}
    except Exception as e:
        warnings.append(f"Не прочитан {rel}: {e}")
        return None


def _py_classes(code):
    """Имена top-level классов через ast. Падать не должен ни на чём."""
    try:
        tree = ast.parse(code)
        return [n.name for n in tree.body if isinstance(n, ast.ClassDef)]
    except Exception:
        return []
