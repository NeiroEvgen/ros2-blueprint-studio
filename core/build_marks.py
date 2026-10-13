"""
Отметки файлов проекта для сборки (вкладка Files).

  yellow — файл (или папка) попадает в CMakeLists.txt
  green  — то же + файл виден на рабочем столе как нода (этап 2)

Хранятся в <проект>/.blueprint/build_marks.json, пути относительно src/
(прямые слэши). Отметка на папке действует на всё содержимое.
Без Qt и docker.
"""
import json
import os

YELLOW = "yellow"
GREEN = "green"
STATES = (YELLOW, GREEN)
SOURCE_EXT = (".cpp", ".cc", ".cxx")
HEADER_EXT = (".hpp", ".h", ".hh")


def _path(project_dir: str) -> str:
    return os.path.join(project_dir, ".blueprint", "build_marks.json")


def norm(rel: str) -> str:
    return os.path.normpath(rel.replace("\\", "/")).replace(os.sep, "/").strip("/")


def load(project_dir: str) -> dict:
    try:
        with open(_path(project_dir), "r", encoding="utf-8") as f:
            data = json.load(f)
    except (OSError, ValueError):
        return {}
    if not isinstance(data, dict):
        return {}
    return {norm(k): v for k, v in data.items() if v in STATES}


def save(project_dir: str, marks: dict) -> None:
    p = _path(project_dir)
    if not marks:
        try:
            os.remove(p)
        except OSError:
            pass
        return
    os.makedirs(os.path.dirname(p), exist_ok=True)
    with open(p, "w", encoding="utf-8") as f:
        json.dump(dict(sorted(marks.items())), f, indent=2, ensure_ascii=False)


def next_state(state):
    """Цикл по клику: нет → жёлтая → зелёная → нет."""
    return {None: YELLOW, YELLOW: GREEN, GREEN: None}[state]


def set_mark(project_dir: str, rel: str, state) -> dict:
    marks = load(project_dir)
    rel = norm(rel)
    if state in STATES:
        marks[rel] = state
    else:
        marks.pop(rel, None)
    save(project_dir, marks)
    return marks


def effective(marks: dict, rel: str):
    """Состояние файла с учётом отметки на родительской папке (ближайшая побеждает)."""
    rel = norm(rel)
    while True:
        if rel in marks:
            return marks[rel]
        if "/" not in rel:
            return None
        rel = rel.rsplit("/", 1)[0]


def marked_files(src_dir: str, marks: dict, states=STATES) -> list:
    """Развернуть отметки в список существующих файлов (пути от src/, отсортирован)."""
    out = set()
    for rel, st in marks.items():
        if st not in states:
            continue
        full = os.path.join(src_dir, rel)
        if os.path.isfile(full):
            out.add(rel)
        elif os.path.isdir(full):
            for root, dirs, files in os.walk(full):
                dirs[:] = [d for d in dirs if not d.startswith(".")]
                for fn in files:
                    r = norm(os.path.relpath(os.path.join(root, fn), src_dir))
                    # вложенная отметка «нет» перебивает папочную: effective решает
                    if effective(marks, r) in states:
                        out.add(r)
    return sorted(out)


def node_language(rel: str):
    ext = os.path.splitext(rel)[1].lower()
    if ext in SOURCE_EXT:
        return "cpp"
    if ext == ".py":
        return "python"
    return None


def green_files(project_dir: str) -> list:
    """Все файлы с зелёной отметкой (пути от src/)."""
    from core.layout import ProjectLayout
    layout = ProjectLayout(project_dir)
    return marked_files(layout.src, load(project_dir), states=(GREEN,))


def green_node_files(project_dir: str, languages):
    """
    Зелёные файлы → (будут_ноды, пропущены).
      будут_ноды — [(language, filename, rel)] для .cpp/.cc/.cxx и .py любой папки под src/
                   (кроме launch/ и __init__.py), язык которых есть в проекте;
      пропущены  — [(rel, причина)]: заголовки, чужие расширения, язык вне проекта.
    """
    nodes, skipped = [], []
    for rel in green_files(project_dir):
        lang = node_language(rel)
        base = os.path.basename(rel)
        if lang is None:
            skipped.append((rel, "not_source"))
        elif rel.startswith("launch/") or base.startswith("__"):
            skipped.append((rel, "not_source"))
        elif lang not in languages:
            skipped.append((rel, "language"))
        else:
            nodes.append((lang, base, rel))
    return nodes, skipped
