"""
Build marks of project files (Files tab).

  yellow: the file (or folder) goes into CMakeLists.txt
  green:  the same + the file is shown on the canvas as a node

Stored in <project>/.blueprint/build_marks.json, paths relative to src/
(forward slashes). A mark on a folder applies to everything inside.
No Qt, no docker.
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
    """Click cycle: none -> yellow -> green -> none."""
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
    """State of a file, taking marks on parent folders into account (the nearest wins)."""
    rel = norm(rel)
    while True:
        if rel in marks:
            return marks[rel]
        if "/" not in rel:
            return None
        rel = rel.rsplit("/", 1)[0]


def marked_files(src_dir: str, marks: dict, states=STATES) -> list:
    """Expand marks into a list of existing files (paths relative to src/, sorted)."""
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
                    # a nested "none" mark overrides the folder mark: effective() decides
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
    """All green-marked files (paths relative to src/)."""
    from core.layout import ProjectLayout
    layout = ProjectLayout(project_dir)
    return marked_files(layout.src, load(project_dir), states=(GREEN,))


def green_node_files(project_dir: str, languages):
    """
    Green files -> (code_nodes, file_nodes).
      code_nodes: [(language, filename, rel)]: .cpp/.cc/.cxx and .py in any folder under src/
                     (except launch/ and __init__.py) whose language the project has;
      file_nodes: [rel]: everything else (CMakeLists.txt, package.xml, headers, yaml...);
                     such a file is shown on the canvas as a plain file node.
    """
    code, plain = [], []
    for rel in green_files(project_dir):
        lang = node_language(rel)
        base = os.path.basename(rel)
        if lang in languages and not rel.startswith("launch/") and not base.startswith("__"):
            code.append((lang, base, rel))
        else:
            plain.append(rel)
    return code, plain
