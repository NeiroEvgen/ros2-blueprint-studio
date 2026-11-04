"""
What the node palette shows: entries, groups, search, language filter, favorites. No Qt.

    entries = builtin_entries(ALL_NODE_CLASSES, "mixed") + user_entries(PALETTES_ROOT, "mixed")
    groups = arrange(entries, query="serv", language="cpp", favorites={"ros.cpp.CppServiceServerNode"})
    -> [("favorites", [...]), ("services", [...]), ...]
"""
import os
from dataclasses import dataclass

# group keys in display order; user palettes come after them, one group per palette
FAVORITES = "favorites"
CATEGORY_ORDER = ("publishers", "subscribers", "services", "logic", "tools")
LANG_TAGS = {"python": "Py", "cpp": "C++"}


@dataclass(frozen=True)
class Entry:
    id: str             # what is dropped on the canvas: node type or USER_NODE:<palette>/<node>
    name: str
    category: str       # one of CATEGORY_ORDER, or "palette:<name>"
    language: str = ""  # "python" | "cpp" | "" (both / not a code node)
    doc: str = ""

    def label(self, show_language):
        tag = LANG_TAGS.get(self.language) if show_language else None
        return f"{self.name}  · {tag}" if tag else self.name


def _language(node_type):
    if node_type.startswith("ros.py."):
        return "python"
    if node_type.startswith("ros.cpp."):
        return "cpp"
    return ""


def _visible_in(project_type, language):
    """Single-language projects show only their language (and language-neutral nodes)."""
    if project_type == "python":
        return language != "cpp"
    if project_type == "cpp":
        return language != "python"
    return True


def _category(cls):
    name = cls.__name__
    if "PubNode" in name:
        return "publishers"
    if "SubNode" in name:
        return "subscribers"
    if "Service" in name:
        return "services"
    if any(k in name for k in ("Monitor", "Group", "Note")):
        return "tools"
    return "logic"


def builtin_entries(node_classes, project_type):
    out = []
    for cls in node_classes:
        node_type = cls.type_
        if "internal" in node_type or node_type.endswith(".FileNode"):
            continue                    # created by Studio itself, not from the palette
        lang = _language(node_type)
        if not _visible_in(project_type, lang):
            continue
        doc = (cls.__doc__ or "").strip().split("\n")[0]
        out.append(Entry(node_type, cls.NODE_NAME, _category(cls), lang, doc))
    return out


def user_entries(root, project_type):
    """Nodes saved into user palettes: <root>/<palette>/<node>/node.yaml."""
    import yaml
    out = []
    if not root or not os.path.isdir(root):
        return out
    for palette in sorted(os.listdir(root)):
        pdir = os.path.join(root, palette)
        if not os.path.isdir(pdir):
            continue
        for node in sorted(os.listdir(pdir)):
            meta = os.path.join(pdir, node, "node.yaml")
            if not os.path.isfile(meta):
                continue
            try:
                with open(meta, encoding="utf-8") as f:
                    node_type = str((yaml.safe_load(f) or {}).get("type") or "")
            except (OSError, yaml.YAMLError, AttributeError):
                node_type = ""
            lang = _language(node_type)
            if _visible_in(project_type, lang):
                out.append(Entry(f"USER_NODE:{palette}/{node}", node, f"palette:{palette}", lang,
                                 f"Palette: {palette}"))
    return out


def matches(entry, query, language="all"):
    """Every word of the query is found in the name, group, language or description."""
    if language in ("python", "cpp") and entry.language not in (language, ""):
        return False
    hay = " ".join((entry.name, entry.category, entry.doc, entry.language,
                    LANG_TAGS.get(entry.language, ""), entry.id)).lower()
    return all(word in hay for word in (query or "").lower().split())


def arrange(entries, query="", language="all", favorites=(), favorites_only=False):
    """[(group key, [entries sorted A-Z])]: favorites first, then the categories, then user palettes."""
    favorites = set(favorites)
    shown = [e for e in entries if matches(e, query, language)]
    if favorites_only:
        shown = [e for e in shown if e.id in favorites]
    key = lambda e: (e.name.lower(), e.language)
    groups = []
    fav = sorted((e for e in shown if e.id in favorites), key=key)
    if fav:
        groups.append((FAVORITES, fav))
    rest = [e for e in shown if e.id not in favorites]
    for cat in CATEGORY_ORDER:
        items = sorted((e for e in rest if e.category == cat), key=key)
        if items:
            groups.append((cat, items))
    palettes = sorted({e.category for e in rest if e.category.startswith("palette:")})
    for cat in palettes:
        groups.append((cat, sorted((e for e in rest if e.category == cat), key=key)))
    return groups
