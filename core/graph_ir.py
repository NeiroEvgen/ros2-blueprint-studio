"""
Graph IR: язык-нейтральный слой над сериализованным графом NodeGraphQt.

Идея: узел НЕ привязан к «своему» холсту. Язык (cpp / python) определяется по типу узла
(ros.cpp.* / ros.py.*) или явному custom.language; нейтральные узлы (группы, заметки,
монитор) языка не имеют и идут туда, куда их позовёт контекст. Это позволяет:
  * генерировать один проект сразу на C++ и Python (общий граф);
  * хранить один граф вместо двух (state v4) и мигрировать старые проекты (v3.6)
    без потерь — вперёд (merge) и назад (split).

Всё — чистые функции над dict-ами; сырые данные узлов не теряются (round-trip).
Без Qt и NodeGraphQt.
"""
import copy
from typing import Dict, List, Optional, Tuple

CPP, PYTHON = "cpp", "python"
LANGUAGES = (CPP, PYTHON)

_GROUP_TYPES = ("ros.nodes.RosGroup", "ros.nodes.RosGroupNode")
_INTERNAL_PREFIX = "ros.nodes.internal."


# ---------------------------------------------------------------- классификация узлов
def node_type(node: dict) -> str:
    return node.get("type_") or node.get("type") or ""


def language_of(node: dict) -> Optional[str]:
    """'cpp' | 'python' | None (нейтральный узел)."""
    explicit = (node.get("custom") or {}).get("language")
    if explicit in LANGUAGES:
        return explicit
    t = node_type(node)
    if t.startswith("ros.cpp.") or ".cpp." in t:
        return CPP
    if t.startswith("ros.py.") or ".py." in t:
        return PYTHON
    return None


def is_group(node: dict) -> bool:
    return node_type(node) in _GROUP_TYPES


def is_internal(node: dict) -> bool:
    return node_type(node).startswith(_INTERNAL_PREFIX)


def partition_by_language(flat_nodes: List[dict], default: str) -> Dict[str, List[dict]]:
    """
    flat_nodes — результат node_sources.flatten_nodes ([{'node':..., 'namespace':..., ...}]).
    Нейтральные/неизвестные узлы получают язык `default` (язык проекта) — как и раньше
    в одноязычных проектах.
    """
    out = {CPP: [], PYTHON: []}
    for entry in flat_nodes:
        lang = language_of(entry["node"]) or default
        out[lang if lang in out else default].append(entry)
    return out


def project_languages(project_type: str) -> Tuple[str, ...]:
    """'cpp' → ('cpp',), 'python' → ('python',), 'mixed' → ('cpp','python')."""
    if project_type == "mixed":
        return (CPP, PYTHON)
    return (CPP,) if project_type == CPP else (PYTHON,)


def default_language(project_type: str) -> str:
    return CPP if project_type in (CPP, "mixed") else PYTHON


# ---------------------------------------------------------------- доступ к сессии
def nodes_as_dict(session: dict) -> Dict[str, dict]:
    """{id: node} независимо от того, список в сессии или словарь."""
    nodes = session.get("nodes", {})
    if isinstance(nodes, dict):
        return nodes
    return {n["id"]: n for n in nodes if isinstance(n, dict) and "id" in n}


def _node_items(session: dict):
    nodes = session.get("nodes", {})
    if isinstance(nodes, dict):
        return list(nodes.items())
    return [(n.get("id"), n) for n in nodes if isinstance(n, dict)]


# ---------------------------------------------------------------- merge / split (v3.6 ⇄ v4)
def _pos(node) -> Optional[List[float]]:
    p = node.get("pos")
    if isinstance(p, (list, tuple)) and len(p) == 2:
        return [float(p[0]), float(p[1])]
    return None


def _bbox_bottom(session: dict) -> Optional[float]:
    ys = [p[1] for _, n in _node_items(session) if (p := _pos(n))]
    return max(ys) if ys else None


def _bbox_top(session: dict) -> Optional[float]:
    ys = [p[1] for _, n in _node_items(session) if (p := _pos(n))]
    return min(ys) if ys else None


GAP = 500.0   # вертикальный зазор между раскладками python и cpp при слиянии


def merge_sessions(py: dict, cpp: dict) -> Tuple[dict, dict]:
    """
    Склеивает две сессии (две вкладки v3.6) в одну.
    Возвращает (merged_session, merge_meta). merge_meta нужен split_session для точного
    обратного разделения: id узлов cpp, сдвиг раскладки, переименования id.
    Узлы cpp сдвигаются вниз, чтобы не налезать на python.
    """
    py = copy.deepcopy(py or {})
    cpp = copy.deepcopy(cpp or {})
    py_items, cpp_items = _node_items(py), _node_items(cpp)
    py_ids = {i for i, _ in py_items}

    # конфликт id → переименовать узел cpp (и все ссылки на него)
    remap: Dict[str, str] = {}
    used = set(py_ids)
    for nid, _ in cpp_items:
        if nid in used:
            n = 2
            while f"{nid}_cpp{n}" in used:
                n += 1
            remap[nid] = f"{nid}_cpp{n}"
        used.add(remap.get(nid, nid))
    if remap:
        _apply_remap(cpp, remap)

    bottom, top = _bbox_bottom(py), _bbox_top(cpp)
    dy = 0.0
    if bottom is not None and top is not None:
        dy = bottom + GAP - top
    if dy:
        for _, n in _node_items(cpp):
            p = _pos(n)
            if p:
                n["pos"] = [p[0], p[1] + dy]

    list_mode = isinstance(py.get("nodes"), list) or (not py.get("nodes") and isinstance(cpp.get("nodes"), list))
    merged = {k: v for k, v in py.items() if k not in ("nodes", "connections")}
    if list_mode:
        merged["nodes"] = [n for _, n in py_items] + [n for _, n in _node_items(cpp)]
    else:
        merged["nodes"] = {**{i: n for i, n in py_items}, **{i: n for i, n in _node_items(cpp)}}
    merged["connections"] = list(py.get("connections", [])) + list(cpp.get("connections", []))
    cpp_items = _node_items(cpp)

    meta = {"cpp_ids": [i for i, _ in cpp_items], "cpp_dy": dy, "remap": remap,
            "py_extra": {k: v for k, v in py.items() if k not in ("nodes", "connections")},
            "cpp_extra": {k: v for k, v in cpp.items() if k not in ("nodes", "connections")}}
    return merged, meta


def _apply_remap(session: dict, remap: Dict[str, str]) -> None:
    def fix(v):
        return remap.get(v, v) if isinstance(v, str) else v
    items = _node_items(session)
    for _, n in items:
        if n.get("id") in remap:
            n["id"] = remap[n["id"]]
        c = n.get("custom")
        if isinstance(c, dict):
            if "parent_group_id" in c:
                c["parent_group_id"] = fix(c["parent_group_id"])
            if isinstance(c.get("internal_nodes"), list):
                c["internal_nodes"] = [fix(i) for i in c["internal_nodes"]]
    if isinstance(session.get("nodes"), dict):
        session["nodes"] = {remap.get(k, k): v for k, v in session["nodes"].items()}
    for conn in session.get("connections", []):
        for side in ("out", "in"):
            if isinstance(conn.get(side), list) and conn[side]:
                conn[side][0] = fix(conn[side][0])


def split_session(merged: dict, meta: dict) -> Tuple[dict, dict]:
    """Обратная операция к merge_sessions: (py_session, cpp_session)."""
    merged = copy.deepcopy(merged)
    cpp_ids = set(meta.get("cpp_ids", []))
    dy = float(meta.get("cpp_dy", 0.0))
    inv = {v: k for k, v in meta.get("remap", {}).items()}

    items = _node_items(merged)
    list_mode = not isinstance(merged.get("nodes"), dict)
    py_nodes, cpp_nodes = ({} if not list_mode else []), ({} if not list_mode else [])
    for nid, n in items:
        if nid in cpp_ids:
            p = _pos(n)
            if p and dy:
                n["pos"] = [p[0], p[1] - dy]
            (cpp_nodes.__setitem__(nid, n) if not list_mode else cpp_nodes.append(n))
        else:
            (py_nodes.__setitem__(nid, n) if not list_mode else py_nodes.append(n))

    py_conn, cpp_conn = [], []
    for c in merged.get("connections", []):
        src, dst = c.get("out", [None])[0], c.get("in", [None])[0]
        if (src in cpp_ids) != (dst in cpp_ids):
            continue            # связь между вкладками: в двух графах её не выразить (см. cross_links)
        (cpp_conn if src in cpp_ids else py_conn).append(c)

    py = {**meta.get("py_extra", {}), "nodes": py_nodes, "connections": py_conn}
    cpp = {**meta.get("cpp_extra", {}), "nodes": cpp_nodes, "connections": cpp_conn}
    if inv:
        _apply_remap(cpp, inv)
    return py, cpp


def single_meta(merged: dict) -> dict:
    """
    merge_meta для графа, который изначально ОДИН (одно-холстовый режим): по нему
    split_session делит граф на python/cpp-части для режима двух вкладок.
    Язык узла — по его типу; группа идёт туда же, куда все её дети (иначе при делении
    группа и содержимое окажутся на разных вкладках). Сдвига раскладки нет.
    """
    items = _node_items(merged)
    lang_of = {nid: language_of(n) for nid, n in items}
    uid = {nid: (n.get("custom") or {}).get("group_uid") for nid, n in items if is_group(n)}
    kids: Dict[str, List[str]] = {}
    for nid, n in items:
        parent = (n.get("custom") or {}).get("parent_group_id")
        if parent:
            kids.setdefault(parent, []).append(nid)
    for _ in range(len(items) + 1):                        # вложенные группы: до неподвижной точки
        changed = False
        for nid, g_uid in uid.items():
            if lang_of.get(nid) is not None or not g_uid:
                continue
            langs = {lang_of.get(k) for k in kids.get(g_uid, [])}
            if langs == {CPP}:
                lang_of[nid] = CPP
                changed = True
        if not changed:
            break
    return {"cpp_ids": [nid for nid, _ in items if lang_of.get(nid) == CPP],
            "cpp_dy": 0.0, "remap": {},
            "py_extra": {k: v for k, v in merged.items() if k not in ("nodes", "connections")},
            "cpp_extra": {}}


def cross_links(merged: dict, meta: dict) -> List[dict]:
    """Связи между «python-частью» и «cpp-частью» — теряются при split_session."""
    cpp_ids = set(meta.get("cpp_ids", []))
    return [c for c in merged.get("connections", [])
            if (c.get("out", [None])[0] in cpp_ids) != (c.get("in", [None])[0] in cpp_ids)]
