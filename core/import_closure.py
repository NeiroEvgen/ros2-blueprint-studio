"""
Замыкание импортов Python от точки входа: какие файлы проекта реально нужны узлу.

Статический разбор через ast, без выполнения кода и без Qt. Результат:
  local        — файлы внутри корней поиска, достижимые от точки входа (включая __init__.py пакетов);
  third_party  — верхнеуровневые имена не из stdlib и не из корней (кандидаты в requirements/apt);
  stdlib       — имена стандартной библиотеки;
  optional     — импорты внутри try/except ImportError (их отсутствие не ломает запуск);
  dynamic      — importlib.import_module / __import__ с нелитеральным аргументом (статически не видно);
  unresolved   — относительные импорты, которые не удалось найти на диске.
"""
import ast
import os
import sys
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Set

STDLIB = set(getattr(sys, "stdlib_module_names", ())) | {"__future__"}


@dataclass
class Closure:
    entry: str
    local: List[str] = field(default_factory=list)
    third_party: Set[str] = field(default_factory=set)
    stdlib: Set[str] = field(default_factory=set)
    optional: Set[str] = field(default_factory=set)
    dynamic: List[str] = field(default_factory=list)       # "file:line"
    unresolved: List[str] = field(default_factory=list)    # "file:line: .name"
    errors: List[str] = field(default_factory=list)        # файлы, которые не удалось разобрать


def _find_module(parts: List[str], roots: List[str]) -> Optional[str]:
    """Файл модуля a.b.c в одном из корней: a/b/c.py или a/b/c/__init__.py."""
    for root in roots:
        base = os.path.join(root, *parts)
        if os.path.isfile(base + ".py"):
            return base + ".py"
        init = os.path.join(base, "__init__.py")
        if os.path.isfile(init):
            return init
    return None


def _package_inits(parts: List[str], roots: List[str]) -> List[str]:
    """__init__.py всех родительских пакетов: import a.b.c выполняет a/__init__ и a/b/__init__."""
    out = []
    for i in range(1, len(parts)):
        for root in roots:
            init = os.path.join(root, *parts[:i], "__init__.py")
            if os.path.isfile(init):
                out.append(init)
                break
    return out


def _is_import_error_guard(node: ast.Try) -> bool:
    for h in node.handlers:
        names = []
        if h.type is None:
            return True
        types = h.type.elts if isinstance(h.type, ast.Tuple) else [h.type]
        for t in types:
            names.append(getattr(t, "id", getattr(t, "attr", "")))
        if {"ImportError", "ModuleNotFoundError", "Exception"} & set(names):
            return True
    return False


class _Visitor(ast.NodeVisitor):
    def __init__(self):
        self.imports = []          # (kind, module, names, level, lineno, optional)
        self.dynamic = []          # lineno
        self._optional = 0

    def visit_Try(self, node):
        guarded = _is_import_error_guard(node)
        self._optional += guarded
        for n in node.body:
            self.visit(n)
        self._optional -= guarded
        for n in node.handlers + node.orelse + node.finalbody:
            self.visit(n)

    def visit_Import(self, node):
        for a in node.names:
            self.imports.append(("import", a.name, [], 0, node.lineno, bool(self._optional)))

    def visit_ImportFrom(self, node):
        self.imports.append(("from", node.module or "", [a.name for a in node.names],
                             node.level, node.lineno, bool(self._optional)))

    def visit_Call(self, node):
        f = node.func
        name = getattr(f, "id", getattr(f, "attr", ""))
        if name in ("import_module", "__import__"):
            arg = node.args[0] if node.args else None
            if isinstance(arg, ast.Constant) and isinstance(arg.value, str):
                self.imports.append(("import", arg.value, [], 0, node.lineno, bool(self._optional)))
            else:
                self.dynamic.append(node.lineno)
        self.generic_visit(node)


def compute(entry: str, roots: Optional[List[str]] = None) -> Closure:
    """entry — путь к .py; roots — корни поиска (по умолчанию каталог точки входа)."""
    entry = os.path.abspath(entry)
    roots = [os.path.abspath(r) for r in (roots or [os.path.dirname(entry)])]
    if os.path.dirname(entry) not in roots:
        roots.append(os.path.dirname(entry))        # как sys.path[0] при запуске скрипта

    res = Closure(entry=entry)
    seen: Dict[str, None] = {}
    queue = [entry]

    def add(path):
        if path not in seen:
            seen[path] = None
            queue.append(path)

    def rel(p):
        return os.path.relpath(p, roots[0])

    while queue:
        cur = queue.pop(0)
        seen.setdefault(cur, None)
        try:
            with open(cur, "r", encoding="utf-8") as f:
                tree = ast.parse(f.read(), filename=cur)
        except (OSError, SyntaxError, UnicodeDecodeError) as e:
            res.errors.append(f"{rel(cur)}: {type(e).__name__}")
            continue
        v = _Visitor()
        v.visit(tree)
        res.dynamic += [f"{rel(cur)}:{ln}" for ln in v.dynamic]

        for kind, module, names, level, lineno, optional in v.imports:
            if level:                                           # относительный импорт
                pkg = os.path.dirname(cur)
                for _ in range(level - 1):
                    pkg = os.path.dirname(pkg)
                base_parts = module.split(".") if module else []
                found = _resolve_from(pkg, base_parts, names)
                if not found:
                    res.unresolved.append(f"{rel(cur)}:{lineno}: {'.' * level}{module}")
                for p in found:
                    add(p)
                continue

            parts = module.split(".")
            top = parts[0]
            target = _find_module(parts, roots)
            if target:
                for init in _package_inits(parts, roots):
                    add(init)
                add(target)
                if kind == "from":                              # from pkg import submodule
                    for n in names:
                        sub = _find_module(parts + [n], roots)
                        if sub:
                            add(sub)
            elif top in STDLIB:
                res.stdlib.add(top)
            else:
                (res.optional if optional else res.third_party).add(top)

    res.local = sorted(rel(p) for p in seen if p != entry)
    res.local.insert(0, rel(entry))
    return res


def _resolve_from(pkg_dir: str, base_parts: List[str], names: List[str]) -> List[str]:
    out = []
    mod_dir = os.path.join(pkg_dir, *base_parts)
    init = os.path.join(mod_dir, "__init__.py")
    if base_parts:
        f = mod_dir + ".py"
        if os.path.isfile(f):
            out.append(f)
        elif os.path.isfile(init):
            out.append(init)
    elif os.path.isfile(os.path.join(pkg_dir, "__init__.py")):
        out.append(os.path.join(pkg_dir, "__init__.py"))
    for n in names:
        for cand in (os.path.join(mod_dir, n + ".py"), os.path.join(mod_dir, n, "__init__.py")):
            if os.path.isfile(cand):
                out.append(cand)
                break
    return out
