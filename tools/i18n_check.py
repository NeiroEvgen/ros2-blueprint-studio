"""
Проверка словарей локализации.  python -m tools.i18n_check

  * в каждом языке те же ключи, что в en (нет пропущенных / лишних);
  * плейсхолдеры {name} в переводе совпадают с оригиналом;
  * каждый ключ, использованный в коде как t("...") / t_plural("..."), есть в en;
  * для t_plural есть форма .other.
Код возврата 1, если есть проблемы.
"""
import ast
import json
import os
import string
import sys
from typing import Dict, List, Set

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LOCALES = os.path.join(ROOT, "locales")
SKIP_DIRS = {"venv", ".git", "__pycache__", "tests", "examples", "docker_image", "build", "node_modules"}


def load_locales() -> Dict[str, Dict[str, str]]:
    out = {}
    for fn in sorted(os.listdir(LOCALES)):
        if fn.endswith(".json"):
            with open(os.path.join(LOCALES, fn), encoding="utf-8") as f:
                raw = json.load(f)
            out[fn[:-5]] = {k: v for k, v in raw.items() if not k.startswith("_")}
    return out


def placeholders(text: str) -> Set[str]:
    try:
        return {name for _, name, _, _ in string.Formatter().parse(text) if name}
    except ValueError:
        return set()


def used_keys(root: str = ROOT):
    """(plain_keys, plural_keys) — литералы из вызовов t(...) / t_plural(...)."""
    plain, plural = {}, {}
    for dp, dns, fns in os.walk(root):
        dns[:] = [d for d in dns if d not in SKIP_DIRS and not d.startswith(".")]
        for fn in fns:
            if not fn.endswith(".py"):
                continue
            path = os.path.join(dp, fn)
            try:
                tree = ast.parse(open(path, encoding="utf-8").read())
            except (SyntaxError, UnicodeDecodeError):
                continue
            for node in ast.walk(tree):
                if isinstance(node, ast.Call) and node.args and isinstance(node.args[0], ast.Constant) \
                        and isinstance(node.args[0].value, str):
                    name = node.func.id if isinstance(node.func, ast.Name) else \
                        node.func.attr if isinstance(node.func, ast.Attribute) else None
                    where = f"{os.path.relpath(path, root)}:{node.lineno}"
                    if name == "t":
                        plain.setdefault(node.args[0].value, where)
                    elif name == "t_plural":
                        plural.setdefault(node.args[0].value, where)
    return plain, plural


def check() -> List[str]:
    problems: List[str] = []
    loc = load_locales()
    en = loc.get("en")
    if en is None:
        return ["locales/en.json отсутствует"]
    for code, d in loc.items():
        if code == "en":
            continue
        for k in sorted(set(en) - set(d)):
            if not _is_plural_variant_ok(k, d, code):
                problems.append(f"[{code}] нет перевода: {k}")
        for k in sorted(set(d) - set(en)):
            if not _is_plural_extra_ok(k, en):
                problems.append(f"[{code}] лишний ключ (нет в en): {k}")
        for k in sorted(set(d) & set(en)):
            if placeholders(d[k]) != placeholders(en[k]):
                problems.append(f"[{code}] плейсхолдеры {k}: {sorted(placeholders(d[k]))} != {sorted(placeholders(en[k]))}")
    plain, plural = used_keys()
    for k, where in sorted(plain.items()):
        if k not in en:
            problems.append(f"{where}: ключ t('{k}') не определён в en.json")
    for k, where in sorted(plural.items()):
        if f"{k}.other" not in en:
            problems.append(f"{where}: для t_plural('{k}') нужен ключ {k}.other в en.json")
    return problems


_PLURAL_SUFFIXES = (".one", ".few", ".many", ".other")


def _is_plural_variant_ok(key: str, d: dict, code: str) -> bool:
    """en имеет .one/.other; в ru могут быть .few/.many — отсутствие .few в en не ошибка, и наоборот
    у языка без нужной формы (напр. ja имеет только .other) пропуск допустим."""
    return key.endswith(_PLURAL_SUFFIXES[:2]) and code in ("ja", "zh", "ko", "vi") and key.endswith(".one")


def _is_plural_extra_ok(key: str, en: dict) -> bool:
    """Формы .few/.many есть только у части языков — их нет в en, и это нормально."""
    return key.endswith((".few", ".many")) and f"{key.rsplit('.', 1)[0]}.other" in en


if __name__ == "__main__":
    probs = check()
    for p in probs:
        print(p)
    print("OK" if not probs else f"{len(probs)} problem(s)")
    sys.exit(1 if probs else 0)
