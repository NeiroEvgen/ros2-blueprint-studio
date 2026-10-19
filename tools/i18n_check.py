"""
Localization dictionary check.  python -m tools.i18n_check

  * every language has the same keys as en (nothing missing / extra);
  * {name} placeholders in a translation match the original;
  * every key used in code as t("...") / t_plural("...") exists in en;
  * t_plural has an .other form.
Exit code 1 if there are problems.
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
    """(plain_keys, plural_keys): literals from t(...) / t_plural(...) calls."""
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
        return ["locales/en.json is missing"]
    for code, d in loc.items():
        if code == "en":
            continue
        for k in sorted(set(en) - set(d)):
            if not _is_plural_variant_ok(k, d, code):
                problems.append(f"[{code}] missing translation: {k}")
        for k in sorted(set(d) - set(en)):
            if not _is_plural_extra_ok(k, en):
                problems.append(f"[{code}] extra key (not in en): {k}")
        for k in sorted(set(d) & set(en)):
            if placeholders(d[k]) != placeholders(en[k]):
                problems.append(f"[{code}] placeholders {k}: {sorted(placeholders(d[k]))} != {sorted(placeholders(en[k]))}")
    plain, plural = used_keys()
    for k, where in sorted(plain.items()):
        if k not in en:
            problems.append(f"{where}: key t('{k}') is not defined in en.json")
    for k, where in sorted(plural.items()):
        if f"{k}.other" not in en:
            problems.append(f"{where}: t_plural('{k}') needs a key {k}.other in en.json")
    return problems


_PLURAL_SUFFIXES = (".one", ".few", ".many", ".other")


def _is_plural_variant_ok(key: str, d: dict, code: str) -> bool:
    """en has .one/.other; ru may have .few/.many: a .few missing in en is not an error, and conversely
    a language without a form (e.g. ja has only .other) may skip it."""
    return key.endswith(_PLURAL_SUFFIXES[:2]) and code in ("ja", "zh", "ko", "vi") and key.endswith(".one")


def _is_plural_extra_ok(key: str, en: dict) -> bool:
    """The .few/.many forms exist only in some languages: they are not in en, and that is fine."""
    return key.endswith((".few", ".many")) and f"{key.rsplit('.', 1)[0]}.other" in en


if __name__ == "__main__":
    probs = check()
    for p in probs:
        print(p)
    print("OK" if not probs else f"{len(probs)} problem(s)")
    sys.exit(1 if probs else 0)
