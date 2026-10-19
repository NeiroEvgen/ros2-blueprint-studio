"""
Studio localization: dictionaries in locales/<code>.json; to add a language, add a file.

    from core.i18n import t, set_language
    t("colcon.build_failed", packages="demo_stage")   # → "Build FAILED: demo_stage"

Rules:
  * a key is looked up in the current language -> in en -> otherwise the key itself is returned (the UI never fails);
  * {name} substitution; a missing parameter stays as is, extra ones are ignored;
  * plurals: t_plural("k", n) picks k.one / k.few / k.many / k.other by the language rules;
  * a language file may contain "_meta": {"name": "Deutsch"} for the list in the settings.
No Qt.
"""
import json
import locale
import os
import string
import threading
from typing import Dict, List, Optional

LOCALES_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "locales")
FALLBACK = "en"

_lock = threading.RLock()
_cache: Dict[str, Dict[str, str]] = {}
_current = None            # None -> detect on first use


# ------------------------------------------------------------------ loading
def _load(lang: str) -> Dict[str, str]:
    with _lock:
        if lang in _cache:
            return _cache[lang]
        data: Dict[str, str] = {}
        path = os.path.join(LOCALES_DIR, f"{lang}.json")
        try:
            with open(path, "r", encoding="utf-8") as f:
                raw = json.load(f)
            data = {k: v for k, v in raw.items() if isinstance(v, str) and not k.startswith("_")}
            data["_name"] = (raw.get("_meta") or {}).get("name", lang)
        except (OSError, ValueError):
            data = {}
        _cache[lang] = data
        return data


def reload() -> None:
    """Reset the cache (after editing the dictionary files)."""
    with _lock:
        _cache.clear()


def available_languages() -> List[Dict[str, str]]:
    """[{'code': 'ru', 'name': '...'}, ...] from the files in locales/."""
    out = []
    try:
        for fn in sorted(os.listdir(LOCALES_DIR)):
            if fn.endswith(".json"):
                code = fn[:-5]
                out.append({"code": code, "name": _load(code).get("_name", code)})
    except OSError:
        pass
    return out


# ------------------------------------------------------------------ current language
def detect_system_language() -> str:
    env = os.environ.get("STUDIO_LANG")
    if env:
        return env.split("_")[0].split("-")[0].lower()
    try:
        loc = locale.getdefaultlocale()[0] or ""
    except Exception:
        loc = ""
    code = loc.split("_")[0].lower()
    return code if code else FALLBACK


def get_language() -> str:
    global _current
    if _current is None:
        code = detect_system_language()
        _current = code if _load(code) else FALLBACK
    return _current


def set_language(code: str) -> bool:
    """Switch the language. False if there is no dictionary (the language stays)."""
    global _current
    code = (code or "").split("_")[0].split("-")[0].lower()
    if not _load(code):
        return False
    _current = code
    return True


# ------------------------------------------------------------------ translation
class _Safe(dict):
    def __missing__(self, key):
        return "{" + key + "}"


def _format(template: str, params: dict) -> str:
    try:
        return string.Formatter().vformat(template, (), _Safe(params))
    except (ValueError, IndexError, KeyError, AttributeError):
        return template


def lookup(key: str, lang: Optional[str] = None) -> Optional[str]:
    lang = lang or get_language()
    for code in (lang, FALLBACK):
        val = _load(code).get(key)
        if val is not None:
            return val
    return None


def t(key: str, **params) -> str:
    val = lookup(key)
    if val is None:
        return key
    return _format(val, params) if params or "{" in val else val


def _plural_category(lang: str, n: int) -> str:
    n = abs(int(n))
    if lang in ("ru", "uk", "be"):
        if n % 10 == 1 and n % 100 != 11:
            return "one"
        if 2 <= n % 10 <= 4 and not 12 <= n % 100 <= 14:
            return "few"
        return "many"
    if lang in ("ja", "zh", "ko", "vi"):
        return "other"
    return "one" if n == 1 else "other"


def t_plural(key: str, n: int, **params) -> str:
    lang = get_language()
    for cat in (_plural_category(lang, n), "other"):
        val = lookup(f"{key}.{cat}")
        if val is not None:
            return _format(val, {"n": n, **params})
    return key
