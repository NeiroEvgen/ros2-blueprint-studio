"""
Studio user settings (not per project): UI language etc.

File: <dir>/settings.json, where dir = $STUDIO_SETTINGS_DIR or ~/.ros_blueprint_studio.
Reading never raises (broken/missing file -> defaults);
writing is atomic. No Qt.
"""
import json
import os
from typing import Any

DEFAULTS = {"language": None, "single_canvas": True}   # None -> detect from the system


def settings_dir() -> str:
    return os.environ.get("STUDIO_SETTINGS_DIR") or os.path.join(os.path.expanduser("~"), ".ros_blueprint_studio")


def settings_path() -> str:
    return os.path.join(settings_dir(), "settings.json")


def load() -> dict:
    data = dict(DEFAULTS)
    try:
        with open(settings_path(), "r", encoding="utf-8") as f:
            raw = json.load(f)
        if isinstance(raw, dict):
            data.update({k: v for k, v in raw.items() if k in DEFAULTS})
    except (OSError, ValueError):
        pass
    return data


def get(key: str) -> Any:
    return load().get(key, DEFAULTS.get(key))


def set(key: str, value: Any) -> bool:      # noqa: A001 (API name)
    """Save one setting. False if it could not be written (the UI keeps working)."""
    if key not in DEFAULTS:
        raise KeyError(key)
    data = load()
    data[key] = value
    path = settings_path()
    try:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        tmp = path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
        os.replace(tmp, path)
        return True
    except OSError:
        return False


def apply_language() -> str:
    """Apply the saved language (if any and its dictionary exists). Returns the current language."""
    from core import i18n
    saved = get("language")
    if saved:
        i18n.set_language(saved)
    return i18n.get_language()
