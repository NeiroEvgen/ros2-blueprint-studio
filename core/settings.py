"""
Пользовательские настройки Studio (не проект): язык интерфейса и т.п.

Файл: <dir>/settings.json, где dir = $STUDIO_SETTINGS_DIR или ~/.ros_blueprint_studio.
Чтение никогда не бросает исключений (битый/отсутствующий файл → значения по умолчанию);
запись атомарная. Без Qt.
"""
import json
import os
from typing import Any

DEFAULTS = {"language": None, "single_canvas": False}   # None → определить по системе


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


def set(key: str, value: Any) -> bool:      # noqa: A001 — имя API
    """Сохранить одну настройку. False, если записать не удалось (интерфейс продолжает работать)."""
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
    """Применить сохранённый язык (если он есть и словарь существует). Возвращает текущий язык."""
    from core import i18n
    saved = get("language")
    if saved:
        i18n.set_language(saved)
    return i18n.get_language()
