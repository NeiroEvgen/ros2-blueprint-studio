"""
Подстановка полей в НЕИЗМЕНЁННЫЙ шаблон ноды палитры ({class_name}, {node_name}, {topic_name}…).

Шаблон кладётся в code_content «сырым» (имя ноды ещё могут поменять). Перед записью файла
его надо заполнить, иначе в .cpp/.py попадает `class {class_name}`. Если пользователь уже
правил код — файл не трогаем: в его коде `{node_name}` может быть обычной f-строкой.
Нетривиальные exec-порты ({EXEC_*}) здесь превращаются в пустоту.
"""
import re

from nodes.templates import CPP_TEMPLATES, PYTHON_TEMPLATES, template_name

_EXEC = ("{EXEC_IN_INIT}", "{EXEC_OUT_INIT}", "{EXEC_FIRE}", "{EXEC_VARS}")


def _class_name(custom, node_name):
    name = custom.get("class_name") or node_name or "Node"
    name = re.sub(r"\W", "_", str(name))
    return name if not name[0].isdigit() else "N" + name


def render_pristine_template(code, custom, node_name, is_cpp):
    key = custom.get("template_key")
    if not key or not code:
        return code
    tmpl = (CPP_TEMPLATES if is_cpp else PYTHON_TEMPLATES).get(template_name(key))
    if tmpl is None or code.strip() != tmpl.strip():
        return code
    values = {
        "{class_name}": _class_name(custom, node_name),
        "{node_name}": str(custom.get("node_name") or node_name or "node"),
        "{topic_name}": str(custom.get("topic_name") or "/chatter"),
        "{interval}": str(custom.get("interval") or 1.0),
    }
    out = tmpl
    for k, v in values.items():
        out = out.replace(k, v)
    for k in _EXEC:
        out = out.replace(k, "")
    return out
