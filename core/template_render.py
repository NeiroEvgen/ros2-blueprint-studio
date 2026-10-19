"""
Fills the fields of an UNCHANGED palette node template ({class_name}, {node_name}, {topic_name}...).

The template goes into code_content "raw" (the node name may still change). It has to be
filled before the file is written, otherwise `class {class_name}` lands in the .cpp/.py. If the user
has already edited the code, the file is left alone: `{node_name}` may be a normal f-string there.
Non-trivial exec ports ({EXEC_*}) are turned into nothing here.
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
