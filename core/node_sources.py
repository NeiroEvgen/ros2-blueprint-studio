"""
Graph nodes -> source files.

  flatten_nodes()          — flat node list with namespace and deploy group (expands groups)
  process_and_save_files() — writes <code_dir>/<node>.(cpp|py); the disk is the source of truth for code

No Qt, no docker. The graph arrives here already serialized (dict).
"""
import hashlib
import logging
import os
import re
from core.template_render import render_pristine_template

from core.generators.cpp_generator import CppGenerator
from core.generators.python_generator import PythonGenerator


def _normalize_nodes(session_data):
    nodes = session_data.get('nodes', [])
    if isinstance(nodes, dict):
        # the dict key is the node ID; put it inside the node
        tmp = []
        for node_id, node_val in nodes.items():
            if isinstance(node_val, dict):
                node_val = dict(node_val)
                node_val.setdefault('id', node_id)
                tmp.append(node_val)
        return tmp
    return [n for n in nodes if isinstance(n, dict) and 'id' in n]


_GROUP_TYPES = ('ros.nodes.RosGroup', 'ros.nodes.RosGroupNode')
_SKIP_TYPES = ('ros.nodes.internal.SubGraphInputNode', 'ros.nodes.internal.SubGraphOutputNode')


def _node_type(node):
    return node.get('type_') or node.get('type') or ''


def _walk(nodes, nodes_dict, current_ns, inherited_group, seen):
    flat = []
    for node in nodes:
        nid = node.get('id')
        pgid = node.get('custom', {}).get('parent_group_id')
        if current_ns == "" and pgid:
            continue                      # group child: the group itself emits it
        node_type = _node_type(node)
        if node_type in _SKIP_TYPES or 'meta' in node_type:
            continue
        if nid in seen:
            continue                      # node already emitted (by the deepest group)
        seen.add(nid)

        if node_type in _GROUP_TYPES:
            custom = node.get('custom', {})
            group_name = custom.get('node_name', node.get('name', 'group'))
            safe_group_name = re.sub(r'[^a-zA-Z0-9_]', '_', group_name)
            new_ns = f"{current_ns}/{safe_group_name}".replace("//", "/")

            internal_ids = [i for i in custom.get('internal_nodes', []) if i in nodes_dict]
            # If a group lists both a nested group and its children, the child
            # belongs to the deepest group and is not emitted twice.
            nested = set()
            for i in internal_ids:
                inner = nodes_dict[i]
                if _node_type(inner) in _GROUP_TYPES:
                    nested.update(inner.get('custom', {}).get('internal_nodes', []))
            children = [nodes_dict[i] for i in internal_ids if i not in nested]
            logging.debug(f"[FLATTEN] group '{group_name}': children={len(children)}, ns='{new_ns}'")

            # Deploy group of the children: the group's explicit one -> otherwise (nested) inherited from
            # the outer group -> otherwise the top-level group name. Files and launch use the same rule.
            group_container = (custom.get('container_group')
                               or (inherited_group if current_ns else safe_group_name.lower()))
            flat.extend(_walk(children, nodes_dict, new_ns, group_container, seen))
        else:
            own_group = node.get('custom', {}).get('container_group') or inherited_group
            flat.append({'node': node, 'namespace': current_ns, 'container_group': own_group})
    return flat


def flatten_nodes(session_data, current_ns="", inherited_group="main"):
    """
    Collects all session nodes, expanding groups (nested ones too).
    Returns a list: [{'node':..., 'namespace':..., 'container_group':...}]
    Every node is emitted exactly once, in its deepest group.
    """
    nodes = _normalize_nodes(session_data)
    nodes_dict = {n['id']: n for n in nodes}
    return _walk(nodes, nodes_dict, current_ns, inherited_group, set())


def process_and_save_files(session_data, target_dir, project_type):
    """Graph session -> files in target_dir (expands groups and calls process_flat_nodes)."""
    try:
        flat_nodes = flatten_nodes(session_data)
    except Exception:
        logging.exception("flatten_nodes failed, nothing generated")
        flat_nodes = []
    return process_flat_nodes(flat_nodes, target_dir, project_type)


def code_hash(text):
    """Code fingerprint that ignores line endings (Windows writes \\r\\n, reads \\n)."""
    return hashlib.sha1((text or "").replace("\r\n", "\n").encode("utf-8")).hexdigest()


def choose_code(node_code, disk_code, synced_hash):
    """
    Whose code to write: the Studio edit (node_code) or the file on disk (disk_code).
    synced_hash is the code fingerprint at the node's last sync with the file.
      node unchanged               -> disk wins (edit in an IDE / external);
      node changed, disk did not   -> node wins (edit in the properties window);
      both changed                 -> disk (never overwrite someone else's data), reason "conflict".
    Without a fingerprint (old project, state unknown): disk, do not overwrite other edits;
    the fingerprint appears after the first save.
    Returns (code, reason).
    """
    if not disk_code or not disk_code.strip():
        return node_code, "no_disk"
    if not node_code or not node_code.strip() or code_hash(node_code) == code_hash(disk_code):
        return disk_code, "same"
    if synced_hash and code_hash(node_code) == synced_hash:
        return disk_code, "disk"
    if not synced_hash:
        return disk_code, "disk"
    if code_hash(disk_code) == synced_hash:
        return node_code, "node"
    return disk_code, "conflict"


def process_flat_nodes(flat_nodes, target_dir, project_type):
    """Flat node list ([{'node','namespace',...}]) -> files <target_dir>/<node>.(cpp|py)."""
    is_cpp_mode = (project_type == "cpp")
    forced_extension = ".cpp" if is_cpp_mode else ".py"

    # template generator
    project_name = os.path.basename(os.path.dirname(target_dir))
    generator = CppGenerator(project_name) if is_cpp_mode else PythonGenerator(project_name)

    created = []
    used_names = set()
    for node_entry in flat_nodes:
        node = node_entry['node']
        namespace = node_entry['namespace']
        container_group = node_entry.get('container_group') or 'main'

        custom = node.get('custom', {})
        visual_name = node.get('name', 'Unnamed')

        logging.debug(f"node '{visual_name}' namespace='{namespace}' -> group='{container_group}'")
    
        bound = (custom.get('bound_file') or '').replace('\\', '/').strip('/')
        if bound:
            # node bound to a file in place (green mark): its code lives at that path
            if not bound.endswith(('.cpp', '.cc', '.cxx') if is_cpp_mode else ('.py',)):
                continue                                  # a file of the other language: not for this pass
            file_path = os.path.normpath(os.path.join(os.path.dirname(target_dir), bound))
            filename = os.path.basename(bound)
            used_names.add(bound)
        else:
            raw_filename = custom.get('source_file') or ''
            base_name = os.path.splitext(os.path.basename(raw_filename))[0]
            if not base_name or base_name.startswith('.'):
                base_name = re.sub(r'[^a-zA-Z0-9_]', '_', visual_name)
            filename = base_name + forced_extension
            if filename in used_names:
                n = 2
                while f"{base_name}_{n}{forced_extension}" in used_names:
                    n += 1
                logging.warning(f"File name '{filename}' is taken by another node; '{visual_name}' "
                                f"gets '{base_name}_{n}{forced_extension}'")
                filename = f"{base_name}_{n}{forced_extension}"
            used_names.add(filename)
            file_path = os.path.join(target_dir, filename)

        # data for the Jinja2 template
        node_data = {
            'name': custom.get('node_name', visual_name),
            'namespace': namespace,
            'publishers': [],
            'subscribers': []
        }
        
        # ports -> publishers / subscribers
        for port in node.get('ports', []):
            port_name = port.get('name', 'port')
            # port name usable as an identifier in code
            safe_port_name = re.sub(r'[^a-zA-Z0-9_]', '_', port_name).lower()
            
            # Inside a subgraph the topic gets the namespace,
            # unless it is a global topic (starts with /)
            topic_name = port_name
            if namespace and not topic_name.startswith('/'):
                topic_name = f"{namespace}/{topic_name}"

            if port.get('type') == 'out':
                node_data['publishers'].append({'name': safe_port_name, 'topic': topic_name})
            elif port.get('type') == 'in':
                node_data['subscribers'].append({'name': safe_port_name, 'topic': topic_name})

        existing_code = custom.get('code_content', '')
        try:
            if os.path.exists(file_path):
                with open(file_path, 'r', encoding='utf-8') as df:
                    disk_code = df.read()
                existing_code, why = choose_code(existing_code, disk_code, custom.get('code_sync_hash'))
                if why == "conflict":
                    logging.warning(f"{filename}: changed both in Studio and on disk; the disk version wins")
                custom['code_content'] = existing_code
        except Exception as e:
            logging.warning(f"Disk re-read skipped for {filename}: {e}")

        existing_code = render_pristine_template(
            existing_code, custom, node_data['name'], is_cpp_mode)
        freshly_generated = generator.generate(node_data)

        if existing_code and existing_code.strip():
            
            code_to_write = existing_code
        else:
            
            code_to_write = freshly_generated

        if code_to_write:
            custom['code_content'] = code_to_write
            custom['code_sync_hash'] = code_hash(code_to_write)
            custom['source_file'] = filename
            if bound:
                custom['bound_file'] = bound

            entry = {
                'language': 'cpp' if is_cpp_mode else 'python',
                'filename': filename,
                'group': container_group,
            }
            if bound:
                entry['relpath'] = os.path.relpath(file_path, target_dir).replace(os.sep, '/')
            created.append(entry)

            write_needed = True
            if os.path.exists(file_path):
                try:
                    with open(file_path, 'r', encoding='utf-8') as existing_f:
                        if existing_f.read() == code_to_write:
                            write_needed = False
                except Exception:
                    pass

            if write_needed:
                try:
                    with open(file_path, 'w', encoding='utf-8') as f:
                        f.write(code_to_write)
                    logging.info(f"File generated/updated: {filename}")
                except Exception as e:
                    logging.error(f"Write error {filename}: {e}")
            else:
                logging.info(f"Skipped (no changes): {filename}")

    return created

