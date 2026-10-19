import os
import yaml
import logging
import traceback
import re
from core.workspace_manager import WorkspaceManager
from core.generators.launch_generator import write_launch_files
from core.i18n import t

# YAML: write tuples (colours, positions) as plain lists and read them back as tuples
def tuple_representer(dumper, data):
    return dumper.represent_sequence('tag:yaml.org,2002:seq', data)
yaml.add_representer(tuple, tuple_representer)

def tuple_constructor(loader, node):
    return tuple(loader.construct_sequence(node))
yaml.add_constructor('tag:yaml.org,2002:python/tuple', tuple_constructor, Loader=yaml.SafeLoader)

try:
    from nodes.templates import PYTHON_TEMPLATES, CPP_TEMPLATES
except ImportError:
    PYTHON_TEMPLATES = {}; CPP_TEMPLATES = {}

# Generic short names -> concrete template names
KEY_MAPPING = {
    "timer": "Timer", "pub": "Publisher", "sub": "Subscriber",
    "trigger": "Subscriber", "action": "ActionClient", "client": "ActionClient", "service": "Service"
}


# CMake zones and generation live in core.generators.build_generator (re-exported for compatibility)
from core.generators.build_generator import (  # noqa: F401
    CMAKE_AUTO_BEGIN, CMAKE_AUTO_END, CMAKE_USER_BEGIN, CMAKE_USER_END, CMAKE_USER_TEMPLATE,
)
from core.generators import build_generator
from core import node_sources
from core import graph_ir
from core import state_store
from core import cmake_lint
from core.layout import ProjectLayout

class ProjectManager:
    def __init__(self, graph_py, graph_cpp):
        self.graph_py = graph_py
        self.graph_cpp = graph_cpp
        # Single canvas: both "language" graphs are the same object (see ui/graph_set.py)
        self.single = graph_py is graph_cpp
        self.last_warnings = []     # warnings of the last Save, already in the UI language
        # Project whose state.yaml failed to load: the canvas is empty, and Save would overwrite graph and code
        self.load_failed_path = None

    # ==========================================
    #               SAVE
    # ==========================================
    @staticmethod
    def _sync_live_nodes(pairs):
        """
        Push the final code and its sync hash back into the live nodes: otherwise the next edit
        in the properties window looks like "both sides changed" and gets lost.
        """
        for graph, session in pairs:
            try:
                items = dict(graph_ir._node_items(session))
                for node in graph.all_nodes():
                    custom = (items.get(node.id) or {}).get("custom") or {}
                    for key in ("code_content", "code_sync_hash"):
                        if key in custom and node.has_property(key) and node.get_property(key) != custom[key]:
                            node.set_property(key, custom[key])
            except Exception as e:      # syncing is a convenience, not a reason to fail the save
                logging.warning(f"Live node sync skipped: {e}")

    FILE_NODE_TYPE = "ros.nodes.meta.FileNode"

    def sync_file_nodes(self, src_dir):
        """
        File nodes (green mark) <-> files on disk, with the same rules as code nodes
        (core.node_sources.choose_code): edit in Studio -> file, edit on disk -> node,
        both -> disk wins. Returns the number of updated nodes.
        """
        graphs = [self.graph_py] if self.single else [self.graph_py, self.graph_cpp]
        changed = 0
        for g in graphs:
            for n in g.all_nodes():
                if getattr(n, "type_", "") != self.FILE_NODE_TYPE or not n.has_property('bound_file'):
                    continue
                rel = (n.get_property('bound_file') or '').replace('\\', '/').strip('/')
                full = os.path.normpath(os.path.join(src_dir, rel))
                if not rel or not os.path.isfile(full):
                    continue
                try:
                    with open(full, 'r', encoding='utf-8') as f:
                        disk = f.read()
                except (OSError, UnicodeDecodeError):
                    continue                     # binary file: nothing to show in the node
                node_code = n.get_property('code_content') or ''
                code, why = node_sources.choose_code(node_code, disk, n.get_property('code_sync_hash'))
                if why == "conflict":
                    logging.warning(f"{rel}: changed both in Studio and on disk; the disk version wins")
                if node_sources.code_hash(code) != node_sources.code_hash(disk):
                    with open(full, 'w', encoding='utf-8') as f:
                        f.write(code)
                if code != node_code:
                    n.set_property('code_content', code)
                    changed += 1
                n.set_property('code_sync_hash', node_sources.code_hash(code))
        return changed

    def save_project(self, project_path):
        self.last_warnings = []
        if self.load_failed_path and os.path.normpath(project_path) == self.load_failed_path:
            msg = t("save.state_unreadable")
            logging.error(msg)
            self.last_warnings.append(msg)
            return False
        try:
            pkg_name = build_generator.package_name_from_dir(project_path)
            
            blueprint_dir = os.path.join(project_path, ".blueprint")
            src_dir = os.path.join(project_path, "src")
            
            project_type = WorkspaceManager.get_project_type(project_path)
            languages = graph_ir.project_languages(project_type)   # ('cpp',) | ('python',) | ('cpp','python')
            layout = ProjectLayout(project_path)
            code_dirs = {lang: layout.code_dir(lang) for lang in languages}
            launch_dir = layout.launch

            for d in [blueprint_dir, src_dir, launch_dir, *code_dirs.values()]:
                if not os.path.exists(d): os.makedirs(d)

            logging.info(f"Saving project {pkg_name} ({project_type}) into {sorted(code_dirs.values())}")
            from core import generated_files
            prev_generated = generated_files.previous(project_path)   # before state.yaml is overwritten

            self.sync_file_nodes(src_dir)              # file-node edits -> disk, before serialize
            self._backup_dynamic_ports(self.graph_py)
            if not self.single:
                self._backup_dynamic_ports(self.graph_cpp)

            merged = None
            if self.single:
                # single graph: split by NODE language (not by tab); the raw graph goes into state v4
                merged = self.graph_py.serialize_session()
                py_data = cpp_data = None
            else:
                py_data = self.graph_py.serialize_session()
                cpp_data = self.graph_cpp.serialize_session()

            if self.single:
                flat_by_lang = self._flat_single(languages, merged, project_type)
            else:
                # language -> session of its tab ("mixed" generates both)
                sessions = {graph_ir.CPP: cpp_data, graph_ir.PYTHON: py_data}
                flat_by_lang = self._flat_by_language(languages, sessions)
            created_files = []
            for lang in languages:
                created_files += node_sources.process_flat_nodes(flat_by_lang[lang], code_dirs[lang], lang) or []
            self._sync_live_nodes([(self.graph_py, merged)] if self.single
                                  else [(self.graph_py, py_data), (self.graph_cpp, cpp_data)])

            from core.generators.dependency_resolver import DependencyResolver, DepSet
            from core.dockerfile_manager import DockerfileManager

            resolver = DependencyResolver()
            dep_set = DepSet()
            for lang in languages:
                dep_set.update(resolver.collect_from_nodes(flat_by_lang[lang], lang))

            dockerfile_mgr = DockerfileManager(project_path)
            dockerfile_mgr.ensure_exists()
            dockerfile_changed = dockerfile_mgr.write_auto_block(dep_set)
            if dockerfile_changed:
                logging.info(f"Dockerfile updated. apt: {sorted(dep_set.apt)}")
            if dep_set.unknown:
                logging.warning(f"Unresolved includes (not in registry): {sorted(dep_set.unknown)}")


            if self.single:
                state_store.write_state_v4(project_path, merged, graph_ir.single_meta(merged))
            else:
                state = {"meta": {"version": "3.6"}, "graphs": {"python": py_data, "cpp": cpp_data}}
                with open(os.path.join(blueprint_dir, "state.yaml"), 'w', encoding='utf-8') as f:
                    yaml.dump(state, f, sort_keys=False, allow_unicode=True)
            
            # 3.1 GHOST FILE CLEANUP (over the FULL list, before filtering by executable,
            # otherwise .cpp helpers without main() would be considered stale and removed).
            # Only the code folder of each language, only files with its extension, and only files
            # Studio generated itself (generated.json): other people's code in src/cpp is never touched.
            # A ghost is not deleted but moved to .blueprint/trash/<time>/.
            from core import build_marks
            marks = build_marks.load(project_path)
            now_generated = set()
            trash_dir = None
            for lang in languages:
                valid_files = {cf['filename'] for cf in created_files if cf['language'] == lang}
                ext = ".cpp" if lang == graph_ir.CPP else ".py"
                try:
                    for f in os.listdir(code_dirs[lang]):
                        rel = build_marks.norm(os.path.relpath(os.path.join(code_dirs[lang], f), src_dir))
                        if f in valid_files:
                            now_generated.add(rel)
                            continue
                        if build_marks.effective(marks, rel):
                            if rel in prev_generated:
                                now_generated.add(rel)   # if the mark is removed, it becomes a ghost again
                            continue       # marked in the Files tab: not a ghost
                        if f.endswith(ext) and rel in prev_generated:
                            trash_dir = trash_dir or generated_files.new_trash_dir(project_path)
                            generated_files.move_to_trash(src_dir, rel, trash_dir)
                            logging.info(f"Stale file {f} moved to {trash_dir}")
                except Exception as e:
                    logging.warning(f"Cleanup skipped: {e}")
            generated_files.save(project_path, now_generated)

            # 3.5 CMAKE & PACKAGE.XML before launch generation, so that created_files is
            # filtered by real executables before anything reaches the launch file
            cmake_path = layout.cmake
            cpp_apt_deps = set()
            if graph_ir.CPP not in languages:
                if os.path.exists(cmake_path):
                    os.remove(cmake_path)
            else:
                cpp_apt_deps, executable_names = self._generate_cpp_build_files(src_dir, created_files)
                self._lint_cmake(cmake_path)
                # Helpers without main() (pulled into other nodes via #include) must not
                # appear in launch as separate processes.
                # Python nodes are not filtered.
                before = {cf['filename'] for cf in created_files if cf['language'] == graph_ir.CPP}
                created_files = [cf for cf in created_files
                                 if cf['language'] != graph_ir.CPP
                                 or os.path.splitext(cf['filename'])[0] in executable_names]
                kept = {cf['filename'] for cf in created_files if cf['language'] == graph_ir.CPP}
                if before - kept:
                    logging.info(f"Excluded from launch (no main(), library helpers): {sorted(before - kept)}")

            # 4. LAUNCH FILES (created_files is already filtered for C++)
            groups = write_launch_files(launch_dir, created_files, pkg_name)

            try:
                from core.compose_generator import ComposeGenerator
                from core.container_config import ContainerConfigStore
                store = ContainerConfigStore(project_path)
                ComposeGenerator(project_path, pkg_name, store).generate(groups)
            except Exception as ce:
                logging.warning(f"Compose skipped: {ce}")

            self.sync_file_nodes(src_dir)              # CMakeLists/launch regenerated -> nodes show the fresh file
            return True, cpp_apt_deps

        except Exception as e:
            logging.error(f"Save Error: {e}")
            traceback.print_exc()
            return False

    def _flat_by_language(self, languages, sessions):
        """
        {language: flat list of nodes to generate}. In a single-language project: all nodes of its tab.
        In mixed: both tabs, but a node with an explicit language (ros.py.* on the C++ tab) still
        goes to its own language, not to the tab's.
        """
        flats = {lang: self._flatten_nodes(sessions[lang]) for lang in languages}
        if len(languages) == 1:
            return flats
        out = {lang: [] for lang in languages}
        for tab_lang, flat in flats.items():
            for entry in flat:
                lang = graph_ir.language_of(entry["node"]) or tab_lang
                out[lang if lang in out else tab_lang].append(entry)
        return out

    def _flat_single(self, languages, merged, project_type):
        """
        Single graph -> {language: flat list}. A node's language comes from its type; neutral nodes
        (notes etc.) get the project language. In a single-language project the whole graph goes to it.
        """
        flat = self._flatten_nodes(merged)
        if len(languages) == 1:
            return {languages[0]: flat}
        parts = graph_ir.partition_by_language(flat, graph_ir.default_language(project_type))
        return {lang: parts[lang] for lang in languages}

    def _process_and_save_files(self, session_data, target_dir, project_type):
        return node_sources.process_and_save_files(session_data, target_dir, project_type)

    def _flatten_nodes(self, session_data, current_ns="", inherited_group="main"):
        return node_sources.flatten_nodes(session_data, current_ns, inherited_group)

    def _lint_cmake(self, cmake_path):
        """Finds typical CMake breakages (including in the USER zone) before colcon and stores the warnings."""
        try:
            with open(cmake_path, 'r', encoding='utf-8') as f:
                issues = cmake_lint.lint(f.read())
        except OSError:
            return
        for issue in issues:
            self.last_warnings.append(issue.message)
            logging.warning(issue.message)

    def _resolve_cmake_user_block(self, cmake_path, new_auto):
        return build_generator.resolve_user_block(cmake_path, new_auto)

    def _generate_cpp_build_files(self, src_dir, created_files):
        return build_generator.generate_cpp_build_files(src_dir, created_files)

    # ==========================================
    #               LOAD
    # ==========================================
    def load_project(self, project_path):
        import copy
        try:
            if project_path.endswith('.json') or project_path.endswith('.yaml'):
                project_path = os.path.dirname(os.path.dirname(project_path))
            
            state_file = os.path.join(project_path, ".blueprint", "state.yaml")
            src_dir = os.path.join(project_path, "src")
            self.load_failed_path = None

            # ALWAYS clear the canvas: otherwise the previous project's nodes end up in the new one on Save
            for g in ([self.graph_py] if self.single else [self.graph_py, self.graph_cpp]):
                g.clear_session()

            if not os.path.exists(state_file): return False
            
            # Any supported state.yaml version (3.6: two tabs; 4: one shared graph, split into
            # python/cpp parts using the merge metadata) -> two sessions.
            loaded_state = state_store.read_state(project_path)
            if loaded_state is None: return False

            if self.single:
                return self._load_into_single(loaded_state, src_dir)

            self.graph_py.clear_session()
            self.graph_cpp.clear_session()

            py_data_raw = loaded_state.py
            cpp_data_raw = loaded_state.cpp
            
            # 1. Copy connections before deserializing: the engine drops those whose ports do not exist yet
            py_conns = copy.deepcopy(py_data_raw.get('connections', []))
            cpp_conns = copy.deepcopy(cpp_data_raw.get('connections', []))
            
            py_data, py_map = self._prepare_data_for_loading(py_data_raw)
            cpp_data, cpp_map = self._prepare_data_for_loading(cpp_data_raw)
            
            # 2. Nodes
            self._safe_deserialize(self.graph_py, py_data)
            self._safe_deserialize(self.graph_cpp, cpp_data)
            
            # 3. Code from files on disk
            self._restore_files_and_code(self.graph_py, py_map, src_dir)
            self._restore_files_and_code(self.graph_cpp, cpp_map, src_dir)
            
            # 4. Dynamic ports
            self._restore_dynamic_ports(self.graph_py)
            self._restore_dynamic_ports(self.graph_cpp)
            
            # 5. Connections, by node name (ids are new after deserialization)
            self._restore_connections(self.graph_py, py_conns, py_data_raw.get('nodes', {}))
            self._restore_connections(self.graph_cpp, cpp_conns, cpp_data_raw.get('nodes', {}))
            
            from nodes.group_logic import fix_group_after_load
            fix_group_after_load(self.graph_py, self.graph_py.all_nodes())
            fix_group_after_load(self.graph_cpp, self.graph_cpp.all_nodes())
            
            return True
        except Exception as e:
            logging.error(f"Load Error: {e}")
            traceback.print_exc()
            if os.path.exists(os.path.join(project_path, ".blueprint", "state.yaml")):
                self.load_failed_path = os.path.normpath(project_path)
            return False

    def _load_into_single(self, loaded_state, src_dir):
        """Load into the single shared canvas (state v4 as is; v3.6 is already merged in loaded_state.merged)."""
        import copy
        from nodes.group_logic import fix_group_after_load
        graph = self.graph_py
        graph.clear_session()
        raw = loaded_state.merged
        conns = copy.deepcopy(raw.get('connections', []))
        data, files_map = self._prepare_data_for_loading(raw)
        self._safe_deserialize(graph, data)
        self._restore_files_and_code(graph, files_map, src_dir)
        self._restore_dynamic_ports(graph)
        self._restore_connections(graph, conns, raw.get('nodes', {}))
        fix_group_after_load(graph, graph.all_nodes())
        return True

    def _prepare_data_for_loading(self, session_data):
        files_map = {}
        nodes = session_data.get('nodes', [])
        if isinstance(nodes, list):
            for n in nodes:
                c = n.get('custom', {})
                if 'source_file' in c: 
                    files_map[n.get('name')] = c['source_file'] 
                    del c['source_file']
        elif isinstance(nodes, dict):
            for nid, n in nodes.items():
                c = n.get('custom', {})
                if 'source_file' in c: 
                    files_map[n.get('name')] = c['source_file'] 
                    del c['source_file']
        return session_data, files_map

    def _safe_deserialize(self, graph, data):
        """
        Deserialization that tolerates properties missing from the node class
        (old projects / new fields). Missing properties are created on the fly.
        """
        from NodeGraphQt.base.model import NodeModel
        orig_set = NodeModel.set_property

        def patched_set(self_model, name, value, *a, **kw):
            if name not in self_model.properties and name not in self_model.custom_properties:
                # no such property: create it so that loading does not fail
                try:
                    self_model.custom_properties[name] = value
                    return
                except Exception:
                    pass
            return orig_set(self_model, name, value, *a, **kw)

        NodeModel.set_property = patched_set
        try:
            graph.deserialize_session(data)
        finally:
            NodeModel.set_property = orig_set

    def _restore_files_and_code(self, graph, files_map, src_dir):
        """Restores node code by looking up files by node name."""
        for node in graph.all_nodes():
            if node.name() in files_map:  
                fname = files_map[node.name()]
                 # look in the language subfolders
                possible_paths = [
                    os.path.join(src_dir, "cpp", fname),
                    os.path.join(src_dir, "python", fname),
                    os.path.join(src_dir, fname)
                ]
                
                found_path = None
                for p in possible_paths:
                    if os.path.exists(p):
                        found_path = p
                        break

                # node properties
                if not node.has_property('source_file'):
                    node.create_property('source_file', value=fname, widget_type=0)
                else:
                    node.set_property('source_file', fname)

                # read the code
                if found_path:
                    try:
                        with open(found_path, 'r', encoding='utf-8') as f:
                            text = f.read()
                        node.set_property('code_content', text)
                        if node.has_property('code_sync_hash'):
                            node.set_property('code_sync_hash', node_sources.code_hash(text))
                    except Exception:
                        pass
                else:
                    node.set_property('code_content', "// Source file not found on disk.")

    def _restore_dynamic_ports(self, graph):
        for node in graph.all_nodes():
            if not node.has_property('saved_ports_config'): continue
            saved = node.get_property('saved_ports_config')
            if not saved: continue
            curr_in = [p.name() for p in node.input_ports()]
            curr_out = [p.name() for p in node.output_ports()]
            for p in saved:
                name = p['name']; clr = tuple(p.get('color', (255,255,255)))
                try:
                    if p['type'] == 'in' and name not in curr_in:
                        node.add_input(name, color=clr, multi_input=True)
                    elif p['type'] == 'out' and name not in curr_out:
                        node.add_output(name, color=clr, multi_output=True)
                except: pass

    # ==========================================
    #               CODE EDITOR
    # ==========================================
    def open_node_in_editor(self, node, project_path):
        import subprocess
        if not project_path: return
        
        src_dir = os.path.join(project_path, "src")
        project_type = WorkspaceManager.get_project_type(project_path)
        
        # pick the code subfolder (cpp or python)
        code_dir = os.path.join(src_dir, "cpp" if project_type == "cpp" else "python")
        if not os.path.exists(code_dir): 
            os.makedirs(code_dir)
            
        is_cpp = 'cpp' in node.type_
        ext = '.cpp' if is_cpp else '.py'
        
        # file name
        filename = node.get_property('source_file') if node.has_property('source_file') else None
        if not filename:
            safe_name = re.sub(r'[^a-zA-Z0-9_]', '_', node.name())
            filename = f"{safe_name}{ext}"
            if not node.has_property('source_file'): 
                node.create_property('source_file', value=filename, widget_type=0)
            else: 
                node.set_property('source_file', filename)
                
        full_path = os.path.join(code_dir, filename)
        
        # no file on disk yet (new node) but the node has code: create it
        if not os.path.exists(full_path):
            code = node.get_property('code_content') if node.has_property('code_content') else ""
            try:
                with open(full_path, 'w', encoding='utf-8') as f: 
                    f.write(code)
            except Exception as e: 
                logging.error(f"Failed to create file {full_path}: {e}")
                
        # open the system editor (VS Code if it is the default)
        try: 
            if os.name == 'nt':
                os.startfile(full_path)
            else:
                subprocess.call(['xdg-open', full_path])
            logging.info(f"Successfully opened: {filename}")
        except Exception as e: 
            logging.error(f"Failed to open editor: {e}")

    # ==========================================
    #               SYNC FROM FILE
    # ==========================================
    def sync_node_from_file(self, filename, content, ports, graphs, log_callback):
        incoming_name = os.path.basename(filename)
        target_node = None
        
        for g in graphs:
            for node in g.all_nodes():
                node_file = node.get_property('source_file')
                if node_file and os.path.basename(node_file) == incoming_name:
                    target_node = node; break
            if target_node: break
            
        if not target_node: return
        
        target_node.set_property('code_content', content)
        
        current_outputs = [p.name() for p in target_node.output_ports()]
        current_inputs = [p.name() for p in target_node.input_ports()]
        updated = False
        
        for port_data in ports:
            p_name = port_data['name']
            color = (255, 255, 255)
            if "twist" in port_data['topic'].lower(): color = (255, 152, 0)
            elif "string" in port_data['topic'].lower(): color = (255, 235, 59)
            
            if port_data['mode'] == 'pub' and p_name not in current_outputs:
                target_node.add_output(name=p_name, multi_output=True, display_name=True, color=color)
                updated = True
            elif port_data['mode'] == 'sub' and p_name not in current_inputs:
                target_node.add_input(name=p_name, multi_input=True, display_name=True, color=color)
                updated = True
                
        if updated: 
            log_callback(f"Sync: Added ports to {incoming_name}")
            target_node.update()

    # ==========================================
    #               HELPERS
    # ==========================================
    def _get_nodes_iterable(self, session_data):
        nodes = session_data.get('nodes', [])
        if isinstance(nodes, dict): return nodes.values()
        return nodes
    
    
    def _backup_dynamic_ports(self, graph):
        for node in graph.all_nodes():
            try:
                ports_config = []
                
                # the attribute may be a method or a plain value
                def get_safe(attr):
                    return attr() if callable(attr) else attr

                for port in node.input_ports():
                    # read multi_connection from the view, do not call it
                    is_multi = port.view.multi_connection if hasattr(port, 'view') else False
                    
                    ports_config.append({
                        'type': 'in', 
                        'name': port.name(), 
                        'color': list(get_safe(port.color)), 
                        'multi': is_multi
                    })

                for port in node.output_ports():
                    is_multi = port.view.multi_connection if hasattr(port, 'view') else False
                    
                    ports_config.append({
                        'type': 'out', 
                        'name': port.name(), 
                        'color': list(get_safe(port.color)), 
                        'multi': is_multi
                    })

                if not node.has_property('saved_ports_config'):
                    node.create_property('saved_ports_config', value=ports_config, widget_type=0)
                else:
                    node.set_property('saved_ports_config', ports_config)
                    
            except Exception as e:
                logging.error(f"Port backup failed for node {node.name()}: {e}")
    
    def _restore_connections(self, graph, connections_data, raw_nodes_data):
        if not connections_data: return

        # 1. Node names by their old (dead) ids
        id_to_name = {}
        if isinstance(raw_nodes_data, dict):
            for nid, ndata in raw_nodes_data.items():
                id_to_name[nid] = ndata.get('name')
        elif isinstance(raw_nodes_data, list):
            for ndata in raw_nodes_data:
                id_to_name[ndata.get('id')] = ndata.get('name')

        # 2. Map of the new, live nodes by name
        name_to_node = {node.name(): node for node in graph.all_nodes()}
        
        restored_count = 0

        for conn in connections_data:
            out_data = conn.get('out', [])
            in_data = conn.get('in', [])
            
            if len(out_data) == 2 and len(in_data) == 2:
                # old ids -> names
                out_name = id_to_name.get(out_data[0])
                in_name = id_to_name.get(in_data[0])
                
                # names -> new nodes
                out_node = name_to_node.get(out_name)
                in_node = name_to_node.get(in_name)
                
                if out_node and in_node:
                    out_port_name = out_data[1]
                    in_port_name = in_data[1]
                    
                    out_port = next((p for p in out_node.output_ports() if p.name() == out_port_name), None)
                    in_port = next((p for p in in_node.input_ports() if p.name() == in_port_name), None)
                    
                    if out_port and in_port:
                        try:
                            out_port.connect_to(in_port)
                            restored_count += 1
                        except Exception as e:
                            logging.warning(f"Connection {out_name}.{out_port_name} -> {in_name}.{in_port_name} failed: {e}")
                    else:
                        logging.warning(f"Ports not found: {out_name}.{out_port_name} -> {in_name}.{in_port_name}")
                else:
                    logging.warning(f"Nodes not found by name: {out_name} -> {in_name}")

        logging.info(f"Connections restored: {restored_count} of {len(connections_data)}")