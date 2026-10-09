import os
import yaml
import logging
import traceback
import re
from core.workspace_manager import WorkspaceManager
from core.generators.launch_generator import write_launch_files

# YAML настройки для корректного сохранения кортежей (цветов и позиций)
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

# Маппинг для перевода "generic" имен в конкретные шаблоны
KEY_MAPPING = {
    "timer": "Timer", "pub": "Publisher", "sub": "Subscriber",
    "trigger": "Subscriber", "action": "ActionClient", "client": "ActionClient", "service": "Service"
}


# CMake-зоны и генерация живут в core.generators.build_generator (реэкспорт для совместимости)
from core.generators.build_generator import (  # noqa: F401
    CMAKE_AUTO_BEGIN, CMAKE_AUTO_END, CMAKE_USER_BEGIN, CMAKE_USER_END, CMAKE_USER_TEMPLATE,
)
from core.generators import build_generator
from core import node_sources
from core import graph_ir
from core.layout import ProjectLayout

class ProjectManager:
    def __init__(self, graph_py, graph_cpp):
        self.graph_py = graph_py
        self.graph_cpp = graph_cpp

    # ==========================================
    #               СОХРАНЕНИЕ
    # ==========================================
    def save_project(self, project_path):
        print("[SAVE DEBUG] save_project ВЫЗВАН, path=", project_path)
        try:
            raw_name = os.path.basename(project_path)
            pkg_name = re.sub(r'[^a-z0-9_]', '_', raw_name.lower())
            
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

            self._backup_dynamic_ports(self.graph_py)
            self._backup_dynamic_ports(self.graph_cpp)
            
            py_data = self.graph_py.serialize_session()
            cpp_data = self.graph_cpp.serialize_session()
            print(f"[SAVE DEBUG] project_type={project_type}")
            print(f"[SAVE DEBUG] cpp_data nodes={len(cpp_data.get('nodes', {}))}, py_data nodes={len(py_data.get('nodes', {}))}")
            print(f"[SAVE DEBUG] cpp_data keys={list(cpp_data.keys())}")

            # язык → сессия его вкладки (в «mixed» генерируются обе)
            sessions = {graph_ir.CPP: cpp_data, graph_ir.PYTHON: py_data}
            flat_by_lang = self._flat_by_language(languages, sessions)
            created_files = []
            for lang in languages:
                created_files += node_sources.process_flat_nodes(flat_by_lang[lang], code_dirs[lang], lang) or []

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

            print(f"[SAVE DEBUG] created_files = {created_files}")

            state = {"meta": {"version": "3.6"}, "graphs": {"python": py_data, "cpp": cpp_data}}
            with open(os.path.join(blueprint_dir, "state.yaml"), 'w', encoding='utf-8') as f:
                yaml.dump(state, f, sort_keys=False, allow_unicode=True)
            
            # 3.1 ОЧИСТКА ФАЙЛОВ-ПРИЗРАКОВ (по ПОЛНОМУ списку — до фильтрации по executable,
            # иначе .cpp-хелперы без main() будут ошибочно сочтены "устаревшими" и удалены).
            # Чистим только код-папку своего языка и только файлы его расширения.
            for lang in languages:
                valid_files = {cf['filename'] for cf in created_files if cf['language'] == lang}
                ext = ".cpp" if lang == graph_ir.CPP else ".py"
                try:
                    for f in os.listdir(code_dirs[lang]):
                        if f.endswith(ext) and f not in valid_files:
                            os.remove(os.path.join(code_dirs[lang], f))
                            logging.info(f"🗑 Удалён устаревший файл: {f}")
                except Exception as e:
                    logging.warning(f"Cleanup skipped: {e}")

            # 3.5 CMAKE & PACKAGE.XML — ДО генерации launch, чтобы отфильтровать
            # created_files по реальным executable раньше, чем они попадут в launch
            cmake_path = layout.cmake
            cpp_apt_deps = set()
            if graph_ir.CPP not in languages:
                if os.path.exists(cmake_path):
                    os.remove(cmake_path)
            else:
                cpp_apt_deps, executable_names = self._generate_cpp_build_files(src_dir, created_files)
                # Хелперы без main() (заинклюженные в другие ноды через #include)
                # не должны появляться в launch как отдельные запускаемые процессы.
                # Python-ноды фильтру не подлежат.
                before = {cf['filename'] for cf in created_files if cf['language'] == graph_ir.CPP}
                created_files = [cf for cf in created_files
                                 if cf['language'] != graph_ir.CPP
                                 or os.path.splitext(cf['filename'])[0] in executable_names]
                kept = {cf['filename'] for cf in created_files if cf['language'] == graph_ir.CPP}
                if before - kept:
                    logging.info(f"Excluded from launch (no main(), library helpers): {sorted(before - kept)}")

            # 4. ГЕНЕРАЦИЯ LAUNCH ФАЙЛОВ (created_files уже отфильтрован для C++)
            groups = write_launch_files(launch_dir, created_files, pkg_name)
            print(f"[SAVE DEBUG] groups = { {k: len(v) for k, v in groups.items()} }")

            try:
                from core.compose_generator import ComposeGenerator
                from core.container_config import ContainerConfigStore
                store = ContainerConfigStore(project_path)
                ComposeGenerator(project_path, pkg_name, store).generate(groups)
            except Exception as ce:
                logging.warning(f"Compose skipped: {ce}")

            return True, cpp_apt_deps

        except Exception as e:
            logging.error(f"Save Error: {e}")
            traceback.print_exc()
            return False

    def _flat_by_language(self, languages, sessions):
        """
        {язык: плоский список нод для генерации}. В одноязычном проекте — все ноды его вкладки.
        В mixed — обе вкладки, но нода с явным языком (ros.py.* на C++-вкладке) всё равно
        уходит в свой язык, а не в язык вкладки.
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

    def _process_and_save_files(self, session_data, target_dir, project_type):
        return node_sources.process_and_save_files(session_data, target_dir, project_type)

    def _flatten_nodes(self, session_data, current_ns="", inherited_group="main"):
        return node_sources.flatten_nodes(session_data, current_ns, inherited_group)

    def _resolve_cmake_user_block(self, cmake_path, new_auto):
        return build_generator.resolve_user_block(cmake_path, new_auto)

    def _generate_cpp_build_files(self, src_dir, created_files):
        return build_generator.generate_cpp_build_files(src_dir, created_files)

    # ==========================================
    #               ЗАГРУЗКА (RESTORED)
    # ==========================================
    def load_project(self, project_path):
        import copy # Добавляем импорт глубокого копирования
        try:
            if project_path.endswith('.json') or project_path.endswith('.yaml'):
                project_path = os.path.dirname(os.path.dirname(project_path))
            
            state_file = os.path.join(project_path, ".blueprint", "state.yaml")
            src_dir = os.path.join(project_path, "src")
            
            if not os.path.exists(state_file): return False
            
            with open(state_file, 'r', encoding='utf-8') as f:
                project_state = yaml.safe_load(f)
            
            self.graph_py.clear_session()
            self.graph_cpp.clear_session()
            
            py_data_raw = project_state["graphs"].get("python", {})
            cpp_data_raw = project_state["graphs"].get("cpp", {})
            
            # --- 1. ЖЕСТКО ЗАПОМИНАЕМ СВЯЗИ (До того, как движок их зачистит) ---
            py_conns = copy.deepcopy(py_data_raw.get('connections', []))
            cpp_conns = copy.deepcopy(cpp_data_raw.get('connections', []))
            
            py_data, py_map = self._prepare_data_for_loading(py_data_raw)
            cpp_data, cpp_map = self._prepare_data_for_loading(cpp_data_raw)
            
            # --- 2. ВОССТАНАВЛИВАЕМ НОДЫ ---
            self._safe_deserialize(self.graph_py, py_data)
            self._safe_deserialize(self.graph_cpp, cpp_data)
            
            # --- 3. ВОССТАНАВЛИВАЕМ КОД ---
            self._restore_files_and_code(self.graph_py, py_map, src_dir)
            self._restore_files_and_code(self.graph_cpp, cpp_map, src_dir)
            
            # --- 4. СОЗДАЕМ ДИНАМИЧЕСКИЕ ПОРТЫ ---
            self._restore_dynamic_ports(self.graph_py)
            self._restore_dynamic_ports(self.graph_cpp)
            
            # --- 5. ПРИНУДИТЕЛЬНО КЛЕИМ СВЯЗИ ОБРАТНО ---
            self._restore_connections(self.graph_py, py_conns, py_data_raw.get('nodes', {}))
            self._restore_connections(self.graph_cpp, cpp_conns, cpp_data_raw.get('nodes', {}))
            
            from nodes.group_logic import fix_group_after_load
            fix_group_after_load(self.graph_py, self.graph_py.all_nodes())
            fix_group_after_load(self.graph_cpp, self.graph_cpp.all_nodes())
            
            return True
        except Exception as e:
            logging.error(f"Load Error: {e}")
            traceback.print_exc()
            return False

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
        Десериализация, устойчивая к свойствам, которых нет в классе ноды
        (старые проекты / новые поля). Недостающие свойства создаются на лету.
        """
        from NodeGraphQt.base.model import NodeModel
        orig_set = NodeModel.set_property

        def patched_set(self_model, name, value, *a, **kw):
            if name not in self_model.properties and name not in self_model.custom_properties:
                # свойства нет — создаём, чтобы загрузка не падала
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
        """Восстанавливает код, ища файлы по имени ноды"""
        for node in graph.all_nodes():
            if node.name() in files_map:  
                fname = files_map[node.name()]
                 # Пытаемся найти файл в подпапках
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

                # Свойства ноды
                if not node.has_property('source_file'):
                    node.create_property('source_file', value=fname, widget_type=0)
                else:
                    node.set_property('source_file', fname)

                # Читаем код
                if found_path:
                    try:
                        with open(found_path, 'r', encoding='utf-8') as f:
                            node.set_property('code_content', f.read())
                    except: pass
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
    #               РЕДАКТОР КОДА
    # ==========================================
    def open_node_in_editor(self, node, project_path):
        import subprocess
        if not project_path: return
        
        src_dir = os.path.join(project_path, "src")
        project_type = WorkspaceManager.get_project_type(project_path)
        
        # Умный поиск подпапки (cpp или python)
        code_dir = os.path.join(src_dir, "cpp" if project_type == "cpp" else "python")
        if not os.path.exists(code_dir): 
            os.makedirs(code_dir)
            
        is_cpp = 'cpp' in node.type_
        ext = '.cpp' if is_cpp else '.py'
        
        # Выясняем имя файла
        filename = node.get_property('source_file') if node.has_property('source_file') else None
        if not filename:
            safe_name = re.sub(r'[^a-zA-Z0-9_]', '_', node.name())
            filename = f"{safe_name}{ext}"
            if not node.has_property('source_file'): 
                node.create_property('source_file', value=filename, widget_type=0)
            else: 
                node.set_property('source_file', filename)
                
        full_path = os.path.join(code_dir, filename)
        
        # Если файла еще нет на диске (новая нода), но код в ней есть - создаем
        if not os.path.exists(full_path):
            code = node.get_property('code_content') if node.has_property('code_content') else ""
            try:
                with open(full_path, 'w', encoding='utf-8') as f: 
                    f.write(code)
            except Exception as e: 
                logging.error(f"Failed to create file {full_path}: {e}")
                
        # Дергаем системный редактор (откроется VS Code, если он стоит по умолчанию)
        try: 
            if os.name == 'nt':  # Для Windows
                os.startfile(full_path)
            else:                # Для Linux / Mac
                subprocess.call(['xdg-open', full_path])
            logging.info(f"Successfully opened: {filename}")
        except Exception as e: 
            logging.error(f"Failed to open editor: {e}")

    # ==========================================
    #               СИНХРОНИЗАЦИЯ С ФАЙЛОМ
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
            log_callback(f"⚡ Sync: Added ports to {incoming_name}")
            target_node.update()

    # ==========================================
    #               ВСПОМОГАТЕЛЬНЫЕ
    # ==========================================
    def _get_nodes_iterable(self, session_data):
        nodes = session_data.get('nodes', [])
        if isinstance(nodes, dict): return nodes.values()
        return nodes
    
    
    def _backup_dynamic_ports(self, graph):
        for node in graph.all_nodes():
            try:
                ports_config = []
                
                # Безопасно извлекаем значение: если это функция — вызываем, если нет — берем как есть
                def get_safe(attr):
                    return attr() if callable(attr) else attr

                for port in node.input_ports():
                    # Проверяем multi_connection через view или напрямую, без вызова как функции
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
                logging.error(f"Ошибка бэкапа портов в ноде {node.name()}: {e}")
    
    def _restore_connections(self, graph, connections_data, raw_nodes_data):
        if not connections_data: return

        # 1. Достаем имена нод по их старым (мертвым) ID
        id_to_name = {}
        if isinstance(raw_nodes_data, dict):
            for nid, ndata in raw_nodes_data.items():
                id_to_name[nid] = ndata.get('name')
        elif isinstance(raw_nodes_data, list):
            for ndata in raw_nodes_data:
                id_to_name[ndata.get('id')] = ndata.get('name')

        # 2. Создаем карту новых, живых нод по их имени
        name_to_node = {node.name(): node for node in graph.all_nodes()}
        
        restored_count = 0

        for conn in connections_data:
            out_data = conn.get('out', [])
            in_data = conn.get('in', [])
            
            if len(out_data) == 2 and len(in_data) == 2:
                # Получаем имена нод по старым ID
                out_name = id_to_name.get(out_data[0])
                in_name = id_to_name.get(in_data[0])
                
                # Ищем новые ноды по именам
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
                            print(f" Ошибка соединения портов: {e}")
                    else:
                        print(f" Порты не найдены! Искали: {out_port_name} -> {in_port_name}")
                else:
                    print(f" Ноды не найдены по именам! {out_name} -> {in_name}")

        print(f"🔗 ВОССТАНОВЛЕНО СВЯЗЕЙ: {restored_count} из {len(connections_data)}")