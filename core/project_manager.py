import os
import yaml
import logging
import traceback
import re
import shutil
from core.workspace_manager import WorkspaceManager
from core.generators.cpp_generator import CppGenerator
from core.generators.python_generator import PythonGenerator
from ui.launch_compiler import LaunchCompiler

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
            # === ШАГ 0: ГЕНЕРАЦИЯ ДИНАМИЧЕСКОГО ИМЕНИ ПАКЕТА ===
            raw_name = os.path.basename(project_path)
            pkg_name = re.sub(r'[^a-z0-9_]', '_', raw_name.lower())
            
            blueprint_dir = os.path.join(project_path, ".blueprint")
            src_dir = os.path.join(project_path, "src")
            
            # 1. ЖЕСТКОЕ РАЗДЕЛЕНИЕ ПАПОК
            project_type = WorkspaceManager.get_project_type(project_path)
            
            if project_type == "cpp":
                code_dir = os.path.join(src_dir, "cpp")
            else:
                code_dir = os.path.join(src_dir, "python")
            
            launch_dir = os.path.join(src_dir, "launch")

            for d in [blueprint_dir, src_dir, code_dir, launch_dir]:
                if not os.path.exists(d): os.makedirs(d)

            logging.info(f"Saving project {pkg_name} ({project_type}) into {code_dir}")

            # 2. Сохраняем графы
            self._backup_dynamic_ports(self.graph_py)
            self._backup_dynamic_ports(self.graph_cpp)
            
            py_data = self.graph_py.serialize_session()
            cpp_data = self.graph_cpp.serialize_session()
            print(f"[SAVE DEBUG] project_type={project_type}")
            print(f"[SAVE DEBUG] cpp_data nodes={len(cpp_data.get('nodes', {}))}, py_data nodes={len(py_data.get('nodes', {}))}")
            print(f"[SAVE DEBUG] cpp_data keys={list(cpp_data.keys())}")
            # 3. ГЕНЕРАЦИЯ КОДА (только активный граф; возвращает файлы с группами)
            if project_type == "cpp":
                created_files = self._process_and_save_files(cpp_data, code_dir, project_type)
            else:
                created_files = self._process_and_save_files(py_data, code_dir, project_type)
            created_files = created_files or []

            print(f"[SAVE DEBUG] created_files = {created_files}")

            state = {"meta": {"version": "3.6"}, "graphs": {"python": py_data, "cpp": cpp_data}}
            with open(os.path.join(blueprint_dir, "state.yaml"), 'w', encoding='utf-8') as f:
                yaml.dump(state, f, sort_keys=False)
            
            # 3.1 ОЧИСТКА ФАЙЛОВ-ПРИЗРАКОВ (удалённые ноды, терминалы)
            valid_files = {cf['filename'] for cf in created_files}
            ext = ".cpp" if project_type == "cpp" else ".py"
            try:
                for f in os.listdir(code_dir):
                    if f.endswith(ext) and f not in valid_files:
                        os.remove(os.path.join(code_dir, f))
                        logging.info(f"🗑 Удалён устаревший файл: {f}")
            except Exception as e:
                logging.warning(f"Cleanup skipped: {e}")

            pycache_dir = os.path.join(launch_dir, "__pycache__")
            if os.path.isdir(pycache_dir):
                shutil.rmtree(pycache_dir, ignore_errors=True)
                
            # 4. ГЕНЕРАЦИЯ LAUNCH ФАЙЛОВ
            from ui.launch_compiler import LaunchCompiler

            # 4.1 Главный launch — ВСЕ ноды (Run запускает весь проект разом)
            main_compiler = LaunchCompiler(created_files, package_name=pkg_name)
            with open(os.path.join(launch_dir, "project_launch.py"), 'w', encoding='utf-8') as f:
                f.write(main_compiler.compile())

            # 4.2 Группировка по 'group' (приходит из _process_and_save_files)
            groups = {}
            for cf in created_files:
                groups.setdefault(cf.get('group', 'main'), []).append(cf)
            print(f"[SAVE DEBUG] groups = { {k: len(v) for k, v in groups.items()} }")

            # 4.3 Per-group launch
            for gname, gfiles in groups.items():
                gc = LaunchCompiler(gfiles, package_name=pkg_name)
                with open(os.path.join(launch_dir, f"{gname}_launch.py"), 'w', encoding='utf-8') as f:
                    f.write(gc.compile())

            # 4.4 docker-compose из групп
            try:
                from core.compose_generator import ComposeGenerator
                from core.container_config import ContainerConfigStore
                store = ContainerConfigStore(project_path)
                ComposeGenerator(project_path, pkg_name, store).generate(groups)
            except Exception as ce:
                logging.warning(f"Compose skipped: {ce}")

            # 5. CMAKE & PACKAGE.XML
            cmake_path = os.path.join(src_dir, "CMakeLists.txt")
            if project_type == "python":
                if os.path.exists(cmake_path): os.remove(cmake_path)
            else:
                # Метод сам вычислит имя пакета внутри, если ты обновил его как мы обсуждали
                self._generate_cpp_build_files(src_dir, created_files)

            return True

        except Exception as e:
            logging.error(f"Save Error: {e}")
            traceback.print_exc()
            return False

    def _map_files_to_groups(self, created_files, cpp_data, py_data, project_type):
        """Сопоставляет созданные файлы с деплой-группами по source_file нод."""
        # строим карту source_file -> container_group
        file_to_group = {}
        data = cpp_data if project_type == "cpp" else py_data
        nodes = data.get('nodes', {})
        if isinstance(nodes, dict):
            tmp = []
            for node_id, node_val in nodes.items():
                if isinstance(node_val, dict):
                    node_val = dict(node_val)  # копия, чтобы не портить исходник
                    node_val.setdefault('id', node_id)
                    tmp.append(node_val)
            nodes = tmp
        else:
            nodes = [n for n in nodes if isinstance(n, dict) and 'id' in n]

        nodes_dict = {n['id']: n for n in nodes}
        for n in nodes:
            print(f"[FLAT DEBUG]   нода type={n.get('type')}, parent_group_id={n.get('custom', {}).get('parent_group_id')!r}")
        groups = {}
        for cf in created_files:
            grp = file_to_group.get(cf['filename'], 'main')
            groups.setdefault(grp, []).append(cf)
        return groups

    def _process_and_save_files(self, session_data, target_dir, project_type):
        print("[PROC DEBUG] _process_and_save_files ВЫЗВАН!!!")
        is_cpp_mode = (project_type == "cpp")
        forced_extension = ".cpp" if is_cpp_mode else ".py"
        
        # Инициализируем генератор
        project_name = os.path.basename(os.path.dirname(target_dir))
        generator = CppGenerator(project_name) if is_cpp_mode else PythonGenerator(project_name)

        # Собираем все ноды в плоский список, учитывая неймспейсы сабграфов
        try:
            flat_nodes = self._flatten_nodes(session_data)
            print(f"[PROC DEBUG] flat_nodes получено: {len(flat_nodes)}")
        except Exception as e:
            import traceback
            print(f"[PROC DEBUG] _flatten_nodes УПАЛ: {e}")
            traceback.print_exc()
            flat_nodes = []

        created = []
        for node_entry in flat_nodes:
            node = node_entry['node']
            namespace = node_entry['namespace']
            container_group = node_entry.get('container_group', 'main')
            
            custom = node.get('custom', {})
            visual_name = node.get('name', 'Unnamed')

            ns_clean = (namespace or "").strip("/")
            container_group = ns_clean.split("/")[0].lower() if ns_clean else "main"
            print(f"[GROUP DEBUG] нода '{visual_name}' namespace='{namespace}' -> group='{container_group}'")
        
            raw_filename = custom.get('source_file') or ''
            base_name = os.path.splitext(os.path.basename(raw_filename))[0]
            if not base_name or base_name.startswith('.'):
                base_name = re.sub(r'[^a-zA-Z0-9_]', '_', visual_name)
            filename = base_name + forced_extension
            file_path = os.path.join(target_dir, filename)

            # Сбор данных для Jinja2 шаблона
            node_data = {
                'name': custom.get('node_name', visual_name),
                'namespace': namespace,
                'publishers': [],
                'subscribers': []
            }
            
            # Парсим порты для определения паблишеров/подписчиков
            for port in node.get('ports', []):
                port_name = port.get('name', 'port')
                # Очистка имени порта для использования в коде
                safe_port_name = re.sub(r'[^a-zA-Z0-9_]', '_', port_name).lower()
                
                # Если нода внутри сабграфа, топик должен учитывать неймспейс, 
                # если только это не глобальный топик (начинается с /)
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
                    if disk_code.strip():
                        existing_code = disk_code
                        custom['code_content'] = disk_code
            except Exception as e:
                logging.warning(f"Disk re-read skipped for {filename}: {e}")

            freshly_generated = generator.generate(node_data)

            if existing_code and existing_code.strip():
                
                code_to_write = existing_code
            else:
                
                code_to_write = freshly_generated

            if code_to_write:
                custom['code_content'] = code_to_write
                custom['source_file'] = filename

                created.append({
                    'language': 'cpp' if is_cpp_mode else 'python',
                    'filename': filename,
                    'group': container_group,
                })

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
                        logging.info(f"Сгенерирован/обновлен файл: {filename}")
                    except Exception as e:
                        logging.error(f"Write error {filename}: {e}")
                else:
                    logging.info(f"Пропущен (нет изменений): {filename}")

        return created

    def _flatten_nodes(self, session_data, current_ns="", inherited_group="main"):
        """
        Рекурсивно собирает все ноды из сессии, учитывая вложенность групп.
        Возвращает список: [{'node':..., 'namespace':..., 'container_group':...}]
        """
        flat_list = []
        nodes = session_data.get('nodes', [])

        if isinstance(nodes, dict):
            # ключ словаря — ID ноды; вписываем его внутрь
            tmp = []
            for node_id, node_val in nodes.items():
                if isinstance(node_val, dict):
                    node_val = dict(node_val)
                    node_val.setdefault('id', node_id)
                    tmp.append(node_val)
            nodes = tmp
        else:
            nodes = [n for n in nodes if isinstance(n, dict) and 'id' in n]

        nodes_dict = {n['id']: n for n in nodes}
        
        for node in nodes:
            _pgid = node.get('custom', {}).get('parent_group_id')
            print(f"[FLAT DEBUG] нода type={node.get('type')}, parent_group_id={_pgid!r}")
            if current_ns == "" and _pgid:
                print(f"[FLAT DEBUG]   -> ПРОПУЩЕНА (parent_group_id={_pgid})")
                continue

            node_type = node.get('type_') or node.get('type') or ''
            if node_type in ('ros.nodes.internal.SubGraphInputNode',
                             'ros.nodes.internal.SubGraphOutputNode'):
                continue
            
            if 'meta' in node_type:
                continue
            is_group = node_type in ['ros.nodes.RosGroup', 'ros.nodes.RosGroupNode']

            if is_group:
                group_name = node.get('custom', {}).get('node_name', node.get('name', 'group'))
                safe_group_name = re.sub(r'[^a-zA-Z0-9_]', '_', group_name)
                new_ns = f"{current_ns}/{safe_group_name}".replace("//", "/")

                internal_ids = node.get('custom', {}).get('internal_nodes', [])
                internal_nodes_data = [nodes_dict[nid] for nid in internal_ids if nid in nodes_dict]
                print(f"[FLATTEN DEBUG] группа '{group_name}': internal_ids={len(internal_ids)}, найдено нод={len(internal_nodes_data)}, new_ns='{new_ns}'")

                # имя группы становится деплой-группой для её детей
                group_container = node.get('custom', {}).get('container_group') or safe_group_name.lower()
                flat_list.extend(self._flatten_nodes(
                    {'nodes': internal_nodes_data}, new_ns, group_container))
            else:
                own_group = node.get('custom', {}).get('container_group') or inherited_group
                flat_list.append({'node': node, 'namespace': current_ns,
                                  'container_group': own_group})

        return flat_list

    def _generate_cpp_build_files(self, src_dir, created_files):
        # 1. ДИНАМИЧЕСКОЕ ИМЯ ПРОЕКТА (берем из папки, где лежит src)
        project_root = os.path.dirname(src_dir)
        raw_name = os.path.basename(project_root)
        # Приводим к стандарту ROS: маленькие буквы, только цифры и подчеркивания
        pkg_name = re.sub(r'[^a-z0-9_]', '_', raw_name.lower())
        
        cpp_dir = os.path.join(src_dir, "cpp")
        
        # БАЗОВЫЕ ЗАВИСИМОСТИ
        deps = set(['rclcpp', 'rclcpp_action', 'rclcpp_lifecycle', 'std_msgs', 'geometry_msgs', 'sensor_msgs', 'example_interfaces'])

        # УМНЫЙ СКАНЕР (Оставляем твою логику с OpenCV)
        pattern_msg = re.compile(r'#include\s+["<]([^/]+)/(msg|srv|action)/[^">]+[">]')
        pattern_libs = re.compile(r'#include\s+["<](tf2_ros|tf2|tf2_geometry_msgs|image_transport|cv_bridge|opencv2|rclcpp_lifecycle|rclcpp_components|lifecycle_msgs)/[^">]+[">]')

        if os.path.exists(cpp_dir):
            for file in os.listdir(cpp_dir):
                if file.endswith(('.cpp', '.hpp', '.h')):
                    with open(os.path.join(cpp_dir, file), 'r', encoding='utf-8') as f:
                        content = f.read()
                        for match in pattern_msg.findall(content):
                            deps.add(match[0])
                        for match in pattern_libs.findall(content):
                            if match == "opencv2":
                                deps.add("opencv")
                            else:
                                deps.add(match)

        # 3. ГЕНЕРАЦИЯ CMakeLists.txt
        # ВНИМАНИЕ: Здесь используем обычные строки, чтобы не путаться с f-строками и скобками
        cmake_content = "cmake_minimum_required(VERSION 3.8)\n"
        cmake_content += f"project({pkg_name})\n\n"
        
        cmake_content += "if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES \"Clang\")\n"
        cmake_content += "  add_compile_options(-Wall -Wextra -Wpedantic)\nendif()\n\n"
        
        cmake_content += "find_package(ament_cmake REQUIRED)\n"

        for dep in sorted(deps):
            cmake_name = "OpenCV" if dep == "opencv" else dep
            cmake_content += f"find_package({cmake_name} REQUIRED)\n"

        cmake_content += "\ninclude_directories(include)\n"
        cmake_content += "file(GLOB_RECURSE CPP_SOURCES \"cpp/*.cpp\")\n\n"
        
        # Исправленный цикл foreach (убраны двойные скобки)
        cmake_content += "foreach(source_file ${CPP_SOURCES})\n"
        cmake_content += "  get_filename_component(exec_name ${source_file} NAME_WE)\n"
        cmake_content += "  add_executable(${exec_name} ${source_file})\n"

        cmake_deps_list = [("OpenCV" if d == "opencv" else d) for d in sorted(deps)]
        deps_str = " ".join(cmake_deps_list)
        
        cmake_content += f"  ament_target_dependencies(${{exec_name}} {deps_str})\n"
        cmake_content += f"  install(TARGETS ${{exec_name}} DESTINATION lib/${{PROJECT_NAME}})\n"
        cmake_content += "endforeach()\n\n"

        cmake_content += "install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})\n"
        cmake_content += "ament_package()\n"

        # === SMART WRITE ДЛЯ CMakeLists.txt ===
        cmake_path = os.path.join(src_dir, "CMakeLists.txt")
        write_cmake = True
        if os.path.exists(cmake_path):
            try:
                with open(cmake_path, 'r', encoding='utf-8') as f:
                    if f.read() == cmake_content:
                        write_cmake = False # Контент идентичен, запись не нужна
            except Exception:
                pass
                
        if write_cmake:
            with open(cmake_path, 'w', encoding='utf-8') as f:
                f.write(cmake_content)
            logging.info("✨ CMakeLists.txt сгенерирован/обновлен")
        else:
            logging.info("⏭ CMakeLists.txt пропущен (нет изменений)")


        # 4. ГЕНЕРАЦИЯ package.xml
        pkg_xml = '<?xml version="1.0"?>\n'
        pkg_xml += '<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>\n'
        pkg_xml += f'<package format="3">\n  <name>{pkg_name}</name>\n  <version>0.0.0</version>\n' 
        pkg_xml += '  <description>Auto-generated by ROS2 Blueprint Studio</description>\n'
        pkg_xml += '  <maintainer email="user@email.com">User</maintainer>\n'
        pkg_xml += '  <license>TODO</license>\n  <buildtool_depend>ament_cmake</buildtool_depend>\n\n'

        # Динамически добавляем теги depend
        for dep in sorted(deps):
            pkg_xml += f"  <depend>{dep}</depend>\n"

        pkg_xml += '\n  <export><build_type>ament_cmake</build_type></export>\n</package>'

        # === SMART WRITE ДЛЯ package.xml ===
        pkg_path = os.path.join(src_dir, "package.xml")
        write_pkg = True
        if os.path.exists(pkg_path):
            try:
                with open(pkg_path, 'r', encoding='utf-8') as f:
                    if f.read() == pkg_xml:
                        write_pkg = False # Контент идентичен, запись не нужна
            except Exception:
                pass
                
        if write_pkg:
            with open(pkg_path, 'w', encoding='utf-8') as f:
                f.write(pkg_xml)
            logging.info("package.xml сгенерирован/обновлен")
        else:
            logging.info("⏭ package.xml пропущен (нет изменений)")

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