import os
import yaml
import logging
import traceback
import re
import shutil
from core.workspace_manager import WorkspaceManager

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
        try:
            blueprint_dir = os.path.join(project_path, ".blueprint")
            src_dir = os.path.join(project_path, "src")
            
            # 1. ЖЕСТКОЕ РАЗДЕЛЕНИЕ ПАПОК
            project_type = WorkspaceManager.get_project_type(project_path) # 'cpp' или 'python'
            
            # Определяем подпапку: src/cpp или src/python
            if project_type == "cpp":
                code_dir = os.path.join(src_dir, "cpp")
            else:
                code_dir = os.path.join(src_dir, "python")
            
            launch_dir = os.path.join(src_dir, "launch")

            # Создаем структуру
            for d in [blueprint_dir, src_dir, code_dir, launch_dir]:
                if not os.path.exists(d): os.makedirs(d)

            logging.info(f"Saving project type: {project_type} into {code_dir}")

            # 2. Сохраняем графы
            self._backup_dynamic_ports(self.graph_py)
            self._backup_dynamic_ports(self.graph_cpp)
            
            py_data = self.graph_py.serialize_session()
            cpp_data = self.graph_cpp.serialize_session()
            
            # 3. ГЕНЕРАЦИЯ КОДА
            self._process_and_save_files(py_data, code_dir, project_type)
            self._process_and_save_files(cpp_data, code_dir, project_type)

            # Сохраняем состояние графов
            state = {"meta": {"version": "3.6"}, "graphs": {"python": py_data, "cpp": cpp_data}}
            with open(os.path.join(blueprint_dir, "state.yaml"), 'w', encoding='utf-8') as f:
                yaml.dump(state, f, sort_keys=False)

            # 4. ГЕНЕРАЦИЯ LAUNCH ФАЙЛА
            created_files = []
            for f in os.listdir(code_dir):
                if project_type == "cpp" and f.endswith(".cpp"):
                    created_files.append({'language': 'cpp', 'filename': f})
                elif project_type == "python" and f.endswith(".py"):
                    created_files.append({'language': 'python', 'filename': f})

            from ui.launch_compiler import LaunchCompiler
            compiler = LaunchCompiler(created_files)
            launch_code = compiler.compile()
            
            with open(os.path.join(launch_dir, "project_launch.py"), 'w', encoding='utf-8') as f:
                f.write(launch_code)

            # 5. CMAKE & PACKAGE.XML (Только для C++)
            cmake_path = os.path.join(src_dir, "CMakeLists.txt")
            if project_type == "python":
                if os.path.exists(cmake_path): os.remove(cmake_path)
            else:
                self._generate_cpp_build_files(src_dir, created_files)

            return True

        except Exception as e:
            logging.error(f"Save Error: {e}")
            traceback.print_exc()
            return False

    def _process_and_save_files(self, session_data, target_dir, project_type):
        is_cpp_mode = (project_type == "cpp")
        forced_extension = ".cpp" if is_cpp_mode else ".py"

        for node in self._get_nodes_iterable(session_data):
            custom = node.get('custom', {})
            # Визуальное имя ноды (например "Cpp Timer")
            visual_name = node.get('name', 'Unnamed')
            node_type = node.get('type', '')

            # === ROS NODE NAME FIX ===
            # 1. Пытаемся взять имя из свойства 'node_name' (там обычно "cpp_timer")
            # 2. Если нет, берем визуальное имя и меняем пробелы на _
            raw_ros_name = custom.get('node_name', visual_name)
            safe_ros_name = re.sub(r'[^a-zA-Z0-9_]', '_', raw_ros_name)

            # ИМЯ ФАЙЛА (Source File)
            raw_filename = custom.get('source_file', re.sub(r'[^a-zA-Z0-9_]', '_', visual_name))
            base_name = os.path.splitext(raw_filename)[0]
            filename = base_name + forced_extension
            file_path = os.path.join(target_dir, filename)

            # ПОДБОР ШАБЛОНА
            check_str = (visual_name + node_type).lower()
            template_key = "Default"
            for key_substr, t_name in KEY_MAPPING.items():
                if key_substr in check_str:
                    template_key = t_name
                    break

            templates_dict = CPP_TEMPLATES if is_cpp_mode else PYTHON_TEMPLATES
            base_template = templates_dict.get(template_key, templates_dict.get("Default", ""))

            # КОНТЕНТ (КАРАНТИН)
            current_code = custom.get('code_content', "")
            if is_cpp_mode:
                if "import rclpy" in current_code or "def __init__" in current_code:
                    current_code = base_template 
            if not is_cpp_mode:
                if "#include" in current_code or "rclcpp::" in current_code:
                    current_code = base_template

            code_to_write = current_code if current_code else base_template

            # ГЕНЕРАЦИЯ
            if code_to_write:
                # Имя класса (без цифр в начале)
                class_name = re.sub(r'[^a-zA-Z0-9_]', '', base_name)
                if is_cpp_mode and class_name and class_name[0].isdigit():
                    class_name = "Node" + class_name
                if not class_name: class_name = "MyNode"

                code_to_write = code_to_write.replace("{class_name}", class_name)
                
                # ВАЖНО: Вставляем безопасное имя ноды (без пробелов)
                code_to_write = code_to_write.replace("{node_name}", safe_ros_name)
                
                code_to_write = code_to_write.replace("{topic_name}", "topic_1")
                code_to_write = code_to_write.replace("{interval}", "0.5")
                
                for tag in ["{EXEC_IN_INIT}", "{EXEC_OUT_INIT}", "{EXEC_FIRE}", "{EXEC_VARS}"]:
                    code_to_write = code_to_write.replace(tag, "")

                custom['code_content'] = code_to_write
                custom['source_file'] = filename
                
                try:
                    with open(file_path, 'w', encoding='utf-8') as f:
                        f.write(code_to_write)
                except Exception as e:
                    logging.error(f"Write error {filename}: {e}")
    
    
    def _generate_cpp_build_files(self, src_dir, created_files):
        pkg_name = "cpp_blueprints_pkg"
        
        cmake = f"""cmake_minimum_required(VERSION 3.8)
project({pkg_name})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(example_interfaces REQUIRED)

include_directories(include)

# === AUTO DISCOVERY ===
# ИСПРАВЛЕНИЕ: Ищем просто в "cpp/", так как мы уже внутри src
file(GLOB_RECURSE CPP_SOURCES "cpp/*.cpp")

foreach(source_file ${{CPP_SOURCES}})
  get_filename_component(exec_name ${{source_file}} NAME_WE)
  
  add_executable(${{exec_name}} ${{source_file}})
  ament_target_dependencies(${{exec_name}} 
    rclcpp rclcpp_action std_msgs geometry_msgs sensor_msgs example_interfaces
  )
  install(TARGETS ${{exec_name}} DESTINATION lib/${{PROJECT_NAME}})
endforeach()

# ИСПРАВЛЕНИЕ: Инсталлируем папку "launch" (без src/), так как она рядом
install(DIRECTORY launch DESTINATION share/${{PROJECT_NAME}})

ament_package()
"""
        with open(os.path.join(src_dir, "CMakeLists.txt"), 'w', encoding='utf-8') as f:
            f.write(cmake)

        # package.xml остается тем же, но на всякий случай перезапишем
        pkg_xml = f"""<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>{pkg_name}</name>
  <version>0.0.0</version>
  <description>Auto-generated</description>
  <maintainer email="user@email.com">User</maintainer>
  <license>TODO</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>rclcpp</depend>
  <depend>rclcpp_action</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>example_interfaces</depend>
  <export><build_type>ament_cmake</build_type></export>
</package>"""
        with open(os.path.join(src_dir, "package.xml"), 'w', encoding='utf-8') as f:
            f.write(pkg_xml)

    # ==========================================
    #               ЗАГРУЗКА (RESTORED)
    # ==========================================
    def load_project(self, project_path):
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
            
            py_data, py_map = self._prepare_data_for_loading(project_state["graphs"].get("python", {}))
            cpp_data, cpp_map = self._prepare_data_for_loading(project_state["graphs"].get("cpp", {}))
            
            self.graph_py.deserialize_session(py_data)
            self.graph_cpp.deserialize_session(cpp_data)
            
            # Восстанавливаем код, с учетом новых папок
            self._restore_files_and_code(self.graph_py, py_map, src_dir)
            self._restore_files_and_code(self.graph_cpp, cpp_map, src_dir)
            
            self._restore_dynamic_ports(self.graph_py)
            self._restore_dynamic_ports(self.graph_cpp)
            
            return True
        except Exception as e:
            logging.error(f"Load Error: {e}")
            traceback.print_exc()
            return False

    def _prepare_data_for_loading(self, session_data):
        files_map = {}
        nodes = session_data.get('nodes', [])
        # Поддержка старого и нового формата (list vs dict)
        if isinstance(nodes, list):
            for n in nodes:
                c = n.get('custom', {})
                if 'source_file' in c: files_map[n.get('id')] = c['source_file']; del c['source_file']
        elif isinstance(nodes, dict):
            for nid, n in nodes.items():
                c = n.get('custom', {})
                if 'source_file' in c: files_map[nid] = c['source_file']; del c['source_file']
        return session_data, files_map

    def _restore_files_and_code(self, graph, files_map, src_dir):
        """
        Восстанавливает код в нодах. Ищет файлы в src/cpp или src/python.
        """
        for node in graph.all_nodes():
            if node.id in files_map:
                fname = files_map[node.id]
                
                # Пытаемся найти файл в подпапках
                possible_paths = [
                    os.path.join(src_dir, "cpp", fname),
                    os.path.join(src_dir, "python", fname),
                    os.path.join(src_dir, fname) # Для старых проектов
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
                def get_val(obj): return obj() if callable(obj) else obj
                def get_color(obj): return list(get_val(obj)) 
                for port in node.input_ports():
                    ports_config.append({'type': 'in', 'name': port.name(), 'color': get_color(port.color), 'multi': get_val(port.multi_connection)})
                for port in node.output_ports():
                    ports_config.append({'type': 'out', 'name': port.name(), 'color': get_color(port.color), 'multi': get_val(port.multi_connection)})
                if not node.has_property('saved_ports_config'): node.create_property('saved_ports_config', value=ports_config, widget_type=0)
                else: node.set_property('saved_ports_config', ports_config)
            except: pass