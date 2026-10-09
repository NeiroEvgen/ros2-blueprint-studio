"""
Ноды графа → файлы исходников.

  flatten_nodes()          — плоский список нод с namespace и деплой-группой (раскрывает группы)
  process_and_save_files() — пишет <code_dir>/<node>.(cpp|py); диск — источник истины для кода

Без Qt, без docker. Граф сюда приходит уже сериализованным (dict).
"""
import logging
import os
import re

from core.generators.cpp_generator import CppGenerator
from core.generators.python_generator import PythonGenerator


def flatten_nodes(session_data, current_ns="", inherited_group="main"):
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
        logging.debug(f"[FLAT DEBUG] нода type={node.get('type')}, parent_group_id={_pgid!r}")
        if current_ns == "" and _pgid:
            logging.debug(f"[FLAT DEBUG]   -> ПРОПУЩЕНА (parent_group_id={_pgid})")
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
            logging.debug(f"[FLATTEN DEBUG] группа '{group_name}': internal_ids={len(internal_ids)}, найдено нод={len(internal_nodes_data)}, new_ns='{new_ns}'")

            # имя группы становится деплой-группой для её детей
            group_container = node.get('custom', {}).get('container_group') or safe_group_name.lower()
            flat_list.extend(flatten_nodes(
                {'nodes': internal_nodes_data}, new_ns, group_container))
        else:
            own_group = node.get('custom', {}).get('container_group') or inherited_group
            flat_list.append({'node': node, 'namespace': current_ns,
                              'container_group': own_group})

    return flat_list




def process_and_save_files(session_data, target_dir, project_type):
    logging.debug("[PROC DEBUG] _process_and_save_files ВЫЗВАН!!!")
    is_cpp_mode = (project_type == "cpp")
    forced_extension = ".cpp" if is_cpp_mode else ".py"
    
    # Инициализируем генератор
    project_name = os.path.basename(os.path.dirname(target_dir))
    generator = CppGenerator(project_name) if is_cpp_mode else PythonGenerator(project_name)

    # Собираем все ноды в плоский список, учитывая неймспейсы сабграфов
    try:
        flat_nodes = flatten_nodes(session_data)
        logging.debug(f"[PROC DEBUG] flat_nodes получено: {len(flat_nodes)}")
    except Exception as e:
        import traceback
        logging.debug(f"[PROC DEBUG] _flatten_nodes УПАЛ: {e}")
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
        logging.debug(f"[GROUP DEBUG] нода '{visual_name}' namespace='{namespace}' -> group='{container_group}'")
    
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

