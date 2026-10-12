"""
Ноды графа → файлы исходников.

  flatten_nodes()          — плоский список нод с namespace и деплой-группой (раскрывает группы)
  process_and_save_files() — пишет <code_dir>/<node>.(cpp|py); диск — источник истины для кода

Без Qt, без docker. Граф сюда приходит уже сериализованным (dict).
"""
import logging
import os
import re
from core.template_render import render_pristine_template

from core.generators.cpp_generator import CppGenerator
from core.generators.python_generator import PythonGenerator


def _normalize_nodes(session_data):
    nodes = session_data.get('nodes', [])
    if isinstance(nodes, dict):
        # ключ словаря — ID ноды; вписываем его внутрь
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
            continue                      # ребёнок группы: его выведет сама группа
        node_type = _node_type(node)
        if node_type in _SKIP_TYPES or 'meta' in node_type:
            continue
        if nid in seen:
            continue                      # нода уже выведена (глубочайшей группой)
        seen.add(nid)

        if node_type in _GROUP_TYPES:
            custom = node.get('custom', {})
            group_name = custom.get('node_name', node.get('name', 'group'))
            safe_group_name = re.sub(r'[^a-zA-Z0-9_]', '_', group_name)
            new_ns = f"{current_ns}/{safe_group_name}".replace("//", "/")

            internal_ids = [i for i in custom.get('internal_nodes', []) if i in nodes_dict]
            # Если группа перечисляет и вложенную группу, и её детей — ребёнок
            # принадлежит самой глубокой группе, а не выводится дважды.
            nested = set()
            for i in internal_ids:
                inner = nodes_dict[i]
                if _node_type(inner) in _GROUP_TYPES:
                    nested.update(inner.get('custom', {}).get('internal_nodes', []))
            children = [nodes_dict[i] for i in internal_ids if i not in nested]
            logging.debug(f"[FLATTEN] группа '{group_name}': детей={len(children)}, ns='{new_ns}'")

            # имя группы становится деплой-группой для её детей
            group_container = custom.get('container_group') or safe_group_name.lower()
            flat.extend(_walk(children, nodes_dict, new_ns, group_container, seen))
        else:
            own_group = node.get('custom', {}).get('container_group') or inherited_group
            flat.append({'node': node, 'namespace': current_ns, 'container_group': own_group})
    return flat


def flatten_nodes(session_data, current_ns="", inherited_group="main"):
    """
    Собирает все ноды сессии, раскрывая группы (в том числе вложенные).
    Возвращает список: [{'node':..., 'namespace':..., 'container_group':...}]
    Каждая нода выводится ровно один раз — в самой глубокой своей группе.
    """
    nodes = _normalize_nodes(session_data)
    nodes_dict = {n['id']: n for n in nodes}
    return _walk(nodes, nodes_dict, current_ns, inherited_group, set())


def process_and_save_files(session_data, target_dir, project_type):
    """Сессия графа → файлы в target_dir (раскрывает группы и вызывает process_flat_nodes)."""
    try:
        flat_nodes = flatten_nodes(session_data)
        logging.debug(f"[PROC DEBUG] flat_nodes получено: {len(flat_nodes)}")
    except Exception as e:
        logging.debug(f"[PROC DEBUG] flatten_nodes УПАЛ: {e}")
        traceback.print_exc()
        flat_nodes = []
    return process_flat_nodes(flat_nodes, target_dir, project_type)


def process_flat_nodes(flat_nodes, target_dir, project_type):
    """Плоский список нод ([{'node','namespace',...}]) → файлы <target_dir>/<node>.(cpp|py)."""
    is_cpp_mode = (project_type == "cpp")
    forced_extension = ".cpp" if is_cpp_mode else ".py"

    # Инициализируем генератор
    project_name = os.path.basename(os.path.dirname(target_dir))
    generator = CppGenerator(project_name) if is_cpp_mode else PythonGenerator(project_name)

    created = []
    used_names = set()
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
        if filename in used_names:
            n = 2
            while f"{base_name}_{n}{forced_extension}" in used_names:
                n += 1
            logging.warning(f"Имя файла '{filename}' уже занято другой нодой — '{visual_name}' "
                            f"получит '{base_name}_{n}{forced_extension}'")
            filename = f"{base_name}_{n}{forced_extension}"
        used_names.add(filename)
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

        existing_code = render_pristine_template(
            existing_code, custom, node_data['name'], is_cpp_mode)
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

