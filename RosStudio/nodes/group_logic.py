import uuid
from NodeGraphQt import BaseNode

# === 1. КЛАСС ГРУППЫ (Proxy Node) ===
class RosGroupNode(BaseNode):
    __identifier__ = 'ros.nodes'
    NODE_NAME = 'RosGroup'
    
    def __init__(self):
        super(RosGroupNode, self).__init__()
        self.set_color(40, 40, 40)
        self.add_text_input('node_name', 'Group Name', text='MySubGraph')
        
        # ID для связи при сохранении/загрузке
        self.create_property('group_uid', '') 
        
        # Список ID внутренних нод (обновляется при загрузке)
        self.create_property('internal_nodes', [])
        
        # Карта связей для компилятора: { 'Group_Out_Port': 'Group_In_Port' }
        self.create_property('port_links', {})
        
        self.create_property('is_expanded', False)

    def get_template_data(self):
        return None 

# === 2. СОЗДАНИЕ ГРУППЫ ===
def create_group_from_selection(graph, name="SubGraph", color=(50, 50, 50)):
    selected_nodes = graph.selected_nodes()
    if not selected_nodes:
        return None, "Select nodes first!"

    # 1. Центр и создание
    xs = [n.pos()[0] for n in selected_nodes]
    ys = [n.pos()[1] for n in selected_nodes]
    cx = sum(xs) / len(selected_nodes)
    cy = sum(ys) / len(selected_nodes)

    # Используем правильное имя класса (с суффиксом Node или без, как зарегистрировано)
    # Обычно это 'ros.nodes.RosGroupNode' если библиотека добавила суффикс
    node_type = 'ros.nodes.RosGroupNode' 
    if node_type not in graph.registered_nodes():
        node_type = 'ros.nodes.RosGroup'

    try:
        group_node = graph.create_node(node_type, pos=[cx, cy])
    except Exception as e:
        return None, f"Error creating group node ({node_type}): {e}"

    group_node.set_name(name)
    group_node.set_property('node_name', name)
    group_node.set_color(color[0], color[1], color[2])
    
    # === ГЕНЕРАЦИЯ УНИКАЛЬНОГО ID ГРУППЫ ===
    # Это свяжет группу и детей даже после перезагрузки/копирования
    gid = str(uuid.uuid4())
    group_node.set_property('group_uid', gid)
    
    internal_ids = []
    for n in selected_nodes:
        internal_ids.append(n.id)
        # Метим детей этим же ID
        if not n.has_property('group_uid'):
            n.create_property('group_uid', gid)
        else:
            n.set_property('group_uid', gid)

    group_node.set_property('internal_nodes', internal_ids)

    # 3. ПЕРЕПОДКЛЮЧЕНИЕ (REWIRING)
    port_links = {} 

    # --- ВХОДЯЩИЕ (External -> Internal) ---
    for node in selected_nodes:
        for in_port in node.input_ports():
            # Список соединений
            connected = list(in_port.connected_ports())
            for source_port in connected:
                source_node = source_port.node()
                # Если источник снаружи
                if source_node not in selected_nodes:
                    # Имена портов
                    pname_in = f"In_{source_port.name()}_{node.name()}"
                    pname_out = f"To_{node.name()}_{in_port.name()}"
                    
                    # Создаем порты на группе
                    if not group_node.get_input(pname_in): group_node.add_input(pname_in)
                    if not group_node.get_output(pname_out): group_node.add_output(pname_out)
                    
                    g_in = group_node.get_input(pname_in)
                    g_out = group_node.get_output(pname_out)
                    
                    # Записываем связь
                    port_links[g_out.name()] = g_in.name()
                    
                    # Переподключаем
                    in_port.disconnect_from(source_port)
                    source_port.connect_to(g_in)
                    g_out.connect_to(in_port)

    # --- ИСХОДЯЩИЕ (Internal -> External) ---
    for node in selected_nodes:
        for out_port in node.output_ports():
            connected = list(out_port.connected_ports())
            for target_port in connected:
                target_node = target_port.node()
                if target_node not in selected_nodes:
                    pname_in = f"From_{node.name()}_{out_port.name()}"
                    pname_out = f"Out_{target_port.name()}_{target_node.name()}"
                    
                    if not group_node.get_input(pname_in): group_node.add_input(pname_in)
                    if not group_node.get_output(pname_out): group_node.add_output(pname_out)
                    
                    g_in = group_node.get_input(pname_in)
                    g_out = group_node.get_output(pname_out)
                    
                    port_links[g_out.name()] = g_in.name()
                    
                    target_port.disconnect_from(out_port)
                    out_port.connect_to(g_in)
                    g_out.connect_to(target_port)

    group_node.set_property('port_links', port_links)

    # 4. Скрываем внутренние ноды
    for node in selected_nodes:
        if hasattr(node, 'view'):
            node.view.setVisible(False)

    graph.clear_selection()
    group_node.set_selected(True)
    
    return group_node, "OK"

# === 3. ВОССТАНОВЛЕНИЕ ПОСЛЕ ЗАГРУЗКИ/СПАВНА ===
def fix_group_after_load(graph, loaded_nodes):
    """
    Вызывается после вставки (Paste) или загрузки шаблона.
    Ищет группы и их детей по 'group_uid', обновляет ID и скрывает детей.
    """
    groups = [n for n in loaded_nodes if n.type_ in ['ros.nodes.RosGroup', 'ros.nodes.RosGroupNode']]
    if not groups: return

    # Пробегаем по всем группам
    for grp in groups:
        gid = grp.get_property('group_uid')
        if not gid: continue
        
        # Ищем детей среди загруженных нод, у которых такой же group_uid
        children = []
        children_ids = []
        
        for n in loaded_nodes:
            if n == grp: continue
            if n.has_property('group_uid') and n.get_property('group_uid') == gid:
                children.append(n)
                children_ids.append(n.id)
        
        # Обновляем список ID в группе (теперь они знают новые ID своих детей)
        grp.set_property('internal_nodes', children_ids)
        
        # Скрываем детей (чтобы не было "потроха наружу")
        grp.set_property('is_expanded', False)
        for child in children:
            if hasattr(child, 'view'):
                child.view.setVisible(False)
        
        # Обновляем цвет
        grp.set_color(40, 40, 40)

# === 4. ПЕРЕКЛЮЧЕНИЕ ВИДИМОСТИ ===
def toggle_group_visibility(graph, group_node):
    internal_ids = group_node.get_property('internal_nodes')
    if not internal_ids: return
    
    new_state = not group_node.get_property('is_expanded')
    group_node.set_property('is_expanded', new_state)
    
    # Ищем ноды по ID
    all_nodes = {n.id: n for n in graph.all_nodes()}
    
    for nid in internal_ids:
        if nid in all_nodes:
            node = all_nodes[nid]
            if hasattr(node, 'view'):
                node.view.setVisible(new_state)
            
    group_node.set_color(80, 80, 80) if new_state else group_node.set_color(40, 40, 40)