import logging
from nodes.custom_nodes import TEMPLATES

class GraphCompiler:
    def __init__(self, graph):
        self.graph = graph

    def compile(self):
        compiled_scripts = []
        all_nodes = self.graph.all_nodes()

        print("\n--- COMPILER START ---")

        for node in all_nodes:
            # 1. Пропускаем мусор
            if node.type_ in ['nodeGraphQt.nodes.BackdropNode', 'nodes.utility.MonitorNode', 'ros.nodes.RosGroupNode']: 
                continue
            
            if not hasattr(node, 'get_template_data'): continue
            data = node.get_template_data()
            
            # Имя ноды и текущий топик из текстового поля
            node_name = data.get('name', 'unknown')
            topic_name = data.get('topic', '/default_topic')
            
            print(f"Processing Node: {node_name} (Default Topic: {topic_name})")

            # === 2. ЛОГИКА АВТО-ПОДКЛЮЧЕНИЯ (Smart Link) ===
            # Если есть входящее соединение, МЫ ОБЯЗАНЫ ВЗЯТЬ ТОПИК ОТТУДА
            
            connected_source_node = None
            
            # Перебираем все входы ноды
            for port in node.input_ports():
                if port.connected_ports():
                    # Нашли подключение!
                    source_port = port.connected_ports()[0]
                    connected_source_node = source_port.node()
                    print(f"  -> Found connection from: {connected_source_node.name()}")
                    break # Берем первый найденный вход
            
            if connected_source_node:
                # Пытаемся украсть имя топика у источника
                if hasattr(connected_source_node, 'get_property'):
                    src_topic = connected_source_node.get_property('topic_name')
                    if src_topic:
                        print(f"  -> OVERRIDE: Changing topic '{topic_name}' -> '{src_topic}'")
                        topic_name = src_topic
                    else:
                        print("  -> WARNING: Source node has no 'topic_name' property!")
            else:
                print("  -> No input connections. Using default topic.")

            # === 3. ПОДГОТОВКА КОДА ===
            # Сначала ищем код внутри самой ноды (если юзер редактировал)
            raw_code = data.get('code')
            
            # Если пусто, берем шаблон по ключу
            if not raw_code:
                key = data.get('template_key')
                if key and key in TEMPLATES:
                    raw_code = TEMPLATES[key]
            
            if not raw_code:
                print(f"  -> SKIP: No code found for {node_name}")
                continue

            # === 4. ПОДСТАНОВКА ===
            lang = data.get('language', 'python')
            class_name = "".join(x.title() for x in node_name.split('_'))

            try:
                # 1. Заменяем известные переменные
                code = raw_code.replace("{class_name}", class_name)\
                               .replace("{node_name}", node_name)\
                               .replace("{topic_name}", topic_name)
                
                # 2. ЧИСТИЛЬЩИК: Если вдруг остались старые плейсхолдеры, убираем их,
                # чтобы Python не падал с NameError.
                code = code.replace("{subscribe_code}", "pass # (removed placeholder)")
                code = code.replace("{publish_code}", "pass # (removed placeholder)")
                
            except Exception as e:
                code = f"// Error compiling: {e}"

            ext = 'cpp' if lang == 'cpp' else 'py'
            compiled_scripts.append({'language': lang, 'filename': f"{node_name}.{ext}", 'code': code})

        print("--- COMPILER END ---\n")
        return compiled_scripts