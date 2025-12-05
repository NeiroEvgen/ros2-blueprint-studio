import logging

class GraphCompiler:
    def __init__(self, graph):
        self.graph = graph

    def compile(self):
        compiled_scripts = []
        
        # 1. Рекурсивно собираем все ноды (включая те, что внутри групп)
        all_nodes = self._collect_nodes_recursive(self.graph)
        
        logging.info(f"Compile: Found {len(all_nodes)} real nodes inside/outside groups")

        for node in all_nodes:
            # Пропускаем служебные
            if node.type_ == 'nodes.utility.MonitorNode': continue
            # Пропускаем сами групповые контейнеры (они не генерируют код, только содержат)
            if node.type_ == 'ros.nodes.SubGraph': continue 
            
            if not hasattr(node, 'get_template_data'): continue

            data = node.get_template_data()
            lang = data.get('language', 'python')
            node_name = data.get('name', 'unknown')
            class_name = node_name.capitalize().replace('_', '') + 'Node'
            
            # --- ЛОГИКА СОЕДИНЕНИЙ ---
            
            # 1. Output / Publisher (Кто слушает нас?)
            # Для ROS 2 это просто: мы пишем в топик /<node_name>_topic
            # Топик всегда уникален для ноды-писателя
            publish_code = ""
            publish_call = ""
            
            out_port = node.get_output('out')
            if out_port:
                topic_name = f"/{node_name}_topic"
                
                if lang == 'python':
                    publish_code = f"self.publisher_ = self.create_publisher(String, '{topic_name}', 10)"
                    publish_call = "self.publisher_.publish(msg if 'msg' in locals() or 'msg' in globals() else out_msg)"
                elif lang == 'cpp':
                    publish_code = f"publisher_ = this->create_publisher<std_msgs::msg::String>(\"{topic_name}\", 10);"
                    publish_call = "if(publisher_) publisher_->publish(message);"

            # 2. Input / Subscriber (Кого слушаем мы?)
            subscribe_code = ""
            
            in_port = node.get_input('in')
            if in_port and in_port.connected_ports():
                # ТУТ МАГИЯ: Нужно найти реальную ноду-источник, даже через границы групп
                source_node = self._resolve_source_node(in_port)
                
                if source_node and hasattr(source_node, 'get_property'):
                    source_node_name = source_node.get_property('node_name')
                    topic_to_listen = f"/{source_node_name}_topic"
                    
                    if lang == 'python':
                        subscribe_code = f"self.subscription = self.create_subscription(String, '{topic_to_listen}', self.listener_callback, 10)"
                    elif lang == 'cpp':
                        subscribe_code = f"subscription_ = this->create_subscription<std_msgs::msg::String>(\"{topic_to_listen}\", 10, std::bind(&{class_name}::listener_callback, this, _1));"
                else:
                    logging.warning(f"Node {node_name}: connected to something that is not a RosNode")

            # 3. Генерация текста
            template = data['template']
            try:
                code = template.replace("{class_name}", class_name)
                code = code.replace("{node_name}", node_name)
                code = code.replace("{msg_data}", data.get('msg', ''))
                code = code.replace("{publish_code}", publish_code)
                code = code.replace("{publish_call}", publish_call)
                code = code.replace("{subscribe_code}", subscribe_code)
            except Exception as e:
                code = "# ERROR GENERATING CODE"

            ext = 'py' if lang == 'python' else 'cpp'
            filename = f"{node_name}.{ext}"
            
            compiled_scripts.append({
                'language': lang,
                'filename': filename,
                'code': code
            })

        return compiled_scripts

    def _collect_nodes_recursive(self, graph_or_group):
        """Рекурсивно собирает все ноды из графа и подграфов"""
        nodes = []
        for node in graph_or_group.all_nodes():
            if node.type_ == 'ros.nodes.SubGraph':
                # Это группа! Ныряем внутрь.
                sub_graph = node.get_sub_graph()
                nodes.extend(self._collect_nodes_recursive(sub_graph))
            else:
                nodes.append(node)
        return nodes

    def _resolve_source_node(self, target_input_port):
        """
        Ищет реальную ноду-источник, проходя сквозь PortNode групп.
        """
        connected_ports = target_input_port.connected_ports()
        if not connected_ports:
            return None
        
        source_port = connected_ports[0]
        source_node = source_port.node()
        
        # Если мы подключены к PortNode (вход внутрь группы)
        # PortNode - это "переходник" внутри группы.
        if source_node.type_ in ['graph.nodes.PortInputNode', 'graph.nodes.PortOutputNode']:
            # Нам нужно найти, к чему подключен этот переходник СНАРУЖИ
            # Но NodeGraphQt хранит связи хитро.
            # Логика: 
            # 1. Мы внутри группы слушаем PortInputNode.
            # 2. Этот PortInputNode соответствует порту на самой GroupNode снаружи.
            
            # К сожалению, API NodeGraphQt для обратного поиска порта группы не тривиально.
            # Но для ROS 2 топиков нам достаточно знать ИМЯ ТОПИКА.
            # Если мы внутри группы, и провод идет от "Input", значит данные приходят снаружи.
            # Попробуем найти внешний порт.
            
            # HACK: Для простоты в этой версии, если мы наткнулись на границу группы,
            # мы можем не найти источник.
            # FIX: Правильный обход требует доступа к родительскому графу, что сложно.
            # В данном IDE мы упростим: ноды внутри группы видят топики глобально по именам.
            # Но для авто-подключения (генерации имени топика) нам нужен источник.
            pass

        return source_node