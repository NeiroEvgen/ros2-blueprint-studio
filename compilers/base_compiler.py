import re

class BaseCompiler:
    def __init__(self, graph):
        self.graph = graph
        self.connection_map = {}

    def _sanitize_name(self, name):
        """Превращает 'My Node' в 'My_Node'"""
        clean = re.sub(r'[^a-zA-Z0-9_/]', '_', name)
        return re.sub(r'_+', '_', clean)

    def build_connection_map(self):
        """Строит карту: (node_id, port_name) -> global_topic_name"""
        self.connection_map = {}
        for node in self.graph.all_nodes():
            if node.type_ in ['nodeGraphQt.nodes.BackdropNode', 'nodes.utility.MonitorNode']:
                continue
            
            node_name = self._sanitize_name(node.name())
            
            for out_port in node.output_ports():
                port_name = self._sanitize_name(out_port.name())
                
                # Разделяем Поток Данных и Поток Управления
                if "Exec" in port_name:
                    topic = f"/trig/{node_name}_{port_name}"
                else:
                    topic = f"/{node_name}_{port_name}"
                
                self.connection_map[(node.id, out_port.name())] = topic
        
        return self.connection_map