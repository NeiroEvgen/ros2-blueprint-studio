from PySide6 import QtCore
from NodeGraphQt import NodeGraph

# === ИМПОРТЫ ВСЕХ НОД ===
from nodes.library.py_basic import PyStringPubNode, PyStringSubNode
from nodes.library.cpp_basic import CppStringPubNode, CppStringSubNode
from nodes.library.py_logic import PyTimerNode, PyPrintNode
from nodes.library.cpp_logic import CppTimerNode, CppPrintNode
from nodes.group_logic import RosGroupNode
from nodes.monitor_node import MonitorNode

TYPE_COLORS = {
    "std_msgs/String": (255, 235, 59),
    "geometry_msgs/Twist": (255, 152, 0),
    "DEFAULT": (100, 100, 100)
}

ALL_NODE_CLASSES = [
    PyStringPubNode, PyStringSubNode, PyTimerNode, PyPrintNode,
    CppStringPubNode, CppStringSubNode, CppTimerNode, CppPrintNode,
    MonitorNode, RosGroupNode
]

class GraphDropFilter(QtCore.QObject):
    """Фильтр для перетаскивания нод из палитры на граф"""
    def __init__(self, graph, node_map_callback):
        super().__init__()
        self.graph = graph
        self.get_node_type = node_map_callback

    def eventFilter(self, watched, event):
        if event.type() == QtCore.QEvent.Drop:
            code = event.mimeData().text()
            node_type = self.get_node_type(code)
            if node_type:
                pos = event.position().toPoint()
                scene_pos = self.graph.viewer().mapToScene(pos)
                self.graph.create_node(node_type, pos=[scene_pos.x(), scene_pos.y()])
                event.acceptProposedAction()
                return True
        return super().eventFilter(watched, event)

def setup_graphs(ui_tabs, callback_node_resolver):
    """Создает и настраивает два графа (Python и C++)"""
    graph_py = NodeGraph()
    graph_cpp = NodeGraph()
    
    # Настройки
    for g in [graph_py, graph_cpp]:
        g.set_pipe_style(0)
        g.set_acyclic(False)
        g.register_nodes(ALL_NODE_CLASSES)
        
        # Drag & Drop support
        g.widget.setAcceptDrops(True)
        g.viewer().setAcceptDrops(True)
        g.viewer().viewport().setAcceptDrops(True)

    # Добавляем во вкладки
    ui_tabs.addTab(graph_py.widget, "Python Blueprints")
    ui_tabs.addTab(graph_cpp.widget, "C++ Blueprints")

    # Устанавливаем фильтры
    filter_py = GraphDropFilter(graph_py, callback_node_resolver)
    graph_py.viewer().viewport().installEventFilter(filter_py)
    
    filter_cpp = GraphDropFilter(graph_cpp, callback_node_resolver)
    graph_cpp.viewer().viewport().installEventFilter(filter_cpp)

    return graph_py, graph_cpp, filter_py, filter_cpp