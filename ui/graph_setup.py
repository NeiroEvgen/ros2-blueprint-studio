from PySide6 import QtCore, QtGui
from NodeGraphQt import NodeGraph

from nodes import ALL_NODE_CLASSES
TYPE_COLORS = {
    "std_msgs/String": (255, 235, 59),
    "geometry_msgs/Twist": (255, 152, 0),
    "DEFAULT": (100, 100, 100)
}


def _node_from_item(graph, item):
    """Находит BaseNode по QGraphicsItem (поднимаясь по родителям)."""
    while item is not None:
        for n in graph.all_nodes():
            if n.view is item:
                return n
        item = item.parentItem()
    return None


class GraphDropFilter(QtCore.QObject):
    """Drop из палитры на граф + Alt+drag ноды с графа в палитру."""
    def __init__(self, graph, node_map_callback):
        super().__init__()
        self.graph = graph
        self.get_node_type = node_map_callback

    def eventFilter(self, watched, event):
        # 1) Drop из палитры -> создать ноду
        if event.type() == QtCore.QEvent.Drop:
            code = event.mimeData().text()
            node_type = self.get_node_type(code)
            if node_type:
                pos = event.position().toPoint()
                scene_pos = self.graph.viewer().mapToScene(pos)
                self.graph.create_node(node_type, pos=[scene_pos.x(), scene_pos.y()])
                event.acceptProposedAction()
                return True

        # 2) Alt+клик по ноде -> начать drag в палитру
        if (event.type() == QtCore.QEvent.MouseButtonPress
                and event.modifiers() & QtCore.Qt.AltModifier):
            viewer = self.graph.viewer()
            pos = event.position().toPoint()
            item = viewer.itemAt(pos)
            node = _node_from_item(self.graph, item)
            if node is not None:
                drag = QtGui.QDrag(viewer)
                mime = QtCore.QMimeData()
                mime.setData("application/x-bp-node-id", node.id.encode())
                drag.setMimeData(mime)
                drag.exec(QtCore.Qt.CopyAction)
                return True  # съедаем событие, чтобы нода не поехала по холсту

        return super().eventFilter(watched, event)
    
def setup_graphs(ui_tabs, callback_node_resolver, delete_handler=None):
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

    # Удаление через QShortcut (работает независимо от фокуса viewport)
    if delete_handler:
        from PySide6 import QtGui
        for g in (graph_py, graph_cpp):
            # Delete = распустить / обычное удаление
            sc_del = QtGui.QShortcut(QtGui.QKeySequence(QtCore.Qt.Key_Delete), g.widget)
            sc_del.setContext(QtCore.Qt.WidgetWithChildrenShortcut)
            sc_del.activated.connect(lambda gr=g: delete_handler(gr, False))
            # Ctrl+Delete = удалить с содержимым
            sc_del_all = QtGui.QShortcut(
                QtGui.QKeySequence("Ctrl+Delete"), g.widget)
            sc_del_all.setContext(QtCore.Qt.WidgetWithChildrenShortcut)
            sc_del_all.activated.connect(lambda gr=g: delete_handler(gr, True))
            # держим ссылки, чтобы не собрал GC
            g._sc_del = sc_del
            g._sc_del_all = sc_del_all

    return graph_py, graph_cpp, filter_py, filter_cpp