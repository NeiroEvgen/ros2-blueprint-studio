# Экспортируем шаблоны и цвета для использования извне
from .templates import TEMPLATES
from .base import MSG_COLORS

# Импортируем классы нод
from nodes.library.py_basic import PyStringPubNode, PyStringSubNode
from nodes.library.cpp_basic import CppStringPubNode, CppStringSubNode, CppCustomNode
from nodes.library.py_logic import PyTimerNode, PyPrintNode

from nodes.library.cpp_logic import CppTimerNode, CppPrintNode 

from nodes.group_logic import RosGroupNode, SubGraphInputNode, SubGraphOutputNode
from nodes.monitor_node import MonitorNode
from .library.meta import NoteNode  



REGISTERED_NODES= [
    PyStringPubNode, PyStringSubNode, PyTimerNode, PyPrintNode, 
    CppStringPubNode, CppStringSubNode, CppTimerNode, CppPrintNode, CppCustomNode,
    MonitorNode, RosGroupNode, SubGraphInputNode, SubGraphOutputNode, NoteNode
]

ALL_NODE_CLASSES = REGISTERED_NODES