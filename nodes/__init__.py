# Экспортируем шаблоны и цвета для использования извне
from .templates import TEMPLATES
from .base import MSG_COLORS

# Импортируем классы нод
from .library.py_logic import PyTimerNode, PyPrintNode
from .library.py_basic import PyStringPubNode, PyStringSubNode
from .library.cpp_basic import CppStringPubNode, CppStringSubNode
from .library.cpp_logic import CppTimerNode, CppPrintNode

# Собираем все ноды в один список для регистрации в NodeGraph
REGISTERED_NODES = [
    PyStringPubNode, PyStringSubNode,
    CppStringPubNode, CppStringSubNode,
    PyTimerNode, CppTimerNode, PyPrintNode, CppPrintNode
]