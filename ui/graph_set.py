"""
Набор холстов проекта.

Раньше код везде знал, что холстов ровно два (graph_py, graph_cpp), и выбирал нужный по
номеру вкладки. Чтобы перейти к одному общему холсту (см. core/graph_ir.py), всё
обращение к холстам идёт через GraphSet: сегодня он хранит два графа, завтра — один и
тот же граф под обоими именами. Остальной код при этом не меняется.

Без Qt: работает с любыми объектами графов (в тестах — заглушки).
"""


class GraphSet:
    def __init__(self, graph_py, graph_cpp):
        self.py = graph_py
        self.cpp = graph_cpp

    @property
    def is_single(self):
        """True, когда оба «языковых» холста — один и тот же граф."""
        return self.py is self.cpp

    def all(self):
        """Все уникальные графы (в одно-холстовом режиме — один)."""
        return [self.py] if self.is_single else [self.py, self.cpp]

    def for_language(self, language):
        return self.cpp if language == "cpp" else self.py

    def current(self, tab_index):
        """Граф активной вкладки (в одно-холстовом режиме вкладка не важна)."""
        if self.is_single:
            return self.py
        return self.py if tab_index == 0 else self.cpp

    def find_node(self, node_id):
        """Нода по id среди всех графов или None."""
        for g in self.all():
            for n in g.all_nodes():
                if n.id == node_id:
                    return n
        return None
