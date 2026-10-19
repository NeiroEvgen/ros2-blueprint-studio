"""
The project's set of canvases.

All canvas access goes through GraphSet, not through the tab index: in two-tab mode
it holds two graphs, in shared-canvas mode (see core/graph_ir.py) one and the same
graph under both names. The rest of the code does not depend on the mode.

No Qt: works with any graph objects (stubs in tests).
"""


class GraphSet:
    def __init__(self, graph_py, graph_cpp):
        self.py = graph_py
        self.cpp = graph_cpp

    @property
    def is_single(self):
        """True when both "language" canvases are the same graph."""
        return self.py is self.cpp

    def all(self):
        """All unique graphs (one in single-canvas mode)."""
        return [self.py] if self.is_single else [self.py, self.cpp]

    def for_language(self, language):
        return self.cpp if language == "cpp" else self.py

    def current(self, tab_index):
        """Graph of the active tab (in single-canvas mode the tab does not matter)."""
        if self.is_single:
            return self.py
        return self.py if tab_index == 0 else self.cpp

    def find_node(self, node_id):
        """Node by id across all graphs, or None."""
        for g in self.all():
            for n in g.all_nodes():
                if n.id == node_id:
                    return n
        return None
