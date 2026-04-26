import math
import networkx as nx


class Airspace:
    """Immutable base graph + mutable closure state.

    The base graph (_base) is never mutated. All routing consumers call
    get_routing_graph() which returns a fresh copy with closed nodes removed.
    This eliminates the ghost-node pollution that resulted from adding R_{id}
    temporary nodes directly to the shared graph.
    """

    def __init__(self, graph: nx.Graph):
        self._base = graph
        self.closed_nodes: set = set()

    def close_node(self, node: str):
        self.closed_nodes.add(node)

    def open_node(self, node: str):
        self.closed_nodes.discard(node)

    def get_routing_graph(self, exclude_virtual: bool = False) -> nx.Graph:
        """Return a routing copy: closed nodes removed, all edge weights preserved.

        exclude_virtual=True also strips C_UP_1 and C_UP_2 so that normal
        route initialisation does not accidentally route through bypass arcs.
        """
        g = self._base.copy()
        for node in self.closed_nodes:
            if g.has_node(node):
                g.remove_node(node)
        if exclude_virtual:
            for vn in ("C_UP_1", "C_UP_2"):
                if g.has_node(vn):
                    g.remove_node(vn)
        return g

    def node_pos(self, node: str):
        return self._base.nodes[node]["pos"]

    def has_node(self, node: str) -> bool:
        return self._base.has_node(node)


def build_airspace() -> Airspace:
    G = nx.Graph()

    nodes = {
        "A": (0, 0),
        "B": (2, 2),
        "C": (5, 2),
        "G": (8, 2),
        "E": (2, -2),
        "F": (5, -2),
        "D": (6, 0),
        "H": (8, -2),

        # Virtual bypass nodes used only when C is closed as an ascent zone.
        "C_UP_1": (4.25, 3.15),
        "C_UP_2": (5.75, 3.15),
    }

    for node_id, pos in nodes.items():
        G.add_node(node_id, pos=pos)

    edges = [
        ("A", "B"),
        ("A", "E"),
        ("B", "C"),
        ("B", "E"),
        ("C", "G"),
        ("C", "D"),
        ("D", "F"),
        ("D", "G"),
        ("D", "H"),
        ("E", "F"),
        ("F", "H"),

        # Bypass arc — only reachable after C is closed.
        ("B", "C_UP_1"),
        ("C_UP_1", "C_UP_2"),
        ("C_UP_2", "G"),
    ]

    for u, v in edges:
        x1, y1 = nodes[u]
        x2, y2 = nodes[v]
        G.add_edge(u, v, weight=math.dist((x1, y1), (x2, y2)))

    return Airspace(G)
