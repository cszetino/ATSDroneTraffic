import math
import networkx as nx


def build_airspace():
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

        # Case-specific virtual bypass nodes. These are hidden unless the
        # dynamic ascent-zone scenario is running.
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

        # Hidden wraparound path used only after C becomes an ascent zone.
        ("B", "C_UP_1"),
        ("C_UP_1", "C_UP_2"),
        ("C_UP_2", "G"),
    ]

    for u, v in edges:
        x1, y1 = nodes[u]
        x2, y2 = nodes[v]
        dist = math.dist((x1, y1), (x2, y2))
        G.add_edge(u, v, weight=dist)

    return G
