import math
import networkx as nx

# =========================================================
# CONFIG
# =========================================================
SAFE_SEPARATION = 1.5
TIME_STEP = 0.5
SIM_DURATION = 20.0
PREDICTION_HORIZON = 5.0
BATTERY_DRAIN_PER_STEP = 0.2

LOW_PRIORITY = 1
NORMAL_PRIORITY = 2
HIGH_PRIORITY = 3

PRIORITY_NAMES = {
    LOW_PRIORITY: "low",
    NORMAL_PRIORITY: "normal",
    HIGH_PRIORITY: "high",
}

ALTITUDE_LAYERS = {
    LOW_PRIORITY: 1,
    NORMAL_PRIORITY: 2,
    HIGH_PRIORITY: 3,
}


# =========================================================
# AIRSPACE
# =========================================================
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
    ]

    for u, v in edges:
        x1, y1 = nodes[u]
        x2, y2 = nodes[v]
        dist = math.dist((x1, y1), (x2, y2))
        G.add_edge(u, v, weight=dist)

    return G


# =========================================================
# DRONE
# =========================================================
class Drone:
    def __init__(self, drone_id, start_node, goal_node, priority, speed=0.2, battery=100.0):
        self.id = drone_id
        self.start_node = start_node
        self.goal_node = goal_node
        self.priority = priority
        self.speed = speed
        self.battery = battery

        self.altitude = ALTITUDE_LAYERS[priority]

        self.route = []
        self.route_index = 0
        self.progress = 0.0

        self.x = 0.0
        self.y = 0.0

        self.completed = False
        self.hold = False
        self.slowed = False

        self.history = []
        self.event_log = []

    def set_position(self, x, y):
        self.x = x
        self.y = y

    def current_position_3d(self):
        return (self.x, self.y, self.altitude)

    def log_event(self, message):
        self.event_log.append(message)

    def reset_flags(self):
        self.hold = False
        self.slowed = False


# =========================================================
# SCENARIOS
# =========================================================
def scenario_no_conflict():
    return [
        Drone("D1", "A", "G", HIGH_PRIORITY, speed=0.20),
        Drone("D2", "A", "H", NORMAL_PRIORITY, speed=0.18),
        Drone("D3", "B", "H", LOW_PRIORITY, speed=0.16),
    ]


def scenario_current_conflict():
    d1 = Drone("D1", "A", "G", HIGH_PRIORITY, speed=0.20)
    d2 = Drone("D2", "B", "A", NORMAL_PRIORITY, speed=0.20)
    d3 = Drone("D3", "D", "H", LOW_PRIORITY, speed=0.15)
    return [d1, d2, d3]


def scenario_future_conflict():
    return [
        Drone("D1", "A", "G", HIGH_PRIORITY, speed=0.25),
        Drone("D2", "F", "C", NORMAL_PRIORITY, speed=0.22),
        Drone("D3", "B", "H", LOW_PRIORITY, speed=0.18),
    ]


def scenario_destination_completion():
    return [
        Drone("D1", "A", "B", HIGH_PRIORITY, speed=0.25),
        Drone("D2", "H", "A", NORMAL_PRIORITY, speed=0.18),
        Drone("D3", "B", "G", LOW_PRIORITY, speed=0.16),
    ]


SCENARIOS = {
    "no_conflict": scenario_no_conflict,
    "current_conflict": scenario_current_conflict,
    "future_conflict": scenario_future_conflict,
    "destination_completion": scenario_destination_completion,
}


# =========================================================
# CONFLICT DETECTION
# =========================================================
def distance_3d(p1, p2):
    return math.sqrt(
        (p1[0] - p2[0]) ** 2 +
        (p1[1] - p2[1]) ** 2 +
        (p1[2] - p2[2]) ** 2
    )


def current_conflict(drone1, drone2, threshold=SAFE_SEPARATION):
    if drone1.completed or drone2.completed:
        return False, None

    d = distance_3d(drone1.current_position_3d(), drone2.current_position_3d())
    return d < threshold, d


def future_position(drone, t, graph):
    if drone.completed or not drone.route or drone.route_index >= len(drone.route) - 1:
        return drone.current_position_3d()

    current_node = drone.route[drone.route_index]
    next_node = drone.route[drone.route_index + 1]

    x1, y1 = graph.nodes[current_node]["pos"]
    x2, y2 = graph.nodes[next_node]["pos"]

    dx = x2 - x1
    dy = y2 - y1
    edge_len = math.sqrt(dx**2 + dy**2)

    if edge_len == 0:
        return drone.current_position_3d()

    ux = dx / edge_len
    uy = dy / edge_len

    travel = drone.progress + drone.speed * t
    if travel > edge_len:
        travel = edge_len

    fx = x1 + ux * travel
    fy = y1 + uy * travel

    return (fx, fy, drone.altitude)


def predict_conflict(drone1, drone2, graph, horizon=PREDICTION_HORIZON, dt=TIME_STEP, threshold=SAFE_SEPARATION):
    if drone1.completed or drone2.completed:
        return False, None, None

    t = 0.0
    min_distance = float("inf")
    first_conflict_time = None

    while t <= horizon:
        p1 = future_position(drone1, t, graph)
        p2 = future_position(drone2, t, graph)
        d = distance_3d(p1, p2)

        if d < min_distance:
            min_distance = d

        if d < threshold:
            first_conflict_time = t
            return True, first_conflict_time, min_distance

        t += dt

    return False, None, min_distance


def scan_conflicts(drones, graph):
    events = []

    for i in range(len(drones)):
        for j in range(i + 1, len(drones)):
            d1 = drones[i]
            d2