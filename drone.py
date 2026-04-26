from config import ALTITUDE_LAYERS


class Drone:
    def __init__(
        self,
        drone_id,
        start_node,
        goal_node,
        priority,
        speed=0.2,
        battery=100.0,
        initial_progress=0.0,
        forced_route=None,
        role="traffic"
    ):
        self.id = drone_id
        self.start_node = start_node
        self.goal_node = goal_node
        self.priority = priority
        self.speed = speed
        self.battery = battery
        self.initial_progress = initial_progress
        self.forced_route = forced_route
        self.role = role

        self.altitude = ALTITUDE_LAYERS[priority]

        self.route = []
        self.route_index = 0
        self.progress = 0.0

        self.x = 0.0
        self.y = 0.0

        self.completed = False
        self.hold = False
        self.hold_until = 0.0
        self.hold_cooldown_until = 0.0
        self.slowed = False
        self.slowed_until = 0.0      # FIX: slowdown now uses a timer like hold
        self.lane_slow = False
        self.rerouted = False
        self.ascent_zone = False

        self.history = []
        self.event_log = []

    def set_position(self, x, y):
        self.x = x
        self.y = y

    def current_position_3d(self):
        return (self.x, self.y, self.altitude)

    def log_event(self, message):
        self.event_log.append(message)
