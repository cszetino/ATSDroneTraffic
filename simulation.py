import networkx as nx

from config import (
    TIME_STEP,
    SIM_DURATION,
    BATTERY_DRAIN_PER_STEP,
    FOLLOW_DISTANCE,
    LANE_SLOW_DISTANCE,
)
from conflict_detection import scan_conflicts
from resolution import resolve_conflicts
from path_utils import (
    set_position_along_route,
    route_distance_traveled,
    route_total_length,
    route_remaining_distance,
)


class Simulation:
    def __init__(self, graph, drones):
        self.graph = graph
        self.drones = drones
        self.time = 0.0

        self.conflict_history = []
        self.action_history = []
        self.state_history = []

        self.closure_active = False
        self.closure_node = None

        self._has_ascent_trigger = any(d.role == "ascent_trigger" for d in drones)

        self._initialize_drones()

    def _route_avoiding_virtual_nodes(self, start, goal):
        graph_copy = self.graph.copy()
        for node in ["C_UP_1", "C_UP_2"]:
            if graph_copy.has_node(node):
                graph_copy.remove_node(node)
        return nx.shortest_path(graph_copy, start, goal, weight="weight")

    def _initialize_drones(self):
        for drone in self.drones:
            if drone.forced_route is not None:
                drone.route = list(drone.forced_route)
            else:
                drone.route = self._route_avoiding_virtual_nodes(
                    drone.start_node, drone.goal_node
                )
            drone.route_index = 0
            drone.progress = 0.0
            x, y = self.graph.nodes[drone.start_node]["pos"]
            drone.set_position(x, y)
            if drone.initial_progress > 0:
                set_position_along_route(drone, self.graph, drone.initial_progress)

    def _activate_ascent_zone_if_needed(self):
        if not self._has_ascent_trigger or self.closure_active:
            return []
        actions = []
        for drone in self.drones:
            if drone.role != "ascent_trigger":
                continue
            if drone.route_index >= len(drone.route) - 1:
                drone.completed = False
                drone.hold = True
                drone.ascent_zone = True
                drone.set_position(*self.graph.nodes["C"]["pos"])
                self.closure_active = True
                self.closure_node = "C"
                actions.append({
                    "action": "activate_ascent_zone",
                    "target": drone.id,
                    "reason": "Drone reached node C and converted it into an ascent zone"
                })
                actions.extend(self._reroute_traffic_around_c())
                break
        return actions

    def _reroute_traffic_around_c(self):
        actions = []
        for drone in self.drones:
            if drone.role == "ascent_trigger" or drone.completed:
                continue
            if "C" not in drone.route:
                continue
            try:
                c_index = drone.route.index("C")
            except ValueError:
                continue
            if drone.route_index >= c_index:
                continue

            reroute_node = f"R_{drone.id}"
            if self.graph.has_node(reroute_node):
                self.graph.remove_node(reroute_node)
            self.graph.add_node(reroute_node, pos=(drone.x, drone.y))
            goal = drone.route[-1]
            if drone.x < 1.75:
                self.graph.add_edge(reroute_node, "B", weight=1.0)
                drone.route = [reroute_node, "B", "C_UP_1", "C_UP_2", goal]
            else:
                self.graph.add_edge(reroute_node, "C_UP_1", weight=1.0)
                drone.route = [reroute_node, "C_UP_1", "C_UP_2", goal]

            drone.route_index = 0
            drone.progress = 0.0
            drone.rerouted = True
            drone.slowed = False
            drone.lane_slow = False
            drone.hold = False
            drone.log_event(f"{drone.id} rerouted around node C via bypass arc")
            actions.append({
                "action": "reroute",
                "target": drone.id,
                "reason": "Node C closed; rerouted around ascent zone"
            })
        return actions

    def _release_expired_holds(self):
        for drone in self.drones:
            if drone.role == "ascent_trigger" and drone.ascent_zone:
                continue
            if drone.hold and self.time >= drone.hold_until:
                drone.hold = False
                drone.log_event(f"{drone.id} released from HOLD at {self.time:.1f}s")

    def _reset_per_tick_flags(self):
        """Reset per-tick flags each step.

        lane_slow is always per-tick.
        slowed uses a timer (slowed_until) — clear it only when the timer expires,
        so it persists for SLOWDOWN_DURATION without re-firing every tick.
        """
        for drone in self.drones:
            drone.lane_slow = False
            # FIX: expire slowdown by timer, not by blanket reset every tick
            if drone.slowed and self.time >= getattr(drone, "slowed_until", 0.0):
                drone.slowed = False
                drone.log_event(f"{drone.id} slowdown expired at {self.time:.1f}s")

    def _nearest_leader_distance(self, follower):
        follower_s = route_distance_traveled(follower, self.graph)
        nearest_gap = None
        for leader in self.drones:
            if leader is follower or leader.completed:
                continue
            if leader.role == "ascent_trigger":
                continue
            if leader.altitude != follower.altitude:
                continue
            if leader.route != follower.route:
                continue
            leader_s = route_distance_traveled(leader, self.graph)
            gap = leader_s - follower_s
            if gap > 0:
                if nearest_gap is None or gap < nearest_gap:
                    nearest_gap = gap
        return nearest_gap

    def _apply_preemptive_lane_following(self):
        for follower in self.drones:
            if follower.completed or follower.role == "ascent_trigger":
                continue
            gap = self._nearest_leader_distance(follower)
            if gap is None:
                continue
            if gap < FOLLOW_DISTANCE:
                follower.hold = True
                follower.hold_until = max(follower.hold_until, self.time + 0.5)
            elif gap < LANE_SLOW_DISTANCE:
                follower.lane_slow = True

    def _move_drone(self, drone):
        if drone.completed:
            return
        if drone.hold:
            return
        if drone.route_index >= len(drone.route) - 1:
            if drone.role == "ascent_trigger":
                drone.completed = False
                drone.hold = True
                return
            drone.completed = True
            drone.log_event(f"{drone.id} completed mission")
            return

        current_s = route_distance_traveled(drone, self.graph)
        speed_factor = 1.0
        if drone.slowed:
            speed_factor *= 0.65
        if drone.lane_slow:
            speed_factor *= 0.45

        proposed_s = current_s + drone.speed * speed_factor * TIME_STEP

        for leader in self.drones:
            if leader is drone or leader.completed or leader.role == "ascent_trigger":
                continue
            if leader.altitude != drone.altitude:
                continue
            if leader.route != drone.route:
                continue
            leader_s = route_distance_traveled(leader, self.graph)
            if leader_s > current_s:
                proposed_s = min(proposed_s, leader_s - FOLLOW_DISTANCE)

        proposed_s = max(proposed_s, current_s)
        set_position_along_route(drone, self.graph, proposed_s)

        if proposed_s >= route_total_length(self.graph, drone.route):
            if drone.role == "ascent_trigger":
                drone.completed = False
                drone.hold = True
            else:
                drone.completed = True

    def _update_battery(self, drone):
        if not drone.completed and not drone.hold:
            drone.battery -= BATTERY_DRAIN_PER_STEP
            if drone.battery < 0:
                drone.battery = 0

    def _log_state(self):
        snapshot = {
            "time": self.time,
            "closure_active": self.closure_active,
            "closure_node": self.closure_node,
            "drones": []
        }
        for drone in self.drones:
            snapshot["drones"].append({
                "id": drone.id,
                "x": drone.x,
                "y": drone.y,
                "z": drone.altitude,
                "battery": drone.battery,
                "priority": drone.priority,
                "completed": drone.completed,
                "hold": drone.hold,
                "slowed": drone.slowed,
                "lane_slow": drone.lane_slow,
                "rerouted": drone.rerouted,
                "ascent_zone": drone.ascent_zone,
                "role": drone.role,
                "route": drone.route,
                "route_index": drone.route_index,
            })
        self.state_history.append(snapshot)

    def run(self):
        steps = int(SIM_DURATION / TIME_STEP)

        for _ in range(steps):
            self._release_expired_holds()
            # FIX: reset both lane_slow and slowed each tick
            self._reset_per_tick_flags()
            self._apply_preemptive_lane_following()

            # FIX: skip conflict scanning for drones that are effectively done
            # (remaining distance < 0.05) to prevent phantom holds against
            # drones sitting stationary at their goal before completed flag flips
            active_drones = [
                d for d in self.drones
                if not d.completed and route_remaining_distance(d, self.graph) > 0.05
                or d.role == "ascent_trigger"
            ]

            conflicts = scan_conflicts(active_drones, self.graph)
            actions = resolve_conflicts(conflicts, self.graph, self.time)

            closure_actions = self._activate_ascent_zone_if_needed()
            actions.extend(closure_actions)

            self.conflict_history.append({"time": self.time, "events": conflicts})
            self.action_history.append({"time": self.time, "actions": actions})

            for drone in self.drones:
                self._move_drone(drone)
                self._update_battery(drone)
                drone.history.append((drone.x, drone.y, drone.altitude))

            self._log_state()
            self.time += TIME_STEP

        return {
            "conflict_history": self.conflict_history,
            "action_history": self.action_history,
            "state_history": self.state_history,
            "drones": self.drones,
        }
