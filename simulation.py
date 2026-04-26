import networkx as nx

from config import (
    TIME_STEP,
    SIM_DURATION,
    BATTERY_DRAIN_PER_STEP,
    HOVER_DRAIN_MULTIPLIER,
    SLOW_DRAIN_MULTIPLIER,
    FOLLOW_DISTANCE,
    LANE_SLOW_DISTANCE,
)
from conflict_detection import scan_conflicts, is_scan_eligible
from resolution import resolve_conflicts
from path_utils import (
    set_position_along_route,
    route_distance_traveled,
    route_total_length,
    route_remaining_distance,
)


class Simulation:
    def __init__(self, airspace, drones):
        self.airspace = airspace
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
        """Compute initial route using the base graph with virtual bypass nodes excluded."""
        g = self.airspace.get_routing_graph(exclude_virtual=True)
        return nx.shortest_path(g, start, goal, weight="weight")

    def _initialize_drones(self):
        base = self.airspace._base
        for drone in self.drones:
            if drone.forced_route is not None:
                drone.route = list(drone.forced_route)
            else:
                drone.route = self._route_avoiding_virtual_nodes(
                    drone.start_node, drone.goal_node
                )
            drone.route_index = 0
            drone.progress = 0.0
            x, y = base.nodes[drone.start_node]["pos"]
            drone.set_position(x, y)
            if drone.initial_progress > 0:
                set_position_along_route(drone, base, drone.initial_progress)
                # set_position_along_route no longer sets completed; check here
                if route_remaining_distance(drone, base) <= 0:
                    drone.completed = True

    def _activate_ascent_zone_if_needed(self):
        if not self._has_ascent_trigger or self.closure_active:
            return []
        actions = []
        base = self.airspace._base
        for drone in self.drones:
            if drone.role != "ascent_trigger":
                continue
            # Trigger on arrival: remaining distance near zero, not on last edge start
            if route_remaining_distance(drone, base) < 0.05:
                drone.completed = False
                drone.hold = True
                drone.ascent_zone = True
                drone.set_position(*base.nodes["C"]["pos"])
                self.closure_active = True
                self.closure_node = "C"
                self.airspace.close_node("C")  # marks C closed in Airspace; no graph mutation
                actions.append({
                    "action": "activate_ascent_zone",
                    "target": drone.id,
                    "reason": "Drone reached node C and converted it into an ascent zone",
                })
                actions.extend(self._reroute_traffic_around_c())
                break
        return actions

    def _reroute_traffic_around_c(self):
        """Reroute all drones whose remaining route passes through C.

        Uses airspace.get_routing_graph() — C is already closed so the shortest
        path naturally routes through C_UP_1 → C_UP_2. No temporary nodes are
        added to the base graph.
        """
        actions = []
        base = self.airspace._base
        routing_graph = self.airspace.get_routing_graph()  # C removed, bypass nodes kept

        for drone in self.drones:
            if drone.role == "ascent_trigger" or drone.completed:
                continue
            if "C" not in drone.route:
                continue
            c_index = drone.route.index("C")
            if drone.route_index >= c_index:
                continue  # drone already past C

            goal = drone.route[-1]

            # Route from the next waypoint so the drone finishes its current edge first
            next_node_idx = drone.route_index + 1
            if next_node_idx >= len(drone.route):
                continue
            next_node = drone.route[next_node_idx]

            if not routing_graph.has_node(next_node) or not routing_graph.has_node(goal):
                continue

            try:
                detour = nx.shortest_path(routing_graph, next_node, goal, weight="weight")
            except nx.NetworkXNoPath:
                continue

            # Preserve the current segment prefix so route_index/progress remain valid
            new_route = drone.route[:drone.route_index + 1] + detour
            drone.route = new_route
            drone.rerouted = True
            drone.slowed = False
            drone.lane_slow = False
            drone.hold = False
            drone.log_event(f"{drone.id} rerouted around node C via bypass: {drone.route}")
            actions.append({
                "action": "reroute",
                "target": drone.id,
                "reason": "Node C closed; rerouted around ascent zone",
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
        """Reset per-tick flags and expire timer-based states."""
        for drone in self.drones:
            drone.lane_slow = False
            if drone.slowed and self.time >= drone.slowed_until:
                drone.slowed = False
                drone.log_event(f"{drone.id} slowdown expired at {self.time:.1f}s")

    def _nearest_leader_distance(self, follower):
        """Return the gap (in route-distance units) to the closest leader on the same route."""
        base = self.airspace._base
        follower_s = route_distance_traveled(follower, base)
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
            leader_s = route_distance_traveled(leader, base)
            gap = leader_s - follower_s
            if gap > 0:
                if nearest_gap is None or gap < nearest_gap:
                    nearest_gap = gap
        return nearest_gap

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
            drone.log_event(f"{drone.id} completed mission at {self.time:.1f}s")
            return

        base = self.airspace._base
        current_s = route_distance_traveled(drone, base)
        speed_factor = 1.0
        if drone.slowed:
            speed_factor *= 0.65
        if drone.lane_slow:
            speed_factor *= 0.45

        proposed_s = current_s + drone.speed * speed_factor * TIME_STEP

        # Hard physical cap: never advance within FOLLOW_DISTANCE of any leader
        # on the same route. This is a silent constraint — conflict detection
        # handles logging and action assignment.
        for leader in self.drones:
            if leader is drone or leader.completed or leader.role == "ascent_trigger":
                continue
            if leader.altitude != drone.altitude:
                continue
            if leader.route != drone.route:
                continue
            leader_s = route_distance_traveled(leader, base)
            if leader_s > current_s:
                proposed_s = min(proposed_s, leader_s - FOLLOW_DISTANCE)

        proposed_s = max(proposed_s, current_s)
        set_position_along_route(drone, base, proposed_s)

        if proposed_s >= route_total_length(base, drone.route):
            if drone.role == "ascent_trigger":
                drone.completed = False
                drone.hold = True
            else:
                drone.completed = True
                drone.log_event(f"{drone.id} completed mission at {self.time:.1f}s")

    def _update_battery(self, drone):
        """Drain battery with physically motivated rates.

        Hovering (hold) costs more than cruising. Slow flight is slightly less
        efficient per timestep than normal cruise.
        """
        if drone.completed:
            return
        if drone.hold:
            drone.battery -= BATTERY_DRAIN_PER_STEP * HOVER_DRAIN_MULTIPLIER
        elif drone.slowed or drone.lane_slow:
            drone.battery -= BATTERY_DRAIN_PER_STEP * SLOW_DRAIN_MULTIPLIER
        else:
            drone.battery -= BATTERY_DRAIN_PER_STEP
        drone.battery = max(drone.battery, 0.0)

    def _log_state(self):
        snapshot = {
            "time": self.time,
            "closure_active": self.closure_active,
            "closure_node": self.closure_node,
            "drones": [],
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
                "route": list(drone.route),
                "route_index": drone.route_index,
            })
        self.state_history.append(snapshot)

    def run(self):
        base = self.airspace._base
        steps = int(SIM_DURATION / TIME_STEP)

        for _ in range(steps):
            self._release_expired_holds()
            self._reset_per_tick_flags()

            # Canonical eligibility filter lives in is_scan_eligible.
            # ascent_trigger drones are never scan-eligible and are excluded here.
            active_drones = [d for d in self.drones if is_scan_eligible(d, base)]

            conflicts = scan_conflicts(active_drones, base)
            actions = resolve_conflicts(conflicts, self.airspace, self.time)

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
