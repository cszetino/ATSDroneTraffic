import math
import networkx as nx
from config import HOLD_DURATION, HOLD_COOLDOWN, SLOWDOWN_DURATION
from path_utils import route_distance_traveled, route_remaining_distance
from llm_interface import get_llm_decision

BATTERY_PRIORITY_THRESHOLD = 5.0
REROUTE_COST_THRESHOLD = 0.5
_VIRTUAL_NODES = {"C_UP_1", "C_UP_2"}

def choose_loser(drone1, drone2, graph, event_type=None):
    """Deterministic safety fallback only. The LLM controller is the primary decision layer."""
    if event_type == "same_lane_spacing" and drone1.route == drone2.route and drone1.route:
        s1 = route_distance_traveled(drone1, graph)
        s2 = route_distance_traveled(drone2, graph)
        if abs(s1 - s2) > 0.03:
            return (drone1, drone2) if s1 < s2 else (drone2, drone1)

    if drone1.priority < drone2.priority:
        return drone1, drone2
    if drone2.priority < drone1.priority:
        return drone2, drone1

    battery_diff = abs(drone1.battery - drone2.battery)
    if battery_diff > BATTERY_PRIORITY_THRESHOLD:
        return (drone2, drone1) if drone1.battery < drone2.battery else (drone1, drone2)

    r1 = route_remaining_distance(drone1, graph)
    r2 = route_remaining_distance(drone2, graph)
    if abs(r1 - r2) > REROUTE_COST_THRESHOLD:
        return (drone2, drone1) if r1 > r2 else (drone1, drone2)

    return (drone1, drone2) if drone1.id > drone2.id else (drone2, drone1)

def decision_explanation(loser, winner, graph, event_type, llm_decision=None):
    r_loser = route_remaining_distance(loser, graph)
    r_winner = route_remaining_distance(winner, graph)
    mode = llm_decision.get("mode", "unknown") if llm_decision else "fallback"
    return (
        f"LLM selected Yield={loser.id}, Right-of-way={winner.id} | "
        f"priority {loser.priority}/{winner.priority}, "
        f"battery {loser.battery:.1f}/{winner.battery:.1f}, "
        f"remaining route {r_loser:.1f}/{r_winner:.1f}, "
        f"event={event_type}, mode={mode}"
    )

def apply_hold(drone, current_time, duration=HOLD_DURATION):
    if current_time < drone.hold_cooldown_until:
        return False
    if not drone.hold:
        drone.hold = True
        drone.hold_until = current_time + duration
        drone.hold_cooldown_until = drone.hold_until + HOLD_COOLDOWN
        drone.log_event(f"{drone.id} assigned HOLD until {drone.hold_until:.1f}s")
        return True
    drone.hold_until = max(drone.hold_until, current_time + duration)
    return False

def apply_slowdown(drone, current_time):
    if current_time < getattr(drone, "slowed_until", 0.0):
        return False
    drone.slowed = True
    drone.slowed_until = current_time + SLOWDOWN_DURATION
    drone.log_event(f"{drone.id} assigned SLOWDOWN until {drone.slowed_until:.1f}s")
    return True

def apply_lane_slow(drone):
    if not drone.lane_slow:
        drone.lane_slow = True
        drone.log_event(f"{drone.id} assigned LANE SLOW")
        return True
    return False

def _find_conflict_node(loser, winner):
    winner_nodes = set(winner.route[winner.route_index:])
    for node in loser.route[loser.route_index:]:
        if node in winner_nodes:
            return node
    return None

def _build_detour_graph(graph):
    g = graph.copy()
    for vn in _VIRTUAL_NODES:
        if g.has_node(vn):
            g.remove_node(vn)
    return g

def apply_reroute(loser, winner, graph, current_time):
    if loser.rerouted:
        return False

    conflict_node = _find_conflict_node(loser, winner)
    if conflict_node is None:
        return False

    goal = loser.route[-1]
    if goal == conflict_node:
        return False

    detour_graph = _build_detour_graph(graph)
    if detour_graph.has_node(conflict_node):
        detour_graph.remove_node(conflict_node)

    current_node = loser.route[loser.route_index]
    if not detour_graph.has_node(current_node) or not detour_graph.has_node(goal):
        return False

    try:
        detour_path = nx.shortest_path(detour_graph, current_node, goal, weight="weight")
    except nx.NetworkXNoPath:
        return False

    reroute_node = f"R_{loser.id}"
    if graph.has_node(reroute_node):
        graph.remove_node(reroute_node)

    graph.add_node(reroute_node, pos=(loser.x, loser.y))
    next_node = detour_path[1] if len(detour_path) > 1 else goal
    x1, y1 = loser.x, loser.y
    x2, y2 = graph.nodes[next_node]["pos"]
    graph.add_edge(reroute_node, next_node, weight=math.dist((x1, y1), (x2, y2)))

    loser.route = [reroute_node] + detour_path[1:]
    loser.route_index = 0
    loser.progress = 0.0
    loser.rerouted = True
    loser.slowed = False
    loser.lane_slow = False
    loser.hold = False
    loser.log_event(f"{loser.id} REROUTED around {conflict_node} (conflict with {winner.id}) via {loser.route}")
    return True

def _drone_by_id(drone1, drone2, drone_id):
    if drone1.id == drone_id:
        return drone1
    if drone2.id == drone_id:
        return drone2
    return None

def _append_action(actions, action, target, row, decision_rank, reason, llm_decision):
    actions.append({
        "action": action,
        "target": target.id,
        "right_of_way": row.id if row else None,
        "decision_rank": decision_rank,
        "llm_reason": llm_decision.get("reason", ""),
        "llm_mode": llm_decision.get("mode", "unknown"),
        "reason": reason,
    })

def resolve_conflicts(conflict_events, graph, current_time):
    """Resolve conflicts with LLM as the decision-maker and Python as safety executor."""
    actions = []
    priority_order = {"current_conflict": 0, "future_conflict": 1, "same_lane_spacing": 2}
    sorted_events = sorted(conflict_events, key=lambda e: priority_order.get(e["type"], 99))

    for event in sorted_events:
        d1 = event["drone_1"]
        d2 = event["drone_2"]
        event_type = event["type"]

        llm_decision = get_llm_decision(event, graph)
        target = _drone_by_id(d1, d2, llm_decision.get("target"))
        row = _drone_by_id(d1, d2, llm_decision.get("right_of_way"))

        if target is None or row is None or target is row:
            target, row = choose_loser(d1, d2, graph, event_type)
            llm_decision = {
                "action": "hold" if event_type == "current_conflict" else "slowdown",
                "target": target.id,
                "right_of_way": row.id,
                "reason": "Invalid LLM command; deterministic safety fallback used.",
                "mode": "safety_fallback",
            }

        decision_rank = decision_explanation(target, row, graph, event_type, llm_decision)
        requested_action = llm_decision.get("action", "no_action")

        if event_type == "same_lane_spacing":
            applied = apply_lane_slow(target)
            if applied:
                _append_action(actions, "lane_slow", target, row, decision_rank,
                               f"LLM-controlled lane spacing behind {row.id}", llm_decision)
            continue

        if requested_action == "reroute":
            rerouted = apply_reroute(target, row, graph, current_time)
            if rerouted:
                _append_action(actions, "reroute", target, row, decision_rank,
                               f"LLM selected reroute to avoid conflict with {row.id}", llm_decision)
                continue
            requested_action = "hold"

        if requested_action == "slowdown":
            applied = apply_slowdown(target, current_time)
            if applied:
                _append_action(actions, "slowdown", target, row, decision_rank,
                               f"LLM selected slowdown near {row.id}", llm_decision)
            continue

        if requested_action == "hold":
            applied = apply_hold(target, current_time, duration=HOLD_DURATION)
            if applied:
                _append_action(actions, "hold", target, row, decision_rank,
                               f"LLM selected hold near {row.id}", llm_decision)
            continue

        if requested_action == "lane_slow":
            applied = apply_lane_slow(target)
            if applied:
                _append_action(actions, "lane_slow", target, row, decision_rank,
                               f"LLM selected lane slow behind {row.id}", llm_decision)
            continue

        # no_action is allowed only for non-current conflicts. Current conflicts get safety hold.
        if requested_action == "no_action" and event_type != "current_conflict":
            continue

        applied = apply_hold(target, current_time, duration=HOLD_DURATION)
        if applied:
            _append_action(actions, "hold", target, row, decision_rank,
                           f"Safety fallback hold near {row.id}", llm_decision)

    return actions
