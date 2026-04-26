import networkx as nx
from config import (
    HOLD_DURATION, HOLD_COOLDOWN,
    SLOWDOWN_DURATION, SLOWDOWN_COOLDOWN,
    LANE_SLOW_COOLDOWN,
)
from path_utils import route_distance_traveled, route_remaining_distance
from llm_interface import get_llm_decision

BATTERY_PRIORITY_THRESHOLD = 5.0
REROUTE_COST_THRESHOLD = 0.5
REROUTE_PENALTY_FACTOR = 50.0  # edge-weight multiplier applied to conflict node's edges


def choose_loser(drone1, drone2, graph, event_type=None):
    """Deterministic safety fallback. Used when LLM output is invalid."""
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
        drone.log_event(f"{drone.id} HOLD until {drone.hold_until:.1f}s")
        return True
    drone.hold_until = max(drone.hold_until, current_time + duration)
    return False


def apply_slowdown(drone, current_time):
    if current_time < getattr(drone, "slowed_until", 0.0):
        return False
    # Suppress re-log within the cooldown window after the last slowdown expired
    last_slow_end = getattr(drone, "slowed_until", 0.0)
    if current_time < last_slow_end + SLOWDOWN_COOLDOWN and not drone.slowed:
        return False
    drone.slowed = True
    drone.slowed_until = current_time + SLOWDOWN_DURATION
    drone.log_event(f"{drone.id} SLOWDOWN until {drone.slowed_until:.1f}s")
    return True


def apply_lane_slow(drone, current_time):
    """Apply lane-slow speed reduction.

    drone.lane_slow is a per-tick flag reset each step, so it must be set True
    every tick it should be active. The cooldown prevents duplicate log entries
    while the condition persists across many ticks.
    """
    drone.lane_slow = True  # always set for this tick regardless of cooldown
    if current_time < drone.lane_slow_cooldown_until:
        return False  # still within window — flag is set but don't re-log
    drone.lane_slow_cooldown_until = current_time + LANE_SLOW_COOLDOWN
    drone.log_event(f"{drone.id} LANE SLOW at {current_time:.1f}s")
    return True


def _find_conflict_node(loser, winner):
    winner_nodes = set(winner.route[winner.route_index:])
    for node in loser.route[loser.route_index:]:
        if node in winner_nodes:
            return node
    return None


def validate_action(action, target, winner, airspace, current_time):
    """Pre-execution safety check for any LLM-selected action.

    Returns (is_valid: bool, reason: str).
    """
    if action == "no_action":
        return True, "ok"

    if action == "hold":
        if current_time < target.hold_cooldown_until:
            return False, f"{target.id} is in hold cooldown until {target.hold_cooldown_until:.1f}s"
        return True, "ok"

    if action == "slowdown":
        if target.slowed and current_time < target.slowed_until:
            return False, f"{target.id} is already slowed until {target.slowed_until:.1f}s"
        return True, "ok"

    if action == "lane_slow":
        return True, "ok"

    if action == "reroute":
        if target.rerouted:
            return False, f"{target.id} already rerouted — choose hold or slowdown"
        conflict_node = _find_conflict_node(target, winner)
        if conflict_node is None:
            return False, "no shared conflict node found for reroute"
        routing_graph = airspace.get_routing_graph()
        current_node = target.route[target.route_index]
        goal = target.route[-1]
        if not routing_graph.has_node(current_node) or not routing_graph.has_node(goal):
            return False, "current or goal node not in routing graph"
        penalized = routing_graph.copy()
        if penalized.has_node(conflict_node):
            for nbr in list(penalized.neighbors(conflict_node)):
                penalized[conflict_node][nbr]["weight"] *= REROUTE_PENALTY_FACTOR
        try:
            candidate = nx.shortest_path(penalized, current_node, goal, weight="weight")
            if conflict_node in candidate:
                return False, "no viable detour around conflict node"
        except nx.NetworkXNoPath:
            return False, "NetworkXNoPath: no detour available"
        return True, "ok"

    return False, f"unknown action: {action}"


def apply_reroute(loser, winner, airspace, current_time):
    """Reroute loser around the conflict node using edge-cost penalties.

    No nodes are added to or removed from the airspace graph. A temporary
    penalized copy is used for path-finding only and is discarded afterward.
    """
    if loser.rerouted:
        return False

    conflict_node = _find_conflict_node(loser, winner)
    if conflict_node is None:
        return False

    goal = loser.route[-1]
    if goal == conflict_node:
        return False

    current_node = loser.route[loser.route_index]
    routing_graph = airspace.get_routing_graph()

    if not routing_graph.has_node(current_node) or not routing_graph.has_node(goal):
        return False

    penalized = routing_graph.copy()
    if penalized.has_node(conflict_node):
        for nbr in list(penalized.neighbors(conflict_node)):
            penalized[conflict_node][nbr]["weight"] *= REROUTE_PENALTY_FACTOR

    try:
        new_route = nx.shortest_path(penalized, current_node, goal, weight="weight")
    except nx.NetworkXNoPath:
        return False

    if conflict_node in new_route:
        return False  # solver chose conflict node despite penalty — no viable detour

    loser.route = new_route
    loser.route_index = 0
    loser.progress = 0.0
    loser.rerouted = True
    loser.slowed = False
    loser.lane_slow = False
    loser.hold = False
    loser.log_event(
        f"{loser.id} REROUTED around {conflict_node} (conflict with {winner.id}) via {loser.route}"
    )
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


def resolve_conflicts(conflict_events, airspace, current_time):
    """Resolve conflicts with LLM as the decision-maker and Python as safety executor.

    Takes an Airspace object so that apply_reroute can call
    airspace.get_routing_graph() without mutating the shared base graph.
    """
    actions = []
    base_graph = airspace._base
    priority_order = {"current_conflict": 0, "future_conflict": 1, "same_lane_spacing": 2}
    sorted_events = sorted(conflict_events, key=lambda e: priority_order.get(e["type"], 99))

    for event in sorted_events:
        d1 = event["drone_1"]
        d2 = event["drone_2"]
        event_type = event["type"]

        llm_decision = get_llm_decision(event, base_graph)
        target = _drone_by_id(d1, d2, llm_decision.get("target"))
        row = _drone_by_id(d1, d2, llm_decision.get("right_of_way"))

        if target is None or row is None or target is row:
            target, row = choose_loser(d1, d2, base_graph, event_type)
            llm_decision = {
                "action": "hold" if event_type == "current_conflict" else "slowdown",
                "target": target.id,
                "right_of_way": row.id,
                "reason": "Invalid LLM output; deterministic safety fallback used.",
                "mode": "safety_fallback",
            }

        decision_rank = decision_explanation(target, row, base_graph, event_type, llm_decision)
        requested_action = llm_decision.get("action", "no_action")

        # Safety pre-check before executing
        is_valid, val_reason = validate_action(requested_action, target, row, airspace, current_time)
        if not is_valid:
            requested_action = "hold" if event_type == "current_conflict" else "slowdown"
            llm_decision["reason"] += f" [validator overrode to {requested_action}: {val_reason}]"

        if event_type == "same_lane_spacing":
            # Honour the LLM action for same_lane events; fall back to lane_slow
            if requested_action in ("lane_slow", "no_action", "reroute"):
                apply_lane_slow(target, current_time)
                _append_action(actions, "lane_slow", target, row, decision_rank,
                                f"lane spacing behind {row.id}", llm_decision)
            elif requested_action == "hold":
                applied = apply_hold(target, current_time)
                if applied:
                    _append_action(actions, "hold", target, row, decision_rank,
                                   f"LLM selected hold for lane spacing near {row.id}", llm_decision)
            elif requested_action == "slowdown":
                applied = apply_slowdown(target, current_time)
                if applied:
                    _append_action(actions, "slowdown", target, row, decision_rank,
                                   f"LLM selected slowdown for lane spacing near {row.id}", llm_decision)
            continue

        if requested_action == "reroute":
            rerouted = apply_reroute(target, row, airspace, current_time)
            if rerouted:
                _append_action(actions, "reroute", target, row, decision_rank,
                                f"LLM selected reroute to avoid conflict with {row.id}", llm_decision)
                continue
            requested_action = "hold"  # reroute failed, escalate

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
            apply_lane_slow(target, current_time)
            _append_action(actions, "lane_slow", target, row, decision_rank,
                            f"LLM selected lane slow behind {row.id}", llm_decision)
            continue

        # no_action is only allowed for non-current conflicts
        if requested_action == "no_action" and event_type != "current_conflict":
            continue

        # Final safety net: current_conflict with no valid action → force hold
        applied = apply_hold(target, current_time, duration=HOLD_DURATION)
        if applied:
            _append_action(actions, "hold", target, row, decision_rank,
                            f"Safety fallback hold near {row.id}", llm_decision)

    return actions
