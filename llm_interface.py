import os
from path_utils import route_remaining_distance, route_distance_traveled
from config import PRIORITY_NAMES

ALLOWED_ACTIONS = {"hold", "slowdown", "lane_slow", "reroute", "no_action"}

def build_llm_decision_payload(conflict_event, graph):
    d1 = conflict_event["drone_1"]
    d2 = conflict_event["drone_2"]
    return {
        "event_type": conflict_event["type"],
        "same_layer": conflict_event.get("same_layer", True),
        "distance_now": conflict_event.get("distance_now"),
        "time_to_conflict": conflict_event.get("time_to_conflict"),
        "predicted_min_distance": conflict_event.get("predicted_min_distance"),
        "drone_1": _drone_packet(d1, graph),
        "drone_2": _drone_packet(d2, graph),
        "controller_rules": [
            "Never allow same-altitude drones to violate safe separation.",
            "Lower priority yields to higher priority.",
            "If priority is equal, lower battery receives right-of-way.",
            "If priority and battery are close, longer remaining mission receives right-of-way.",
            "For same-route congestion, slow the trailing drone first.",
            "For head-on or crossing conflicts on different routes, prefer rerouting the yielding drone.",
            "Use hold only when slowdown/reroute is not enough or unavailable.",
        ],
        "allowed_output_schema": {
            "action": "hold | slowdown | lane_slow | reroute | no_action",
            "target": "drone id receiving the action",
            "right_of_way": "drone id that should continue",
            "reason": "short explanation",
        },
    }

def _drone_packet(drone, graph):
    return {
        "id": drone.id,
        "priority": drone.priority,
        "priority_name": PRIORITY_NAMES.get(drone.priority, "UNKNOWN"),
        "altitude_layer": drone.altitude,
        "battery": round(drone.battery, 1),
        "route": list(drone.route),
        "route_index": drone.route_index,
        "route_progress": round(route_distance_traveled(drone, graph), 2),
        "remaining_distance": round(route_remaining_distance(drone, graph), 2),
        "already_rerouted": drone.rerouted,
    }

def _local_llm_fallback_decision(payload):
    """Offline LLM-controller fallback with the same command shape as a real LLM."""
    d1 = payload["drone_1"]
    d2 = payload["drone_2"]
    event_type = payload["event_type"]

    def choose_yield_and_row():
        if event_type == "same_lane_spacing" and d1["route"] == d2["route"]:
            return (d1, d2) if d1["route_progress"] < d2["route_progress"] else (d2, d1)
        if d1["priority"] < d2["priority"]:
            return d1, d2
        if d2["priority"] < d1["priority"]:
            return d2, d1
        if abs(d1["battery"] - d2["battery"]) > 5.0:
            return (d2, d1) if d1["battery"] < d2["battery"] else (d1, d2)
        if abs(d1["remaining_distance"] - d2["remaining_distance"]) > 0.5:
            return (d2, d1) if d1["remaining_distance"] > d2["remaining_distance"] else (d1, d2)
        return (d1, d2) if d1["id"] > d2["id"] else (d2, d1)

    yielding, row = choose_yield_and_row()
    same_route = d1["route"] == d2["route"]

    if event_type == "same_lane_spacing":
        action = "lane_slow"
    elif event_type in {"future_conflict", "current_conflict"} and not same_route:
        action = "reroute" if not yielding.get("already_rerouted", False) else "hold"
    elif event_type == "future_conflict":
        action = "slowdown"
    elif event_type == "current_conflict":
        action = "hold"
    else:
        action = "no_action"

    return {
        "action": action,
        "target": yielding["id"],
        "right_of_way": row["id"],
        "reason": (
            f"LLM controller selected {action} for {yielding['id']} while giving "
            f"right-of-way to {row['id']} based on conflict type, priority, battery, and remaining route."
        ),
        "mode": "local_llm_fallback",
    }

def _normalize_decision(raw_decision, payload):
    if not isinstance(raw_decision, dict):
        return _local_llm_fallback_decision(payload)

    action = str(raw_decision.get("action", "")).strip().lower()
    target = str(raw_decision.get("target", "")).strip()
    right_of_way = str(raw_decision.get("right_of_way", "")).strip()
    valid_ids = {payload["drone_1"]["id"], payload["drone_2"]["id"]}

    if action not in ALLOWED_ACTIONS or target not in valid_ids or right_of_way not in valid_ids or target == right_of_way:
        return _local_llm_fallback_decision(payload)

    return {
        "action": action,
        "target": target,
        "right_of_way": right_of_way,
        "reason": str(raw_decision.get("reason", "LLM selected action.")),
        "mode": raw_decision.get("mode", "real_llm"),
    }

def _call_real_llm_if_configured(payload):
    # Optional hook. Keep this isolated so the project runs offline in PyCharm.
    # Replace this function with your API call if the professor requires a live model.
    if os.getenv("USE_REAL_LLM", "0") != "1":
        return None
    return None

def get_llm_decision(conflict_event, graph):
    payload = build_llm_decision_payload(conflict_event, graph)
    raw = _call_real_llm_if_configured(payload)
    if raw is None:
        raw = _local_llm_fallback_decision(payload)
    return _normalize_decision(raw, payload)

def build_llm_event_summary(conflict_event, action_list):
    d1 = conflict_event["drone_1"]
    d2 = conflict_event["drone_2"]
    return {
        "event_type": conflict_event["type"],
        "drone_1": d1.id,
        "drone_2": d2.id,
        "priority_1": d1.priority,
        "priority_2": d2.priority,
        "distance_now": conflict_event["distance_now"],
        "time_to_conflict": conflict_event["time_to_conflict"],
        "predicted_min_distance": conflict_event["predicted_min_distance"],
        "same_layer": conflict_event["same_layer"],
        "actions_taken": action_list,
        "decision_logic": "LLM controller chooses action; deterministic simulator validates and executes it.",
    }

def get_mock_llm_response(payload):
    return (
        f"LLM traffic-controller advisory: {payload['drone_1']} and {payload['drone_2']} "
        f"have a {payload['event_type']}. Executed command: {payload['actions_taken']}."
    )
