import csv
import math
from config import SAFE_SEPARATION


def compute_min_separation(state_history):
    """Post-hoc audit: find the minimum 3D separation observed between any two
    drones at the same altitude across the entire simulation.

    Returns (min_sep, worst_pair, worst_time). If no eligible pairs exist,
    returns (float('inf'), None, None).
    """
    min_sep = float("inf")
    worst_pair = None
    worst_time = None

    for snap in state_history:
        drones = snap["drones"]
        for i in range(len(drones)):
            for j in range(i + 1, len(drones)):
                d1 = drones[i]
                d2 = drones[j]
                if d1["completed"] or d2["completed"]:
                    continue
                # ascent_trigger drones are excluded from conflict detection;
                # exclude them here too so the metric isn't polluted by the
                # deliberate spatial overlap at the ascent zone node.
                if d1.get("role") == "ascent_trigger" or d2.get("role") == "ascent_trigger":
                    continue
                if d1["z"] != d2["z"]:
                    continue
                dist = math.sqrt(
                    (d1["x"] - d2["x"]) ** 2
                    + (d1["y"] - d2["y"]) ** 2
                    + (d1["z"] - d2["z"]) ** 2
                )
                if dist < min_sep:
                    min_sep = dist
                    worst_pair = (d1["id"], d2["id"])
                    worst_time = snap["time"]

    return min_sep, worst_pair, worst_time


def _count_actions(drone_id, action_history):
    counts = {"hold": 0, "slowdown": 0, "reroute": 0, "lane_slow": 0}
    for entry in action_history:
        for action in entry["actions"]:
            if action["target"] == drone_id:
                key = action["action"]
                if key in counts:
                    counts[key] += 1
    return counts


def _mission_completion_time(drone_id, state_history):
    for snap in state_history:
        for d in snap["drones"]:
            if d["id"] == drone_id and d["completed"]:
                return snap["time"]
    return None


def export_metrics_csv(sim_result, case_name, output_path):
    """Write per-drone metrics to a CSV file and return the rows for inspection."""
    drones = sim_result["drones"]
    action_history = sim_result["action_history"]
    state_history = sim_result["state_history"]

    min_sep, worst_pair, worst_time = compute_min_separation(state_history)
    total_conflicts = sum(len(e["events"]) for e in sim_result["conflict_history"])

    rows = []
    for drone in drones:
        mission_time = _mission_completion_time(drone.id, state_history)
        action_counts = _count_actions(drone.id, action_history)
        rows.append({
            "case": case_name,
            "drone_id": drone.id,
            "role": drone.role,
            "priority": drone.priority,
            "altitude": drone.altitude,
            "completed": drone.completed,
            "mission_time_s": mission_time,
            "final_battery": round(drone.battery, 1),
            "battery_consumed": round(max(0.0, 100.0 - drone.battery), 1),
            "holds_issued": action_counts["hold"],
            "slowdowns_issued": action_counts["slowdown"],
            "reroutes_issued": action_counts["reroute"],
            "lane_slows_issued": action_counts["lane_slow"],
            "rerouted": drone.rerouted,
            "total_conflicts_in_case": total_conflicts,
            "min_separation_observed": round(min_sep, 3) if min_sep != float("inf") else "N/A",
            "separation_violated": (
                "YES" if min_sep < SAFE_SEPARATION else "NO"
            ) if min_sep != float("inf") else "N/A",
            "worst_pair": str(worst_pair) if worst_pair else "N/A",
            "worst_pair_time_s": worst_time if worst_time is not None else "N/A",
        })

    fieldnames = list(rows[0].keys()) if rows else []
    with open(output_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    return rows


def print_summary(sim_result, case_name):
    """Concise per-drone summary plus aggregate stats. Replaces print_results
    and print_resolution_summary in main.py."""
    drones = sim_result["drones"]
    action_history = sim_result["action_history"]
    state_history = sim_result["state_history"]

    min_sep, worst_pair, worst_time = compute_min_separation(state_history)
    total_conflicts = sum(len(e["events"]) for e in sim_result["conflict_history"])

    print(f"\n{'='*70}")
    print(f"RESULTS: {case_name}")
    print(f"{'='*70}")
    print(f"Total conflict events detected : {total_conflicts}")
    if min_sep != float("inf"):
        sep_flag = " *** VIOLATION ***" if min_sep < SAFE_SEPARATION else " (OK)"
        print(
            f"Minimum separation observed    : {min_sep:.3f} units"
            f"{sep_flag}"
            f"  [pair={worst_pair}, t={worst_time:.1f}s]"
        )
    else:
        print("Minimum separation observed    : N/A (no same-altitude pairs)")

    print("\n--- Drone Summary ---")
    header = f"{'ID':<8} {'Role':<14} {'Pri':<4} {'Alt':<4} {'Done':<6} {'Bat%':<6} {'Hold':>5} {'Slow':>5} {'Rerout':>6} {'LnSlw':>6}"
    print(header)
    print("-" * len(header))
    for drone in drones:
        ac = _count_actions(drone.id, action_history)
        mt = _mission_completion_time(drone.id, state_history)
        done_str = f"{mt:.1f}s" if mt is not None else ("yes" if drone.completed else "NO")
        print(
            f"{drone.id:<8} {drone.role:<14} {drone.priority:<4} {drone.altitude:<4} "
            f"{done_str:<6} {drone.battery:<6.1f} "
            f"{ac['hold']:>5} {ac['slowdown']:>5} {ac['reroute']:>6} {ac['lane_slow']:>6}"
        )

    print("\n--- Resolution Log ---")
    found = False
    for entry in action_history:
        for action in entry["actions"]:
            found = True
            print(
                f"  t={entry['time']:.1f}s | {action['target']} -> "
                f"{action['action'].upper()} | {action['reason']}"
            )
    if not found:
        print("  No resolution actions taken.")
