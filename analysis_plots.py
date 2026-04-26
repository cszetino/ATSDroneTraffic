import os
import math
import pandas as pd
import matplotlib.pyplot as plt

from airspace import build_airspace
from scenarios import SCENARIOS
from simulation import Simulation
from path_utils import route_remaining_distance


OUTPUT_DIR = "analysis_outputs"
os.makedirs(OUTPUT_DIR, exist_ok=True)


def distance_2d(a, b):
    return math.dist((a["x"], a["y"]), (b["x"], b["y"]))


def nearest_same_layer_separation(snapshot):
    drones = snapshot["drones"]
    min_sep = None

    for i in range(len(drones)):
        for j in range(i + 1, len(drones)):
            d1 = drones[i]
            d2 = drones[j]

            if d1["completed"] or d2["completed"]:
                continue

            if d1["z"] != d2["z"]:
                continue

            sep = distance_2d(d1, d2)

            if min_sep is None or sep < min_sep:
                min_sep = sep

    return min_sep


def run_all_cases():
    results = {}

    for case_name, scenario_func in SCENARIOS.items():
        graph = build_airspace()
        drones = scenario_func()

        sim = Simulation(graph, drones)
        result = sim.run()

        results[case_name] = {
            "graph": graph,
            "result": result,
        }

    return results


def build_summary_table(results):
    rows = []

    for case_name, data in results.items():
        result = data["result"]
        drones = result["drones"]

        action_count = 0
        reroutes = 0
        slowdowns = 0
        holds = 0
        lane_slows = 0

        for entry in result["action_history"]:
            for action in entry["actions"]:
                action_count += 1
                action_type = action["action"]

                if action_type == "reroute":
                    reroutes += 1
                elif action_type == "slowdown":
                    slowdowns += 1
                elif action_type == "hold":
                    holds += 1
                elif action_type == "lane_slow":
                    lane_slows += 1

        completed = sum(1 for d in drones if d.completed)
        total = len(drones)
        completion_rate = 100 * completed / total

        final_battery_avg = sum(d.battery for d in drones) / total

        last_active_time = 0.0
        for snapshot in result["state_history"]:
            if any(not d["completed"] for d in snapshot["drones"] if d["role"] != "ascent_trigger"):
                last_active_time = snapshot["time"]

        rows.append({
            "case": case_name,
            "total_drones": total,
            "completed_drones": completed,
            "completion_rate_percent": completion_rate,
            "total_ai_actions": action_count,
            "reroutes": reroutes,
            "slowdowns": slowdowns,
            "holds": holds,
            "lane_slows": lane_slows,
            "avg_final_battery_percent": final_battery_avg,
            "last_active_time_sec": last_active_time,
        })

    return pd.DataFrame(rows)


def build_battery_table(results):
    rows = []

    for case_name, data in results.items():
        for drone in data["result"]["drones"]:
            rows.append({
                "case": case_name,
                "drone": drone.id,
                "priority": drone.priority,
                "final_battery_percent": drone.battery,
                "completed": drone.completed,
                "rerouted": drone.rerouted,
            })

    return pd.DataFrame(rows)


def build_separation_table(results):
    rows = []

    for case_name, data in results.items():
        for snapshot in data["result"]["state_history"]:
            sep = nearest_same_layer_separation(snapshot)

            if sep is not None:
                rows.append({
                    "case": case_name,
                    "time_sec": snapshot["time"],
                    "nearest_same_layer_separation": sep,
                })

    return pd.DataFrame(rows)


def plot_actions_by_case(summary_df):
    plt.figure(figsize=(10, 6))
    plt.bar(summary_df["case"], summary_df["total_ai_actions"])
    plt.xticks(rotation=30, ha="right")
    plt.ylabel("Number of AI Actions")
    plt.title("AI Traffic Controller Interventions by Scenario")
    plt.tight_layout()
    plt.savefig(os.path.join(OUTPUT_DIR, "figure_ai_actions_by_case.png"), dpi=300)
    plt.close()


def plot_action_types(summary_df):
    action_cols = ["reroutes", "slowdowns", "holds", "lane_slows"]

    bottom = [0] * len(summary_df)

    plt.figure(figsize=(10, 6))

    for col in action_cols:
        plt.bar(summary_df["case"], summary_df[col], bottom=bottom, label=col)
        bottom = [bottom[i] + summary_df[col].iloc[i] for i in range(len(summary_df))]

    plt.xticks(rotation=30, ha="right")
    plt.ylabel("Number of Actions")
    plt.title("Breakdown of AI Actions by Scenario")
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(OUTPUT_DIR, "figure_action_type_breakdown.png"), dpi=300)
    plt.close()


def plot_completion_rate(summary_df):
    plt.figure(figsize=(10, 6))
    plt.bar(summary_df["case"], summary_df["completion_rate_percent"])
    plt.xticks(rotation=30, ha="right")
    plt.ylabel("Mission Completion Rate (%)")
    plt.ylim(0, 110)
    plt.title("Mission Completion Rate by Scenario")
    plt.tight_layout()
    plt.savefig(os.path.join(OUTPUT_DIR, "figure_completion_rate.png"), dpi=300)
    plt.close()


def plot_average_battery(summary_df):
    plt.figure(figsize=(10, 6))
    plt.bar(summary_df["case"], summary_df["avg_final_battery_percent"])
    plt.xticks(rotation=30, ha="right")
    plt.ylabel("Average Final Battery (%)")
    plt.ylim(0, 100)
    plt.title("Average Final Battery Remaining by Scenario")
    plt.tight_layout()
    plt.savefig(os.path.join(OUTPUT_DIR, "figure_avg_battery_by_case.png"), dpi=300)
    plt.close()


def plot_final_battery_by_drone(battery_df):
    labels = battery_df["case"] + "\n" + battery_df["drone"]

    plt.figure(figsize=(12, 6))
    plt.bar(labels, battery_df["final_battery_percent"])
    plt.xticks(rotation=45, ha="right")
    plt.ylabel("Final Battery (%)")
    plt.ylim(0, 100)
    plt.title("Final Battery Remaining by Drone")
    plt.tight_layout()
    plt.savefig(os.path.join(OUTPUT_DIR, "figure_final_battery_by_drone.png"), dpi=300)
    plt.close()


def plot_nearest_separation(separation_df):
    for case_name in separation_df["case"].unique():
        case_df = separation_df[separation_df["case"] == case_name]

        plt.figure(figsize=(10, 6))
        plt.plot(case_df["time_sec"], case_df["nearest_same_layer_separation"])
        plt.axhline(y=1.15, linestyle="--", label="Safe Separation Threshold")
        plt.xlabel("Time (s)")
        plt.ylabel("Nearest Same-Layer Separation")
        plt.title(f"Nearest Same-Layer Separation Over Time: {case_name}")
        plt.legend()
        plt.tight_layout()
        plt.savefig(os.path.join(OUTPUT_DIR, f"figure_separation_{case_name}.png"), dpi=300)
        plt.close()


def plot_last_active_time(summary_df):
    plt.figure(figsize=(10, 6))
    plt.bar(summary_df["case"], summary_df["last_active_time_sec"])
    plt.xticks(rotation=30, ha="right")
    plt.ylabel("Last Active Mission Time (s)")
    plt.title("Scenario Completion Time Estimate")
    plt.tight_layout()
    plt.savefig(os.path.join(OUTPUT_DIR, "figure_completion_time_by_case.png"), dpi=300)
    plt.close()


def main():
    results = run_all_cases()

    summary_df = build_summary_table(results)
    battery_df = build_battery_table(results)
    separation_df = build_separation_table(results)

    summary_df.to_csv(os.path.join(OUTPUT_DIR, "summary_metrics.csv"), index=False)
    battery_df.to_csv(os.path.join(OUTPUT_DIR, "battery_metrics.csv"), index=False)
    separation_df.to_csv(os.path.join(OUTPUT_DIR, "separation_metrics.csv"), index=False)

    plot_actions_by_case(summary_df)
    plot_action_types(summary_df)
    plot_completion_rate(summary_df)
    plot_average_battery(summary_df)
    plot_final_battery_by_drone(battery_df)
    plot_nearest_separation(separation_df)
    plot_last_active_time(summary_df)

    print("\nAnalysis complete.")
    print(f"Graphs and CSV files saved to: {OUTPUT_DIR}")
    print("\nSummary Metrics:")
    print(summary_df)


if __name__ == "__main__":
    main()