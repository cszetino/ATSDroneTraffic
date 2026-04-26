from airspace import build_airspace
from scenarios import SCENARIOS
from simulation import Simulation
from llm_interface import build_llm_event_summary, get_mock_llm_response
from animation import animate_simulation


CASES_TO_RUN = [
    "case_1_no_conflict",
    "case_2_altitude_deconfliction",
    "case_3_same_layer_future_conflict",
    "case_4_dynamic_ascent_zone",
    "case_5_congested_lane",
]


def print_results(sim_result):
    print("\n=== FINAL DRONE STATES ===")

    for drone in sim_result["drones"]:
        print(
            f"{drone.id}: "
            f"pos=({drone.x:.2f}, {drone.y:.2f}, z={drone.altitude}) | "
            f"battery={drone.battery:.1f} | completed={drone.completed} | "
            f"hold={drone.hold} | slowed={drone.slowed} | lane_slow={drone.lane_slow} | "
            f"rerouted={drone.rerouted} | route={drone.route}"
        )

    print("\n=== ACTION LOG ===")
    found_action = False

    for entry in sim_result["action_history"]:
        if entry["actions"]:
            found_action = True
            print(f"\nt={entry['time']:.1f}s")

            for action in entry["actions"]:
                print(f"  {action}")

    if not found_action:
        print("  No actions taken.")


def print_resolution_summary(sim_result):
    print("\n=== RESOLUTION SUMMARY ===")

    found_action = False

    for entry in sim_result["action_history"]:
        for action in entry["actions"]:
            found_action = True
            print(
                f"t={entry['time']:.1f}s | "
                f"{action['target']} assigned {action['action'].upper()} | "
                f"{action['reason']}"
            )

    if not found_action:
        print("No resolution action was required.")


def run_llm_demo(sim_result):
    latest_payload = None

    for conflict_entry, action_entry in zip(
        sim_result["conflict_history"],
        sim_result["action_history"]
    ):
        if conflict_entry["events"] and action_entry["actions"]:
            event = conflict_entry["events"][0]
            actions = action_entry["actions"]
            latest_payload = build_llm_event_summary(event, actions)

    print("\n=== LLM DEMO ===")

    if latest_payload is None:
        print("No conflict events available for LLM summary.")
        return

    response = get_mock_llm_response(latest_payload)

    print(latest_payload)
    print("\nLLM OUTPUT:")
    print(response)


def run_case(case_name):
    print("\n" + "=" * 70)
    print(f"RUNNING: {case_name}")
    print("=" * 70)

    if case_name not in SCENARIOS:
        raise ValueError(f"Unknown scenario: {case_name}")

    graph = build_airspace()
    drones = SCENARIOS[case_name]()

    sim = Simulation(graph, drones)
    result = sim.run()

    print_results(result)
    print_resolution_summary(result)
    run_llm_demo(result)

    gif_name = f"{case_name}.gif"
    title = f"LLM-Assisted Drone Traffic Simulation: {case_name}"

    animate_simulation(
        result,
        graph,
        title=title,
        output_file=gif_name
    )

    print(f"\nSaved animation: {gif_name}")

    return result


if __name__ == "__main__":
    print("Starting drone ATC simulation test cases...")

    for case in CASES_TO_RUN:
        run_case(case)

    print("\nAll simulation cases finished.")
