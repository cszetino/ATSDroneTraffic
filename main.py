from airspace import build_airspace
from scenarios import SCENARIOS
from simulation import Simulation
from llm_interface import build_llm_event_summary, get_mock_llm_response
from animation import animate_simulation
from metrics import print_summary, export_metrics_csv


CASES_TO_RUN = [
    "case_1_no_conflict",
    "case_2_altitude_deconfliction",
    "case_3_same_layer_future_conflict",
    "case_4_dynamic_ascent_zone",
    "case_5_congested_lane",
]


def run_llm_demo(sim_result):
    latest_payload = None

    for conflict_entry, action_entry in zip(
        sim_result["conflict_history"],
        sim_result["action_history"],
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

    airspace = build_airspace()
    drones = SCENARIOS[case_name]()

    sim = Simulation(airspace, drones)
    result = sim.run()

    print_summary(result, case_name)
    run_llm_demo(result)

    csv_path = f"{case_name}_metrics.csv"
    export_metrics_csv(result, case_name, csv_path)
    print(f"\nMetrics exported: {csv_path}")

    gif_name = f"{case_name}.gif"
    title = f"LLM-Assisted Drone Traffic Simulation: {case_name}"

    # Pass airspace._base (the immutable nx.Graph) so animation.py needs no changes
    animate_simulation(
        result,
        airspace._base,
        title=title,
        output_file=gif_name,
    )

    print(f"Saved animation: {gif_name}")

    return result


if __name__ == "__main__":
    print("Starting drone ATC simulation test cases...")

    for case in CASES_TO_RUN:
        run_case(case)

    print("\nAll simulation cases finished.")
