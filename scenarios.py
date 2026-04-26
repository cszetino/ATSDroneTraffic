from drone import Drone
from config import HIGH_PRIORITY, NORMAL_PRIORITY, LOW_PRIORITY


def case_1_no_conflict():
    return [
        Drone("D1", "A", "G", HIGH_PRIORITY, speed=0.26, battery=96.0),
        Drone("D2", "E", "H", NORMAL_PRIORITY, speed=0.24, battery=88.0),
        Drone("D3", "B", "D", LOW_PRIORITY, speed=0.22, battery=92.0),
    ]


def case_2_altitude_deconfliction():
    return [
        Drone("D1", "A", "G", HIGH_PRIORITY, speed=0.25, battery=94.0),
        Drone("D2", "G", "A", NORMAL_PRIORITY, speed=0.25, battery=72.0),
        Drone("D3", "E", "H", LOW_PRIORITY, speed=0.22, battery=85.0),
    ]


def case_3_same_layer_future_conflict():
    return [
        Drone("D1", "A", "G", HIGH_PRIORITY, speed=0.27, battery=96.0),
        Drone("D2", "F", "C", HIGH_PRIORITY, speed=0.24, battery=58.0),
        Drone("D3", "E", "H", HIGH_PRIORITY, speed=0.18, battery=84.0),
    ]


def case_4_dynamic_ascent_zone():
    return [
        Drone("ASC", "B", "C", HIGH_PRIORITY, speed=0.35, battery=90.0, role="ascent_trigger"),
        Drone("D1", "A", "G", HIGH_PRIORITY, speed=0.20, battery=82.0, forced_route=["A", "B", "C", "G"]),
        Drone("D2", "A", "G", NORMAL_PRIORITY, speed=0.18, battery=68.0, forced_route=["A", "B", "C", "G"]),
        Drone("D3", "B", "G", LOW_PRIORITY, speed=0.16, battery=76.0, forced_route=["B", "C", "G"]),
    ]


def case_5_congested_lane():
    route = ["A", "B", "C", "G"]
    return [
        Drone("D1", "A", "G", HIGH_PRIORITY, speed=0.18, battery=91.0, initial_progress=7.2, forced_route=route),
        Drone("D2", "A", "G", HIGH_PRIORITY, speed=0.21, battery=78.0, initial_progress=5.0, forced_route=route),
        Drone("D3", "A", "G", HIGH_PRIORITY, speed=0.24, battery=66.0, initial_progress=2.8, forced_route=route),
        Drone("D4", "A", "G", HIGH_PRIORITY, speed=0.25, battery=84.0, initial_progress=0.6, forced_route=route),
    ]


SCENARIOS = {
    "case_1_no_conflict": case_1_no_conflict,
    "case_2_altitude_deconfliction": case_2_altitude_deconfliction,
    "case_3_same_layer_future_conflict": case_3_same_layer_future_conflict,
    "case_4_dynamic_ascent_zone": case_4_dynamic_ascent_zone,
    "case_5_congested_lane": case_5_congested_lane,
}
