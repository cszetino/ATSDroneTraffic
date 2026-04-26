SAFE_SEPARATION = 1.15
TIME_STEP = 0.5
SIM_DURATION = 45.0
PREDICTION_HORIZON = 8.0
BATTERY_DRAIN_PER_STEP = 0.2

FOLLOW_DISTANCE = 0.85
LANE_SLOW_DISTANCE = 1.35
HOLD_DURATION = 2.0          # FIX: was 0.75 — too short, caused rapid hold-release-hold cycling
HOLD_COOLDOWN = 1.0          # after a hold expires, drone cannot be re-held for this long
SLOWDOWN_DURATION = 3.0      # how long a slowdown persists before re-evaluating

LOW_PRIORITY = 1
NORMAL_PRIORITY = 2
HIGH_PRIORITY = 3

PRIORITY_NAMES = {
    LOW_PRIORITY: "LOW",
    NORMAL_PRIORITY: "NORMAL",
    HIGH_PRIORITY: "HIGH",
}

ALTITUDE_LAYERS = {
    LOW_PRIORITY: 1,
    NORMAL_PRIORITY: 2,
    HIGH_PRIORITY: 3,
}
