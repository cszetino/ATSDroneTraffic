SAFE_SEPARATION = 1.15
TIME_STEP = 0.5
SIM_DURATION = 45.0
PREDICTION_HORIZON = 8.0
BATTERY_DRAIN_PER_STEP = 0.2
HOVER_DRAIN_MULTIPLIER = 1.5   # holding/hovering costs more power than cruising
SLOW_DRAIN_MULTIPLIER = 1.1    # slow forward flight is slightly less efficient per step

FOLLOW_DISTANCE = 0.85
LANE_SLOW_DISTANCE = 1.35
NEAR_GOAL_THRESHOLD = 0.15     # drones within this distance of goal are excluded from conflict scans

HOLD_DURATION = 2.0
HOLD_COOLDOWN = 1.0
SLOWDOWN_DURATION = 3.0
SLOWDOWN_COOLDOWN = 2.0        # minimum gap before a new slowdown can be logged/applied
LANE_SLOW_COOLDOWN = 1.5       # minimum gap before a new lane_slow log entry is emitted

LOW_PRIORITY = 1
NORMAL_PRIORITY = 2
HIGH_PRIORITY = 3

PRIORITY_NAMES = {
    LOW_PRIORITY: "LOW",
    NORMAL_PRIORITY: "NORMAL",
    HIGH_PRIORITY: "HIGH",
}

# Default altitude layer per priority level. Altitude is now a separate physical
# parameter from priority — drones can be constructed with an explicit altitude.
ALTITUDE_LAYERS = {
    LOW_PRIORITY: 1,
    NORMAL_PRIORITY: 2,
    HIGH_PRIORITY: 3,
}
