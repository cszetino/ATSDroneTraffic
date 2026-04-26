# LLM-Assisted Drone Traffic Control Simulation

## Project Overview

This project is a Python-based simulation demonstrating how autonomous delivery drones can be managed in a shared urban airspace using deterministic traffic control logic with Large Language Model (LLM) support.

The system simulates multiple drones navigating waypoint-based routes while avoiding collisions, resolving congestion, rerouting around temporary airspace restrictions, and prioritizing urgent missions.

The LLM is **not** the flight controller.

Instead, the simulator performs all safety-critical motion and traffic decisions while the LLM provides operator-style summaries, explanations, and advisory outputs.

---

# Core Concept

The project models a simplified drone traffic management system using:

- Priority-based altitude layers
- Conflict detection
- Future conflict prediction
- Lane-following congestion control
- Dynamic rerouting around closed airspace zones
- Battery-aware right-of-way logic
- LLM decision summaries

---

# Priority / Altitude Model

Mission priority determines reserved altitude layers:

| Priority | Altitude Layer |
|--------|----------------|
| LOW | z = 1 |
| NORMAL | z = 2 |
| HIGH | z = 3 |

This helps separate urgent traffic vertically and reduce same-layer conflicts.

Examples:

### HIGH Priority
- Medical delivery
- Emergency battery swap
- Defense / surveillance mission
- Time-sensitive package

### NORMAL Priority
- Standard commercial delivery

### LOW Priority
- Repositioning flight
- Non-urgent delivery
- Training / maintenance route

---

# Decision Logic

Every simulation timestep:

## Step 1: Vertical Separation

Drones on different altitude layers are treated as separated.

## Step 2: Same-Layer Conflict Detection

For drones on the same altitude layer:

- Current separation distance checked
- Future separation predicted
- Same-lane spacing checked

## Step 3: Right-of-Way Logic

If conflict exists:

1. Same-lane follower yields first
2. Higher priority receives right-of-way
3. Lower battery drone receives priority
4. Longer remaining route distance receives priority
5. Drone ID used only as deterministic fallback

## Step 4: Actions Applied

Depending on severity:

- `slowdown`
- `hold`
- `lane_slow`
- `reroute`

---

# LLM Role

The LLM receives structured simulator events such as:

```json
{
  "event_type": "future_conflict",
  "drone_1": "D1",
  "drone_2": "D2",
  "recommended_action": "slowdown"
}
