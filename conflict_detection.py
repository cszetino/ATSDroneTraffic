import math
from config import SAFE_SEPARATION, PREDICTION_HORIZON, TIME_STEP, NEAR_GOAL_THRESHOLD
from path_utils import route_distance_traveled, route_remaining_distance


def is_scan_eligible(drone, graph):
    """Single canonical check for conflict scan inclusion.

    A drone is excluded when:
    - it has completed its mission
    - it is an ascent_trigger (managed separately by the simulation)
    - it is within NEAR_GOAL_THRESHOLD of its goal (avoids phantom conflicts
      from a nearly-arrived drone being projected past a stationary one)
    """
    if drone.completed:
        return False
    if drone.role == "ascent_trigger":
        return False
    if route_remaining_distance(drone, graph) < NEAR_GOAL_THRESHOLD:
        return False
    return True


def distance_3d(p1, p2):
    return math.sqrt(
        (p1[0] - p2[0]) ** 2 +
        (p1[1] - p2[1]) ** 2 +
        (p1[2] - p2[2]) ** 2
    )


def current_conflict(drone1, drone2, threshold=SAFE_SEPARATION):
    """Check immediate 3D separation. Eligibility is the caller's responsibility."""
    d = distance_3d(drone1.current_position_3d(), drone2.current_position_3d())
    return d < threshold, d


def future_position(drone, t, graph):
    """Project drone position t seconds into the future, walking across multiple edges."""
    if drone.completed or not drone.route or drone.route_index >= len(drone.route) - 1:
        return drone.current_position_3d()

    if drone.hold:
        return drone.current_position_3d()

    speed_factor = 1.0
    if drone.slowed:
        speed_factor *= 0.65
    if drone.lane_slow:
        speed_factor *= 0.45

    remaining_travel = drone.speed * speed_factor * t

    seg_index = drone.route_index
    seg_progress = drone.progress

    while seg_index < len(drone.route) - 1:
        n1 = drone.route[seg_index]
        n2 = drone.route[seg_index + 1]

        x1, y1 = graph.nodes[n1]["pos"]
        x2, y2 = graph.nodes[n2]["pos"]

        dx = x2 - x1
        dy = y2 - y1
        edge_len = math.sqrt(dx ** 2 + dy ** 2)

        if edge_len == 0:
            seg_index += 1
            seg_progress = 0.0
            continue

        remaining_on_edge = edge_len - seg_progress

        if remaining_travel <= remaining_on_edge:
            ratio = (seg_progress + remaining_travel) / edge_len
            fx = x1 + ratio * dx
            fy = y1 + ratio * dy
            return (fx, fy, drone.altitude)

        remaining_travel -= remaining_on_edge
        seg_index += 1
        seg_progress = 0.0

    gx, gy = graph.nodes[drone.route[-1]]["pos"]
    return (gx, gy, drone.altitude)


def predict_conflict(drone1, drone2, graph,
                     horizon=PREDICTION_HORIZON,
                     dt=TIME_STEP,
                     threshold=SAFE_SEPARATION):
    """Predict whether drone1 and drone2 will conflict within the horizon.

    A conflict is flagged only when drones are actively converging at the moment
    they breach the separation threshold. This prevents false positives from
    drones that briefly approach and then diverge, and from drones whose
    projected goal positions happen to be close together.
    """
    t = 0.0
    min_distance = float("inf")
    conflict_time = None
    prev_distance = None

    while t <= horizon:
        p1 = future_position(drone1, t, graph)
        p2 = future_position(drone2, t, graph)
        d = distance_3d(p1, p2)

        if d < min_distance:
            min_distance = d

        if d < threshold:
            # Only flag if drones are converging at this specific timestep
            if prev_distance is not None and d < prev_distance:
                if conflict_time is None:
                    conflict_time = t

        prev_distance = d
        t += dt

    return (conflict_time is not None), conflict_time, min_distance


def same_lane_spacing_event(d1, d2, graph, threshold):
    """Detect same-route drones with insufficient spacing.

    Skips pairs where either drone is already on hold — hold already resolves
    the spacing issue and piling on lane_slow creates unnecessary log entries.
    """
    if d1.route != d2.route or not d1.route:
        return None

    if d1.hold or d2.hold:
        return None

    s1 = route_distance_traveled(d1, graph)
    s2 = route_distance_traveled(d2, graph)
    gap = abs(s1 - s2)

    if gap < threshold:
        return {
            "type": "same_lane_spacing",
            "pair": (d1.id, d2.id),
            "distance_now": gap,
            "time_to_conflict": 0.0,
            "predicted_min_distance": gap,
            "same_layer": True,
            "drone_1": d1,
            "drone_2": d2,
        }

    return None


def scan_conflicts(drones, graph):
    """Scan all eligible drone pairs for conflicts.

    Eligibility is determined by is_scan_eligible so the filter lives in one place.
    """
    events = []
    eligible = [d for d in drones if is_scan_eligible(d, graph)]

    for i in range(len(eligible)):
        for j in range(i + 1, len(eligible)):
            d1 = eligible[i]
            d2 = eligible[j]

            if d1.altitude != d2.altitude:
                continue

            lane_event = same_lane_spacing_event(d1, d2, graph, SAFE_SEPARATION)
            if lane_event is not None:
                events.append(lane_event)
                continue

            current_flag, current_distance = current_conflict(d1, d2)
            future_flag, time_to_conflict, min_distance = predict_conflict(d1, d2, graph)

            if current_flag:
                events.append({
                    "type": "current_conflict",
                    "pair": (d1.id, d2.id),
                    "distance_now": current_distance,
                    "time_to_conflict": 0.0,
                    "predicted_min_distance": current_distance,
                    "same_layer": True,
                    "drone_1": d1,
                    "drone_2": d2,
                })
            elif future_flag:
                events.append({
                    "type": "future_conflict",
                    "pair": (d1.id, d2.id),
                    "distance_now": current_distance,
                    "time_to_conflict": time_to_conflict,
                    "predicted_min_distance": min_distance,
                    "same_layer": True,
                    "drone_1": d1,
                    "drone_2": d2,
                })

    return events
