import math
from config import SAFE_SEPARATION, PREDICTION_HORIZON, TIME_STEP
from path_utils import route_distance_traveled


def distance_3d(p1, p2):
    return math.sqrt(
        (p1[0] - p2[0]) ** 2 +
        (p1[1] - p2[1]) ** 2 +
        (p1[2] - p2[2]) ** 2
    )


def current_conflict(drone1, drone2, threshold=SAFE_SEPARATION):
    if drone1.completed or drone2.completed:
        return False, None

    if drone1.role == "ascent_trigger" or drone2.role == "ascent_trigger":
        return False, None

    d = distance_3d(drone1.current_position_3d(), drone2.current_position_3d())
    return d < threshold, d


def future_position(drone, t, graph):
    """Project drone position t seconds into the future, walking across multiple edges.

    The original single-edge version clamped at the first edge end, so head-on conflicts
    that happen after a node transition (cases 3 and 6) were never detected. This version
    walks the full remaining route so converging drones on different paths are caught
    well before they actually meet.
    """
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
    if drone1.completed or drone2.completed:
        return False, None, None

    if drone1.role == "ascent_trigger" or drone2.role == "ascent_trigger":
        return False, None, None

    t = 0.0
    min_distance = float("inf")
    prev_distance = None
    converging = False

    while t <= horizon:
        p1 = future_position(drone1, t, graph)
        p2 = future_position(drone2, t, graph)

        d = distance_3d(p1, p2)

        if d < min_distance:
            min_distance = d

        # FIX: only flag a conflict if the drones are actually converging —
        # i.e. their distance is decreasing. This prevents phantom conflicts
        # where one drone is far behind and the prediction horizon projects it
        # past a drone that is effectively already done and stationary.
        if prev_distance is not None and d < prev_distance:
            converging = True

        if converging and d < threshold:
            return True, t, min_distance

        prev_distance = d
        t += dt

    return False, None, min_distance


def same_lane_spacing_event(d1, d2, graph, threshold):
    if d1.role == "ascent_trigger" or d2.role == "ascent_trigger":
        return None

    if d1.route != d2.route or not d1.route:
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
    events = []

    for i in range(len(drones)):
        for j in range(i + 1, len(drones)):
            d1 = drones[i]
            d2 = drones[j]

            if d1.completed or d2.completed:
                continue

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
