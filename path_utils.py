import math


def edge_length(graph, n1, n2):
    x1, y1 = graph.nodes[n1]["pos"]
    x2, y2 = graph.nodes[n2]["pos"]
    return math.dist((x1, y1), (x2, y2))


def route_total_length(graph, route):
    total = 0.0
    for i in range(len(route) - 1):
        total += edge_length(graph, route[i], route[i + 1])
    return total


def route_distance_traveled(drone, graph):
    if not drone.route:
        return 0.0

    total = 0.0

    for i in range(drone.route_index):
        if i + 1 >= len(drone.route):
            break

        total += edge_length(graph, drone.route[i], drone.route[i + 1])

    total += drone.progress
    return total


def set_position_along_route(drone, graph, distance_along_route):
    if not drone.route or len(drone.route) < 2:
        return

    route_length = route_total_length(graph, drone.route)

    if distance_along_route >= route_length:
        drone.route_index = len(drone.route) - 1
        drone.progress = 0.0
        gx, gy = graph.nodes[drone.route[-1]]["pos"]
        drone.set_position(gx, gy)
        drone.completed = True
        return

    remaining = max(distance_along_route, 0.0)

    for i in range(len(drone.route) - 1):
        n1 = drone.route[i]
        n2 = drone.route[i + 1]
        seg_len = edge_length(graph, n1, n2)

        if remaining <= seg_len:
            x1, y1 = graph.nodes[n1]["pos"]
            x2, y2 = graph.nodes[n2]["pos"]

            ratio = 0.0 if seg_len == 0 else remaining / seg_len

            drone.route_index = i
            drone.progress = remaining
            drone.x = x1 + ratio * (x2 - x1)
            drone.y = y1 + ratio * (y2 - y1)
            drone.completed = False
            return

        remaining -= seg_len


def route_remaining_distance(drone, graph):
    if not drone.route:
        return 0.0

    return max(route_total_length(graph, drone.route) - route_distance_traveled(drone, graph), 0.0)
