import matplotlib.pyplot as plt
import networkx as nx
import numpy as np
from matplotlib.animation import FuncAnimation
from config import PRIORITY_NAMES


def _build_action_map(sim_result):
    """FIX: precompute dict keyed by time — avoids linear scan + float equality every frame."""
    action_map = {}
    for entry in sim_result["action_history"]:
        action_map[entry["time"]] = entry["actions"]
    return action_map


def _action_text_at_time(action_map, time_value):
    actions = action_map.get(time_value, [])
    messages = []

    for action in actions:
        target = action.get("target", "?")
        act = action.get("action", "").upper()
        row = action.get("right_of_way")

        if row:
            messages.append(f"{target} -> {act} | ROW={row}")
        else:
            messages.append(f"{target} -> {act}")

    if not messages:
        return "Decision: No action"

    if len(messages) > 2:
        return "Decision: " + " | ".join(messages[:2]) + f" | +{len(messages) - 2} more"

    return "Decision: " + " | ".join(messages)


def _battery_color(battery):
    """Return a color string based on battery level."""
    if battery > 60:
        return "green"
    elif battery > 30:
        return "orange"
    else:
        return "red"


def animate_simulation(sim_result, graph, title, output_file, frame_skip=2):
    fig, ax = plt.subplots(figsize=(10, 6))
    pos = nx.get_node_attributes(graph, "pos")

    is_closure_case = "case_4_dynamic_ascent_zone" in title

    visible_nodes = [n for n in graph.nodes if not str(n).startswith("C_UP") and not str(n).startswith("R_")]
    virtual_nodes = [n for n in graph.nodes if str(n).startswith("C_UP")]

    visible_edges = [
        e for e in graph.edges
        if not str(e[0]).startswith("C_UP") and not str(e[1]).startswith("C_UP") and not str(e[0]).startswith("R_") and not str(e[1]).startswith("R_")
    ]

    virtual_edges = [
        e for e in graph.edges
        if str(e[0]).startswith("C_UP") or str(e[1]).startswith("C_UP")
    ]

    nx.draw_networkx_edges(
        graph,
        pos,
        edgelist=visible_edges,
        edge_color="gray",
        ax=ax
    )

    if is_closure_case:
        nx.draw_networkx_edges(
            graph,
            pos,
            edgelist=virtual_edges,
            edge_color="goldenrod",
            style="--",
            width=2,
            alpha=0.65,
            ax=ax
        )

    nx.draw_networkx_nodes(
        graph,
        pos,
        nodelist=visible_nodes,
        node_size=800,
        node_color="lightgray",
        ax=ax
    )

    if is_closure_case:
        nx.draw_networkx_nodes(
            graph,
            pos,
            nodelist=virtual_nodes,
            node_size=120,
            node_color="gold",
            alpha=0.45,
            ax=ax
        )

    nx.draw_networkx_labels(
        graph,
        pos,
        labels={n: n for n in visible_nodes},
        ax=ax
    )

    drone_ids = [drone.id for drone in sim_result["drones"]]
    colors = ["black", "red", "orange", "green", "purple", "brown", "blue", "pink", "cyan"]

    offsets = [
        (0.0, 0.0),
        (-0.08, 0.06),
        (0.08, -0.06),
        (0.08, 0.06),
        (-0.08, -0.06),
        (0.12, 0.0),
        (-0.12, 0.0),
        (0.0, 0.12),
        (0.0, -0.12),
    ]

    for i, drone in enumerate(sim_result["drones"]):
        if is_closure_case and drone.id != "ASC":
            continue

        color = colors[i % len(colors)]
        dx, dy = offsets[i % len(offsets)]

        route_points = []
        for node in drone.route:
            x, y = graph.nodes[node]["pos"]
            route_points.append((x + dx, y + dy))

        if len(route_points) >= 2:
            xs = [p[0] for p in route_points]
            ys = [p[1] for p in route_points]

            ax.plot(
                xs,
                ys,
                linestyle="--",
                linewidth=1.3,
                color=color,
                alpha=0.42,
                label=f"{drone.id} route"
            )

    drone_markers = {}
    drone_labels = {}
    drone_battery_bars = {}
    drone_battery_texts = {}

    for i, drone_id in enumerate(drone_ids):
        color = colors[i % len(colors)]
        marker_style = "^" if drone_id == "ASC" else "o"

        marker, = ax.plot(
            [],
            [],
            marker=marker_style,
            markersize=11 if drone_id == "ASC" else 9,
            linestyle="None",
            color=color
        )

        label = ax.text(
            0,
            0,
            "",
            fontsize=8,
            fontweight="bold"
        )

        # FIX: small battery indicator bar below each drone label
        batt_bar, = ax.plot(
            [],
            [],
            linewidth=4,
            solid_capstyle="butt",
            color="green",
            alpha=0.85,
            zorder=6
        )

        batt_text = ax.text(
            0, 0, "",
            fontsize=7,
            color="dimgray",
            zorder=7
        )

        drone_markers[drone_id] = marker
        drone_labels[drone_id] = label
        drone_battery_bars[drone_id] = batt_bar
        drone_battery_texts[drone_id] = batt_text

    status_box = ax.text(
        0.02,
        0.97,
        "",
        transform=ax.transAxes,
        fontsize=8,
        verticalalignment="top",
        bbox=dict(boxstyle="round", facecolor="white", alpha=0.85)
    )

    altitude_note = ax.text(
        0.02,
        0.05,
        "Altitude layers: LOW z=1 | NORMAL z=2 | HIGH z=3",
        transform=ax.transAxes,
        fontsize=8,
        bbox=dict(boxstyle="round", facecolor="white", alpha=0.75)
    )

    closure_artists = []

    if is_closure_case:
        cx, cy = graph.nodes["C"]["pos"]

        ax.text(
            cx,
            cy + 0.70,
            "ASCENT ZONE TRIGGER",
            ha="center",
            va="bottom",
            fontsize=8,
            color="darkgoldenrod",
            alpha=0.55
        )

        zone_ring = ax.scatter(
            [],
            [],
            s=1450,
            facecolors="none",
            edgecolors="gold",
            linewidths=4,
            zorder=8
        )

        zone_label = ax.text(
            cx,
            cy + 0.98,
            "",
            ha="center",
            va="bottom",
            fontsize=9,
            fontweight="bold",
            color="darkgoldenrod",
            zorder=9
        )

        blocked_lines = []
        for _ in [("B", "C"), ("C", "G"), ("C", "D")]:
            line, = ax.plot(
                [],
                [],
                color="red",
                linewidth=3,
                alpha=0.45,
                linestyle=":",
                zorder=4
            )
            blocked_lines.append(line)

        closure_artists = [zone_ring, zone_label] + blocked_lines

    ax.set_title(title)
    ax.set_xlabel("X Position")
    ax.set_ylabel("Y Position")
    ax.grid(True)

    if len(drone_ids) <= 4:
        ax.legend(loc="lower right")
    else:
        ax.legend(loc="upper right", fontsize=8)

    snapshots = sim_result["state_history"]

    # FIX: precompute action lookup dict — no more per-frame linear scan
    action_map = _build_action_map(sim_result)

    frame_indices = list(range(0, len(snapshots), frame_skip))
    if frame_indices[-1] != len(snapshots) - 1:
        frame_indices.append(len(snapshots) - 1)

    def update(frame_index):
        snapshot = snapshots[frame_index]
        time_value = snapshot["time"]

        active_action_text = _action_text_at_time(action_map, time_value)

        status_box.set_text(
            f"Time: {time_value:.1f}s\n"
            f"{active_action_text}"
        )

        artists = [status_box, altitude_note]

        if is_closure_case and closure_artists:
            closure_active = snapshot.get("closure_active", False)
            cx, cy = graph.nodes["C"]["pos"]

            zone_ring = closure_artists[0]
            zone_label = closure_artists[1]
            blocked_lines = closure_artists[2:]

            if closure_active:
                zone_ring.set_offsets([[cx, cy]])
                zone_label.set_text("ASCENT ZONE ACTIVE\nNODE C CLOSED")

                blocked_specs = [
                    ("B", "C"),
                    ("C", "G"),
                    ("C", "D"),
                ]

                for line, (n1, n2) in zip(blocked_lines, blocked_specs):
                    x1, y1 = graph.nodes[n1]["pos"]
                    x2, y2 = graph.nodes[n2]["pos"]
                    line.set_data([x1, x2], [y1, y2])
            else:
                zone_ring.set_offsets(np.empty((0, 2)))
                zone_label.set_text("")

                for line in blocked_lines:
                    line.set_data([], [])

            artists.extend(closure_artists)

        # FIX: build per-drone state dict once per frame instead of scanning per drone
        state_by_id = {s["id"]: s for s in snapshot["drones"]}

        for i, drone_id in enumerate(drone_ids):
            state = state_by_id.get(drone_id)

            if state is None:
                continue

            dx, dy = offsets[i % len(offsets)]
            x = state["x"] + dx
            y = state["y"] + dy

            drone_markers[drone_id].set_data([x], [y])

            if state.get("role") == "ascent_trigger":
                label_text = "ASC"
                if state.get("ascent_zone", False):
                    label_text += " ZONE"
            else:
                priority_name = PRIORITY_NAMES.get(state["priority"], "UNK")
                label_text = f"{drone_id} {priority_name} z={state['z']}"

                if state.get("rerouted", False):
                    label_text += " REROUTE"
                elif state.get("hold", False):
                    label_text += " HOLD"
                elif state.get("lane_slow", False):
                    label_text += " LANE SLOW"
                elif state.get("slowed", False):
                    label_text += " SLOW"

                if state.get("completed", False):
                    label_text += " DONE"

            drone_labels[drone_id].set_position((x + 0.06, y + 0.06))
            drone_labels[drone_id].set_text(label_text)

            # Battery bar — colored fill showing charge level
            batt = state.get("battery", 100)
            bar_color = _battery_color(batt)
            BAR_MAX = 0.35        # full-charge width in map units
            BAR_Y_OFFSET = 0.22  # distance below drone marker centre
            bar_width = BAR_MAX * (max(batt, 0) / 100.0)
            bar_y = y - BAR_Y_OFFSET

            batt_bar = drone_battery_bars[drone_id]
            # Always draw at least a tiny nub so the bar is visible even at 0 %
            batt_bar.set_data([x, x + max(bar_width, 0.02)], [bar_y, bar_y])
            batt_bar.set_color(bar_color)

            # Percentage label sits to the right of the full bar width
            batt_text = drone_battery_texts[drone_id]
            batt_text.set_position((x + BAR_MAX + 0.04, bar_y - 0.04))
            batt_text.set_text(f"{batt:.0f}%")
            batt_text.set_color(bar_color)

            artists.append(drone_markers[drone_id])
            artists.append(drone_labels[drone_id])
            artists.append(batt_bar)
            artists.append(batt_text)

        return artists

    anim = FuncAnimation(
        fig,
        update,
        frames=frame_indices,
        interval=100,
        blit=True,
        repeat=False
    )

    print(f"Saving {output_file} with {len(frame_indices)} frames...")
    anim.save(output_file, writer="pillow", fps=8)
    plt.close(fig)

    return anim
