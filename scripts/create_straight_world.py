"""
Script to create straight-road world.yaml files for the pnc-sim.

Given a list of (x, y) waypoints the script:
  1. Connects them with straight segments.
  2. Offsets left / right lane boundaries by lane_width / 2.
  3. Optionally places stop-sign entities at waypoints or segment midpoints.

Coordinate system:  +X FORWARD, +Y LEFT  (same as create_circle_world.py)

Stop-sign placement
-------------------
Pass a list of waypoint indices to `stop_sign_indices`. A single stop sign
will be placed at that waypoint, facing oncoming traffic (opposite to the
direction of travel).
"""

import math
import yaml

OUTPUT_FILEPATH = "../config/generated_straight.yaml"


# ── helpers ────────────────────────────────────────────────────────────────────


def _unit_normal(ax, ay, bx, by):
    """Return the LEFT-pointing unit normal of segment A→B."""
    dx = bx - ax
    dy = by - ay
    length = math.hypot(dx, dy)
    if length == 0:
        return (0.0, 0.0)
    # rotate 90° CCW  →  left side of travel direction
    return (-dy / length, dx / length)


def _heading(ax, ay, bx, by):
    """Heading angle (rad) from A to B  (atan2(dy, dx))."""
    return math.atan2(by - ay, bx - ax)


def _pt(x, y):
    return {"x": round(x, 2), "y": round(y, 2), "z": 0.0}


# ── lane boundaries ───────────────────────────────────────────────────────────


def line_points(waypoints, lane_width):
    """
    Return (left_pts, right_pts) offset from the centerline by ±lane_width/2.

    At interior waypoints the offset direction is the average of the normals of
    the two adjoining segments so the boundary stays smooth at bends.
    """
    hw = lane_width / 2.0
    n = len(waypoints)
    left, right = [], []

    for i in range(n):
        # compute averaged normal
        if i == 0:
            nx, ny = _unit_normal(*waypoints[0], *waypoints[1])
        elif i == n - 1:
            nx, ny = _unit_normal(*waypoints[-2], *waypoints[-1])
        else:
            n1x, n1y = _unit_normal(*waypoints[i - 1], *waypoints[i])
            n2x, n2y = _unit_normal(*waypoints[i], *waypoints[i + 1])
            nx = (n1x + n2x) / 2.0
            ny = (n1y + n2y) / 2.0
            length = math.hypot(nx, ny)
            if length:
                nx /= length
                ny /= length

        cx, cy = waypoints[i]
        left.append(_pt(cx + nx * hw, cy + ny * hw))
        right.append(_pt(cx - nx * hw, cy - ny * hw))

    return left, right


# ── stop signs ─────────────────────────────────────────────────────────────────


def stop_sign_at(x, y, heading, lane_width, offset=1.0):
    """
    Single stop-sign entity placed at (x, y) but shifted to the right side of the lane.
    Facing *heading* (radians).
    """
    # Shift to the right of the lane
    r = lane_width / 2.0 + offset
    # angle to the right of the heading
    angle = heading - math.pi / 2.0
    sx = x + r * math.cos(angle)
    sy = y + r * math.sin(angle)

    # Face inward (opposite of travel direction)
    face = heading + math.pi

    return {
        "x": round(sx, 2),
        "y": round(sy, 2),
        "z": 1.0,
        "gamma": round(face, 5),
    }


# ── map generator ─────────────────────────────────────────────────────────────


def generate_straight_map(
    waypoints,
    lane_width,
    stop_sign_indices=None,
    stop_sign_distance=5.0,
    filename=OUTPUT_FILEPATH,
):
    """
    Write a straight-road YAML world file.

    Parameters
    ----------
    waypoints : list of (x, y) tuples – centerline coordinates.
    lane_width : float – total lane width (metres).
    stop_sign_indices : list of int – indices of waypoints where a stop sign should be placed.
    stop_sign_distance: float – distance (metres) to place the stop sign *before* the waypoint.
    filename : str – output path.
    """
    if stop_sign_indices is None:
        stop_sign_indices = []

    left, right = line_points(waypoints, lane_width)

    # ── entities (stop signs) ──────────────────────────────────────────────
    entities = []

    for i in stop_sign_indices:
        cx, cy = waypoints[i]
        # heading = direction of travel arriving at this waypoint
        if i > 0:
            h = _heading(*waypoints[i - 1], cx, cy)
        else:
            h = _heading(cx, cy, *waypoints[i + 1])
            
        # Offset the stop sign backwards along the approach vector
        sx = cx - stop_sign_distance * math.cos(h)
        sy = cy - stop_sign_distance * math.sin(h)
        
        entities.append(stop_sign_at(sx, sy, h, lane_width))

    # ── car starts at the first waypoint, facing the second ────────────────
    car_heading = _heading(*waypoints[0], *waypoints[1])

    data = {
        "lane_boundary": {
            "left": left,
            "right": right,
        },
        "entities": entities,
        "car_pos": {
            "x": round(waypoints[0][0], 2),
            "y": round(waypoints[0][1], 2),
            "z": 0.0,
            "gamma": round(car_heading, 5),
        },
    }

    with open(filename, "w") as f:
        yaml.dump(data, f, sort_keys=False)

    print(f"Wrote {filename}")


if __name__ == "__main__":
    generate_straight_map(
        waypoints=[(0, 0), (10, 0), (20, 0), (30, 0), (40, 0), (50, 0), (60, 0), (70, 0), (80, 0), (90, 0), (100, 0)],
        lane_width=5.0,
        stop_sign_indices=[3, 7],
        filename="../config/straight.yaml",
    )

    generate_straight_map(
        waypoints=[
            (0, 0),
            (25, 0),
            (50, 25),
            (75, 25),
            (100, 0),
        ],
        lane_width=5.0,
        stop_sign_indices=[1, 3],
        filename="../config/turn_thingie.yaml",
    )
