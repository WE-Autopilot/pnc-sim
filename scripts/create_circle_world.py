"""
This is a script to create circular world.yaml files to load into the sim.
Assumes the car will move CLOCKWISE and start
"""

import math
import yaml

DO_LOOP_BACK = True

def circle_points(radius, num_points):
    pts = []
    for i in range(num_points):
        theta = 2.0 * math.pi * i / num_points
        x = radius * math.cos(theta)
        y = radius * math.sin(theta)
        pts.append(
            {
                "x": round(x, 2),
                "y": round(y, 2),
                "z": 0.0,
            }
        )

    # if a loop add the first waypoint to the end
    if DO_LOOP_BACK:
        pts.append(pts[0])

    return pts


def stop_sign(radius, theta):
    return {
        "x": round(radius * math.cos(theta), 2),
        "y": round(radius * math.sin(theta), 2),
        "z": 1.0,
        "gamma": round(theta + math.pi, 5),  # face inward
    }


def generate_circle_map(
    center_radius, lane_width, num_points, num_stop_signs, filename
):
    left_r = center_radius + lane_width / 2.0
    right_r = center_radius - lane_width / 2.0

    data = {
        "lane_boundary": {
            "left": circle_points(left_r, num_points),
            "right": circle_points(right_r, num_points),
        },
        "entities": [
            stop_sign(right_r - 2.0, 2.0 * i * math.pi / num_stop_signs)
            for i in range(num_stop_signs)
        ],
        "car_pos": {
            "x": round(center_radius, 2),
            "y": 0.0,
            "z": 0.0,
            "gamma": math.pi,
        },
    }

    with open(filename, "w") as f:
        yaml.dump(data, f, sort_keys=False)

    print(f"Wrote {filename}")


if __name__ == "__main__":
    generate_circle_map(
        center_radius=40.0,  # m
        lane_width=5.0,  # m
        num_points=32,
        num_stop_signs=3,
        filename="../config/generated_circle.yaml",
    )
