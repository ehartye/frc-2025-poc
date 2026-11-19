"""Screen first contact with the centered receiving shelf up to its top-plane crossing."""
import argparse
import json
import math

if __package__:
    from .station_edge_contact import DEFAULT_SPEEDS_MPS, GRAVITY_MPS2, RADIUS_M, SLOPE_RAD, departure
else:
    from station_edge_contact import DEFAULT_SPEEDS_MPS, GRAVITY_MPS2, RADIUS_M, SLOPE_RAD, departure

# Station frame in metres: x increases robotward and z increases upward.
LIP_X = -0.0034863154833308166
LIP_Z = 0.9509805343062192
PANEL_RIGHT = 0.0
PANEL_TOP = 0.933958
SHELF_TOP = 0.880
SHELF_LENGTH = 0.265
EXTENDED_FRONT = 0.049525
RETRACTED_FRONT = EXTENDED_FRONT + 0.220


def supported_center(angle):
    return LIP_X + RADIUS_M * math.sin(angle), LIP_Z + RADIUS_M * math.cos(angle)


def flight_center(state, seconds):
    x = LIP_X + state["center_x_m_from_edge"] + state["velocity_x_mps"] * seconds
    z = (LIP_Z + state["center_z_m_from_edge"] + state["velocity_z_mps"] * seconds
         - 0.5 * GRAVITY_MPS2 * seconds**2)
    return x, z


def panel_clearance_bound(points):
    """For each monotone path interval use the nearest corner of its enclosing box."""
    bounds = []
    for start, end in zip(points, points[1:]):
        dx = max(0, start[0] - PANEL_RIGHT)
        dz = max(0, end[1] - PANEL_TOP)
        bounds.append(math.hypot(dx, dz) - RADIUS_M)
    return min(bounds)


def time_at_center_height(state, height):
    _, initial_z = flight_center(state, 0)
    drop = initial_z - height
    vz = state["velocity_z_mps"]
    return 2 * drop / (math.sqrt(vz**2 + 2 * GRAVITY_MPS2 * drop) - vz)


def path_to_shelf(speed):
    state = departure(speed)
    release_angle = math.radians(state["separation_angle_deg_from_up"])
    time = time_at_center_height(state, SHELF_TOP + RADIUS_M)
    # The circle center moves monotonically right and down on both path segments.
    # Subdivision tightens the continuous panel-clearance bound. No gap between samples is omitted.
    steps = 32
    arc = [supported_center(SLOPE_RAD + (release_angle - SLOPE_RAD) * i / steps)
           for i in range(steps + 1)]
    flight = [flight_center(state, time * i / steps) for i in range(steps + 1)]
    return state, time, arc + flight


def screen(speed, shelf_front):
    state, time, path = path_to_shelf(speed)
    x, z = path[-1]
    panel_bound = panel_clearance_bound(path)
    if panel_bound <= 0:
        result = "panel-clearance-unresolved"
    elif shelf_front <= x <= shelf_front + SHELF_LENGTH:
        result = "first-shelf-contact-on-top"
    elif x + RADIUS_M < shelf_front:
        result = "short-of-shelf-through-plane-crossing"
    else:
        result = "shelf-corner-contact-unresolved"
    return {
        "entry_speed_mps": speed,
        "departure_mode": state["mode"],
        "shelf_front_m": shelf_front,
        "flight_seconds_to_top_plane": time,
        "center_x_m_at_plane": x,
        "center_z_m_at_plane": z,
        "minimum_panel_gap_lower_bound_m": panel_bound,
        "normal_velocity_mps_at_plane": state["velocity_z_mps"] - GRAVITY_MPS2 * time,
        "screen": result,
    }


def stand_off_screen(speed, stand_off_mm):
    """Translate the shelf robotward and find first contact through center-at-top height."""
    if not math.isfinite(stand_off_mm) or stand_off_mm < 0:
        raise ValueError("Additional stand-off must be finite and nonnegative")
    front = EXTENDED_FRONT + stand_off_mm / 1000
    state, bottom_time, path = path_to_shelf(speed)
    bottom_x, _ = path[-1]
    center_time = time_at_center_height(state, SHELF_TOP)
    center_x, _ = flight_center(state, center_time)
    contact_point = None

    if front <= bottom_x <= front + SHELF_LENGTH:
        time = bottom_time
        kind = "top"
        contact_point = (bottom_x, SHELF_TOP)
    elif bottom_x > front + SHELF_LENGTH:
        raise ValueError("Trajectory overshoots the far end of the modeled entrance")
    elif front > center_x + RADIUS_M:
        time = center_time
        kind = "clear-through-center-height"
    else:
        # Until x reaches the front or z reaches shelf height, both components
        # of the center-to-corner distance decrease. They enclose a unique first root.
        low = bottom_time
        high = min(center_time, bottom_time + (front - bottom_x) / state["velocity_x_mps"])
        while high - low > 1e-10:
            mid = (low + high) / 2
            x, z = flight_center(state, mid)
            if math.hypot(x - front, z - SHELF_TOP) > RADIUS_M:
                low = mid
            else:
                high = mid
        time = high
        kind = "entrance-corner"
        contact_point = (front, SHELF_TOP)

    # Past the bottom crossing, positive x only increases. The minimum
    # horizontal gap alone now bounds clearance to the entire lower panel.
    panel_gap = min(panel_clearance_bound(path), bottom_x - PANEL_RIGHT - RADIUS_M)
    if panel_gap <= 0:
        raise ValueError("Lower-panel clearance cannot be certified for this path")
    x, z = flight_center(state, time)
    return {
        "entry_speed_mps": speed, "additional_stand_off_mm": stand_off_mm,
        "surface": kind, "flight_time_s": time, "center_m": (x, z),
        "contact_point_m": contact_point, "panel_gap_lower_bound_m": panel_gap,
        "last_top_contact_stand_off_mm": 1000 * (bottom_x - EXTENDED_FRONT),
        "last_upper_corner_contact_stand_off_mm": 1000 * (center_x + RADIUS_M - EXTENDED_FRONT),
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stand-off-mm", nargs="+", type=float,
                        help="additional robotward offsets from the extended receiving pose")
    args = parser.parse_args()
    if args.stand_off_mm:
        try:
            cases = [stand_off_screen(speed, offset) for offset in args.stand_off_mm
                     for speed in DEFAULT_SPEEDS_MPS]
        except ValueError as error:
            parser.error(str(error))
    else:
        cases = [screen(speed, front) for front in (EXTENDED_FRONT, RETRACTED_FRONT)
                 for speed in DEFAULT_SPEEDS_MPS]
    print(json.dumps(cases, indent=2))
