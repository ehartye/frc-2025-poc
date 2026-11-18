"""Screen first contact with the centered receiving shelf up to its top-plane crossing."""
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


def path_to_shelf(speed):
    state = departure(speed)
    release_angle = math.radians(state["separation_angle_deg_from_up"])
    _, initial_z = flight_center(state, 0)
    drop = initial_z - (SHELF_TOP + RADIUS_M)
    vz = state["velocity_z_mps"]
    time = 2 * drop / (math.sqrt(vz**2 + 2 * GRAVITY_MPS2 * drop) - vz)
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


if __name__ == "__main__":
    print(json.dumps([screen(speed, front) for front in (EXTENDED_FRONT, RETRACTED_FRONT)
                      for speed in DEFAULT_SPEEDS_MPS], indent=2))
