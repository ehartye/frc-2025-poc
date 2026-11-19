"""Couple bumper-wall approach clearance to the finite CORAL length on the receiver top."""
import argparse
import json
import math

if __package__:
    from . import station_receiver_impact as receiver
else:
    import station_receiver_impact as receiver

# B4 centered station and source-registered dimensions, documented in STATION-DOCK.md.
BUMPER_HALF_MM = 425
CENTERED_DISTANCE_MM = 434.525
PIPE_LENGTH_MM = 301.625


def interval_in_slab(interval, slope, intercept, low, high):
    """Clip station Y to low <= slope*Y + intercept <= high."""
    if interval is None:
        return None
    if abs(slope) < 1e-14:
        return interval if low <= intercept <= high else None
    a, b = sorted(((low-intercept)/slope, (high-intercept)/slope))
    left, right = max(interval[0], a), min(interval[1], b)
    return (left, right) if right > left else None


def dock(design, speed, yaw_deg, lateral_mm, extension_mm, radius_mm, extra_standoff_mm=0):
    if not all(math.isfinite(v) for v in (speed,yaw_deg,lateral_mm,extension_mm,radius_mm,extra_standoff_mm)):
        raise ValueError("Docking inputs must be finite")
    if abs(yaw_deg) > 45 or min(speed, extra_standoff_mm) < 0:
        raise ValueError("Use a forward-facing approach within 45 degrees and nonnegative speed and stand-off")
    if not 0 <= extension_mm <= design["slide_travel_mm"] or not 0 <= radius_mm <= design["thickness_mm"]:
        raise ValueError("Extension or radius exceeds model dimensions")
    yaw = math.radians(yaw_deg)
    c, s = math.cos(yaw), math.sin(yaw)
    projection = BUMPER_HALF_MM * (abs(c) + abs(s))
    at_fixed_pose = CENTERED_DISTANCE_MM - projection
    nominal_gap = CENTERED_DISTANCE_MM - BUMPER_HALF_MM
    chassis_x = projection + nominal_gap + extra_standoff_mm
    reach = (CENTERED_DISTANCE_MM - 1000*receiver.station.EXTENDED_FRONT
             + extension_mm - design["baseline_extension_mm"])
    shelf_min_x = chassis_x - reach*c - design["width_mm"]*abs(s)/2
    if shelf_min_x <= 0:
        raise ValueError("Shelf crosses the station plane in the adjusted pose")
    _, time, path = receiver.station.path_to_shelf(speed)
    x = path[-1][0] * 1000
    dx = x - chassis_x
    # Global bottom generator: (x,Y,top). After inverse yaw/offset, clip it
    # to the flat portion of the finite shelf in both longitudinal and width axes.
    interval = (-PIPE_LENGTH_MM / 2, PIPE_LENGTH_MM / 2)
    longitudinal = interval_in_slab(interval, s, dx*c-lateral_mm*s+reach,
                                    radius_mm, design["length_mm"])
    covered = interval_in_slab(longitudinal, c, -dx*s-lateral_mm*c,
                               -design["width_mm"]/2, design["width_mm"]/2)
    length = covered[1] - covered[0] if covered else 0.0
    return {"entry_speed_mps": speed, "yaw_deg": yaw_deg, "lateral_mm": lateral_mm,
            "extension_mm": extension_mm, "radius_mm": radius_mm, "extra_standoff_mm": extra_standoff_mm,
            "bumper_gap_at_original_pose_mm": at_fixed_pose,
            "required_chassis_retreat_mm": chassis_x-CENTERED_DISTANCE_MM,
            "after_retreat_chassis_x_mm": chassis_x, "after_retreat_shelf_min_x_mm": shelf_min_x,
            "flight_time_to_top_s": time, "bottom_x_mm": x, "flat_contact_interval_y_mm": covered,
            "flat_contact_length_mm": length, "pipe_length_mm": PIPE_LENGTH_MM,
            "flat_fraction": length/PIPE_LENGTH_MM}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--yaw-deg", type=float, nargs="+", default=[0,2,5,10])
    parser.add_argument("--lateral-mm", type=float, nargs="+", default=[0])
    parser.add_argument("--speeds", type=float, nargs="+", default=[0.5,1,2])
    parser.add_argument("--extra-standoff-mm", type=float, nargs="+", default=[0])
    args = parser.parse_args()
    design = json.loads(receiver.DESIGN.read_text())
    rows = [dock(design,speed,yaw,lateral,ext,radius,extra)
            for yaw in args.yaw_deg for lateral in args.lateral_mm for speed in args.speeds
            for extra in args.extra_standoff_mm
            for ext in design["extension_choices_mm"] for radius in design["radius_choices_mm"]]
    print(json.dumps(rows,indent=2,allow_nan=False))
