"""Screen a project-owned receiver lip from station departure through its first impact."""
import argparse
import json
import math
from pathlib import Path

if __package__:
    from . import receiver_contact as station
else:
    import receiver_contact as station

DESIGN = Path(__file__).resolve().parents[1] / "cad" / "station-receiver" / "design.json"


def first_impact(speed_mps, offset_mm, extension_mm, radius_mm, design):
    """Return first contact through center-at-fillet-center height in the station XZ plane."""
    if not all(math.isfinite(v) and v >= 0 for v in (speed_mps, offset_mm, extension_mm, radius_mm)):
        raise ValueError("Speeds, offsets, extension and radius must be finite and nonnegative")
    if radius_mm > min(design["thickness_mm"], design["length_mm"]) or extension_mm > design["slide_travel_mm"]:
        raise ValueError("Radius or extension exceeds the model dimensions")
    r = radius_mm / 1000
    front = station.EXTENDED_FRONT + (offset_mm + design["baseline_extension_mm"] - extension_mm) / 1000
    if front <= station.PANEL_RIGHT:
        raise ValueError("Shelf enters the station lower-panel envelope")
    cx, cz = front + r, station.SHELF_TOP - r
    state, top_time, path = station.path_to_shelf(speed_mps)
    panel_gap = min(station.panel_clearance_bound(path), path[-1][0] - station.PANEL_RIGHT - station.RADIUS_M)
    if panel_gap <= 0:
        raise ValueError("Pipe clearance to lower panel is unresolved")
    at_top = path[-1][0]
    if at_top > front + design["length_mm"]/1000:
        raise ValueError("Pipe passes the far end of the receiver")
    if at_top >= cx:
        time = top_time
        normal = (0.0, 1.0)
        surface = "top"
    else:
        # The center moves towards the arc center in both coordinates until
        # directly above or level with it. Hence the signed gap is monotone.
        low = top_time
        high = min(station.time_at_center_height(state, cz),
                   top_time + (cx - at_top) / state["velocity_x_mps"])
        def gap(t):
            x, z = station.flight_center(state, t)
            return math.hypot(x-cx, z-cz) - (r + station.RADIUS_M)
        if gap(high) > 1e-12:
            return {"surface": "clear-through-cutoff", "time_s": high, "post_x_mps": None}
        for _ in range(55):
            mid = (low + high) / 2
            if gap(mid) > 0:
                low = mid
            else:
                high = mid
        time = high
        x, z = station.flight_center(state, time)
        dist = math.hypot(x-cx, z-cz)
        normal = ((x-cx)/dist, (z-cz)/dist)
        surface = "fillet" if r else "corner"
    vin = (state["velocity_x_mps"], state["velocity_z_mps"] - station.GRAVITY_MPS2*time)
    approach = sum(v*n for v,n in zip(vin, normal))
    if approach >= 0:
        raise ValueError("First-contact normal does not oppose the incoming velocity")
    return {"surface": surface, "time_s": time, "center_m": station.flight_center(state, time),
            "normal_xz": normal, "velocity_in_mps": vin, "normal_approach_mps": -approach,
            "post_x_mps": [vin[0] - (1+e)*approach*normal[0] for e in design["restitution_samples"]]}


def study(design):
    return [
        {"speed_mps": speed, "extra_standoff_mm": offset, "extension_mm": extension, "radius_mm": radius,
         **first_impact(speed, offset, extension, radius, design)}
        for extension in design["extension_choices_mm"] for radius in design["radius_choices_mm"]
        for speed in design["entry_speeds_mps"] for offset in design["extra_standoff_mm"]
    ]


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--design", type=Path, default=DESIGN)
    args = parser.parse_args()
    design = json.loads(args.design.read_text())
    print(json.dumps({"schema": "station-receiver-first-impact/1",
                      "restitution_samples": design["restitution_samples"], "trials": study(design)},
                     allow_nan=False, indent=2))
