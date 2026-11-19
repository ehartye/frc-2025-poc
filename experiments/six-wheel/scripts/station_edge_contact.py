"""Compare tangent departure and frictionless edge contact for a transverse CORAL."""

import argparse
import json
import math

RADIUS_M = 4.5 * 0.0254 / 2
GRAVITY_MPS2 = 9.80665
SLOPE_RAD = math.radians(35)
DEFAULT_SPEEDS_MPS = (0.5, 1.0, 2.0)


def departure(speed_mps):
    """Return the separation state in a local frame at the ideal sharp lip."""
    if not math.isfinite(speed_mps) or speed_mps < 0:
        raise ValueError("Entry speed must be finite and nonnegative")
    gravity_radius = GRAVITY_MPS2 * RADIUS_M
    initial_cos = math.cos(SLOPE_RAD)
    threshold = math.sqrt(gravity_radius * initial_cos)
    if speed_mps >= threshold:
        mode = "immediate-separation"
        release_angle = SLOPE_RAD
        release_speed = speed_mps
    else:
        mode = "supported-arc"
        release_cos = (2 * initial_cos + speed_mps**2 / gravity_radius) / 3
        release_angle = math.acos(release_cos)
        release_speed = math.sqrt(gravity_radius * release_cos)
    return {
        "entry_speed_mps": speed_mps,
        "threshold_mps": threshold,
        "mode": mode,
        "separation_angle_deg_from_up": math.degrees(release_angle),
        "center_x_m_from_edge": RADIUS_M * math.sin(release_angle),
        "center_z_m_from_edge": RADIUS_M * math.cos(release_angle),
        "velocity_x_mps": release_speed * math.cos(release_angle),
        "velocity_z_mps": -release_speed * math.sin(release_angle),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--speeds", nargs="+", type=float, default=DEFAULT_SPEEDS_MPS,
                        metavar="M/S", help="assumed incoming center speeds")
    args = parser.parse_args()
    try:
        states = [departure(speed) for speed in args.speeds]
    except ValueError as error:
        parser.error(str(error))
    print(json.dumps({"model": "frictionless-sharp-edge", "states": states}, indent=2))


if __name__ == "__main__":
    main()
