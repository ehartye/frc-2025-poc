"""Screen the displacement at a lateral point load on one inner drive rail."""
from math import isfinite


def displacement(load_n, axle_x_mm, support_half_span_mm):
    """Return (pinned, fixed) bearing-point travel magnitudes, in millimeters.

    The uniform, uncut web is loaded through its centroid. The two end supports
    are symmetric about chassis X0. No intermediate lateral support is credited.
    """
    if not all(map(isfinite, (load_n, axle_x_mm, support_half_span_mm))):
        raise ValueError("Load and positions must be finite")
    if support_half_span_mm <= 0 or abs(axle_x_mm) > support_half_span_mm:
        raise ValueError("The bearing must fall within a positive support span")
    span = 2 * support_half_span_mm
    left = support_half_span_mm + axle_x_mm
    right = support_half_span_mm - axle_x_mm
    stiffness = 69_000 * 100 * 6**3 / 12  # E * I, N mm²
    pinned = abs(load_n) * (left * right)**2 / (3 * stiffness * span)
    fixed = pinned * left * right / span**2
    return pinned, fixed


if __name__ == "__main__":
    print("force_N,support_abs_x_mm,bearing_x_mm,pinned_mm,fixed_mm")
    for force in (250, 500):
        for support_x in (300, 312):
            for bearing_x in (0, 243.65, 247.65, 251.65):
                pinned, fixed = displacement(force, bearing_x, support_x)
                print(f"{force},{support_x},{bearing_x},{pinned:.4f},{fixed:.4f}")
