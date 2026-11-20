"""Shop geometry shared by the native issue and the independent source fit checks.

The roboRIO mounting pattern is NI's four bottom 4-40 threaded inserts. The gray
case is a project-owned keepout. Supplier geometry is never redistributed.
"""

from attachments import base, cyl, cut, fuse, tube, hexagon, volume
from clearance import move
from installed_audit import HERE, sha


def mount_rio(plate):
    device = base.block((171.924, -191.09, 293, 318.076, -47.91, 327.709))
    parts = {}
    holes = []
    for i, dx in enumerate((-50.8, 50.8)):
        for j, dy in enumerate((-63.5, 63.5)):
            x, y = 245 + dx, -119.5 + dy
            # Pocket floor z291.60; a full 1.6mm spacer between the skin and
            # a 0.40mm washer gives 3.4mm grip and caps penetration at 2.95mm.
            bore = cyl(x, y, 288, 1.7, 8)
            plate = cut(plate, bore)
            key = f"roborio-{i}-{j}"
            parts[key + "-backing-spacer"] = tube(x, y, 290, 10, 3.4, 1.6)
            parts[key + "-washer"] = tube(x, y, 289.60, 7.0, 3.3, 0.4)
            shank = cyl(x, y, 289.60, 2.845 / 2, 6.35)
            parts[key + "-4-40-screw"] = cut(
                fuse(cyl(x, y, 286.8, 2.75, 2.8), shank),
                hexagon(x, y, 286.7, 2.381, 1.7),
            )
            device = cut(device, cyl(x, y, 292.9, 1.4225, 3.15))
            holes.append([x, y])
    return plate, parts, device, holes


def lock_rings():
    parts = {}
    for index, y in enumerate((-66, 42)):
        for side, z in (("lower", 430), ("upper", 438)):
            parts[f"receiver-absorber-{index}-KM8-{side}"] = cut(
                hexagon(118, y, z, 10, 3), cyl(118, y, z - 1, 4, 5)
            )
    return parts


def machine_platform(plate):
    """Open cells while retaining full contact lands and eight longitudinal ribs."""
    areas = []
    for x0, x1 in ((15, 99), (119, 212), (234, 315)):
        for y0, y1 in ((-93, -12), (12, 91), (118, 189)):
            # Maintain continuous 12 mm-wide rails at the four device fasteners.
            # Larger round buttons on case/board mark rest pads, not extra product.
            cutter = base.block((x0, y0, 289, x1, y1, 295))
            for x in (194.2, 295.8):
                for y in (-183.0, -56.0):
                    cutter = cut(
                        cutter, base.block((x - 8, y0 - 1, 288, x + 8, y1 + 1, 296))
                    )
            # Central transmitter/PDH wiring stays in the frame; positively seated
            # controls require their actual original case fasteners and cable exits.
            before = volume(plate)
            plate = cut(plate, cutter)
            amount = before - volume(plate)
            if amount <= 0:
                raise ValueError((x0, y0, "uncut window"))
            areas.append(dict(boundsMm=[x0, y0, x1, y1], removedMm3=amount))
    return plate, areas


def device_check(path):
    from rio_mount import reference
    from attachments import common, dist

    # Only compare the material and the intended through-insert thread region.
    # Executing two cut and blind-hole gauges per site differentiates the seat
    # from a solid boss. No inference is made about unmodeled threads.
    original = reference(path)
    solids = list(base.subsolids(original))
    results = []
    for x in (194.2, 295.8):
        for y in (-183.0, -56.0):
            pilot = cyl(x, y, 293.01, 1.0, 2.85)
            outer = tube(x, y, 293.37, 5.0, 2.95, 2.68)
            stop = cyl(x, y, 300, 1, 1)
            results.append(
                dict(
                    centerMm=[x, y],
                    minorBorePlugOverlapMm3=sum(common(s, pilot) for s in solids),
                    threadAnnularWitnessMm3=sum(common(s, outer) for s in solids),
                    blindDepthControlMm3=sum(common(s, stop) for s in solids),
                )
            )
    return dict(
        sourceSha256=sha(path),
        catalog="am-3000a",
        source="https://andymark.com/products/ni-roborio-2-0",
        manual="NI roboRIO 2.0 Embedded Controller for FRC User Manual, dimensions Figure 4 and Method Three",
        figure="https://www.manualslib.com/manual/3320782/Ni-Roborio-2-0.html?page=8",
        patternMm=[101.6, 127],
        thread="#4-40 UNC",
        nominalScrewLengthMm=6.35,
        nominalPenetrationMm=2.95,
        modeling="Smooth minor-bore plug and annular material versus the supplier; installed major-diameter threads are represented as nominal.",
        stations=results,
    )


if __name__ == "__main__":
    import argparse, json
    from pathlib import Path

    p = argparse.ArgumentParser()
    p.add_argument("supplier", type=Path)
    r = device_check(p.parse_args().supplier)
    (HERE / "rio-threaded-fit.json").write_bytes(
        (json.dumps(r, indent=2) + "\n").encode()
    )
