"""Check close camera mechanics and project surveyed robot/field witnesses into both eyes."""

import argparse
import itertools
import json
import math
from pathlib import Path

import numpy as np

from mechanics import (
    FIXED,
    FORWARD,
    UP,
    LEFT,
    ROOF_M,
    ROOF_T,
    source,
    pose,
    vendor,
    cylinders,
)
from fixed_locator import HERE, ROOT, read_mount
from camera_parts import box


def project(point, origin, axis, up):
    relative = point - origin
    depth = float(relative @ axis)
    if depth <= 0:
        raise ValueError("Witness lies behind lens")
    focal = 320 / math.tan(math.radians(73 / 2))
    return [
        320 - focal * (relative @ LEFT) / depth,
        240 - focal * (relative @ up) / depth,
    ]


def run(folder):
    raw, eyes, bounds, stops = source(folder)
    reprs = json.loads((HERE / "mount.json").read_bytes())
    fixed_t = pose("fixed-datum-up", bounds, eyes.mean(axis=0))
    rot = fixed_t[0] @ ROOF_M.T
    trans = fixed_t[1] - rot @ ROOF_T
    mounts = read_mount()
    datum = vendor.cad.read_named(
        HERE / "fixed-datum.step", [{"id": k} for k in reprs["reopenedParts"]]
    )
    retained = {
        k: v
        for k, v in mounts.items()
        if k not in ("camera-proxy", "slotted-cradle-weldment", "usb-service-reserve")
    }
    retained.update(datum)
    parts = json.loads((ROOT / "camera-datum-shop/native.json").read_bytes())["parts"]
    roof = vendor.cad.read_named(
        ROOT / "camera-datum-shop/camera-shelf.step", [{"id": k} for k in parts]
    )

    def overlaps(bodies, environment):
        found = {}
        limits = {
            name: np.split(np.array(vendor.bounds(body)), 2)
            for name, body in environment.items()
        }
        for name, body in bodies.items():
            low, high = np.split(np.array(vendor.bounds(body)), 2)
            for other, shape in environment.items():
                left, right = limits[other]
                upper = np.prod(
                    np.maximum(0.0, np.minimum(high, right) - np.maximum(low, left))
                )
                if upper > 0.001:
                    v = vendor.cad.intersection(body, shape)
                    if v > 0.001:
                        found[name + "/" + other] = v
        return found

    roof_case = {str(i): vendor.transform(s, ROOF_M, ROOF_T) for i, s in enumerate(raw)}
    roof_real = {
        k: v
        for k, v in roof.items()
        if k != "camera-envelope" and not k.startswith("camera-M4")
    }
    fixed_case = {str(i): vendor.transform(s, *fixed_t) for i, s in enumerate(raw)}
    mechanical = {
        "fixedCaseToFixedHardware": overlaps(
            fixed_case,
            {k: v for k, v in retained.items() if not k.startswith("fixed-camera-M4-")},
        ),
        "roofCaseToRoofHardware": overlaps(roof_case, roof_real),
    }
    # These plug gauges span the separate roof-source housings. Use them as a transferred fixed-camera
    # connector trial; individual installed stock and the cable beyond the boot remain to be verified.
    trial = {
        "adapter": [19.5, -28.5, 841.3, 39, -2.6, 854.25],
        "data": [-13, -27.1, 837.4, 25.1, -20, 858.2],
        "power-with-transferred-straight-tail": [-53, -11.4, 841.35, 26, -3.7, 854.2],
    }
    gauges = {
        name: vendor.transform(box(*b[:3], *(np.array(b[3:]) - b[:3])), rot, trans)
        for name, b in trial.items()
    }
    mechanical["fixedPlugGaugesToRetainedHardware"] = overlaps(gauges, retained)
    mechanical["fixedPlugGaugesBoundsMm"] = {
        k: vendor.cad.bounds(s) for k, s in gauges.items()
    }
    focal = 320 / math.tan(math.radians(73 / 2))
    plane_margins = {}
    for name, b in trial.items():
        corners = [
            rot @ np.array(p) + trans for p in itertools.product(*zip(b[:3], b[3:]))
        ]
        plane_margins[name] = min(
            float((p - FIXED) @ UP - 240 / focal * ((p - FIXED) @ FORWARD))
            for p in corners
        )
    mechanical["fixedPlugClearanceAboveTopImagePlaneMm"] = plane_margins
    diagonal = math.hypot(320, 240) / focal
    # Test the full corner ray against each observed concentric cylindrical shoulder, using the
    # deepest datum of the finite barrel trial. This checks the close hardware hidden by a 10 mm
    # render clipping plane. Refraction and the actual entrance pupil need real optics.
    aperture = []
    for i, p in enumerate(eyes):
        covers = [
            (r, lo)
            for q, r, lo, hi in cylinders(raw[2])
            if np.linalg.norm(q[:2] - p[:2]) < 0.002 and abs(r - 4) < 0.002
        ]
        if len(covers) < 1:
            raise ValueError("Missing cover aperture")
        for radius, z in sorted(set(tuple(x) for x in [*stops[i], *covers])):
            margin = radius - (p[2] + 0.75 - z) * diagonal
            aperture.append(
                {
                    "eyeSupplierX": p[0],
                    "radiusMm": radius,
                    "zMm": z,
                    "cornerMarginMm": margin,
                }
            )
    reg = reprs["registrations"]
    projections = {}
    for label, x, y in (("reef-A", -490.0, -180.0), ("reef-B", -490.0, 220.0)):
        piece = "coral-" + label[-1]
        nav = json.loads((ROOT / "relieved-match/navigation.json").read_bytes())[
            "cases"
        ]
        ex, ey, h = next(
            r["finalBodyErrorMmDeg"]
            for r in nav
            if r["pipe"] == piece and r["mode"] == "nominal"
        )
        x += ex
        y += ey
        c, s = math.cos(math.radians(h)), math.sin(math.radians(h))
        rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        robot_to_world = lambda p: np.array([x, y, 0.0]) + rz @ p
        row = {}
        for name, axis, up in (
            ("roof", [0, 0, -1], [1, 0, 0]),
            ("fixed-datum-up", FORWARD, UP),
        ):
            offset = np.array([370.0, 0, 0]) if name == "roof" else np.zeros(3)
            pupils = np.array(reg[name]["lensCylinderMidpointsMm"]) + offset
            pipe = np.array([446.0, 0.0, 557.15])
            row[name] = {
                "fieldOriginsMm": [robot_to_world(p).tolist() for p in pupils],
                "fieldAxes": (rz @ np.array(axis)).tolist(),
                "fixturePointFieldMm": robot_to_world(pipe).tolist(),
                "rectifiedPixels": [
                    project(pipe, p, np.array(axis), np.array(up)) for p in pupils
                ],
            }
        projections[piece] = row
    out = {
        "sourceSha256": reprs["enclosureSha256"],
        "fit": mechanical,
        "aperture": aperture,
        "maximumFullFrameCornerSlope": diagonal,
        "fieldFrame": "2025 registered reef local coordinates, millimetres",
        "fieldWitness": projections,
        "surveyedPhysicalPupils": None,
    }
    (HERE / "interfaces.json").write_text(
        json.dumps(out, indent=2) + "\n", encoding="utf-8", newline="\n"
    )
    print(mechanical, min(v["cornerMarginMm"] for v in aperture), flush=True)


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("vendor", type=Path)
    run(p.parse_args().vendor)
