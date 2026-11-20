"""Independently apply relative travel to the corrected camera screws and fixed upper guide."""

import argparse
import json
from pathlib import Path

from attachments import base, common
from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder
from OCP.gp import gp_Ax2, gp_Pnt, gp_Dir
import runpy
from clearance import evaluate
from cold_issue import precise
from installed_audit import HERE, ROOT, box_gap, reopen, sha
from issue import document
from receiver_frames import corrections, inherited_frames
from shop_sources import issue_shapes

OUT = HERE / "receiver-service"
ISSUE = OUT / "issue"


def ranges(box, dx, ylo=0.0, yhi=0.0):
    return [
        box[0] + min(0.0, dx),
        box[1] + ylo,
        box[2],
        box[3] + max(0.0, dx),
        box[4] + yhi,
        box[5],
    ]


def run(held_camera=False):
    manifest = json.loads((ISSUE / "interfaces.json").read_bytes())
    all_parts = {k: p for k, *_, p in reopen()} | issue_shapes(ISSUE, manifest)
    for k in manifest["fixedHeight"]["removed"]:
        all_parts.pop(k, None)
    assert len(all_parts) == manifest["robotSolids"]
    groups = inherited_frames() | manifest["frameAssignments"]
    corrected = corrections()
    if held_camera:
        groups["moving-camera-public-envelope"] = "Fixed chassis"
        corrected.pop("moving-camera-public-envelope")
    yokes = json.loads(
        (ROOT / "experiments/swerve-drive/retention/open-yokes.json").read_bytes()
    )["parts"]
    y = {
        p["id"]: (-35 if "-left-" in p["id"] else 35)
        for p in yokes
        if p["frame"] == "yoke"
    }
    movement = {}
    for k in all_parts:
        dx = {
            "Middle slide": 185,
            "Outer slide": 370,
            "Left capture yoke": 370,
            "Right capture yoke": 370,
        }.get(groups.get(k), 0)
        extra = 1.5 if k.startswith("grip-") else 0.0
        movement[k] = (dx, min(0.0, y.get(k, 0)) - extra, max(0.0, y.get(k, 0)) + extra)
    boxes = {k: precise(s) for k, s in all_parts.items()}
    guides = set(base.named(OUT / "roof/short-boom-keyed-saddle.step")) | set(
        base.named(OUT / "roof/roof-usb-guide-and-yoke.step")
    )
    guides.remove("guide-R-machined-mast")
    targets = [(k, peer) for k, v in movement.items() if v[0] for peer in guides]
    min_pair = min(
        (box_gap(ranges(boxes[k], *movement[k]), boxes[g]), k, g) for k, g in targets
    )
    if min_pair[0] <= 0:
        raise ValueError(("Upper guide requires a narrow-phase check", min_pair))
    cases = {}
    for k in corrected:
        disposition = {
            "sameRigidTravel": 0,
            "separatedBoxes": 0,
            "exact": [],
            "adverse": [],
        }
        for j, t in all_parts.items():
            if k == j:
                continue
            x, ymin, ymax = movement[j]
            dx = 370.0 - x
            if (dx, ymin, ymax) == (0, 0, 0):
                disposition["sameRigidTravel"] += 1
                continue
            if box_gap(ranges(boxes[k], dx, -ymax, -ymin), boxes[j]) > 0.02:
                disposition["separatedBoxes"] += 1
                continue
            if ymin or ymax:
                disposition["adverse"].append(
                    {"peer": j, "reason": "requires two-axis narrow phase"}
                )
                continue
            result = evaluate(all_parts[k], [(j, t, boxes[j])], (1, 0, 0), 0.0, dx)
            disposition["exact"].append({"peer": j, "receipt": result})
            disposition["adverse"].extend(result["adverse"])
        cases[k] = disposition
    if not held_camera:
        route = ROOT / "experiments/six-wheel/moving-harness/routing.py"
        geometry = runpy.run_path(str(route))
        witness = {}
        for cable, ycoord, radius in (("data", -317, 3.275), ("power", -329, 4.30)):
            for stroke in (345, 350, 355, 360, 365, 370):
                start = gp_Pnt(
                    geometry["MOVING_X"] + stroke - 20, ycoord, geometry["HIGH_Z"]
                )
                segment = BRepPrimAPI_MakeCylinder(
                    gp_Ax2(start, gp_Dir(1, 0, 0)), radius, 20
                ).Shape()
                witness[f"{cable}/{stroke}"] = round(
                    common(segment, all_parts["Right NEO550 REV-21-1651"]), 3
                )
        document(
            OUT / "carrier-motor.json",
            dict(
                geometry=manifest["outputSha256"],
                route=sha(route),
                links=geometry["CHAIN_LINKS"],
                lengthMm=20,
                motor="Right NEO550 REV-21-1651",
                jacketMm={"data": 6.55, "power": 8.60},
                overlapMm3=witness,
                scope="Positive common volume on this straight segment rejects the inherited carrier and fixed jack motor combination at full stroke. Zero here does not qualify the rest of the path.",
            ),
        )
    document(
        OUT
        / (
            "support-fixed-camera-control.json"
            if held_camera
            else "support-travel.json"
        ),
        dict(
            heldCamera=held_camera,
            issue=manifest["outputSha256"],
            framesSha256=sha(OUT / "frames.json"),
            installed=len(boxes),
            fixedGuideBodies=len(guides),
            movingReceiverBodies=len({k for k, _ in targets}),
            guidePairs=len(targets),
            worstGuidePair=min_pair,
            correctedBodies=cases,
            scope="Full forward-and-return stroke with 35 mm captured-yoke travel and a conservative signed 1.5 mm displacement on grip articles. Bodies sharing a rigid carrier retain their parked relative positions.",
        ),
    )
    print(min_pair, {k: r["adverse"] for k, r in cases.items()}, flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--hold-camera", action="store_true")
    run(parser.parse_args().hold_camera)
