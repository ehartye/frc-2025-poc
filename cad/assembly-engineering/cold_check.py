"""Reopen released STEP and test its service/fastener datums with adverse models."""

import json
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
from OCP.gp import gp_Pnt
from installed_audit import HERE, ROOT, sha, reopen, bounds, box_gap
from attachments import base, common, dist
from clearance import move


def run():
    receipt = json.loads((HERE / "interfaces.json").read_bytes())
    file = HERE / "station-and-jack-interfaces.step"
    if sha(file) != receipt["outputSha256"]:
        raise ValueError("Not the reviewed cutting file")
    issued = {k: s for k, _, _, _, _, s in reopen()}
    changes = base.named(file)
    installed = {
        k: s
        for k, s in (issued | changes).items()
        if k not in receipt["fixedHeight"]["removed"]
    }
    screw = {}
    for side, sgn in [("Left", "L"), ("Right", "R")]:
        saddle = changes[side + " rail saddle"]
        for x in (270, 317):
            k = sgn + "-" + str(x) + "-screw"
            shape = changes[k]
            screw[k] = [
                {
                    "displacedZmm": dz,
                    "gapMm": dist(move(shape, (0, 0, dz)), saddle),
                    "materialOverlapMm3": common(move(shape, (0, 0, dz)), saddle),
                }
                for dz in (0, 0.15, -0.01)
            ]
            assert (
                screw[k][0]["gapMm"] < 1e-6
                and screw[k][0]["materialOverlapMm3"] < 0.001
            )
            assert (
                screw[k][1]["gapMm"] > 0.1 and screw[k][2]["materialOverlapMm3"] > 0.5
            )
    battery = installed["np18-battery"]
    b = list(bounds(battery))
    sweep = BRepPrimAPI_MakeBox(gp_Pnt(*b[:3]), gp_Pnt(b[3], b[4], b[5] + 172)).Shape()
    excluded = {"battery-hold-down", "np18-battery"} | {
        k
        for k in installed
        if k.startswith("battery-") and ("-upper-washer" in k or "-upper-locknut" in k)
    }
    remaining = {k: s for k, s in installed.items() if k not in excluded}
    hits = {
        k: common(s, sweep)
        for k, s in remaining.items()
        if box_gap(bounds(sweep), bounds(s)) < 0.001
    }
    occupied = common(installed["battery-hold-down"], sweep)
    result = {
        "sourceSha256": sha(file),
        "screws": screw,
        "battery": {
            "remove": sorted(excluded - {"np18-battery"}),
            "trayToClearMm": 172,
            "sweepCheckAgainstBodies": len(remaining),
            "intersectionMm3": {k: v for k, v in hits.items() if v > 0.01},
            "unreleasedTopBarControlMm3": occupied,
            "scope": "Selected bare registered NP18 envelope. Disconnect and cover both leads before service; terminal boots and flexible conductors need a separate mockup.",
        },
    }
    assert not result["battery"]["intersectionMm3"] and occupied > 100
    (HERE / "cold-check.json").write_bytes(
        (json.dumps(result, indent=2) + "\n").encode()
    )


if __name__ == "__main__":
    run()
