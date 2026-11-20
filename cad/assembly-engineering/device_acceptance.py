"""Independent acceptance gauges on real supplier geometry at the issued device coordinates."""

import argparse
import json
import math
from pathlib import Path
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.STEPControl import STEPControl_Reader
from OCP.gp import gp_Trsf
from attachments import base, bounds, box_gap, common, cyl, dist, hexagon, tube
from clearance import move
from installed_audit import HERE, reopen, sha
from pdh_mount import SKU_SHA, SOURCE


def run(supplier):
    if sha(supplier) != SKU_SHA:
        raise ValueError("Incorrect, missing or truncated REV-11-1850 native file")
    rd = STEPControl_Reader()
    rd.ReadFile(str(supplier))
    rd.TransferRoots()
    registration = gp_Trsf()
    registration.SetValues(0, 0, 1, -212.2875, 1, 0, 0, 155.5625, 0, 1, 0, 161)
    parts = [
        BRepBuilderAPI_Transform(s, registration, False).Shape()
        for s in base.subsolids(rd.OneShape())
    ]
    actual = [(s, bounds(s)) for s in parts]
    selected = base.named(HERE / "issue/station-and-jack-interfaces.step")
    robot = {k: s for k, *_, s in reopen()} | selected
    reg = json.loads((HERE / "issue/interfaces.json").read_bytes())
    for k in reg["fixedHeight"]["removed"]:
        robot.pop(k, None)
    robot.pop("rev-pdh")
    rb = {k: bounds(s) for k, s in robot.items()}

    def touch(gauge):
        b = bounds(gauge)
        return sum(common(gauge, s) for s, bb in actual if box_gap(b, bb) < 0.01)

    def block(gauge, skip):
        b = bounds(gauge)
        out = {}
        for k, body in robot.items():
            if k in skip or box_gap(b, rb[k]) > 0.01:
                continue
            v = common(gauge, body)
            if v > 0.01:
                out[k] = round(v, 4)
        return out

    result = {}
    for i, x in enumerate((-320.2375, -104.3375)):
        for j, y in enumerate((104.7625, 206.3625)):
            k = f"pdh-{i}-{j}"
            washer = robot[k + "-M4-top-washer"]
            socket = tube(x, y, 168.15, 10, 8.3, 42)
            shaft = cyl(x, y, 160.9, 2.0, 6.55)
            wrench = block(socket, {k + "-M4-locknut", k + "-M4x16-low-screw"})
            result[k] = dict(
                shaftMaterialMm3=touch(shaft),
                washerMaterialMm3=touch(washer),
                inward001mmBearingAreaMm2=touch(move(washer, (0, 0, -0.01))) / 0.01,
                socketDeviceMm3=touch(socket),
                socketRobotHits=wrench,
                thinSocketMaterialByODmm={
                    str(od): touch(tube(x, y, 168.15, od, 8.3, 42))
                    for od in (9.0, 9.2, 9.4, 9.6, 9.8)
                },
            )
    lead = gauge = None
    for k in list(selected):
        if not k.startswith("receiver-absorber") or "KM8" not in k:
            continue
        _, _, idx, _, side = k.split("-")
        y = (-66, 42)[int(idx)]
        z = 430 if side == "lower" else 438
        # A straight AF10 thin open-end jaw approaches horizontally from +X.
        # Its U-slot remains open around the housing; two prongs flank the nut.
        from attachments import fuse

        a = base.block((115, y - 8, z + 0.2, 128, y - 5.08, z + 2.8))
        b = base.block((115, y + 5.08, z + 0.2, 128, y + 8, z + 2.8))
        c = base.block((125, y - 8, z + 0.2, 177, y + 8, z + 2.8))
        wrench = fuse(fuse(a, c), b)
        skip = {k, "support-machined-bridge"}
        result[k] = dict(
            assembledStraightToolHits=block(wrench, skip),
            preassemblyToolHits={
                n: v
                for n, v in block(wrench, skip).items()
                if n.startswith("receiver-absorber")
            },
        )
    out = dict(
        supplierStepSha256=sha(supplier),
        supplierSource=SOURCE,
        issue=reg["outputSha256"],
        axes="world X,Y,Z = supplier +Z,+X,+Y plus registered translation in mm.",
        stations=result,
    )
    (HERE / "issue/device-acceptance.json").write_bytes(
        (json.dumps(out, indent=2) + "\n").encode()
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("supplier", type=Path)
    run(parser.parse_args().supplier)
