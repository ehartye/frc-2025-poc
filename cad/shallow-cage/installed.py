"""Audit the screw bridge interference and full forward jack against the registered stowed body."""

import argparse
import hashlib
import json
import sys
from pathlib import Path
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.BRepPrimAPI import BRepPrimAPI_MakePrism, BRepPrimAPI_MakeBox
from OCP.STEPControl import STEPControl_Reader
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE
from OCP.gp import gp_Pnt, gp_Vec
from clearance import ROOT, HERE, base, bounds, box_distance, move, read_assembly


def volume(shape):
    return base.cad.volume(shape)[0]


def sweep_z(s, travel):
    lo = bounds(s)[2]
    fs = TopExp_Explorer(s, TopAbs_FACE)
    bottom = []
    while fs.More():
        f = fs.Current()
        fs.Next()
        if abs(bounds(f)[5] - lo) < 0.005:
            bottom.append(f)
    if len(bottom) != 1:
        raise ValueError(f"Ambiguous extruded end ({len(bottom)})")
    return BRepPrimAPI_MakePrism(
        bottom[0], gp_Vec(0, 0, travel + bounds(s)[5] - lo)
    ).Shape()


def load_parts(folder):
    receipt = json.loads((folder / "reopen.json").read_bytes())
    parts = {}
    for group, r in receipt["added"].items():
        path = folder / r["step"]
        if hashlib.sha256(path.read_bytes()).hexdigest() != r["sha256"]:
            raise ValueError(path)
        rd = STEPControl_Reader()
        rd.ReadFile(str(path))
        rd.TransferRoots()
        solids = list(base.subsolids(rd.OneShape()))
        if len(solids) != len(r["bodies"]):
            raise ValueError(group)
        for s, info in zip(solids, r["bodies"]):
            if abs(volume(s) - info["volumeMm3"]) > 0.01:
                raise ValueError(info["name"])
            parts[info["name"]] = (group, s)
    return receipt, parts


def run(folder):
    receipt, parts = load_parts(folder)
    original = [
        (name, s, bounds(s))
        for group, name, s in read_assembly(
            ROOT / "cad/swerve-captive/public-robot.step"
        )
        if group != "Demonstration CORAL"
    ]
    frame = [
        (k, s, bounds(s))
        for k, (g, s) in parts.items()
        if g == "Shallow cage jack frame"
    ]
    clashes = []
    comparisons = 0
    for name, (group, shape) in parts.items():
        static = group == "Shallow cage jack frame"
        v = shape
        if not static:
            v = sweep_z(shape, 328 + (14 if group == "Lower plate clamp" else 0))
        if group in ("Left cage shoe", "Right cage shoe"):
            b = list(bounds(v))
            b[1 if group.startswith("Right") else 4] += (
                -36 if group.startswith("Right") else 36
            )
            v = BRepPrimAPI_MakeBox(gp_Pnt(*b[:3]), gp_Pnt(*b[3:])).Shape()
        b = bounds(v)
        for key, s, bs in original + ([] if static else frame):
            comparisons += 1
            if box_distance(bs, b) > 0.001:
                continue
            vol = volume(BRepAlgoAPI_Common(v, s).Shape())
            if vol > 0.005:
                clashes.append(dict(moving=name, obstacle=key, sweptOverlapMm3=vol))
    total = {}
    for n, (group, s) in parts.items():
        if "drive and pawl" in n:
            continue
        density = (
            7850
            if any(
                k in n
                for k in (
                    "screw core",
                    "shoe",
                    "Bearing pad",
                    "travelling nut",
                    "Latch stem",
                    "dovetail",
                )
            )
            else 2850
        )
        total[n] = volume(s) * density * 1e-9
    material = sum(total.values())
    return dict(
        nativeSha256=receipt["sha256"],
        candidateSolids=len(parts),
        baselineSolids=len(original),
        comparisons=comparisons,
        materialInterferences=clashes,
        materialMassKg=material,
        bodyMassKg=total,
        additionalReceivingUpperKg={
            "two motor, reduction and passive locks": 1.60,
            "installed fasteners, wiring, switches, preload actuator and backing": 0.75,
        },
        accountedUpperIncrementKg=material + 2.35,
        scope="Union of vertical slide stroke and guide envelopes at empty stow. Shoe sweep conservatively bounds both axes. Named actuator packages are space reservations. Retainer fit and strength require an independent design.",
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("folder", type=Path)
    parser.add_argument("out", type=Path)
    args = parser.parse_args()
    args.out.write_bytes((json.dumps(run(args.folder), indent=2) + "\n").encode())
