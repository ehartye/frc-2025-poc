"""Screen the actual finite pipe against the neutral installed bodies during ejection."""

import argparse
import hashlib
import json
import sys
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
ENG = HERE.parents[2] / "cad/assembly-engineering"
sys.path.insert(0, str(ENG))
# The original engineering modules use the name clearance for the shallow-cage CAD utility.
sys.path.insert(0, str(ENG.parent / "shallow-cage"))
from installed_audit import reopen, sha
from shop_sources import issue_shapes
from attachments import common
from OCP.gp import gp_Ax2, gp_Dir, gp_Pnt
from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
from OCP.BRepExtrema import BRepExtrema_DistShapeShape


def read(p):
    return json.loads(p.read_bytes())


def cylinder(p, a):
    line = gp_Ax2(gp_Pnt(*(p - 150.8125 * a)), gp_Dir(*a))
    return BRepAlgoAPI_Cut(
        BRepPrimAPI_MakeCylinder(line, 57.15, 301.625).Shape(),
        BRepPrimAPI_MakeCylinder(line, 50.8, 301.625).Shape(),
    ).Shape()


def shapes():
    folder = ENG / "receiver-service/issue"
    reg = read(folder / "interfaces.json")
    native = {
        b["identity"]: b
        for b in read(folder / "engineering-reopen.json")["bodies"]
        if b["instance"] is not None
    }
    bodies = {k: s for k, *_, s in reopen()} | issue_shapes(folder, reg)
    groups = {}
    for k, s in bodies.items():
        if k in reg["fixedHeight"]["removed"]:
            continue
        r = native[
            hashlib.sha256(("team-1780-engineering-v1/" + k).encode()).hexdigest()
        ]
        g = r["occurrence"].split(":")[0]
        g = (
            g
            if g
            in (
                "Middle slide",
                "Outer slide",
                "Left capture yoke",
                "Right capture yoke",
            )
            else "fixed"
        )
        groups.setdefault(g, []).append((k, s, r["boundsMm"]))
    if sum(map(len, groups.values())) != len(native):
        raise ValueError("Incomplete installed robot")
    return groups


def local(row, q0, group):
    p, a = row[2:5] * 1000, row[5:8]
    if group == "fixed":
        return p, a
    is_middle = group == "Middle slide"
    offset = 2 if is_middle else 4
    dz, phi = (row[20:26] - q0)[offset : offset + 2]
    c, s = np.cos(phi), np.sin(phi)
    rotation = np.array([[c, 0, -s], [0, 1, 0], [s, 0, c]])
    center = (
        p - [1000 * row[1] * (0.5 if is_middle else 1), 0, 467 + 1000 * dz]
    ) @ rotation + [0, 0, 467]
    if group in ("Left capture yoke", "Right capture yoke"):
        center[1] += row[14] * (1 if group == "Left capture yoke" else -1)
    return center, a @ rotation


def run(trace, groups, step):
    result = read(trace.with_suffix(".json"))
    if sha(trace) != result["trace"]:
        raise ValueError("Different tube trace")
    rack = trace.parent / "rack-337mm-31.25us.npz"
    if sha(rack) != read(rack.with_suffix(".json"))["traceSha256"]:
        raise ValueError("Wrong parked frame")
    q0 = np.load(rack)["q"][0, :6]
    samples = np.load(trace)
    rows = samples[(samples[:, 0] >= 3.32) & (samples[:, 0] < 4.91)][::step]
    material = {}
    calls = 0
    ids = set()
    for group, inventory in groups.items():
        bounds = np.array([v[2] for v in inventory])
        for row in rows:
            p, a = local(row, q0, group)
            extreme = 150.8125 * np.abs(a) + 57.15 * np.sqrt(np.maximum(0, 1 - a * a))
            hits = np.flatnonzero(
                np.all(
                    (p + extreme >= bounds[:, :3]) & (p - extreme <= bounds[:, 3:]),
                    axis=1,
                )
            )
            if len(hits) == 0:
                continue
            tube = cylinder(p, a)
            for i in hits:
                key, solid, _ = inventory[i]
                calls += 1
                ids.add(key)
                d = BRepExtrema_DistShapeShape(tube, solid)
                if not d.IsDone():
                    raise ValueError(key)
                if d.Value() > 1e-5:
                    continue
                volume = common(tube, solid)
                if volume > 0.01:
                    data = material.setdefault(
                        key, dict(firstS=float(row[0]), lastS=float(row[0]), peakMm3=0)
                    )
                    data["lastS"] = float(row[0])
                    if volume > data["peakMm3"]:
                        data.update(peakMm3=volume, peakS=float(row[0]))
    return dict(
        trace=sha(trace),
        sourceState=result["sourceState"],
        native=sha(ENG / "receiver-service/issue/engineering-reopen.json"),
        samplingPeriodMs=5 * step,
        sampleRangeS=[float(rows[0, 0]), float(rows[-1, 0])],
        sampled=len(rows),
        bodies=sum(map(len, groups.values())),
        narrowPhasePairs=calls,
        boundingBoxCandidates=sorted(ids),
        positiveUndeformedMaterialMm3=material,
        scope="Surface geometry in the solved stage pitch/heave frame. Neutral CAD omits variable distributed face deflection, measured pad compression, gear backlash and cable flex. Positive overlap needs a component-specific compliance/contact model. Clear discrete samples alone do not bound continuous clearance.",
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("trace", nargs="+", type=Path)
    p.add_argument("--stride", type=int, default=5)
    p.add_argument(
        "--front-supports",
        action="store_true",
        help="Check only the four diagnosed grip-foot and riser solids",
    )
    a = p.parse_args()
    groups = shapes()
    if a.front_supports:
        selected = {
            f"grip-{side}-front-{part}"
            for side in ("L", "R")
            for part in ("support-foot", "riser")
        }
        groups = {"Outer slide": [v for v in groups["Outer slide"] if v[0] in selected]}
        if len(groups["Outer slide"]) != 4:
            raise ValueError("Grip support identity changed")
    for t in a.trace:
        out = run(t, groups, a.stride)
        t.with_suffix(
            ".supports.json" if a.front_supports else ".installed.json"
        ).write_bytes((json.dumps(out, indent=2) + "\n").encode())
        print(out["positiveUndeformedMaterialMm3"], flush=True)
