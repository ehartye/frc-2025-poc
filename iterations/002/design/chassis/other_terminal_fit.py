"""Place the measured port cartridge at the other three rail datums and challenge material and access."""
import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder
from OCP.TopLoc import TopLoc_Location
from OCP.gp import gp_Ax2, gp_Dir, gp_Pnt, gp_Trsf


HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
Z = 94.63336287672979


def transform(angle, shift_x):
    angle = math.radians(angle)
    c, s = round(math.cos(angle)), round(math.sin(angle))
    result = gp_Trsf()
    result.SetValues(c, -s, 0, shift_x, s, c, 0, 0, 0, 0, 1, 0)
    return TopLoc_Location(result)


def cylinder(start, direction, radius, length):
    return BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(*start), gp_Dir(*direction)), radius, length).Shape()


def run(package):
    sys.path.insert(0, str(package / "scripts"))
    from component_shape import volume
    from fusion_authority import register_path, verify, digest
    from mechanical_motion import bounds, common, distance, bearing_area
    from showcase_contacts import load_solids, box_gap

    register = register_path(ROOT)
    line = register.parent / "issue-004/lineage.json"
    authority = verify(ROOT, register, line)
    original, sheets = load_solids(line.with_name("assembly.step"))
    if sheets or len(original) != 504:
        raise ValueError("Unexpected active lower chassis")
    cold_file = register.parent / "private/port-504-native.json"
    native = json.loads(cold_file.read_bytes())
    if native["verifiedAuthority"] != authority:
        raise ValueError("Mismatched cold observations")
    bboxes = {name: np.asarray(bounds(s)) for name, s in original.items()}
    source = {}
    for body in native["coldCartridge"]:
        matches = sorted((max(abs(b - body["boundsMm"])), name) for name, b in bboxes.items())
        if matches[0][0] > .1 or matches[1][0] < .5:
            raise ValueError(f"Ambiguous source {body['path']}")
        source[matches[0][1]] = original[matches[0][1]]
    if len(source) != 16:
        raise ValueError("Lost a port cartridge article")
    rails = {name: np.array([(lo + hi) / 2 for lo, hi in zip(b[:3], b[3:])])
             for name, b in bboxes.items() if "socket rail [" in name}
    if len(rails) != 4:
        raise ValueError("Unresolved socket-rail population")
    tools = [cylinder((x, 328.9375, Z + .2), (0, 0, -1), 6.4, 12.2) for x in (-65, -35, 35, 65)]
    tools += [cylinder((x, 318.1, Z - 3.175 - 7.1), (0, 1, 0), 2.6, 3.5) for x in (-50, 50)]
    targets = [("starboard", 180, 0, (0, -330.975)), ("front", -90, 5, (335.975, 0)),
               ("rear", 90, -5, (-335.975, 0))]
    assembly = dict(original)
    result = dict(authority=authority, inputNativeReportSha256=digest(cold_file), targets={})
    for name, deg, dx, xy in targets:
        host = next(k for k, v in rails.items() if np.linalg.norm(v[:2] - xy) < .01)
        pose = transform(deg, dx)
        material = assembly[host]
        volume_before = volume(material)
        for cutter in tools:
            cut = BRepAlgoAPI_Cut(material, cutter.Moved(pose))
            if not cut.IsDone() or cut.Shape().IsNull():
                raise ValueError(f"Invalid tooling at {name}")
            material = cut.Shape()
        if not BRepCheck_Analyzer(material).IsValid():
            raise ValueError(f"Open cut material on {host}")
        removed = volume_before - volume(material)
        expected = 0 if name == "starboard" else native["machined"]["expectedRemovedMm3"]
        if abs(removed - expected) > .02:
            raise ValueError(f"Commissioned rail holes differed at {name}: {removed} vs {expected}")
        assembly[host] = material
        seen = [(k, s, bounds(s)) for k, s in assembly.items()]
        copies = {k: s.Moved(pose) for k, s in source.items()}
        detail, blocked = [], []
        clear, tight = 0, math.inf
        for article, solid in copies.items():
            box = bounds(solid)
            for other, fixed, limit in seen:
                lower = box_gap(box, limit)
                if lower > .001:
                    clear += 1
                    tight = min(tight, lower)
                    continue
                interference = common(solid, fixed)
                gap = distance(solid, fixed)
                pair = dict(article=article, other=other, gapMm=gap, commonMm3=interference)
                if gap < .001 and other == host:
                    axis = (0, 0, -1)
                    pair["horizontalSeatMm2"] = bearing_area(solid, fixed, axis)
                detail.append(pair)
                if interference > .005:
                    blocked.append(pair)
        result["targets"][name] = dict(host=host, rotationDeg=deg, shiftXmm=dx,
                                      additionalRailRemovalMm3=removed, matches=detail, blocked=blocked,
                                      broadPhaseClear=clear, smallestBroadClearBoundMm=tight)
        assembly.update({name + "/" + k: shape for k, shape in copies.items()})
        print(name, len(assembly), len(blocked), round(removed, 4), flush=True)
        (HERE / "other-terminal-fit.json").write_text(json.dumps(result, indent=2) + "\n")
    if len(assembly) != 552:
        raise ValueError("Incomplete four-terminal assembly")
    result["totalProposedSolids"] = len(assembly)
    result["scope"] = "Three proposed rigid copies on the exact parked 504 chassis. Curved wall stock and receiver fits use the native port construction. Module service paths, support stiffness, full perimeter, operating loads and the three new native attachments still require verification."
    (HERE / "other-terminal-fit.json").write_text(json.dumps(result, indent=2) + "\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("package", type=Path)
    run(parser.parse_args().package)
