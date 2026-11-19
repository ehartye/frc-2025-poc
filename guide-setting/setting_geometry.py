"""Machine two repeatable guide datums per mast and verify the removable setting tools."""
import argparse
import math
import sys
from pathlib import Path

from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.BRepPrimAPI import BRepPrimAPI_MakeCone
from OCP.gp import gp_Ax2, gp_Dir, gp_Pnt

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT / "guide-mounts"))
from parts import MAST_BOLTS, SIDES, STATIONS, carriage, j, xcyl

BORE_RADIUS = 2.01
PIN_RADIUS = 2.
GAUGE_Y = 219.
GAUGE_RISE = 5.


def slot(y, z):
    lower = xcyl(-21, y, z-.30, BORE_RADIUS, 8)
    upper = xcyl(-21, y, z+.30, BORE_RADIUS, 8)
    middle = j.box(-21, -13, y-BORE_RADIUS, y+BORE_RADIUS, z-.30, z+.30)
    return j.fuse(j.fuse(lower, upper), middle)


def setting_pin(y, z):
    # The straight pilot spans both walls. A short lead only guides its entry.
    axis = gp_Ax2(gp_Pnt(-25.5, y, z), gp_Dir(1, 0, 0))
    lead = BRepPrimAPI_MakeCone(axis, 1.5, PIN_RADIUS, .5).Shape()
    shank = xcyl(-25, y, z, PIN_RADIUS, 11)
    grip = xcyl(-14, y, z, 3., 12)
    return j.fuse(j.fuse(lead, shank), grip)


def build_kit(original):
    parts, changes = {}, {}
    for side, sign in SIDES:
        prefix = f"guide-{side}-"
        mast_name = prefix+"machined-mast"
        mast = original[mast_name]
        for station in STATIONS:
            y, z = sign*GAUGE_Y, station+GAUGE_RISE
            key = prefix+str(station)+"-mount-arm"
            before = j.cad.volume(mast)[0]
            mast = j.cut(mast, xcyl(-26, y, z, BORE_RADIUS, 7))
            removed = before - j.cad.volume(mast)[0]
            bore = xcyl(-21, y, z, BORE_RADIUS, 8) if station == STATIONS[0] else slot(y, z)
            root = original[key]
            parts[key] = j.cut(root, bore)
            parts[prefix+str(station)+"-setting-pin"] = setting_pin(y, z)
            changes[key] = {"wallMm": removed/(math.pi*BORE_RADIUS**2),
                            "removedMastMm3": removed,
                            "removedRootMm3": j.cad.volume(root)[0]-j.cad.volume(parts[key])[0]}
        parts[mast_name] = mast
    return parts, changes


def run(registry, dest):
    source = ROOT/"guide-mounts"
    fit = j.read(source/"fit.json")
    if j.digest(registry) != fit["registrySha256"]:
        raise ValueError("Unexpected robot inventory")
    old = carriage.selected(source,"guide-mounts.step","fit.json","stepSha256","volumesMm3")
    scene, edges, _ = carriage.scene(registry, ROOT/"fixed-hoist")
    braces = carriage.selected(ROOT/"carriage-support","carriage-support.step","fit.json","stepSha256","volumesMm3")
    scene.update({k:v for k,v in braces.items() if not k.startswith("support-machined-mast-")})
    scene.update(old)
    parts, changes = build_kit(old)
    dest.mkdir(parents=True,exist_ok=True)
    step = dest/"setting.step"
    j.write_step(step, parts)
    j.finalize_step(step)
    actual = j.cad.read_named(step,[{"id":k} for k in parts])
    for name, shape in actual.items():
        if (not BRepCheck_Analyzer(shape).IsValid() or j.cad.solid_count(shape)!=1
                or abs(j.cad.volume(shape)[0]-j.cad.volume(parts[name])[0])>.003):
            raise ValueError("Reopened geometry changed for "+name)
    scene.update(actual)
    bounds = {k:j.cad.bounds(v) for k,v in scene.items()}

    def contacts(body, omit):
        box = j.cad.bounds(body)
        hits = {}
        for k, other in scene.items():
            if k not in omit and j.lower_bound(box,bounds[k])<.001:
                volume = j.cad.intersection(body,other)
                if volume>.001:
                    hits[k] = round(volume,4)
        cable = {}
        for k,edge,b,r in edges:
            if j.lower_bound(box,b)<r and (distance:=j.distance(body,edge)-r)<-.001:
                cable[k]=min(cable.get(k,0.),distance)
        return {"materialOverlapMm3":hits,"cableIntrusionMm":cable}

    tests = {}
    for side,sign in SIDES:
        for station in STATIONS:
            tag = f"guide-{side}-{station}-"
            key = tag+"mount-arm"
            name = tag+"setting-pin"
            pin, root = actual[name], actual[key]
            y,z = sign*GAUGE_Y, station+GAUGE_RISE
            tests[name+" seated"] = contacts(pin,{name})
            withdrawal = j.fuse(xcyl(-25.5,y,z,2,63.5), xcyl(-14,y,z,3,63.5))
            tests[name+" extraction"] = contacts(withdrawal,{name})
            row = changes[key]
            row["outboardShiftOverlapMm3"] = j.cad.intersection(j.cad.translated(root,(0,sign*.315,0)),pin)
            row["verticalShiftOverlapMm3"] = j.cad.intersection(j.cad.translated(root,(0,0,.25)),pin)
            if (abs(row["wallMm"]-2)>.001 or row["outboardShiftOverlapMm3"]<1
                    or ((row["verticalShiftOverlapMm3"]>1) != (station==STATIONS[0]))):
                raise ValueError("Datum check failed for "+key)
            row["frontSockets"] = []
            for index,(bolt_y,dz) in enumerate(MAST_BOLTS):
                own = {tag+f"root-{index}-screw"}
                # Envelope includes a 12 mm OD driver and ±0.25 mm positioning allowance.
                tool = xcyl(-14,sign*bolt_y,station+dz,6.25,80)
                tests[tag+f"driver {index} with pins"] = contacts(tool,own)
                old_pin = setting_pin(sign*222.5,station)
                row["frontSockets"].append({
                    "workingClearanceMm":j.distance(pin,tool),
                    "originalLocationOverlapMm3":j.cad.intersection(old_pin,tool)})
    for key,test in tests.items():
        if test["materialOverlapMm3"] or test["cableIntrusionMm"]:
            raise ValueError((key,test))
    for k,row in changes.items():
        if min(s["workingClearanceMm"] for s in row["frontSockets"])<1 or not all(
                s["originalLocationOverlapMm3"]>0 for s in row["frontSockets"]):
            raise ValueError("Driver comparison failed for "+k)
    installed = [k for k in parts if not k.endswith("setting-pin")]
    pins = [k for k in parts if k.endswith("setting-pin")]
    j.write(dest/"fit.json",{"guideSha256":fit["stepSha256"],"registrySha256":j.digest(registry),
        "stepSha256":j.digest(step),"changedInstalled":installed,"pitOnly":pins,
        "nominalBoreMm":4.02,"nominalPilotMm":4.,"upperSlotHalfLengthMm":.30,
        "boreCentersMm":[[GAUGE_Y,z+GAUGE_RISE] for z in STATIONS],
        "volumesMm3":{k:round(j.cad.volume(v)[0],5) for k,v in actual.items()},
        "interfaces":changes,"tools":tests,"measuredFacePositions":None})
    print("Reopened",len(actual),"articles and checked",len(tests),"tool positions and sweeps")


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("registry",type=Path)
    p.add_argument("--output",type=Path,default=HERE)
    args=p.parse_args()
    run(args.registry,args.output)
