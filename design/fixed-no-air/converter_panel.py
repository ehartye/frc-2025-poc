"""Find the fixed-base supply panel's clearance on both issued robot exchanges."""
import argparse
import hashlib
import json
import math
from pathlib import Path

from service import cad, distance
from OCP.BRepPrimAPI import BRepPrimAPI_MakeSphere
from OCP.gp import gp_Pnt

HERE = Path(__file__).resolve().parent
REPLACED = {"observer-host-5v-reserve", "observer-camera-5v-reserve"}
PANEL = [-190, -202, 164, 135, 85, 156]
BODY_X = [-184, -140, -96]
FILES = {"pickup": "db52b764d308c59df9b6d3ed4099c2ba9fbb05bd12b48cfbec791e012d5a8603",
         "placement": "c6ba9c988380ee16715f9ad83d8638229c868a2a5e885d74903179b47300e02b"}


def digest(file):
    return hashlib.sha256(file.read_bytes()).hexdigest()


def box(values):
    return cad.brep(["box", values])


def lower_bound(a, b):
    return math.sqrt(sum(max(a[i]-b[i+3], b[i]-a[i+3], 0)**2 for i in range(3)))


def nearest(shape, parts, bounds, exclude=()):
    target = cad.bounds(shape)
    distance_mm = math.inf
    hits = []
    for bound, name in sorted((lower_bound(target, location), k) for k, location in bounds.items() if k not in exclude):
        if bound > distance_mm + .001:
            break
        gap = distance(shape, parts[name])
        if gap < distance_mm - .001:
            hits = []
        if gap <= distance_mm + .001:
            hits.append(name)
        distance_mm = min(distance_mm, gap)
    return {"mm": round(distance_mm, 3), "bodies": sorted(hits)}


def segment(a, b):
    length = math.dist(a,b)
    direction = [(q-p)/length for p,q in zip(a,b)]
    return cad.brep(["cyl", [*a,*direction,4,length]])


def check(folder):
    manifest = json.loads((HERE / "whole-robot.json").read_bytes())
    result = {}
    clearance = box(PANEL)
    mounts = {"leftRear": [-184, -197], "leftFront": [-64, -197],
              "rightRear": [-184, -139], "rightFront": [-64, -139]}
    route = [(-124,-171,277),(-124,-171,295),(-332,-171,295),
             (-334,-45,282),(-334,-45,918),(-334,-63,938),(-327,-63,972)]
    for name, pin in FILES.items():
        step = folder / (name + ".step")
        receipt = json.loads(step.with_suffix(".json").read_bytes())
        if digest(step) != pin or receipt["native_manifest_sha256"] != digest(HERE / "whole-robot.json"):
            raise ValueError("STEP differs from reviewed fixed-low exchange")
        row = manifest["cases"][receipt["pose"]]
        ids = set(row["identical_brep_sha256"]) | set(row["new_or_changed"])
        if len(ids) != 593:
            raise ValueError("unexpected number of native parts")
        parts = cad.read_named(step, [{"id": n} for n in ids])
        for key in REPLACED:
            del parts[key]
        bounds = {k: cad.bounds(v) for k,v in parts.items()}
        feet = {}
        for key,(x,y) in mounts.items():
            support = "observer-pi-and-converter-backplate" if key == "leftRear" else "deck-main"
            z = bounds[support][5]
            foot = cad.brep(["cyl", [x,y,z,0,0,1,4,164-z]])
            tip = cad.brep(["cyl", [x,y,z-.5,0,0,1,4,.5]])
            fraction = cad.intersection(tip,parts[support]) / (math.pi*4**2*.5)
            feet[key] = {"bearingSupport":support, "supportGapMm":round(distance(foot,parts[support]),3),
                         "bearingFraction":round(fraction,5),
                         "otherClearance":nearest(foot,parts,bounds,{support}),
                         "baseMm":[x,y,z]}
        corridors = []
        for i,(a,b) in enumerate(zip(route,route[1:])):
            uses = set()
            if i >= 2:
                uses.add("observer-cable-trunk-reserve")
            if i == 5:
                uses.add("observer-usb-service-reserve")
            leg = segment(a,b)
            corner = BRepPrimAPI_MakeSphere(gp_Pnt(*b),4).Shape()
            gaps = [nearest(shape,parts,bounds,uses) for shape in (leg,corner)]
            corridors.append({"from":a,"to":b,"lengthMm":round(math.dist(a,b),2),"destinations":sorted(uses),
                              "otherClearance":min(gaps,key=lambda row:row["mm"])})
        direct = segment((-334,-45,282),(-334,-45,962))
        clash = cad.intersection(direct,parts["observer-slotted-cradle-weldment"])
        if clash <= .001 or any(f["bearingFraction"] < .999 for f in feet.values()):
            raise ValueError("support or original cable interference changed")
        result[name] = {"sha256":pin, "strokeMm":row["stroke_mm"], "panelClearance":nearest(clearance,parts,bounds),
                        "directLeadCradleInterferenceMm3":round(clash,3),
                        "fixedCameraLeadSegments":corridors,
                        "feet":feet, "retainedParts":len(parts)}
    return {"reservedEnvelopeOriginAndSizeMm":PANEL, "removed":sorted(REPLACED),
            "converterBoundsOriginAndSizeMm":[[x,-193,186,36,59,91] for x in BODY_X], "poses":result}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("folder", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    args.output.write_text(json.dumps(check(args.folder),indent=2)+"\n",encoding="utf-8",newline="\n")
