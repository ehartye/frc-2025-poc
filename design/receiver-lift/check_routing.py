"""Check proposed winch routes against the original named robot solids (millimetres)."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
BASE = HERE.parent / "station-l1"
sys.path.insert(0, str(BASE))
from build_assembly import brep, bounds, occupied, read_named, volume, xcaf_doc
from mechanical import box, cylinder, cut
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPCAFControl import STEPCAFControl_Writer
from OCP.STEPControl import STEPControl_AsIs, STEPControl_Reader
from build_assembly import solid_count
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name

Y, Z = [0,1,0], [0,0,1]


def ring_y(x, y, z, radius, bore, width):
    return cut(cylinder(x,y,z,Y,radius,width), cylinder(x,y-1,z,Y,bore,width+2))


def routing(height):
    shaft_x, low_z, high_z = -92, 330, 975
    rope_r = 3.175/2
    lower_r = 19.05/2 + rope_r
    top_r = 20.5
    sheave_x = shaft_x - lower_r + top_r
    return_x = sheave_x + top_r
    shaft_r = 6.35 / (3**0.5/2)  # round envelope enclosing a 1/2-in across-flats hex shaft
    items = {
        "common-shaft-round-envelope":cylinder(shaft_x,-270,low_z,Y,shaft_r,540),
        "motor-body":cylinder(-45,-95,low_z,Y,30,58.25),
        "sport-ratchet-and-mount-reserve":box(-75,-36.75,low_z-30,60,112,60),
        "mount-riser":box(-55,66,154,20,8,146),
        "drive-pulley-reserve":cylinder(-45,75.25,low_z,Y,15,8),
        "driven-pulley-reserve":ring_y(shaft_x,75.25,low_z,15,shaft_r,8)
    }
    # Straight sections of the 1:1 transfer. Circles bound the two pitch turns.
    for side in [-1,1]:
        items["transfer-reach-"+str(side)] = box(shaft_x,76,low_z+side*16-1,-45-shaft_x,6.5,2)
    for side in [-1,1]:
        y = side*252
        key = lambda name: name+("-R" if side < 0 else "-L")
        for label,x,z,core,width,flange in [
            ("drum",shaft_x,low_z,lower_r-rope_r,26.924,20.7),
            ("sheave",sheave_x,high_z,top_r-rope_r,7,22)]:
            bore = shaft_r if label == "drum" else 6.35
            items[key(label+"-core")] = ring_y(x,y-width/2,z,core,bore,width)
            for i,edge in enumerate([y-width/2-2,y+width/2]):
                items[key(label+"-flange-"+str(i))] = ring_y(x,edge,z,flange,bore,2)
        items[key("upper-pin")] = cylinder(sheave_x,y-17,high_z,Y,6.35,34)
        items[key("running-line")] = cylinder(shaft_x-lower_r,y,low_z,Z,rope_r,high_z-low_z)
        anchor_z = height-136
        items[key("return-line")] = cylinder(return_x,y,anchor_z,Z,rope_r,high_z-anchor_z)
        # Anchor plate touches the end of the existing crossbar. The line exits its top face.
        tab_y = 240 if side > 0 else -258
        items[key("anchor-tab")] = box(-97,tab_y,anchor_z-6,38,18,6)
        # Two support arms bear on the existing mast's outer face.
        arm_y = 235 if side > 0 else -245
        items[key("sheave-support")] = cut(box(sheave_x-10,arm_y,high_z-9,79,10,18),
            cylinder(sheave_x,arm_y-1,high_z,Y,6.36,12))
        bearing_y = 216 if side > 0 else -224
        items[key("bearing-envelope")] = ring_y(shaft_x,bearing_y,low_z,14.285,shaft_r,8)
        items[key("bearing-arm")] = cut(box(shaft_x-18,bearing_y,low_z-18,-70-(shaft_x-18),8,36),
            cylinder(shaft_x,bearing_y-1,low_z,Y,14.29,10))
    return {k:brep(v) for k,v in items.items()}


def clashes(first, second):
    bounds2 = {k:bounds(s) for k,s in second.items()}
    hits = {}
    for k,s in first.items():
        b = bounds(s)
        for name,t in second.items():
            if occupied(b,bounds2[name]):
                common = BRepAlgoAPI_Common(s,t)
                if not common.IsDone(): raise ValueError("Boolean failed for "+k+" and "+name)
                shared = volume(common.Shape())[0]
                if shared > 0.001: hits[k+" / "+name] = round(shared,6)
    return hits


def write_step(path, shapes):
    doc,table=xcaf_doc()
    for name, shape in shapes.items():
        label=table.AddShape(shape,False)
        TDataStd_Name.Set_s(label,TCollection_ExtendedString(name))
    writer=STEPCAFControl_Writer()
    if not writer.Transfer(doc,STEPControl_AsIs) or writer.Write(str(path))!=IFSelect_RetDone:
        raise ValueError("STEP export failed")
    reader=STEPControl_Reader()
    if reader.ReadFile(str(path)) != IFSelect_RetDone or not reader.TransferRoots():
        raise ValueError("Cannot independently reopen merged robot export")
    restored=reader.OneShape()
    if solid_count(restored) != len(shapes):
        raise ValueError("Merged robot STEP lost one or more solid bodies")
    expected=sum(volume(shape)[0] for shape in shapes.values())
    if abs(volume(restored)[0]-expected) > max(0.05, expected*1e-8):
        raise ValueError("Merged robot STEP material volume changed")


def audit(output):
    cfg=json.loads((BASE/"assembly.json").read_text())
    check=json.loads((BASE/"review.json").read_text())["poses"]
    report={}
    for pose,desc in cfg["poses"].items():
        source=BASE/(pose+".step")
        digest=hashlib.sha256(source.read_bytes()).hexdigest()
        if digest != check[pose]["stepSha256"]: raise ValueError("Wrong base STEP")
        base=read_named(source)
        del base["lift-drive"]
        layer=routing(desc["height"])
        obs={"base_step_sha256":digest, "existing_bodies":len(base), "added_routing_bodies":len(layer),
             "external_interferences_mm3":clashes(layer,base)}
        internal = {}
        names = list(layer)
        for i, name in enumerate(names):
            internal.update(clashes({name:layer[name]},{n:layer[n] for n in names[i+1:]}))
        obs["internal_interferences_mm3"] = internal
        if pose=="stowed":
            service = {s["id"]:brep(box(*s["box"])) for s in cfg["service"]}
            obs["service_interferences_mm3"]=clashes(layer,service)
        if output:
            write_step(output/(pose+".step"), base|layer)
        if any(obs.get(key) for key in ["internal_interferences_mm3", "external_interferences_mm3", "service_interferences_mm3"]):
            raise ValueError("Routing conflict in "+pose+": "+str(obs))
        report[pose]=obs
    return report


if __name__ == "__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--step-dir",type=Path)
    args=parser.parse_args()
    if args.step_dir: args.step_dir.mkdir(parents=True,exist_ok=True)
    print(json.dumps(audit(args.step_dir),indent=2))
