"""Check the selected receiver material against the installed red reef at attained scoring poses."""
import argparse
import json
import math
from pathlib import Path
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.gp import gp_Trsf

from clearance import ROOT, cad, digest, dock, mod, poses, read, wire_boxes, native_box


def transform(bodies, sector, error):
    x,y,angle = error
    c,s = math.cos(math.radians(angle)), math.sin(math.radians(angle))
    transform = gp_Trsf()
    transform.SetValues(c,-s,0,3453.050980729168-490+x,0,0,1,0,-s,-c,0,-sector-y)
    return {k:BRepBuilderAPI_Transform(body,transform,False).Shape() for k,body in bodies.items()}


def hits(mounted, reef):
    obstacles = {k:cad.bounds(body) for k,body in reef.items()}
    material = []
    for name, body in mounted.items():
        bounds = cad.bounds(body)
        for key, other in obstacles.items():
            if dock.separation(bounds,other) > .001:
                continue
            volume = cad.intersection(body,reef[key])
            if volume > .001:
                material.append({"body":name,"field":key,"commonMm3":volume})
    return sorted(material,key=lambda row:-row["commonMm3"])


def run(artifacts,lamp,field):
    if digest(field) != read("relieved-match/native-register.json")["fieldSha256"]:
        raise ValueError("Unexpected official field revision")
    assembly = poses(artifacts,lamp)
    fixed, slide = wire_boxes()
    structure = mod("finite_reef_structure",ROOT/"moving-observer/clearance.py").reef_parts(field)
    nav = read("relieved-match/navigation.json")["cases"]
    cases = {}
    for idx,stroke in enumerate((0,370)):
        bodies = dict(assembly[idx])
        reservations = {k:native_box(v) for k,v in fixed.items()}
        reservations.update({k:cad.translated(native_box(v),[stroke,0,0]) for k,v in slide.items()})
        for piece,sector in (("coral-A",-180),("coral-B",220)):
            route = next(r for r in nav if r["pipe"]==piece and r["mode"]=="nominal")
            error = route["finalBodyErrorMmDeg"]
            exact = hits(transform(bodies,sector,error),structure)
            extra = hits(transform(reservations,sector,error),structure)
            cases[f"{piece}-{stroke}"] = {"errorMmDeg":error,"native":exact,"reservations":extra}
            print(piece,stroke,"hits",len(exact),len(extra),exact[:3],flush=True)
    return {"fieldSha256":digest(field),"cases":cases}


if __name__ == "__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    for arg in ("artifacts","lamp","field","output"):
        parser.add_argument(arg,type=Path)
    args=parser.parse_args()
    result=run(args.artifacts,args.lamp,args.field)
    args.output.write_text(json.dumps(result,indent=2,allow_nan=False)+"\n",encoding="utf-8",newline="\n")
