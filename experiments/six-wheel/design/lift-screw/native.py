"""Compare alternative screw-axis reserves against four registered red assemblies."""
import argparse
import json
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent/"lift-arrest"))
from compare import cad, contact, cylinder, digest
from relief import relieved_plate

HEIGHT = {"place":560,"transport":670,"stowed":940,"receive":940}
LOCATIONS = [(-86,264),(-142,255),(-155,255),(-146,300),(-60,298)]
# Package dimensioned from the 3/4-6 steel screw and two 89520 bronze sleeves.
SCREW_R, NUT_R, NUT_LEN = 9.525, 15.875, 38.1


def envelope(x, y, height):
    center = height - 140
    return {"shaft":cylinder(x,y,365,SCREW_R,520),
            "workingNut":cylinder(x,y,center-NUT_LEN/2,NUT_R,NUT_LEN),
            "safetyNut":cylinder(x,y,center+NUT_LEN/2+2,NUT_R,NUT_LEN)}


def review(pose, folder):
    frames = json.loads((folder/"part-frames.json").read_bytes())
    pin = digest(folder/(pose+".step"))
    if pin != json.loads((folder/"review.json").read_bytes())[pose]["export_sha256"]:
        raise ValueError("Native assembly differs from its registration")
    shapes = cad.read_named(folder/(pose+".step"),[{"id":k} for k in frames])
    height=HEIGHT[pose]
    for tag,side in (("L",1),("R",-1)):
        k=f"backup-{tag}-carriage-plate"
        shapes[k]=relieved_plate(shapes[k],side,height)
    bounds = {k:cad.bounds(v) for k,v in shapes.items()}
    locations = {}
    for x,y in LOCATIONS:
        checks = {}
        for side in (-1,1):
            bodies = envelope(x,side*y,height)
            # Native screw and nut intentionally overlap each other as thread reserves.
            checks[str(side)] = {k:contact(v,shapes,bounds) for k,v in bodies.items()}
            # Address collisions *between* operating heights for every stationary solid.
            swept = cylinder(x,side*y,560-140-NUT_LEN/2,NUT_R,380+2*NUT_LEN+2)
            stationary = {k:v for k,v in shapes.items() if frames[k]=="base"}
            checks[str(side)]["sweptNutAgainstFixed"]=contact(swept,stationary,bounds)
        locations[f"{x},{y}"]=checks
    return {"pose":pose,"source":pin,"bodyCount":len(shapes),"locationsMm":locations}


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("pose",choices=HEIGHT)
    p.add_argument("output",type=Path)
    a=p.parse_args()
    red=HERE.parent/"competition-package"/"shop-duplex"/"red"
    a.output.write_text(json.dumps(review(a.pose,red),indent=2)+"\n",encoding="utf-8",newline="\n")
