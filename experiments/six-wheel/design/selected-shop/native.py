"""Screen the 36 wheel screws through a full rotation on the issued competition robot."""
import argparse
import hashlib
import json
import math
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE.parent / "installed-procurement"))
import clamps
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut

cad = clamps.cad


def ring(x, sign, start, end, small, big):
    def tube(rad):
        return cad.brep(clamps.cylinder(x, sign*start, clamps.Z, [0, sign, 0], rad, end-start))
    return BRepAlgoAPI_Cut(tube(big), tube(small)).Shape()


def gap(a, b):
    query = BRepExtrema_DistShapeShape(a, b)
    if not query.IsDone():
        raise ValueError("Native distance did not converge")
    return query.Value()


def lower(a, b):
    return sum(max(0, a[i]-b[i+3], b[i]-a[i+3])**2 for i in range(3))**0.5


def closest(a, boxes, group):
    limits = cad.bounds(a)
    best, body = float("inf"), None
    for bound, key in sorted((lower(limits,b), k) for k,b in boxes.items() if k in group):
        if bound > best:
            break
        observed = gap(a, group[key])
        if observed < best:
            best, body = observed, key
    return {"gap_mm":round(best,5), "body":body}


def screen(path):
    expected = json.loads((ROOT/"camera-datum-shop/native-place.json").read_bytes())
    data = path.read_bytes()
    sha = hashlib.sha256(data).hexdigest()
    if sha != expected["step_sha256"]:
        raise ValueError("Expected the issued 603-part red placement")
    names = {s.decode() for s in re.findall(rb"\bPRODUCT\(\s*'([^']*)'", data)}-{"SOLID","COMPOUND"}
    if len(names)!=expected["independent_root_solids"]:
        raise ValueError("Unexpected native inventory")
    parts = cad.read_named(path,[{"id":v} for v in names])
    for old in clamps.REPLACED:
        del parts[old]
    alternatives = {k:v for sign in (-1,1) for station in clamps.STATIONS
                    for k,v in clamps.parts(sign,station).items()}
    if set(parts)&alternatives.keys() or len(alternatives)!=86:
        raise ValueError("Incorrect positive carrier assembly")
    # Qualify the replacement on this new full pose too; the prior four-pose fit is separate.
    boxes = {k:cad.bounds(v) for k,v in parts.items()}
    clashes = {}
    for key,v in alternatives.items():
        bounds = cad.bounds(v)
        for k,b in boxes.items():
            if cad.occupied(bounds,b):
                volume = cad.intersection(v,parts[k])
                if volume>0.01:
                    clashes[f"{key}/{k}"]=volume
    parts.update(alternatives)
    boxes.update({k:cad.bounds(v) for k,v in alternatives.items()})
    result = {}
    for tag,x in clamps.STATIONS.items():
        for lr,sign in (("L",1),("R",-1)):
            key=tag+"-"+lr
            group={k:v for k,v in parts.items() if k not in {"wheel-"+key,"hub-"+key}}
            # The screw seats on the outer hub flange, .188 inch beyond the outer wheel face.
            land=302.7 + .188*25.4
            r=23.8125
            for label,a,b,rad in (("shank",land-31.8,land,2.45),("receiving-head",land,land+4,5.5),
                                  ("6mm-head-control",land,land+6,6),
                                  ("overheight-control",land,land+16,12)):
                swept=ring(x,sign,a,b,r-rad,r+rad)
                observation=closest(swept,boxes,group)
                if observation["gap_mm"]==0:
                    observation["swept_common_mm3"]=round(cad.intersection(swept,group[observation["body"]]),5)
                if label=="6mm-head-control":
                    # Align one actual head radially with the lower, rearward clamp screw.
                    angle=math.atan2(-17,-19)
                    head=cad.brep(clamps.cylinder(x+r*math.cos(angle),sign*a,clamps.Z+r*math.sin(angle),
                                                  [0,sign,0],rad,b-a))
                    observation["one_head_at_deg"]=round(math.degrees(angle)%360,5)
                    observation["one_head_common_mm3"]=round(cad.intersection(head,group[observation["body"]]),5)
                result[key+"/"+label]=observation
    return {"source_sha256":sha,"original_parts":len(names),"carrier_substitution":[6,86],
            "retained_body_intersections_mm3":clashes,"wheel_sweeps":result,
            "head_land_abs_y_mm":land,"tip_beyond_inner_wheel_face_mm":277.3-(land-31.8),
            "method":"Exact annular swept envelopes across 360 degrees, other than the intentionally threaded wheel and drilled hub. Units mm."}


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("step",type=Path)
    args=parser.parse_args()
    (HERE/"wheel-fit.json").write_text(json.dumps(screen(args.step),indent=2)+"\n",encoding="utf8",newline="\n")
