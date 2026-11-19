"""Check end-screw fit and the center axle's positive outward restraint."""
import argparse
import json
from pathlib import Path
import sys

from OCP.BRepAlgoAPI import BRepAlgoAPI_Fuse

HERE = Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent/"moving-harness"))
from continuous_screen import SHOP_STEP, read_shop
from converter_panel import cad, distance, lower_bound, nearest


def along_y(x, start, sign, radius, length):
    return cad.brep(["cyl",[x,start,76.2,0,sign,0,radius,length]])


def check(artifacts):
    parts,_ = read_shop(artifacts)
    boxes = {k:cad.bounds(v) for k,v in parts.items()}
    locations = {}
    travel = {}
    for pos,x in (("front",247.65),("center",0),("rear",-247.65)):
        for side,sy in (("L",1),("R",-1)):
            axle = f"axle-{pos}-{side}"
            for face,sgn in (("outer",1),("inner",-1)):
                if pos=="center" and face=="inner":
                    continue
                bearing = f"bearing-{pos}-{face}-{side}"
                y = sy*(327 if face=="outer" else 233.4)
                head = along_y(x,y,sy*sgn,8.5,3.8)
                witness = nearest(head,parts,boxes,{axle,bearing})
                locations[f"{pos}-{side}-{face}"] = {
                    "headUndersideToBearingAxialFaceMm":distance(head,parts[bearing]),
                    "closestOtherToHeadEnvelope":witness}
            if pos=="center":
                # The registered bar is a constant extruded rounded hex. Overlapping
                # copies give the exact sweep through the full 10 mm socket engagement.
                swept = BRepAlgoAPI_Fuse(parts[axle],cad.translated(parts[axle],[0,sy*10,0]))
                if not swept.IsDone():
                    raise ValueError("Unable to build axial extraction sweep")
                head = along_y(x,sy*327,sy,8.5,13.8)
                overlaps = {}
                for part,shape in (("shaft",swept.Shape()),("head",head)):
                    bb = cad.bounds(shape)
                    for key,value in parts.items():
                        if key!=axle and lower_bound(bb,boxes[key])<.001:
                            common = cad.intersection(shape,value)
                            if common>.001:
                                overlaps[f"{part}/{key}"] = common
                travel[side] = {"outwardTranslationMm":10,"sweptMaterialOverlapMm3":overlaps}
    return {"shopSha256":SHOP_STEP,"screw":"REV-21-2805 V2, nominal 17 x 3.8 mm head envelope",
            "locations":locations,"centerOutwardExtraction":travel}


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("artifacts",type=Path)
    parser.add_argument("output",type=Path)
    args=parser.parse_args()
    args.output.parent.mkdir(exist_ok=True,parents=True)
    args.output.write_text(json.dumps(check(args.artifacts),indent=2)+"\n",encoding="utf-8",newline="\n")
