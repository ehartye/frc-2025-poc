"""Reopen the registered station occurrence and resolve its central entry corridor in robotward millimetres."""
import argparse
import hashlib
import json
import math
import sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from mission.check_branch_exclusion import cad,direct_children,name,FIRST_SHA
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.gp import gp_Trsf

def parts(step):
    if hashlib.sha256(step.read_bytes()).hexdigest()!=FIRST_SHA:
        raise ValueError("Unexpected FIRST field revision")
    doc,table=cad.xcaf_doc()
    reader=cad.STEPCAFControl_Reader()
    if reader.ReadFile(str(step))!=cad.IFSelect_RetDone or not reader.Transfer(doc):
        raise ValueError("Cannot reopen source STEP")
    roots=cad.Sequence_TDF_Label()
    table.GetFreeShapes(roots)
    root=roots.Value(1)
    station,ref=next((o,r) for o,r in direct_children(table,root) if name(o)=="NAUO34" and name(r)=="GE-25000_Red 3 Corner")
    reg=json.loads((ROOT/"mission/field-registration.json").read_bytes())["redStation"]
    hx,_,hz=reg["humanSideNativeBasis"]
    ox,_,oz=[v*25.4 for v in reg["nativeOriginIn"]]
    xf=gp_Trsf()
    xf.SetValues(-hx,0,-hz,ox*hx+oz*hz,-hz,0,hx,ox*hz-oz*hx,0,1,0,0)
    out={}
    for occ,child in direct_children(table,ref):
        loc=table.GetLocation_s(station).Multiplied(table.GetLocation_s(occ))
        shape=BRepBuilderAPI_Transform(table.GetShape_s(child).Moved(loc),xf,False).Shape()
        out[name(occ)+"/"+name(child)]=shape
    return out

if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("step",type=Path)
    for key,shape in parts(parser.parse_args().step).items():
        print(key,tuple(round(x,3) for x in cad.bounds(shape)))
