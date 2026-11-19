"""Compare removable passive face solids with the selected shop receiver and its translating inventory."""
import argparse
import json
from pathlib import Path
import sys

import numpy as np
from OCP.BRepAlgoAPI import BRepAlgoAPI_Fuse
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace, BRepBuilderAPI_MakePolygon
from OCP.BRepPrimAPI import BRepPrimAPI_MakePrism
from OCP.gp import gp_Pnt, gp_Vec

from guide import Guide, HERE, ROOT, gap, x_center

sys.path.insert(0,str(ROOT/"selected-optics"))
import scenes


def plate(side, shift=0., thickness=.8, short=False, narrow=False):
    """Extrude each joined facet in X. Relative stroke covers every translation of a rigid body."""
    g=Guide(throat_mm=-.3 if short else .3, x_width_mm=40 if narrow else 80, x_offset_mm=20 if narrow else 0)
    def point(z, outer):
        # The swept fixed guide, in a moving body's initial frame, reaches X - shift.
        x=1000*x_center(z/1000)+g.x_offset_mm-g.x_width_mm/2+min(0.,-shift)
        y=side*(150.8125+1000*gap(z/1000,g)+thickness*outer)
        return gp_Pnt(x,y,z)
    solids=[]
    for lo,hi in ((625,770),(770,850 if short else 900)):
        p=BRepBuilderAPI_MakePolygon()
        for z,outer in ((lo,0),(hi,0),(hi,1),(lo,1)):
            p.Add(point(z,outer))
        p.Close()
        f=BRepBuilderAPI_MakeFace(p.Wire()).Face()
        solids.append(BRepPrimAPI_MakePrism(f,gp_Vec(g.x_width_mm+abs(shift),0,0)).Shape())
    return BRepAlgoAPI_Fuse(*solids).Shape()


def run(assets, lamp, short=False, narrow=False):
    model=scenes.load_assembly(ROOT/"selected-match/clearance.py")
    cad=model.cad
    if narrow:
        sys.path.insert(0,str(ROOT/"integrated-slide"))
        from render import select
        a,b=select(model,assets,lamp)
    else:
        a,b=scenes.selected_poses(model,assets,lamp)
    key_ids=[{"id":"guide-positive-y","group":"guide"},{"id":"guide-negative-y","group":"guide"}]
    step=HERE/("narrow-face.step" if narrow else "short-face-profile.step" if short else "face-profile.step")
    cad.write_named(step,[plate(1,thickness=3.8 if short else .8,short=short,narrow=narrow),plate(-1,thickness=3.8 if short else .8,short=short,narrow=narrow)],key_ids,{"guide":"#E7AA42"})
    shapes=cad.read_named(step,key_ids)
    zero={k:cad.bounds(v) for k,v in a.items()}
    end={k:cad.bounds(v) for k,v in b.items()}
    swept={}
    hits=[]
    unresolved=[]
    for desc,side in zip(key_ids,(1,-1)):
        guide=shapes[desc["id"]];gb=np.array(cad.bounds(guide))
        for k,s in a.items():
            low=np.array(zero[k]);high=np.array(end[k])
            union=np.r_[np.minimum(low[:3],high[:3]),np.maximum(low[3:],high[3:])]
            if not cad.occupied(union,gb):continue
            change=high-low
            dx=round((change[0]+change[3])/2,5)
            if max(abs(change-[dx,0,0,dx,0,0]))>.001:
                unresolved.append([side,k]);continue
            if (side,dx) not in swept:swept[(side,dx)]=plate(side,dx,thickness=3.8 if short else .8,short=short,narrow=narrow)
            volume=cad.intersection(s,swept[(side,dx)])
            if volume>.001:
                witnesses=[]
                for stroke in range(0,371,5):
                    stage=cad.translated(s,[stroke/370*dx,0,0])
                    if cad.occupied(cad.bounds(stage),gb):
                        v=cad.intersection(guide,stage)
                        if v>.001:witnesses.append([stroke,v])
                hits.append({"guide":desc["id"],"robot":k,"relativeSweepMm3":volume,
                             "samplesWithInterference":len(witnesses),"first":witnesses[:1],
                             "largest":max(witnesses,key=lambda r:r[1]) if witnesses else None})
    vol=sum(cad.volume(s)[0] for s in shapes.values())
    result={"source":{"selectedKit":model.digest(ROOT/"selected-match/receiver-clearance-kit.step"),
                      "opticalRegistration":model.digest(ROOT/"selected-optics/mount.json")},
            "poseCount":len(a),"stepSha256":model.digest(step),"volumesMm3":{k:cad.volume(s)[0] for k,s in shapes.items()},
            "boundsMm":{k:cad.bounds(s) for k,s in shapes.items()},"interferences":hits,
            "unresolvedVariableGeometry":unresolved,"facePairMassGAt2700KgM3":vol*.00270 if not short else None,
            "roofTopTubeStowedBoundsMm":zero["roof-top-tube"]}
    (HERE/("fit-narrow.json" if narrow else "fit-short.json" if short else "fit.json")).write_text(json.dumps(result,indent=2)+"\n",encoding="utf8",newline="\n")
    print("poses",len(a),"total face volume mm3",vol,"hits",[(r["robot"],r["largest"]) for r in hits],flush=True)


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("assets",type=Path);p.add_argument("lamp",type=Path);p.add_argument("--short", action="store_true");p.add_argument("--narrow",action="store_true");args=p.parse_args();run(args.assets,args.lamp,args.short or args.narrow,args.narrow)
