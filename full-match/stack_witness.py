"""Reopen the physical stowed robot and measure a native intersection with staged game pieces."""
import argparse
import json
import math
from pathlib import Path

from match import COORDS, ROOT, corrected
from mission.heavy_field_clock import revised_motion
from mission.review_combined_departure import digest
import sys
sys.path.insert(0,str(ROOT/"design/station-l1"))
import build_assembly as cad
from OCP.BRepPrimAPI import BRepPrimAPI_MakeSphere, BRepPrimAPI_MakeCylinder
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
from OCP.gp import gp_Ax2,gp_Pnt,gp_Dir


def native(poses):
    meta=json.loads((poses/"review.json").read_text())
    step=poses/"stowed.step"
    if digest(step)!=meta["stowed"]["export_sha256"]:
        raise ValueError("Stowed STEP digest mismatch")
    parts=json.loads((poses/"part-frames.json").read_text())
    shapes=cad.read_named(step,[{"id":k} for k in parts])
    boxes={k:cad.bounds(s) for k,s in shapes.items()}
    # Pick the centerline sample nearest the one upper red stack, without looking
    # at the native material result. Recompute all three occupied solids in robot frame.
    f=revised_motion()
    route=f.routes(-.18)[0]
    x,y=(q*f.IN for q in COORDS[-1])
    witnesses=[]
    for t in [j*.02 for j in range(int(route.duration/.02)+1)]:
        (a,b,h),_=route.at(t)
        local=f.rot((x-a,y-b),-h)
        witnesses.append((math.hypot(*local),t,local,h))
    _,t,(lx,ly),heading=min(witnesses)
    lx*=1000
    ly*=1000
    ball_r=206.375
    middle=gp_Pnt(lx,ly,114.3+ball_r)
    shell=BRepAlgoAPI_Cut(BRepPrimAPI_MakeSphere(middle,ball_r).Shape(),
                          BRepPrimAPI_MakeSphere(middle,ball_r-1).Shape()).Shape()
    tools={"red-upper-algae":shell}
    for axis,h in (("x",0),("y",math.pi/2)):
        dx,dy=math.cos(h-heading),math.sin(h-heading)
        ax=gp_Ax2(gp_Pnt(lx-dx*150.8125,ly-dy*150.8125,57.15),gp_Dir(dx,dy,0))
        pipe=BRepAlgoAPI_Cut(BRepPrimAPI_MakeCylinder(ax,57.15,301.625).Shape(),
                             BRepPrimAPI_MakeCylinder(ax,50.8,301.625).Shape()).Shape()
        tools["red-upper-coral-axis-"+axis]=pipe
    hits=[]
    for obstacle,tool in tools.items():
        limits=cad.bounds(tool)
        for k,b in boxes.items():
            if cad.occupied(limits,b):
                volume=cad.intersection(shapes[k],tool)
                if volume>.05:
                    hits.append({"moving":k,"obstacle":obstacle,"timeS":t,"volumeMm3":round(volume,5)})
    return {"schema":"field-start-native-negative/1","stowed_sha256":digest(step),"timeS":t,"heading_rad":heading,
            "relative_stack_xy_mm":[lx,ly],"ball_shell_mm":1,"ball_radius_mm":ball_r,
            "pipe_od_mm":114.3,"pipe_id_mm":101.6,"pipe_length_mm":301.625,
            "basis":"One stationary centered stack; pipe X/Y are separate orientation alternatives. Ball shell depth is a model assumption.",
            "hits":hits}


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--poses",required=True,type=Path);p.add_argument("--output",required=True,type=Path)
    a=p.parse_args()
    a.output.write_text(json.dumps(native(a.poses),indent=2)+"\n",encoding="utf-8",newline="\n")
