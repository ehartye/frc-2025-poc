"""Enclose the continuous interpolant through the simulated entry-to-seat paths in robot coordinates."""
import argparse
import bisect
import itertools
import json
import math
from pathlib import Path

from design import HERE, ROOT, INWARD, registered_robot, cad, receiver_drives, sha
from capture import simulate, HALF, R, CANDIDATE
from dataclasses import replace
from entry_envelope import convex_hull
from dock_tolerance import separation
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace, BRepBuilderAPI_MakePolygon
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
from OCP.BRepPrimAPI import BRepPrimAPI_MakePrism
from OCP.gp import gp_Pnt,gp_Vec

TIMES = (0,.008,.016,.024,.032,.040,.048,.056,.064,.080,.100,.125,.175,.24,.32,.48,.75,1.2)


def prism(tips,half):
    polygon=BRepBuilderAPI_MakePolygon()
    for x,z in tips:
        polygon.Add(gp_Pnt(x,-half,z))
    polygon.Close()
    return BRepPrimAPI_MakePrism(BRepBuilderAPI_MakeFace(polygon.Wire()).Face(),gp_Vec(0,2*half,0)).Shape()


def sweep(robot, lateral, heading,preload):
    bins=[[] for _ in TIMES[:-1]]
    max_gap_time=0.
    reaches=(.30,.325,.35,.375,.40) if preload==11.5 else (.40,)
    for dx,dy,a,reach in itertools.product((-5,5),(-lateral,lateral),(-heading,heading),reaches):
        path=[]
        run=simulate(dx,dy,a,path=path,cfg=replace(CANDIDATE,preload=preload),reach=reach)
        if run["status"]!="conditional-seat": raise ValueError(run)
        # First sample is one time step after lip; the analytical flight study covers launch.
        for i,(t,x,y,z,h) in enumerate(path):
            k=max(0,min(len(bins)-1,bisect.bisect_right(TIMES,t)-1))
            row=(x,y,z,abs(math.radians(h)))
            bins[k].append(row)
            if i and t-path[i-1][0]>.00050001:raise ValueError("Coarse capture interval")
            if i:
                max_gap_time=max(max_gap_time,t-path[i-1][0])
                if path[i-1][0]<TIMES[k]:
                    _,ox,oy,oz,oh=path[i-1]
                    bins[k].append((ox,oy,oz,abs(math.radians(oh))))
                    bins[k-1].append(row)
    normal_boxes={k:cad.bounds(v) for k,v in robot.items()}
    skip={"receiver-R6-ported","grip-L-420-3M-15","grip-R-420-3M-15"}
    results=[]
    radius=1000*R/math.cos(math.pi/64)
    circle=[(radius*math.cos((2*i+1)*math.pi/64),radius*math.sin((2*i+1)*math.pi/64)) for i in range(64)]
    for t0,t1,rows in zip(TIMES,TIMES[1:],bins):
        centers=convex_hull([(x+b*1000*HALF*math.sin(h),z) for x,y,z,h in rows for b in (-1,1)])
        # Reserve 0.02 mm for curvature between coupled-model samples. Report as an assumption;
        # do not claim the single-time-step numerical integration encloses the actual physical tube.
        ends=convex_hull([(x+u+sx*.02,z+v+sz*.02) for x,z in centers for u,v in circle for sx in (-1,1) for sz in (-1,1)])
        half=1000*HALF+max(abs(y)+1000*R*math.sin(h) for x,y,z,h in rows)+.025
        # Use the largest heading and lateral magnitude independently on the solid interpolation.
        half=max(half,1000*HALF+max(abs(r[1]) for r in rows)+1000*R*math.sin(max(r[3] for r in rows))+.025)
        tools={}
        for travel in (0,INWARD):
            shape=prism(ends,half+travel)
            tools[travel]=(shape,cad.bounds(shape))
        closest=(3.,None); hits={}; intended={}
        for k,shape in robot.items():
            if k in skip:continue
            tool,box=tools[INWARD if receiver_drives.translates_with_grip(k) or k.endswith("-spring") else 0]
            if separation(box,normal_boxes[k])>closest[0]+.001:continue
            distance=BRepExtrema_DistShapeShape(tool,shape)
            if not distance.IsDone():raise ValueError(k)
            d=distance.Value()
            if d<1e-6:
                vol=cad.intersection(tool,shape)
                if vol>.001:
                    if k.startswith("observer-moving-pad-"):intended[k]=vol
                    else:hits[k]=vol
            if not k.startswith("observer-moving-pad-") and d<closest[0]:closest=(d,k)
        results.append({"timeS":[t0,t1],"hardwareLowerMm":closest[0],"closestBody":closest[1],
                        "envelopeOverlapMm3":hits,"pressurePadEnvelopeMm3":intended})
    return {"lateralMm":lateral,"headingDeg":heading,"preloadN":preload,"reachesMm":reaches,"cases":8*len(reaches),"dtMaxS":max_gap_time,"bins":results}


if __name__=="__main__":
    a=argparse.ArgumentParser(description=__doc__)
    a.add_argument("--output",type=Path,required=True)
    p=a.parse_args()
    robot=registered_robot()
    g=json.loads((HERE/"geometry.json").read_bytes())
    for source,ids in ((ROOT/"station-supply/receiver-stops.step",[f"grip-{s}-inboard-hardstop" for s in ("L","R")]),
                       (HERE/"cheeks.step",g["stock"])):
        robot.update(cad.read_named(source,[{"id":k} for k in ids]))
    shim=json.loads((HERE/"shims.json").read_bytes())
    shim_ids=list(shim["parts"])+list(shim["stopChecks"])+[k+"-seat-shim" for k in shim["parts"]]
    shimmed=robot|cad.read_named(HERE/"shims.step",[{"id":k} for k in shim_ids])
    result={"schema":"station-capture-solid-envelope/1","geometrySha256":sha(HERE/"geometry.json"),
            "contactSha256":sha(HERE/"capture.json"),"shimSha256":sha(HERE/"shims.json"),"cases":[sweep(solids,y,a,preload) for solids,preload in ((robot,10),(shimmed,11.5)) for y,a in ((.8,.25),(1.,.15))]}
    p.output.write_text(json.dumps(result,indent=2)+"\n",encoding="utf-8",newline="\n")
