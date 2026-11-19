"""Independently intersect actual finite pipe material at adverse sampled seats."""
import argparse
import itertools
import json
import math
from dataclasses import replace
from pathlib import Path
from capture import CANDIDATE, HALF, R, SHELF, patches, simulate
from design import HERE, registered_robot, cad, receiver_drives, sha
from transfer import tube


def witness(dx,side,yaw,preload,robot):
    trace=[]
    cfg=replace(CANDIDATE,preload=preload)
    simulate(dx,side,yaw,cfg=cfg,path=trace)
    nearest=min(range(len(trace)),key=lambda i:trace[i][1])
    keys=[f"grip-{s}-{part}" for s in ("L","R") for part in ("rear-riser","drive-22T")]
    hits=[]
    for offset in (-50,-30,-10,0,10,30,50):
        t,x,y,z,h=trace[max(0,min(nearest+offset,len(trace)-1))]
        angle=math.radians(h)
        piece=tube(x,z,-h,y)
        b=cad.bounds(piece)
        for key in keys:
            sign=1 if key.startswith("grip-L-") else -1
            shape=robot[key]
            shift=0.
            if receiver_drives.translates_with_grip(key):
                # Unloaded inboard reach -0.40 mm. Surface y of each contacting annular strip.
                points=patches((x-270)/1000-sign*HALF*math.sin(angle),z/1000-SHELF,cfg)
                shift=max([-.4]+[sign*y+1000*(HALF*(math.cos(angle)-1)+sign*xr*math.sin(angle)) for xr,_,_ in points])
                shape=cad.translated(shape,(0,sign*shift,0))
            if not cad.occupied(b,cad.bounds(shape)): continue
            vol=cad.intersection(piece,shape)
            if vol>.001:
                hits.append({"body":key,"timeS":t,"poseMmDeg":[x,y,z,h],"jawOutwardMm":shift,"materialMm3":vol})
    return {"dxMm":dx,"sideMm":side,"yawDeg":yaw,"preloadN":preload,"nativeHits":hits}


if __name__=="__main__":
    a=argparse.ArgumentParser(description=__doc__);a.add_argument("--output",type=Path,required=True);p=a.parse_args()
    robot=registered_robot()
    rows=[witness(x,y,a,preload,robot) for x,y,a,preload in itertools.product((-5,5),(.8,),(-.25,.25),(10.,11.5))]
    p.output.write_text(json.dumps({"sourceSha256":sha(HERE/"capture.json"),"cases":rows},indent=2)+"\n",encoding="utf-8",newline="\n")
