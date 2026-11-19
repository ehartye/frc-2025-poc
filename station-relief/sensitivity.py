"""Challenge the coupled pickup with interior states, contact and numerical perturbations."""
import argparse
import hashlib
import itertools
import json
from dataclasses import replace
from pathlib import Path
from capture import CANDIDATE, HERE, simulate


def run():
    cfg=replace(CANDIDATE,preload=11.5)
    out={}
    for label,side,ang in (("wider-heading",.8,.25),("wider-side",1.,.15)):
        rows=[simulate(dx,y,heading,cfg=cfg) for dx,y,heading in itertools.product((-5,0,5),(-side,0,side),(-ang,0,ang))]
        out[label]={"count":len(rows),"statuses":{s:sum(r["status"]==s for r in rows) for s in {r["status"] for r in rows}},
                    "maximumTravelMm":max(r["largestRequiredOutwardMm"] for r in rows),
                    "rearwardMm":min(r["minimumUMm"] for r in rows)}
    challenges={}
    for label,config,step,reach in (("fine",cfg,.00025,.4),("coarse",cfg,.001,.4),("more-inward-reach",cfg,.0005,.6),
        ("finer-rim-quadrature",replace(cfg,slices=11),.0005,.4),("less-belt-traction",replace(cfg,belt_mu=.30),.0005,.4),
        ("low-ramp-traction",replace(cfg,ramp_mu=.15),.0005,.4),("very-low-grip",replace(cfg,belt_mu=.08,preload=2.),.0005,.4)):
        rows=[simulate(dx,.8,h,dt=step,cfg=config,reach=reach) for dx,h in itertools.product((-5,5),(-.25,.25))]
        challenges[label]=[{"dxMm":r["dxMm"],"yawDeg":r["headingDeg"],"outcome":r["status"],
                            "reachMm":reach,"rearwardMm":r["minimumUMm"],"requestedTravelMm":r["largestRequiredOutwardMm"],
                            "seatMm":r["terminalUMm"]} for r in rows]
    stops=[]
    for reach in (.30,.325,.35,.375,.40,.42,.44,.50,.60):
        rows=[simulate(dx,side,h,cfg=cfg,reach=reach) for dx,(side,angle),sign in itertools.product(
            (-5,5),((.8,.25),(1.,.15)),(-1,1)) for h in (sign*angle,)]
        stops.append({"inwardReachMm":reach,"trialCount":len(rows),"maximumOutwardMm":max(r["largestRequiredOutwardMm"] for r in rows),
                      "allSeated":all(r["status"]=="conditional-seat" for r in rows),"rearwardMm":min(r["minimumUMm"] for r in rows)})
    return {"inwardStops":stops,"sourceSha256":hashlib.sha256((HERE/"capture.json").read_bytes()).hexdigest(),"interiorGrids":out,"challenges":challenges}

if __name__=="__main__":
    a=argparse.ArgumentParser(description=__doc__);a.add_argument("--output",type=Path,required=True);p=a.parse_args()
    p.output.write_text(json.dumps(run(),indent=2)+"\n",encoding="utf-8",newline="\n")
