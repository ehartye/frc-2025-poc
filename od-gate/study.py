"""Source-bound nominal, numerical and adverse pickup and placement checks."""
import argparse
from collections import Counter
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict,replace
import hashlib
import json
import math
from pathlib import Path
from motion import Case,HERE,ROOT,run

FACTORS={"length":(.295275,.307975),"diameter":(.1128,.1158),"mass":(.499,.816466266),
         "normal":(-.010,.010),"lateral":(-.010,.010),"yaw":(-3.,3.),"slice_y":(-.1,.1),
         "station_yaw":(-1.,1.),"target_error":(-.015,.015),"sector":(-.18,.18),
         "mu":(.35,.70),"damping":(.28,.50),"lag":(0.,.025),"close_time":(.35,.45),
         "inertia":(1e-6,7e-6),"travel":(.109,.115)}

def cases():
    base=Case(withdraw=0.)
    probes={"reference-A":{},"reference-B":{"sector":.18},"stationary-shelf":{"travel":0,"withdraw":0.},
            "gate-only-no-withdraw":{"withdraw":0.},"no-counterfeed":{"initial_surface":0.},"hood-fixed-low":{"hood_raise":0.},
            "hood-stuck-high":{"close_time":12.},"single-side-late":{"lag":1.},"gate-stalled":{"travel":0.},
            "low-traction":{"mu":.08},"early-gate":{"gate_delay":-.20},"reduced-power":{"drive_current":2.},
            "coarser":{"dt":.0005},"finest":{"dt":.000125},"near-lip":{"target_error":.10},
            "long-offset":{"length":.307975,"normal":.010,"lateral":.010,"yaw":4.,"slice_y":.10,"lag":.025},
            "short-offset":{"length":.295275,"normal":-.010,"lateral":-.010,"yaw":-4.,"slice_y":-.10,"sector":.18}}
    result=[(n,replace(base,**kw)) for n,kw in probes.items()]
    for i in range(64):
        vals={k:limits[(i&mask).bit_count()%2] for mask,(k,limits) in enumerate(FACTORS.items(),1)}
        vals["yaw"]=vals["station_yaw"]-vals["yaw"]
        result.append((f"grid-{i:02}",replace(base,**vals)))
    return result

def one(item):
    name,c=item
    result=run(c);result.pop("trace")
    r=c.diameter*.5;half=c.length*.5
    yaw=math.radians(c.yaw)
    ya=math.radians(c.station_yaw)
    end=abs(c.lateral)+half*math.cos(yaw)+r*abs(math.sin(yaw))
    open_gap=c.center+c.travel-c.belt_halfwidth-end
    chute_gap=.12199054775091216/2-r*math.cos(ya)-half*abs(math.sin(ya))
    bilateral_on_time=c.gate_duration+c.gate_delay+c.lag<=.335 and c.travel>.100
    candidate=(result["seatedS"] is not None and result["seatedS"]<1.1 and result["incompleteNipDuringCarryS"]<.001
               and result["outcome"]=="sustained-paired-target" and min(open_gap,chute_gap)>0 and bilateral_on_time)
    return {"id":name,"parameters":asdict(c),"conditionalSequence":candidate,"bilateralCommandWindow":bilateral_on_time,"openGateSideGapMm":1000*open_gap,
            "sourceSlotHalfGapMm":1000*chute_gap,**result}

if __name__=="__main__":
    p=argparse.ArgumentParser();p.add_argument("--workers",type=int,default=2);p.add_argument("--output",type=Path,required=True)
    a=p.parse_args()
    with ProcessPoolExecutor(a.workers) as pool:rows=list(pool.map(one,cases()))
    refs=["sourced-match/source-545.json","design/fixed-height/ports-065.json","cradle-receiver/study.json",
          "end-grip-length/witness.json","scripts/station_l1_mission.py","scripts/receiver_contact.py"]
    data={"sourceSha256":{q:hashlib.sha256((ROOT/q).read_bytes()).hexdigest() for q in refs},"rows":rows,
          "fieldSha256":__import__("motion").mission.TARGET["step_sha256"]}
    a.output.write_text(json.dumps(data,indent=2,allow_nan=False)+"\n",encoding="utf-8",newline="\n")
