"""Run fixed design candidates, independent misses, and a deterministic pairwise disturbance screen."""
import argparse
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict, replace
import hashlib
import json
import math

from model import Case, HERE, ROOT, run

BASE = Case(face="backstop",dump_angle=18.,rolling=.04,withdrawal=.15)
# Each independent binary factor has a distinct nonzero six-bit mask. Over all 64 rows every
# pair of individual factor signs occurs sixteen times. This is not a full corner enclosure.
FACTORS = {
    "length": (.301625-.00635,.301625+.00635), "diameter": (.1128,.1158),
    "wall": (.0055,.0066), "mass": (.498951607,.816466266), "normal": (-.010,.010),
    "lateral": (-.010,.010), "chassisYaw": (-3.,3.), "station_pipe_yaw": (-1.,1.),
    "rail_lean": (-1.,1.), "rail_height": (-.0015,.0015), "rail_offset_y": (-.002,.002),
    "dump_angle": (16.,20.), "target_gap": (-.015,.015), "ramp_mu": (.15,.40),
    "rail_mu": (.20,.45), "rolling": (0.,.10), "damping": (.24,.62),
    "rail_slice": (-.100,.100),"sector": (-.18,.18)
}


def challenges():
    out={}
    for key,config in {
        "flat":Case(face="flat"),"V-low-loss":Case(rolling=0.,withdrawal=.15),
        "V-medium-loss":Case(rolling=.04,withdrawal=.15),"V-high-loss":Case(rolling=.10,withdrawal=.15),
        "backstop-A":BASE,"backstop-B":replace(BASE,sector=.18),"no-withdrawal":replace(BASE,withdrawal=0.),
        "overtilt":replace(BASE,dump_angle=40.),"under-tilt-high-loss":replace(BASE,dump_angle=14.,rolling=.10),
        "zero-rolling-loss":replace(BASE,rolling=0.),"high-rolling-loss":replace(BASE,rolling=.10),
        "fine":replace(BASE,dt=.00025),"finer":replace(BASE,dt=.000125),
        "out-of-chute":replace(BASE,station_pipe_yaw=3.,yaw=3.)
    }.items():out[key]=config
    return out


def cases():
    jobs=list(challenges().items())
    for i in range(64):
        params={k:limits[(i&mask).bit_count()%2] for mask,(k,limits) in enumerate(FACTORS.items(),start=1)}
        params["yaw"]=params["station_pipe_yaw"]-params.pop("chassisYaw")
        jobs.append((f"factor-{i:02d}",replace(BASE,**params)))
    return jobs


def one(pair):
    name,cfg=pair
    result=run(cfg)
    result.pop("trace")
    geometric=all(v>=0 for v in result["support"].values())
    cond=result["seatAtExistingTime"] is True and geometric and result["outcome"]=="sustained-paired-target"
    return {"id":name,"parameters":asdict(cfg),"conditionalSequence":cond,**result}


def study(workers):
    with ProcessPoolExecutor(max_workers=workers) as pool:rows=list(pool.map(one,cases()))
    geometry=json.loads((ROOT/"station-relief/geometry.json").read_bytes())
    window=next(r for r in geometry["cases"]["revised-pocket"] if r["lateral_mm"]==.8)
    prior_long=window["max_belt_excursion_mm"]+.00635*500
    return {"schema":"coral-body-supported-cradle/1","inputs":{p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in (
        ROOT/"station-relief/geometry.json",ROOT/"station-relief/capture.json",ROOT/"station-supply/transfer.json",
        ROOT/"sourced-match/source-545.json",ROOT/"scripts/station_l1_mission.py",ROOT/"scripts/receiver_contact.py")},
        "priorLongCutTravelMm":prior_long,"priorAvailableMm":1.4,"cutLengthQualification":False,
        "disturbanceDesign":"64 binary rows, all combinations for every independent pair, ends include shortest and longest pieces",
        "rows":rows}


if __name__=="__main__":
    from pathlib import Path
    a=argparse.ArgumentParser(description=__doc__);a.add_argument("--output",type=Path,required=True);a.add_argument("--workers",type=int,default=2)
    p=a.parse_args(); p.output.write_text(json.dumps(study(p.workers),indent=2,allow_nan=False)+"\n",encoding="utf-8",newline="\n")
