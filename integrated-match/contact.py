"""Propagate the two attained pipe states with actual HAL rack displacement through assessment."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import sys
from unittest.mock import patch
import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
sys.path.insert(0,str(ROOT/"low-observation"))
from source import inputs,handoff


def run(jdk,cache):
    plans={s["id"]:s for s in json.loads((ROOT/"fixed-height-match/placements.json").read_bytes())["placements"]}
    events=list(csv.DictReader((HERE/"events.csv").open()))
    paths=json.loads((HERE/"hal.json").read_bytes())["motionSha256"]
    src=inputs(jdk)
    ledgers={}
    fingerprints={}
    sim=handoff.spin.settle
    original_gap=sim.shelf_gap
    for mode in ("nominal","lateB","single"):
        ledgers[mode]={}
        for pipe,s in plans.items():
            rows={r["event"]:r for r in events if r["scenario"]==mode and r["piece"]==pipe}
            if "release" not in rows:continue
            key=pipe+("-withdraw" if "withdraw" in rows else "-held")
            path=cache/f"path-{mode}-{pipe}.csv"
            when=int(rows["withdraw"]["wall_ms"])/1000 if "withdraw" in rows else math.inf
            tab=np.array([0.])
            if when!=math.inf:
                digest=hashlib.sha256(path.read_bytes()).hexdigest()
                if paths[path.name]!=digest:raise ValueError("unregistered HAL trace")
                key+=digest+str(when)
                tab=np.loadtxt(path,skiprows=1,delimiter=",",usecols=[1])
            if key not in fingerprints:
                ref=src[pipe]
                wall=18+s["lip"]+ref["post"]["start_from_station_s"]
                if abs(wall-18-s["impact"])>1e-7:raise ValueError("Contact source clock changed")
                step=ref["settle_kwargs"]["dt"]
                kw=dict(ref["settle_kwargs"],withdrawal="fixed",duration=math.ceil((156-wall)/step)*step)
                count=0; minimum=math.inf
                half=57.65*math.cos(math.radians(1))+151.0625*math.sin(math.radians(1))
                def measured(x,z,front,top):
                    nonlocal count,minimum
                    moment=wall+count*step;count+=1
                    elapsed=(moment-when)*1000
                    if elapsed<=0:travel=0.
                    elif elapsed>=len(tab)-1:travel=tab[-1]
                    else:
                        i=int(elapsed);travel=tab[i]+(elapsed-i)*(tab[i+1]-tab[i])
                    remaining=370-1000*travel
                    bound=max(425,165+remaining,160+.5*remaining)
                    minimum=min(minimum,490+1000*x-half-bound-1)
                    return original_gap(x,z,front-travel,top)
                with patch.object(sim,"shelf_gap",measured):
                    out=sim.continue_contact(**kw)
                final=out["last_xz_vxvz_spin"][:2]
                ok=out["stop"]=="sustained-paired-floor-and-lip"
                if ok and np.max(np.abs(np.array(final)-s["finalXZ"]))>1e-7:raise ValueError("Final pipe/branch datum changed")
                fingerprints[key]={"targetWallS":wall,"stop":out["stop"],"lastObservedWallS":wall+out["elapsed_s"],
                        "minShelfGapMm":out["min_shelf_clearance_mm"],"wholeFrontLowerMm":minimum,
                        "stillPairedAtEndS":out["ending_still_double_contact_s"],"finalXZ":final,
                        "firstLipAtWallS":None if out["first_lip_after_s"] is None else wall+out["first_lip_after_s"],
                        "branchLowerMm":s["finalBranchLowerMm"] if ok else None,"checkedSteps":count,
                        "settleDtS":step,"admitted":ok and "stow" in rows and minimum>0}
            ledgers[mode][pipe]=fingerprints[key]
    (HERE/"contact.json").write_text(json.dumps(ledgers,indent=2,allow_nan=False)+"\n",encoding="utf-8",newline="\n")


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("java_home",type=Path);p.add_argument("cache",type=Path)
    a=p.parse_args();run(a.java_home.resolve(),a.cache.resolve())
