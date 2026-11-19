"""Apply runtime permissions to the six-robot programs and the season scoring oracles."""
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
sys.path.insert(0,str(ROOT/"fixed-height-match"))
import rehearse as field
BASE_SCENARIO=field.scenario
def read(p):return json.loads(p.read_bytes())


def fieldcase(mode,contact,events):
    plans=read(ROOT/"fixed-height-match/placements.json")["placements"]
    f,auto,team,_=BASE_SCENARIO(plans,inboard=True)
    focal=field.traffic.Program(2,"red"); focal.initial=focal.pose=auto.end;team["red2"]=focal
    updates=[e for e in events if e["scenario"]==mode]
    delivered=[]
    def wait(target,label):
        if target<focal.clock-1e-8:raise ValueError("noncausal "+label)
        focal.append(target-focal.clock,label)
    for s in plans:
        name=s["id"]
        e={r["event"]:int(r["wall_ms"])/1000-18 for r in updates if r["piece"]==name}
        if not e:continue
        if focal.clock>s["release"]-1.1+1e-7:raise ValueError("Unavailable dock/stock")
        if abs(s["release"]-e["feed"])>.001:raise ValueError("changed feed")
        _,loaded,empty=f.routes(s["sector"])
        wait(s["lip"],"station feed "+name);wait(e["seat"],"station camera and port verification "+name)
        if abs(s["drive"]-e["carry"])>.001:raise ValueError("Carried state and route clock differ")
        wait(s["drive"],"verified stationary seat "+name)
        focal.append(loaded.duration,"carrying "+name,loaded)
        wait(e["release"],"placement "+name)
        if "hold" in e:
            wait(e["hold"],"placement released "+name)
            wait(135,"placement held after invalid B frame")
            break
        trial=contact[mode][name]
        if not trial["admitted"] or not e["withdraw"]<=e["motion"]<e["stow"]:raise ValueError("Unverified departure")
        end=s["clear"]+s["retreatDelay"]+.370/.15
        wait(e["stow"],"placement and measured withdrawal "+name)
        wait(end,"empty slide recovery")
        delivered.append(dict(s,supportAt138=trial["stop"],finalXZ=trial["finalXZ"],finalBranchLowerMm=trial["branchLowerMm"]))
        if mode=="single" or name=="coral-B":
            leg=field.park_route(f,focal.pose,True)
            focal.append(leg.duration,"park",leg)
        else:
            focal.append(empty.duration,"return",empty)
    return f,auto,team,delivered


def clearance():
    tube=read(ROOT/"fixed-height-match/native.json")["firstTubeToSecondDockMm"]
    b=tube["inflatedTubeBoxMm"]
    roof=read(ROOT/"moving-observer/mount.json")["parts"]
    scene=read(HERE/"optics.json")["movingRayPrism"]["baseChanges"]
    def gap(box):
        return math.sqrt(sum(max(0,b[i]-box[i+3],box[i]-b[i+3])**2 for i in range(3)))
    ng=[]
    for name,part in roof.items():
        box=part["bounds_mm"].copy()
        box[3]+=370
        ng.append((gap(box),name))
    for item in scene:
        if item["revision"]=="593":
            ng.append((gap(item["aabb"]),item["part"]))
    return {"originalMovingStrokeMm":tube["gap"],"newBodiesMm":min(ng),"combinedLowerMm":min(tube["gap"],min(ng)[0])}


def run():
    cases={}
    events=list(csv.DictReader((HERE/"events.csv").open()))
    contacts=read(HERE/"contact.json")
    plans=read(ROOT/"fixed-height-match/placements.json")["placements"]
    starts={}
    for mode in ("nominal","lateB","single"):
        program=fieldcase(mode,contacts,events)
        f,auto,team,_=program
        for role,p in team.items():
            x,y,heading=auto.start if role=="red2" else p.initial
            red=role.startswith("red")
            tape=(391.375,393.375) if red else (297.5,299.5)
            width=max(0,min(x+.425,tape[1]*f.IN)-max(x-.425,tape[0]*f.IN))
            starts[role]={"xyIn":[round(x/f.IN,3),round(y/f.IN,3)],"headingDeg":round(math.degrees(heading),3),
                          "proxyTapeOverlapMm":round(1000*width,3)}
            if width<=0:raise ValueError("Unregistered starting position "+role)
        with patch.object(field,"scenario",return_value=program):
            result=field.audit(plans,inboard=True)
        result.pop("observerAuthorizedL1Count")
        held=mode=="lateB"
        if held:
            # The prior pipe's nearest remaining part is tested across the second extended stroke.
            proof=clearance()
            if proof["combinedLowerMm"]<=0:raise ValueError("Second receiver overlaps first pipe")
        result["runtimeDeliveries"]=len(program[-1])
        result["uncredited"]= [p for p,v in contacts[mode].items() if not v["admitted"]]
        result["fieldStockMinimumMm"]=min(min(v["stockBoundsMm"].values()) for v in result["field"])
        result["staticObstacleMinimumMm"]=min(v["minimumStaticM"] for v in result["field"])*1000
        result["stockPieces"]=len(read(ROOT/"full-match/installed-stock.json")["stock"])
        result["stowMarginMs"]={s["id"]:1000*(18+s["clear"]+s["retreatDelay"]+.370/.15)-
            next(int(e["wall_ms"]) for e in events if e["piece"]==s["id"] and e["event"]=="stow" and e["scenario"]==mode) for s in program[-1]}
        cases[mode]=result
    result={"cases":cases,"sixStartingPositions":starts,"firstPipeToSecondExtendedRobot":clearance(),
            "nativeStartingTapeAndPark":{k:v for k,v in read(ROOT/"fixed-height-match/native.json").items()
                                         if k in ("startBumperTapeMm3","inboardClothInZoneMm3","autoEndWholeHullToStartTapeLowerM")},
            "sourceSha256":{n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in [
            "integrated-match/hal.json","integrated-match/events.csv","integrated-match/contact.json","integrated-match/optics.json",
            "fixed-height-match/placements.json","fixed-height-match/native.json","full-match/installed-stock.json",
            "slide-feedback/no-air.json","moving-observer/clearance.json","design/fixed-no-air/balance.json","design/fixed-no-air/inspection.json"]}}
    (HERE/"match.json").write_text(json.dumps(result,indent=2,allow_nan=False)+"\n",encoding="utf-8",newline="\n")


if __name__=="__main__":run()
