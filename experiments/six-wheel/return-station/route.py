"""Replay a single preplanned feeder bias from every real A exit using the existing wheel constraints."""
import json
import math
import sys
from pathlib import Path
import numpy as np
HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
sys.path.insert(0,str(ROOT/"fixed-height-match"))
import rehearse as field
from installed_stock import check


def read(p):return json.loads(p.read_bytes())


def replay():
    first=read(ROOT/"spatial-contact/match.json")["cases"]
    plans=read(ROOT/"fixed-height-match/placements.json")["placements"]
    f,_,roster,_=field.scenario(plans,inboard=True)
    prior=f.routes(-.180)[2]
    # The fixed bias is the midpoint of the measured return range, rounded to 0.1 mm.
    arows=[v for v in first if v["pipe"]=="coral-A"]
    sides=[v["returnStationRobotDxDyYawMmDeg"][1] for v in arows]
    bias=-round((min(sides)+max(sides))/2,1)
    destination=prior.end
    shift=f.rot((0,bias/1000),destination[2])
    adapted=f.Route(prior.start)
    for seg in prior.segments[:-2]:adapted.segment(seg["kind"],seg["demand"])
    adapted.go((destination[0]+shift[0],destination[1]+shift[1]))
    adapted.face(destination[2])
    stock=read(ROOT/"full-match/installed-stock.json")["stock"]
    trials=read(ROOT/"sector-alignment/results.json")
    start=18+plans[0]["clear"]+plans[0]["retreatDelay"]+.370/.15
    human=18+plans[1]["release"]
    results={}
    for row in arows:
        mode=row["mode"]
        ex,ey,eh=next(v["finalBodyErrorMmDeg"] for v in trials if v["pipe"]=="coral-A" and v["mode"]==mode)
        route=f.Route((prior.start[0]+ex/1000,prior.start[1]+ey/1000,prior.start[2]+math.radians(eh)))
        for seg in adapted.segments:route.segment(seg["kind"],seg["demand"])
        station=f.rot((route.end[0]-destination[0],route.end[1]-destination[1]),-destination[2])
        delta=[1000*station[0],1000*station[1],math.degrees(f.short(route.end[2]-destination[2]))]
        full_turn_screen=min(v["gap_m"] for v in f.audit(route,field.HULL,field.MARKS))
        prefix=f.Route(route.start)
        for seg in route.segments[:-1]:prefix.segment(seg["kind"],seg["demand"])
        last=route.segments[-1]
        if last["kind"]!="turn" or abs(last["demand"])>.02:raise ValueError("Unexpected docking turn")
        reference=f.Route(last["start"]);reference.segment("drive",1e-7)
        radius=math.hypot(.425,.425)
        swept=min(v["gap_m"] for v in f.audit(reference,field.HULL,field.MARKS)) - 2*radius*math.sin(abs(last["demand"])/2)
        clearance=min(swept,min(v["gap_m"] for v in f.audit(prefix,field.HULL,field.MARKS)))
        inventory=check(f,stock,{"robot":field.HULL},route.segments)
        pair=math.inf
        for t in np.arange(0,human-start+.011,.01):
            p,w=route.at(t)
            for role,prog in roster.items():
                if role=="red2":continue
                pos,other,_=prog.at(start+t-18)
                pair=min(pair,field.traffic.separation(p,pos)-.005*(field.traffic.motion_bound(w)+field.traffic.motion_bound(other))-.00016)
        results[mode]={"stationRobotDxDyYawMmDeg":delta,"minStaticMm":1000*clearance,"unrestrictedRotationScreenMm":1000*full_turn_screen,"finalTurnDeg":math.degrees(last["demand"]),"minStockMm":min(i["lower_bound_mm"] for i in inventory.values()),"minRobotMm":1000*pair,
                       "departWallS":start,"arriveWallS":start+route.duration,"feedWallS":human,"settleBeforeFeedS":human-start-route.duration}
    (HERE/"return.json").write_text(json.dumps({"fixedStationLeftBiasMm":bias,"extraRouteDurationS":adapted.duration-prior.duration,"cases":results},indent=2)+"\n",encoding="utf8",newline="\n")


if __name__=="__main__":replay()

