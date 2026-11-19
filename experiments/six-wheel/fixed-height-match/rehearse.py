"""Run one clock for low-carriage arrivals, all six robot programs and the final assessments."""
import itertools
import json
import math
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT/"full-match"))
from match import corrected, build_park, MARKS, HULL, traffic, rect, others
from installed_stock import check
from scripts import assess_coral


def read(path):
    return json.loads(path.read_bytes())


def park_route(f,start,inboard):
    if not inboard:
        return build_park(f,start)
    ref=read(ROOT/"leave-park/near-tape.json")["cases"]["near-tape"]["finish"]
    route=f.Route(start)
    route.go((425*f.IN,start[1]),reverse=True)
    route.go((425*f.IN,ref[1]))
    route.go(tuple(ref[:2]))
    return route


def cage_margin(f,route):
    model=read(ROOT/"leave-park/cage-motion.json")
    stations={name:((s["axis_mm"][0]+690.875*25.4/2)/1000,
                   (317*25.4/2-s["axis_mm"][1])/1000) for name,s in model["stations"].items()}
    ticks=math.ceil(route.duration/.01)
    dt=route.duration/ticks
    half=dt/2
    best=(math.inf,None,None)
    for i in range(ticks):
        t=(i+.5)*dt
        (x,y,h),w=route.at(t)
        motion=half*traffic.motion_bound(w)+f.DRIVE["maximumWheelAccelMps2"]*(1+2*traffic.RADIUS/f.DRIVE["trackM"])*half**2
        for key,(cx,cy) in stations.items():
            rx,ry=f.rot((cx-x,cy-y),-h)
            gap=math.hypot(max(0,abs(rx)-.425),max(0,abs(ry)-.425))-model["radius_mm"]/1000-motion
            best=min(best,(gap,key,t))
    tracking=model["cases"]["near-tape"]["tracking_mm"]
    residual=1000*best[0]-tracking
    return {"uprightUnlimitedYawLowerMm":1000*best[0],"closest":best[1],"legTimeS":best[2],
            "comparisonTrackingMm":tracking,"remainingLateralSwingMm":residual,
            "assumedConeDeg":math.degrees(math.atan2(max(0,residual),model["upper_height_mm"]))}


def scenario(sources, park=True, partner_clears=True, stock=(0,35), station_hold=False,inboard=False):
    f=corrected()
    team,_,_=traffic.programs()
    auto=f.routes(-.18)[0]
    focal=traffic.Program(2,"red")
    focal.initial=focal.pose=auto.end
    team["red2"]=focal
    for role,delay in (("blue1",11),("blue2",45)):
        p=team[role]
        p.append(delay,"await crossing")
        r=f.Route(p.pose)
        r.go((p.pose[0]-.35,p.pose[1]),reverse=True)
        p.append(r.duration,"back away from midfield",r)
    if partner_clears:
        p=team["red1"]
        r=f.Route(p.pose)
        r.go((485*f.IN,65*f.IN),reverse=True)
        p.append(r.duration,"vacate parking lane",r)
    delivery=[]
    for i,s in enumerate(sources):
        if stock[i] is None or max(focal.clock,stock[i])+1.1>s["release"]+1e-8:
            break
        _,loaded,empty=f.routes(s["sector"])
        if math.dist(focal.pose[:2],loaded.start[:2])>1e-7:
            raise ValueError("Discontinuous station approach")
        focal.append(s["lip"]-focal.clock,"station feed")
        if station_hold:
            focal.append(s["seat"]-focal.clock,"stationary optical seat")
            focal.append(135-focal.clock,"hold at station, continuous custody unqualified")
            break
        focal.append(s["drive"]-focal.clock,"indicated seat and retained hold")
        focal.append(loaded.duration,"carrying "+s["id"],loaded)
        if focal.clock>=s["impact"]:
            raise ValueError("Actuation clock precedes arrival at target")
        focal.append(s["supported"]-focal.clock,"placement "+s["id"])
        finish_retract=s["clear"]+s["retreatDelay"]+.370/.15
        focal.append(max(0,finish_retract-focal.clock),"empty slide recovery")
        delivery.append(s)
        returning=park_route(f,focal.pose,inboard) if i==1 and park else empty
        focal.append(returning.duration,"park" if i==1 and park else "return",returning)
    return f,auto,team,delivery


def audit(sources, **options):
    f,auto,team,delivered=scenario(sources,**options)
    keys=list(team)
    minimum=(math.inf,None,None,None)
    first=None
    others_to_pipe=math.inf
    step=.02
    half=step/2
    acc=f.DRIVE["maximumWheelAccelMps2"]*(1+2*.69/f.DRIVE["trackM"])
    for n in range(round(135/step)):
        t=(n+.5)*step
        states={k:team[k].at(t) for k in keys}
        def extent(role):
            phase=states[role][2]
            return (.535,.425) if role=="red2" and (phase.startswith("placement") or phase=="empty slide recovery") else (.425,.425)
        def speed(role):
            left,right=states[role][1]
            return abs((left+right)/2)+math.hypot(*extent(role))*abs((left-right)/f.DRIVE["trackM"])
        for a,b in itertools.combinations(keys,2):
            raw=traffic.separation(states[a][0],states[b][0],extent(a),extent(b))
            bound=raw-half*(speed(a)+speed(b))-acc*half**2
            if bound<minimum[0]:
                minimum=(bound,a,b,t)
            if raw<0 and first is None:
                first=(a,b,t,raw)
        for s in delivered:
            if t+half<s["supported"]:
                continue
            pos=(f.LIP[0]+s["settledXZ"][0],f.LIP[1]+s["sector"],0)
            for k in keys:
                if k!="red2":
                    others_to_pipe=min(others_to_pipe,traffic.separation(states[k][0],pos,extent(k),(.05725,.1509125))
                                       -half*speed(k)-acc*half**2)
    stock=read(ROOT/"full-match/installed-stock.json")["stock"]
    field=[]
    hboxes={"robot-envelope":HULL}
    def add_leg(role,leg,route,extra):
        static=min(f.audit(route,HULL,MARKS+extra),key=lambda r:r["gap_m"])
        installed=check(f,stock,hboxes,route.segments)
        field.append(dict(role=role,leg=leg,minimumStaticM=static["gap_m"],closest=static["closest"],
                          stockBoundsMm={k:v["lower_bound_mm"] for k,v in installed.items()}))
    add_leg("red2","AUTO",auto,others())
    for role,p in team.items():
        for start,_,phase,route in p.intervals:
            if route:
                placed=[(s["id"],rect(f.LIP[0]+s["settledXZ"][0],f.LIP[1]+s["sector"],.05725,.1509125))
                        for s in delivered if start>=s["supported"]]
                add_leg(role,phase,route,placed)
    focal=team["red2"]
    end=focal.at(135)[0]
    barge=(690.875/2+23)*f.IN
    parked=(0<end[1]-.425 and end[1]+.425<146.5*f.IN and
            min(end[0]+.425,barge)-max(end[0]-.425,barge-46*f.IN)>0)
    tape=(690.875-144-65.5-88)*f.IN
    leave=auto.duration<=15 and auto.end[0]-.425*math.sqrt(2)>tape
    rows=[]
    for s in delivered:
        pos=(f.LIP[0]+s["finalXZ"][0],f.LIP[1]+s["sector"],0)
        gaps={k:traffic.separation(p.at(135)[0],pos,(.425,.425),(.05725,.1509125)) for k,p in team.items()}
        rows.append({"id":s["id"],"observedMs":138000,"touchesOwnTrough":s["supportAt138"]=="sustained-paired-floor-and-lip",
                     "touchesAlliedRobot":False if min(v for k,v in gaps.items() if k.startswith("red"))>0 else None,
                     "touchesOpponentRobot":False if min(v for k,v in gaps.items() if k.startswith("blue"))>0 else None,
                     "scoredOnAnotherReefLevel":False if s["finalBranchLowerMm"]>0 else None})
    editions={}
    for ed in assess_coral.EDITIONS:
        answer=assess_coral.assess({"schema":"reefscape-coral-observations/1","phase":"TELEOP",
             "basis":"Two finite settled sectors and stationary post-buzzer roster","samples":[
              {"timeMs":138000,"robotCommandActive":False,"inventoryComplete":False,"pieces":rows}]},ed)
        editions[ed]=answer["samples"][0]["pieces"]
    clear=minimum[0]>0 and others_to_pipe>0 and all(r["minimumStaticM"]>0 and min(r["stockBoundsMm"].values())>0 for r in field)
    all_l1=all(p["l1CriteriaAtInstant"]=="yes" for r in editions.values() for p in r)
    raw=3*leave+2*len(delivered)*all_l1+2*parked
    cage=next((cage_margin(f,path) for _,_,phase,path in focal.intervals if phase=="park"),None)
    gates=read(HERE/"observer.json")["pieces"]
    return {"nominalProgramClear":clear,"parkCageScreen":cage,
            "observerAuthorizedL1Count":sum(gates[s["id"]]["matchAuthorized"] for s in delivered),
            "sixRobotPaths":{k:[x[2] for x in p.intervals if x[3]] for k,p in team.items()},
            "autoTimeS":auto.duration,"finishTeleopS":focal.clock,"finiteIds":[s["id"] for s in delivered],
            "conditionalPoints":raw if clear and focal.clock<=135 else None,"pairBound":minimum,"firstOverlap":first,
            "otherRobotToSettledPipeBoundM":others_to_pipe if math.isfinite(others_to_pipe) else None,
            "finalPose":end,"park":parked,"leave":leave,"assessments":editions,"field":field}


def main():
    files=read(HERE/"placements.json")["placements"]
    return {k:audit(files,**opts) for k,opts in (("stationHold",{"station_hold":True}),("inboard",{"inboard":True}),
            ("reserved",{}),("blocked",{"partner_clears":False}),
            ("lateSecond",{"stock":(0,112)}),("returnInstead",{"park":False}))}


if __name__=="__main__":
    (HERE/"match.json").write_text(json.dumps(main(),indent=2,allow_nan=False)+"\n",encoding="utf-8",newline="\n")
