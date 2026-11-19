"""Assess scoring and parking from the attained first dock without resetting the odometry."""
import csv
import hashlib
import json
import math
from pathlib import Path
import sys
import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
sys.path.insert(0,str(ROOT/"sector-alignment"))
import clock as approach
from installed_stock import check,coordinate,box_gaps,gap_from_distances
from scripts import assess_coral
field=approach.field


def read(path):
    return json.loads(path.read_bytes())


def copied_demands(f,plan,start):
    actual=f.Route(start)
    for seg in plan.segments:
        actual.segment(seg["kind"],seg["demand"])
    return actual


def branch_lower(row,reference):
    # Hausdorff bound on two congruent expanded cylinders. Both axis signs describe the same solid.
    center=(row["x_m"],row["y_m"],row["z_m"])
    angle=math.acos(min(1,abs(row["axis_y"])))
    radius=math.hypot(.1528125,.05915)
    return reference["minimum_mm"]-1000*(math.dist(center,reference["target_x_y_z_m"])+2*radius*math.sin(angle/2))


def run():
    physics=read(HERE/"physics.json"); source=read(HERE/"source.json")
    registered=read(ROOT/"sector-alignment/results.json")
    terminals=read(ROOT/"sector-alignment/clock.json")
    branches=read(ROOT/"mission/branch-exclusion.json")
    plans=read(ROOT/"fixed-height-match/placements.json")["placements"]
    stock=read(ROOT/"full-match/installed-stock.json")["stock"]
    f,auto,roster,_=field.scenario(plans,inboard=True)
    actors={k:p for k,p in roster.items() if k!="red2"}
    events=list(csv.DictReader((HERE/"events.csv").open()))
    out=[]
    for pipe,mode in [(r["pipe"],r["mode"]) for r in source]:
        trial=next(r for r in registered if (r["pipe"],r["mode"])==(pipe,mode))
        approach_gate=next(r for r in terminals["cases"] if (r["pipe"],r["condition"])==(pipe,mode))
        phys=next(r for r in physics["cases"] if r["case"]==pipe+"-"+mode+"-scheduled")
        ref=next(r for r in branches["trips"] if r["piece"]==pipe)
        exclusion=branch_lower(phys,ref)
        plan=next(s for s in plans if s["id"]==pipe)
        sector=plan["sector"]
        returns=f.routes(sector)[2]
        initial=trial["initial"]
        carry=f.routes(sector)[1]
        terminal=carry.duration-carry.segments[-1]["dt"]
        stock_prior=pair_prior=math.inf
        for u in np.linspace(0,2,201):
            q=u/2; gain=q**3*(10-15*q+6*q*q)
            (x,y,h),w=carry.at(terminal-2+u)
            pose=(x+gain*initial[0]/1000,y+gain*initial[1]/1000,h+gain*math.radians(initial[2]))
            wall=approach_gate["startWallS"]-2+u
            own=.005*(field.traffic.motion_bound(w)+.07)+.00016
            for v in stock.values():
                gap=gap_from_distances(box_gaps(coordinate(f,v,pose),field.HULL),v)
                stock_prior=min(stock_prior,gap-1000*own)
            for other in actors.values():
                pos,ow,_=other.at(wall-18)
                pair_prior=min(pair_prior,field.traffic.separation(pose,pos)-own-.005*field.traffic.motion_bound(ow))
        ex,ey,eh=trial["finalBodyErrorMmDeg"]
        dock=(f.LIP[0]-.49+ex/1000,f.LIP[1]+sector+ey/1000,math.radians(eh))
        return_route=copied_demands(f,returns,dock)
        delta=f.rot((return_route.end[0]-returns.end[0],return_route.end[1]-returns.end[1]),-returns.end[2])
        row={"pipe":pipe,"mode":mode,"pairedSupportThroughAssessment":phys["outcome"]=="supported" and phys["final_dwell_s"]>2,
             "nativeOtherLevelLowerMm":exclusion,"twoSecondPerturbationStockMm":stock_prior,"twoSecondPerturbationTrafficMm":1000*pair_prior,
             "sourceTerminalStockMm":approach_gate["stowedVoxelStockLowerMm"],"returnStationRobotDxDyYawMmDeg":[1000*delta[0],1000*delta[1],eh]}
        if pipe=="coral-B":
            row["scope"]="independent finite second-sector experiment; its issued station feed starts at the reference datum"
            out.append(row);continue
        times={r["event"]:int(r["wall_ms"])/1000 for r in events if r["scenario"]=="single" and r["approach"]==mode and r["piece"]==pipe}
        onset=times["stow"]
        base=f.Route((f.LIP[0]-.49,f.LIP[1]+sector,0.))
        base.segment("drive",425*f.IN-base.start[0])
        destination=read(ROOT/"leave-park/near-tape.json")["cases"]["near-tape"]["finish"]
        base.go((425*f.IN,destination[1]));base.go(tuple(destination[:2]))
        actual=copied_demands(f,base,dock)
        static=min(v["gap_m"] for v in f.audit(actual,field.HULL,field.MARKS))
        inventory=check(f,stock,{"robot":field.HULL},actual.segments)
        stocked=min(v["lower_bound_mm"] for v in inventory.values())
        cage=field.cage_margin(f,actual)
        tube=(f.LIP[0]+phys["x_m"],f.LIP[1]+phys["y_m"],0)
        robot_gap=own_pipe=other_pipe=math.inf
        path=[]
        # The settled cylinder's full dimensions include 2 mm at the radius and each end.
        shape=(.05915,.1528125)
        for j in range(math.ceil((156-onset)/.02)+1):
            t=.02*j;pose,w=actual.at(t);own=.01*field.traffic.motion_bound(w)+.0006
            own_pipe=min(own_pipe,field.traffic.separation(pose,tube,(.425,.425),shape)-own)
            for key,p in actors.items():
                pp,ow,_=p.at(min(135,onset+t-18));others=.01*field.traffic.motion_bound(ow)+.0006
                robot_gap=min(robot_gap,field.traffic.separation(pose,pp)-own-others)
                other_pipe=min(other_pipe,field.traffic.separation(pp,tube,(.425,.425),shape)-others)
            if t<=actual.duration+.25 and j%10==0:path.append([round(onset+t,3),round(pose[0],4),round(pose[1],4)])
        # Check the five other robots from the first release through the start of departure, including
        # the pre-settle flight/bounce. A disk encloses both the signed model and its contact excursion.
        for w in np.arange(times["release"],onset+.021,.02):
            for p in actors.values():
                pp,ow,_=p.at(w-18)
                separation=field.traffic.separation(pp,tube,(.425,.425),(.245,.190))-.01*field.traffic.motion_bound(ow)-.0006
                other_pipe=min(other_pipe,separation)
        x,y,h=actual.end
        xhalf=.425*(abs(math.cos(h))+abs(math.sin(h)));barge=(690.875/2+23)*f.IN
        overlap=min(x+xhalf,barge)-max(x-xhalf,barge-46*f.IN)
        parked=min(y-xhalf,146.5*f.IN-y-xhalf,overlap)>0 and onset+actual.duration<=153
        cleared=min(robot_gap,own_pipe,other_pipe,static,stocked/1000,stock_prior/1000,pair_prior,exclusion/1000,cage["remainingLateralSwingMm"]/1000,approach_gate["stowedVoxelStockLowerMm"]/1000)>0
        witness={"id":pipe,"observedMs":138000,"touchesOwnTrough":row["pairedSupportThroughAssessment"],"touchesAlliedRobot":False if min(own_pipe,other_pipe)>0 else None,
                 "touchesOpponentRobot":False if other_pipe>0 else None,"scoredOnAnotherReefLevel":False if exclusion>0 else None}
        scoring={ed:assess_coral.assess({"schema":"reefscape-coral-observations/1","phase":"TELEOP","basis":"spatial signed contact and stationary roster","samples":[{"timeMs":138000,"robotCommandActive":False,"inventoryComplete":False,"pieces":[witness]}]},ed)["samples"][0]["pieces"][0]["l1CriteriaAtInstant"] for ed in assess_coral.EDITIONS}
        row.update(stowWallS=onset,parkWallS=onset+actual.duration,parkTapeOverlapMm=1000*overlap,parkStaticMm=1000*static,parkStockMm=stocked,
                   parkTrafficMm=1000*robot_gap,parkRobotToScoredPipeMm=1000*own_pipe,otherRobotsToPipeMm=1000*other_pipe,cage=cage,
                   scoring=scoring,conditionalRolePoints=(3+2*parked+2*all(v=="yes" for v in scoring.values()) if cleared else None),
                   path=path)
        out.append(row)
    (HERE/"match.json").write_text(json.dumps({"scope":"five independent first-trip-and-park trials; second sector evaluated independently","cases":out,
        "calibrationFailureFallback":terminals["oneClockPolicy"],"sourceSha256":{str(p.relative_to(ROOT)).replace("\\","/"):hashlib.sha256(p.read_bytes()).hexdigest()
        for p in (HERE/"physics.json",HERE/"events.csv",ROOT/"sector-alignment/clock.json",ROOT/"mission/branch-exclusion.json",ROOT/"bridge-tolerance/field.json",ROOT/"full-match/installed-stock.json")}},indent=2)+"\n",encoding="utf8",newline="\n")


if __name__=="__main__":run()
