"""Join the approach gate to match wall time, actual occupied traffic and a stowed park alternative."""
import csv
import hashlib
import json
import math
from pathlib import Path
import sys
import numpy as np
from types import SimpleNamespace

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
sys.path.insert(0,str(ROOT/"fixed-height-match"))
import rehearse as field
from installed_stock import coordinate,box_gaps,gap_from_distances,check

def read(path):return json.loads(path.read_bytes())
def frame(f,x,y,h):
    return f.LIP[0]-.490+x/1000,f.LIP[1]+y/1000,h

def study(cache):
    plans=read(ROOT/"fixed-height-match/placements.json")["placements"]
    f,auto,team,_=field.scenario(plans,inboard=True)
    roster=read(ROOT/"full-match/installed-stock.json")["stock"]
    source=read(HERE/"results.json")
    bound=read(ROOT/"bridge-tolerance/field.json")
    source_packets=read(ROOT/"integrated-match/match.json")
    actors={k:v for k,v in team.items() if k!="red2"}
    result=[]
    for trial in source:
        pipe=trial["pipe"];s=next(v for v in plans if v["id"]==pipe)
        route=f.routes(s["sector"])[1]
        begin=18+s["drive"]+route.duration-route.segments[-1]["dt"]
        steps=np.loadtxt(cache/(pipe+"-"+trial["mode"]+".csv"),delimiter=",",skiprows=1)
        values=[];minimum=math.inf;stockgap=math.inf;static=math.inf
        optical_stock=math.inf;optical_traffic=math.inf
        centers=[]
        for v in roster.values():
            x,h,z=v["native_center_mm"];wx,wy=f.field_from_native(x/25.4,z/25.4)
            size=.20955 if v["group"]!="floor-pipe" else math.hypot(.151,.058)
            centers.append((np.array([wx,wy,h/1000]),size))
        def segment_dist(c,a,b):
            d=b-a
            k=min(1.,max(0.,float((c-a)@d/(d@d))))
            return float(np.linalg.norm(c-a-k*d))
        # The stock check uses an 8 mm one-robot reserve on 10 ms samples. A separate
        # two-body bound below includes the opponent relative speed and acceleration.
        pad=.008
        for index in range(0,len(steps),2):
            tick,x,y,h=steps[index];t=begin+tick/1000-18
            pose=frame(f,x,y,h)
            # The sphere about the feature center contains the full 220mm printed
            # window and both 12mm bolt heads; linearly interpolating between this
            # and a lens covers every ray. The extra spatial margin spans the 10ms step.
            tail=np.array([f.LIP[0],f.LIP[1],.310])
            eyes=[np.r_[np.array(pose[:2])+np.asarray(f.rot((-.315,lens),h)),.970] for lens in (.0375,-.0375)]
            for eye in eyes:
                optical_stock=min(optical_stock,min(segment_dist(c,eye,tail)-size-.179-.008 for c,size in centers))
            for other,program in actors.items():
                p,w,_=program.at(t)
                for eye in eyes:
                    optical_traffic=min(optical_traffic,segment_dist(np.array(p[:2]),eye[:2],tail[:2])-.602-.179-.021)
                # 10ms samples here. Include each robot's own instantaneous swept speed and acceleration.
                own=.005*(trial["peakWheelMmps"]/1000+.602*.70)
                other_move=.005*field.traffic.motion_bound(w)+.00016
                minimum=min(minimum,field.traffic.separation(pose,p)-own-other_move)
            for v in roster.values():
                value=gap_from_distances(box_gaps(coordinate(f,v,pose),field.HULL),v)-pad*1000
                stockgap=min(stockgap,value)
            # This straight terminal approach is at the field midpoint in Y, more than 3 m from the rails.
            # The reef covers the x interval from the lip. The rotated front uses the complete 850mm hull.
            static=min(static,490-x-425*(abs(math.cos(h))+abs(math.sin(h)))-pad*1000)
            if index%20==0:values.append([round(t+18,3),round(pose[0],3),round(pose[1],3)])
        accepted=trial["accepted"];physically_scoped=not trial["truthOutsideDeclaredCalibration"]
        item={"pipe":pipe,"condition":trial["mode"],"startWallS":begin,"extensionDeadlineWallS":begin+route.segments[-1]["dt"]+1.5,
              "approachDecisionWallS":None if not accepted else begin+trial["decisionMs"]/1000,
              "allStockToOpticalCorridorMm":optical_stock*1000,"allTrafficToOpticalCorridorMm":optical_traffic*1000,"pairedTrafficLowerMm":1000*minimum,"stowedVoxelStockLowerMm":stockgap,"redReefFaceLowerMm":static,
              "permittedInCalibration":accepted and physically_scoped,"controllerAccepted":accepted,
              "scope":"first trip" if pipe=="coral-A" else "independent second-sector rehearsal; prior cleared pipe needed to schedule",
              "extensionAgainstOccupiedBallMm":bound["sectors"][str(s["sector"])]["whole_target_stock"]["reef-ball"]["after_translation_and_camera_allowance_mm"] if accepted and physically_scoped else None}
        if not accepted:
            final=frame(f,*steps[-1,1:]);late=math.inf
            # Wheels have actually stopped. Retain this same footprint while the other five finish.
            for t in np.arange(begin-18+steps[-1,0]/1000,135.001,.02):
                for k,program in actors.items():
                    other,w,_=program.at(t)
                    late=min(late,field.traffic.separation(final,other)-.01*field.traffic.motion_bound(w)-.0006)
            item["stationaryAbortToBuzzerPairLowerMm"]=late*1000
        if pipe=="coral-A" and trial["mode"]=="nominal":
            # Preselect the documented alternate before commanding extension. A fresh, admitted
            # dock interval is required; a blind abort holds the retracted receiver at its stop.
            _,x,y,h=steps[-1];start=frame(f,x,y,h)
            planned=f.Route((f.LIP[0]-.490,f.LIP[1]+s["sector"],0.))
            planned.segment("drive",425*f.IN-planned.start[0])
            end=read(ROOT/"leave-park/near-tape.json")["cases"]["near-tape"]["finish"]
            planned.go((425*f.IN,end[1]));planned.go(tuple(end[:2]))
            actual=f.Route(start)
            for seg in planned.segments:actual.segment(seg["kind"],seg["demand"])
            at=item["extensionDeadlineWallS"];dt=.01;pair=(math.inf,None);points=[]
            for j in range(math.ceil((153-at)/dt)+1):
                u=j*dt;pose,w=actual.at(min(u,actual.duration))
                for k,p in actors.items():
                    other,ow,_=p.at(at+u-18)
                    gap=field.traffic.separation(pose,other)-.005*(field.traffic.motion_bound(w)+field.traffic.motion_bound(ow))-.00016
                    pair=min(pair,(gap,k))
                if j%25==0 and u<actual.duration+.25:points.append([round(at+u,3),round(pose[0],3),round(pose[1],3)])
            stock=check(f,roster,{"robot":field.HULL},actual.segments)
            barrier=min(v["gap_m"] for v in f.audit(actual,field.HULL,field.MARKS))
            parked=actual.end
            barge=(690.875/2+23)*f.IN
            # Coverage of the finally rotated square projected on X; physical cloth X is 425mm.
            clip=barge-(parked[0]-.425*abs(math.cos(parked[2])))
            item["parkIfScoringWithheld"]={"startWallS":at,"stowedThroughout":True,"finishWallS":at+actual.duration,
                "minimumRobotGapMm":1000*pair[0],"nearestRobot":pair[1],"minimumStaticMm":1000*barrier,
                "minimumStockMm":min(v["lower_bound_mm"] for v in stock.values()),"cage":field.cage_margin(f,actual),
                "finalXYHeading":parked,"frontBumperZoneOverlapMm":1000*clip,"path":points,"approachPath":values}
        result.append(item)
    fallback=result[0]["parkIfScoringWithheld"]
    custody=read(HERE/"park-custody.json")["stowedPark"]
    admissible=custody["stayedOnShelf"] and custody["assessmentWallS"]>=156 and all(fallback[k]>0 for k in ["minimumRobotGapMm","minimumStaticMm","minimumStockMm","frontBumperZoneOverlapMm"]) and fallback["finishWallS"]<153
    out={"schema":"reef-sector-docking/v1","cases":result,"oneClockPolicy":{"autoLeave":source_packets["cases"]["nominal"]["leave"],
            "verifiedPipeScores":0,"secondPipeDispatched":False,"parkOptionClear":admissible,"conditionalParkRolePoints":3+2*admissible,
            "blindAbortPoints":3,"physicalContactRestriction":"Both changed shots must have independently admitted support, optical withdrawal and stow before a second pipe is dispatched."},
            "sourceSHA256":{str(p.relative_to(ROOT)).replace("\\","/"):hashlib.sha256(p.read_bytes()).hexdigest() for p in [
                HERE/"scene.json",HERE/"results.json",HERE/"pipe.json",HERE/"park-custody.json",ROOT/"bridge-tolerance/field.json",ROOT/"bridge-tolerance/register.json",
                ROOT/"integrated-match/match.json",ROOT/"full-match/installed-stock.json",ROOT/"fixed-height-match/placements.json"]}}
    (HERE/"clock.json").write_text(json.dumps(out,indent=2)+"\n",encoding="utf8",newline="\n")
    print(out["oneClockPolicy"])
if __name__=="__main__":study(Path(sys.argv[1]))
