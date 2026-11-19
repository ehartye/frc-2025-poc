"""Join the exact robot paths, raw optical and rack outcomes, and final rules assessment."""
import argparse
import csv
import itertools
import json
import math
from pathlib import Path
import sys
import numpy as np
from navigation import HERE,ROOT,placed_route,read,sha
sys.path.insert(0,str(ROOT/"fixed-height-match"))
import rehearse as field
from installed_stock import check,coordinate,box_gaps,gap_from_distances
from scripts import assess_coral


def copied(f,model,start):
    r=f.Route(start)
    for seg in model.segments:r.segment(seg["kind"],seg["demand"])
    return r


def return_route(f, start):
    ref=f.routes(-.18)[2]
    model=f.Route(ref.start)
    for seg in ref.segments[:-2]:model.segment(seg["kind"],seg["demand"])
    shift=f.rot((0,read(ROOT/"return-station/return.json")["fixedStationLeftBiasMm"]/1000),ref.end[2])
    model.go(f.add(ref.end,shift));model.face(ref.end[2])
    return copied(f,model,start)


def park_route(f, start, sector):
    model=f.Route((f.LIP[0]-.49,f.LIP[1]+sector,0))
    destination=read(ROOT/"leave-park/near-tape.json")["cases"]["near-tape"]["finish"]
    model.segment("drive",425*f.IN-model.start[0])
    model.go((425*f.IN,destination[1]));model.go(tuple(destination[:2]))
    return copied(f,model,start)


def run(cache):
    plans=read(ROOT/"fixed-height-match/placements.json")["placements"]
    f,auto,alliance,_=field.scenario(plans,inboard=True)
    nav=read(HERE/"navigation.json")["cases"]
    stock=read(ROOT/"full-match/installed-stock.json")["stock"]
    geometry=read(HERE/"native-register.json")
    with (HERE/"events.csv").open() as file:hal=list(csv.DictReader(file))
    with (HERE/"landing-observed.csv").open() as file:
        placements={r["case"].split("/")[0]:r for r in csv.DictReader(file) if "/250/" in r["case"]}
    corrected={}
    for name,row in placements.items():
        center=np.array([float(row[k]) for k in ("x_m","y_m","z_m")])
        ref=next(s for s in read(ROOT/"mission/branch-exclusion.json")["trips"] if s["piece"]==name)
        delta=1000*(np.linalg.norm(center-ref["target_x_y_z_m"])+2*.164*math.sin(math.acos(min(1,abs(float(row["axis_y"]))))/2))
        corrected[name]=dict(center=center.tolist(),branchLowerMm=ref["minimum_mm"]-delta,deltaMm=delta,supported=row["outcome"]=="supported")
    # Compare finite swept tubes near the shared trough. During other approaches the stowed
    # pipe stays inside the independently screened robot footprint.
    history=np.load(cache/"coral-B-pads-edge-250.npy")
    incoming=history[history[:,0]>=history[-1,0]-4.0]
    heading=incoming[:,-2]
    center_y=np.sin(heading)*incoming[:,1]+np.cos(heading)*incoming[:,2]+incoming[:,-3]-f.LIP[1]
    axis_y=np.sin(heading)*incoming[:,4]+np.cos(heading)*incoming[:,5]
    extent=.1508125*np.abs(axis_y)+.05715*np.sqrt(np.maximum(0,1-axis_y**2))
    low_b=min(float(placements["coral-B"]["swept_min_y_m"]),float(np.min(center_y-extent)))
    tube_clearance=1000*(low_b-float(placements["coral-A"]["swept_max_y_m"]))-4.5
    outcomes={}
    for mode in ("nominal","lateB","single"):
        legs=[];scored=[];last=auto.end;cursor=18.;static=[];inventory=[];routes=[];cage=None;park_end=None
        def add_leg(t,tag,route,duration=None):
            nonlocal last,cursor
            if math.dist(last[:2],route.start[:2])>3e-5:raise ValueError("Chassis discontinuity "+tag)
            duration=route.duration if duration is None else duration
            if t+1e-6<cursor:raise ValueError("Clock overlap "+tag)
            legs.append((t,t+duration,tag,route))
            last=route.end;cursor=t+duration
        def screen(tag,route):
            for seg in route.segments:
                short=f.Route(seg["start"])
                if seg["kind"]=="turn" and abs(seg["demand"])<.02:
                    short.segment("drive",1e-7)
                    value=min(v["gap_m"] for v in f.audit(short,field.HULL,field.MARKS))-2*.602*math.sin(abs(seg["demand"])/2)
                else:
                    short.segment(seg["kind"],seg["demand"])
                    value=min(v["gap_m"] for v in f.audit(short,field.HULL,field.MARKS))
                static.append(1000*value)
            q=check(f,stock,{"stowed":field.HULL},route.segments)
            inventory.append(min(v["lower_bound_mm"] for v in q.values()))
        screen("auto",auto)
        for plan in plans:
            piece=plan["id"]
            events={r["event"]:int(r["wall_ms"])/1000 for r in hal if r["piece"]==piece and r["scenario"]==mode}
            if "carry" not in events:break
            n=next(r for r in nav if r["pipe"]==piece and r["mode"]=="nominal")
            _,before,_=placed_route(piece,n["actualStationMmDeg"])
            start=18+plan["drive"]
            add_leg(start,piece+" carry",before)
            screen("carry",before)
            a=np.loadtxt(cache/(piece+"-nominal.csv"),delimiter=",",skiprows=1)
            if sha(cache/(piece+"-nominal.csv"))!=n["motionSHA256"]:raise ValueError("Unregistered steering")
            data=a.copy();data[:,1:3]/=1000;data[:,1]+=f.LIP[0]-.490;data[:,2]+=f.LIP[1]
            class Steering:
                def __init__(self,samples):
                    self.a=samples;self.start=tuple(samples[0,1:]);self.end=tuple(samples[-1,1:]);self.duration=samples[-1,0]/1000
                def at(self,t):
                    step=min(len(self.a)-1,max(0,int(t/.005)))
                    pt=tuple(np.interp(np.clip(t*1000,0,self.a[-1,0]),self.a[:,0],col) for col in self.a[:,1:].T)
                    if step==len(self.a)-1:return pt,(0.,0.)
                    dx,dy,dh=self.a[step+1,1:]-self.a[step,1:]
                    speed=(dx*math.cos(pt[2])+dy*math.sin(pt[2]))/.005
                    yaw=dh/.005*.290
                    return pt,(speed-yaw,speed+yaw)
            steer=Steering(data)
            add_leg(cursor,piece+" sighted approach",steer)
            # Uniform 8 mm envelope for intersample motion, same discretization as the original study.
            for _,x,y,h in a[::2]:
                p=(f.LIP[0]-.49+x/1000,f.LIP[1]+y/1000,h)
                gap=min(gap_from_distances(box_gaps(coordinate(f,o,p),field.HULL),o) for o in stock.values())-8
                inventory.append(gap)
                static.append(490-x-425*(abs(math.sin(h))+abs(math.cos(h)))-8)
            stop=events.get("stow",153)
            legs.append((cursor,stop,piece+" placement",None))
            cursor=stop
            if "stow" not in events:break
            scored.append(piece)
            periodic=(mode=="single" or piece=="coral-B")
            onward=park_route(f,last,plan["sector"]) if periodic else return_route(f,last)
            add_leg(stop,"park" if periodic else "return to station",onward)
            screen("departure",onward)
            if periodic:
                park_end=cursor
                cage=field.cage_margin(f,onward)
                break
        def focal(t):
            if t<=18:
                p,w=auto.at(min(t,15));return p,w,"AUTO"
            prior=auto.end
            for a,b,label,r in legs:
                if t<a:return prior,(0.,0.),"station hold"
                if t<b:return (*r.at(t-a),label) if r else (prior,(0.,0.),label)
                if r:prior=r.end
            return prior,(0.,0.),"parked" if park_end else legs[-1][2] if legs else "hold"
        margin=math.inf;other_pipe=math.inf;own_prior=math.inf;view=[]
        own_extent=(.425,.425)
        for i in range(7801):
            t=i*.02
            own,w,tag=focal(t)
            actors={"red2":(own,w)}
            for k,p in alliance.items():
                if k!="red2":
                    other,ow,_=p.at(max(0,t-18));actors[k]=(other,ow)
            for ka,kb in itertools.combinations(actors,2):
                pa,wa=actors[ka];pb,wb=actors[kb]
                da=(.535,.425) if ka=="red2" and "placement" in tag else own_extent
                db=(.535,.425) if kb=="red2" and "placement" in tag else own_extent
                margin=min(margin,1000*(field.traffic.separation(pa,pb,da,db)-.01*(field.traffic.motion_bound(wa)+field.traffic.motion_bound(wb))-.0006))
            if i%10==0:view.append([round(t,2),tag,[[k,round(p[0],4),round(p[1],4),round(p[2],5)] for k,(p,w) in actors.items()]])
            for piece in scored:
                row=corrected[piece]
                times={r["event"]:int(r["wall_ms"])/1000 for r in hal if r["scenario"]==mode and r["piece"]==piece}
                if t<times["release"]:continue
                x,y,_=row["center"];tube=(f.LIP[0]+x,f.LIP[1]+y,0.)
                for k,(p,ow) in actors.items():
                    if k=="red2":continue
                    other_pipe=min(other_pipe,1000*(field.traffic.separation(p,tube,own_extent,(.245,.190))-.01*field.traffic.motion_bound(ow)-.0006))
                if t<times["stow"]:continue
                if piece=="coral-A" and "coral-B placement"==tag:
                    own_prior=min(own_prior,geometry["earlierInflatedFirstTubeAtSecondMm"]-row["deltaMm"])
                else:own_prior=min(own_prior,1000*(field.traffic.separation(own,tube,own_extent,(.05915,.15282))-.01*field.traffic.motion_bound(w)-.0006))
        end,_,_=focal(153)
        extent=.425*(abs(math.cos(end[2]))+abs(math.sin(end[2])))
        barge=(690.875/2+23)*f.IN
        over=min(end[0]+extent,barge)-max(end[0]-extent,barge-46*f.IN)
        parked=park_end is not None and park_end<=153 and over>0 and 0<end[1]-extent and end[1]+extent<146.5*f.IN and cage["remainingLateralSwingMm"]>0
        tape=(690.875-144-65.5-88)*f.IN
        leave=auto.duration<=15 and auto.end[0]-.425*math.sqrt(2)>tape
        pieces=[{"id":p,"observedMs":138000,"touchesOwnTrough":corrected[p]["supported"] if mode!="lateB" else None,"touchesAlliedRobot":False if min(own_prior,other_pipe)>0 else None,"touchesOpponentRobot":False if other_pipe>0 else None,"scoredOnAnotherReefLevel":False if corrected[p]["branchLowerMm"]>0 else None} for p in scored]
        oracle={ed:assess_coral.assess({"schema":"reefscape-coral-observations/1","phase":"TELEOP","basis":"conditional 597-body physics, HAL and achieved field roster","samples":[{"timeMs":138000,"robotCommandActive":False,"inventoryComplete":False,"pieces":pieces}]},ed)["samples"][0]["pieces"] for ed in assess_coral.EDITIONS}
        eligible=sum(all(oracle[ed][j]["l1CriteriaAtInstant"]=="yes" for ed in oracle) for j in range(len(pieces)))
        qualified=min(margin,min(static),min(inventory),own_prior,other_pipe)>0 and (mode!="nominal" or tube_clearance>0)
        outcomes[mode]={"leave":leave,"parked":bool(parked),"parkWallS":park_end,"parkTapeOverlapMm":1000*over,"cage":cage,"scoredPieces":scored,"scoring":oracle,"points":3*leave+2*parked+2*eligible if qualified else None,"allRobotLowerMm":margin,"stowedFieldLowerMm":min(static),"stowedStockLowerMm":min(inventory),"priorTubeToRobotMm":own_prior,"otherRobotToTubeMm":other_pipe,"valid":qualified}
        outcomes[mode]["supportedBeforeUnresolvedFault"]=scored if mode=="lateB" else []
        outcomes[mode]["scoredPieces"]=[r["id"] for j,r in enumerate(pieces) if all(oracle[ed][j]["l1CriteriaAtInstant"]=="yes" for ed in oracle)]
        (cache/(mode+"-visual.json")).write_text(json.dumps(view,separators=(",",":")),encoding="utf8")
    out={"actuationRegistration":sha(HERE/"hal.json"),"sourceToFinal":corrected,"successiveTubeLowerMm":tube_clearance,"assessedWallS":156,"cases":outcomes}
    (HERE/"match.json").write_text(json.dumps(out,indent=2)+"\n",encoding="utf8",newline="\n")
    print([(k,r["points"],r["parkWallS"],r["allRobotLowerMm"],r["stowedFieldLowerMm"],r["stowedStockLowerMm"],r["priorTubeToRobotMm"]) for k,r in outcomes.items()],flush=True)

if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("cache",type=Path);run(p.parse_args().cache)
