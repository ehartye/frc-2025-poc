"""Join the six-robot rehearsal's active clock to the protected electrical inventory."""
import argparse
import csv
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
from retreat import DURATION, RISE, TRAVEL, motion
from size_drives import UP4,UP5,motor


def load(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    module=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


match=load("low_registered_match",ROOT/"fixed-height-match/rehearse.py")
electric=load("old_electrical_sizing",ROOT/"design/electrical-inspection/replay.py")
plans=match.read(ROOT/"fixed-height-match/placements.json")["placements"]
rack=match.read(ROOT/"design/receiver-drives/sizing.json")
branches={row["load"]:row for row in electric.CIRCUITS}
phase_keys=tuple(electric.PHASE_R)
DT=.020


def chassis(hold):
    f,auto,team,_=match.scenario(plans,station_hold=hold,inboard=True)
    windows=[(0.,"AUTO approach",auto)]
    windows += [(18+start,name,route) for start,_,name,route in team["red2"].intervals if route]
    segments=[]
    for now,label,route in windows:
        for s in route.segments:
            for duration,initial,acc in ((s["rise"],0,s["a"]),(s["coast"],s["peak"],0),
                                         (s["rise"],s["peak"],-s["a"])):
                if duration>0:
                    segments.append((now,now+duration,label,s["kind"],initial,acc))
                    now+=duration
    return segments,team["red2"]


def slide_speed(t,entry):
    if t<entry["drive"]:
        return 0.,0.,""
    # The published route enters the three-second quintic extension 1.5 s after arrival.
    move_start=entry["_extend"]
    u=(t-move_start)/3
    if 0<=u<=1:
        return TRAVEL/3*30*u*u*(1-u)**2,TRAVEL/9*60*u*(1-u)*(1-2*u),"extend"
    elapsed=t-entry["clear"]-entry["retreatDelay"]
    if 0<=elapsed<DURATION:
        _,v,a=motion(elapsed)
        return -v,-a,"retreat"
    if move_start+3<t<entry["clear"]+entry["retreatDelay"]:
        return 0.,0.,"holding-extended"
    return 0.,0.,""


def make_samples(grips,hold,drag,stuck=False):
    segments,robot=chassis(hold)
    sources=[dict(x) for x in plans]
    for source in sources:
        route=match.corrected().routes(source["sector"])[1]
        source["_extend"]=source["drive"]+route.duration+1.5
    index=0
    params={**electric.DEFAULT,"massKg":67.,"inertiaKgM2":67*2*.425**2}
    for i in range(round(156/DT)):
        left=i*DT
        right=left+DT
        tel=left-18
        while index<len(segments) and segments[index][1]<=left+1e-10:
            index+=1
        phase={k:[0.,0.] for k in phase_keys}
        def add(k,amps,volts):
            phase[k][0]=max(phase[k][0],amps)
            phase[k][1]=max(phase[k][1],volts)
        stage=("AUTO hold" if left<15 else "transition" if left<18 else "post-buzzer" if tel>=135 else robot.at(tel+.01)[2])
        active=left<15 or 18<=left<153
        for j in range(index,len(segments)):
            begin,end,name,kind,initial,acc=segments[j]
            if begin>=right:break
            if active:
                stage=name
                for t in (max(begin,left),min(end,right)):
                    d=electric.demand(params,kind,max(0,initial+acc*(t-begin)),acc)
                    for k in ("DriveLeft","DriveRight"):add(k,d["phaseA"],d["motorV"])
        if 0<=tel<135:
            # Integral clock ticks align across AUTO, transition and TELEOP.
            g=grips["ticks"].get(str(i-900))
            if g and (not hold or tel<=sources[0]["seat"]+.02):
                for side,key in (("L","LeftGrip"),("R","RightGrip")):
                    add(key,*g[side])
            if stuck and tel>sources[0]["seat"]:
                for key in ("LeftGrip","RightGrip"):add(key,8,.12*8)
            if not hold:
                for s in sources:
                    for t in (tel,tel+.01,tel+DT):
                        v,a,action=slide_speed(t,s)
                        if not action:continue
                        # Resist load while holding as well as while moving; no credit for friction or regeneration.
                        force=(2*rack["loaded_outer_mass_kg"]+.5*rack["moving_middle_mass_kg"])*abs(a)+2*drag+4
                        d=motor(force,abs(v)/2,.010,UP5*UP4,.70,abs(a)/2,3e-6)
                        add("Slide",d["phase_A"],d["terminal_V"])
        watts={"Radio":12.,"RIO":15.,"Observer":(5.1*3+6+1)/.85,"PH":1.5}
        needed={}
        for key,(a,v) in phase.items():
            needed[key]=v+a*electric.PHASE_R[key]
            # Lift actuation isolated on this mechanically fixed configuration.
            watts[key]=0. if key=="Lift" else .7+a*needed[key]/.90
        yield left,stage,phase,watts,needed


def critical(watts,needed,compressor,voc):
    demand={k:max(9.5,needed.get(k,0.)) for k in watts}
    branch={k:cut+electric.RESIST[k]*(watts[k]/cut+(compressor if k=="PH" else 0.)) for k,cut in demand.items()}
    key=max(branch,key=branch.get)
    bus=branch[key]
    amps=0.
    for k,p in watts.items():
        fixed=compressor if k=="PH" else 0.
        avail=bus-electric.RESIST[k]*fixed
        term=(avail+math.sqrt(avail*avail-4*electric.RESIST[k]*p))/2
        amps+=fixed+p/term
    return (voc-bus)/amps,key


def evaluate(samples,voc,resistance,compressor,output=None):
    stats={k:{"minTerminalV":math.inf,"peakBranchA":0.,"branchA2s":0.,"peakPhaseA":0.,"phaseA2s":0.} for k in branches}
    total=ah=battery_a2s=0.
    bus_min=device_min=headroom=math.inf
    bad=floor_fail=cap_fail=0.
    worst_rail=worst_r=None
    admissible=math.inf
    cache={}
    csv_file=output.open("w",newline="",encoding="utf-8") if output else None
    writer=csv.writer(csv_file) if csv_file else None
    if writer:writer.writerow(["wall_s","phase","pdh_V","pack_A","minimum_terminal_V",*[k+"_A" for k in branches]])
    try:
        for t,stage,phase,watts,needed in samples:
            cache_key=tuple(watts.values())+tuple(needed.values())
            if cache_key not in cache:
                cap=critical(watts,needed,compressor,voc)
                try:
                    solved=electric.rail(voc,resistance,watts,{"PH":compressor})
                except ValueError:
                    solved=None
                cache[cache_key]=solved,cap
            solved,(limit,key)=cache[cache_key]
            if limit<admissible:admissible,worst_r=limit,[round(t,3),stage,key]
            maxed=any(phase[k][0]>branches[k]["phaseA"] for k in phase)
            if maxed:cap_fail+=DT
            if solved is None:
                bad+=DT
                continue
            bus,term,current=solved
            lowest=min(term,key=term.get)
            if term[lowest]<device_min:
                device_min=term[lowest]
                worst_rail=[round(t,3),stage,lowest]
            if term[lowest]<9.5:floor_fail+=DT
            bus_min=min(bus_min,bus)
            headroom=min(headroom,min(term[k]-v for k,v in needed.items()))
            amps=sum(current.values())
            ah+=amps*DT/3600
            total=max(total,amps)
            battery_a2s+=amps*amps*DT
            for k,a in current.items():
                v=stats[k]
                v["peakBranchA"]=max(v["peakBranchA"],a)
                v["branchA2s"]+=a*a*DT
                v["minTerminalV"]=min(v["minTerminalV"],term[k])
                if k in phase:
                    v["phaseA2s"]+=phase[k][0]**2*DT
                    v["peakPhaseA"]=max(v["peakPhaseA"],phase[k][0])
            if writer:
                writer.writerow([f"{t:.2f}",stage,f"{bus:.4f}",f"{amps:.4f}",f"{term[lowest]:.4f}",
                                 *[f"{current[k]:.4f}" for k in branches]])
    finally:
        if csv_file:csv_file.close()
    for k,b in stats.items():
        b["wireAndContactJ"]=b["branchA2s"]*electric.RESIST[k]
        b["branchRmsAOnSolvedTicks"]=math.sqrt(b["branchA2s"]/(156-bad))
        b["ratingA"]=branches[k]["ratingA"]
        b["maxRatingFraction"]=b["peakBranchA"]/b["ratingA"]
        if k in phase_keys:
            b["windingJ"]=b["phaseA2s"]*(electric.NEO_R if k in ("DriveLeft","DriveRight","Lift") else electric.NEO550_R)
            b["motorLeadJ"]=b["phaseA2s"]*electric.PHASE_R[k]
    return {"voc":voc,"commonOhm":resistance,"compressorA":compressor,"durationS":156,"peakBusA":total,
            "minBusV":bus_min,"minDeviceV":device_min,"worstTerminal":worst_rail,"minMotorHeadroomV":headroom,
            "belowFloorS":floor_fail,"unsolvedS":bad,"overPhaseCapS":cap_fail,"ampHoursOnSolvedTicks":ah,
            "packAndTrunkResistiveJ":battery_a2s*resistance,"maxCommonOhmAtThisVoc":admissible,
            "limitingFloorOrHeadroomSample":worst_r,"branches":stats}


def run(grips,out):
    electric.validate(list(branches.values()))
    result={}
    for name,hold,drag,stuck,settings in (
        ("stationHold",True,25,False,[("fixed",12.4,.025,0),("weakFixed",11.8,.032,0)]),
        ("park372",False,25,False,[("fixed",12.4,.025,0),("compressor",12.4,.025,15),
           ("weak",11.8,.032,15),("weakFixed",11.8,.032,0),("screenedPack",11.8,.027,0),
           ("depleted",10.9,.040,15)]),
        ("heavySlide",False,80,False,[("fixed",12.4,.025,0)]),
        ("jammedSlide",False,140,False,[("fixed",12.4,.025,0)]),
        ("stalledGrip",True,25,True,[("fixed",12.4,.025,0)])):
        samples=list(make_samples(grips,hold,drag,stuck))
        for label,voc,res,current in settings:
            result[name+"-"+label]=evaluate(samples,voc,res,current,out/(name+"-"+label+".csv"))
    return result


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("grips",type=Path)
    p.add_argument("output",type=Path)
    a=p.parse_args()
    a.output.mkdir(parents=True,exist_ok=True)
    grip=match.read(a.grips)
    result=run(grip,a.output)
    files=["fixed-height-match/placements.json","fixed-height-match/match.json","fixed-height-match/observer.json",
           "design/electrical-inspection/circuits.json","design/receiver-drives/sizing.json","fixed-low-power/retreat.json",
           "design/fixed-height/ports-065.json","design/drivetrain/route_check.py","design/electrical-inspection/replay.py"]
    report={"cadAndContactSourceHashes":{k:hashlib.sha256((ROOT/k).read_bytes()).hexdigest() for k in files},
            "gripTraceSha256":hashlib.sha256(a.grips.read_bytes()).hexdigest(),"gripMotorEnvelopeA":grip["peakPerBelt"],
            "independentMainOff":{"sourceConnected":False,"branchA":dict.fromkeys(branches,0),"observerAvailable":False,
            "motionAuthorized":False,"storedMechanicalEnergy":"unknown"},"cases":electric.round_values(result)}
    (HERE/"results.json").write_text(json.dumps(report,indent=2,allow_nan=False)+"\n",encoding="utf-8",newline="\n")
