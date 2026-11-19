"""Sample the actual registered wheel paths and integrate scale and track perturbations."""
import csv
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
sys.path.insert(0,str(ROOT/"relieved-match"))
import field_trial as route
from navigation import placed_route

DT=.005
TRACK=.580

def read(p):return json.loads(p.read_bytes())

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def wheel_steps(path):
    heading=np.unwrap(path[:,2])
    turn=np.diff(heading);bearing=heading[:-1]+turn/2
    delta=np.diff(path[:,:2],axis=0)
    length=(delta[:,0]*np.cos(bearing)+delta[:,1]*np.sin(bearing))/np.sinc(turn/(2*np.pi))
    return np.column_stack((length-TRACK*turn/2,length+TRACK*turn/2))

def integrate(initial,arcs,scale,track,encoder_radii=(1.,1.),encoder_track=TRACK):
    # Physical travel for a fixed encoder distance. Apply center/yaw calibration separately.
    center=np.mean(arcs,axis=1)
    turn=np.diff(arcs,axis=1)[:,0]/TRACK
    calibrated=np.column_stack((center-encoder_track*turn/2,center+encoder_track*turn/2))
    wheels=calibrated*np.asarray(scale)/np.asarray(encoder_radii)
    turn=(wheels[:,1]-wheels[:,0])/track
    h=initial[2]+np.r_[0,np.cumsum(turn)]
    mid=h[:-1]+turn/2
    ds=np.mean(wheels,axis=1)*np.sinc(turn/(2*np.pi))
    return np.column_stack((initial[0]+np.r_[0,np.cumsum(ds*np.cos(mid))],
                            initial[1]+np.r_[0,np.cumsum(ds*np.sin(mid))],h))

def registry(raw,mode):
    plans=read(ROOT/"fixed-height-match/placements.json")["placements"]
    f,auto,_,_=route.field.scenario(plans,inboard=True)
    if mode.startswith("near"):
        case=read(ROOT/"leave-park/near-tape.json")["cases"]["near-tape"]
        auto=f.Route(tuple(case["start"]))
        speed,acc=f.DRIVE["maximumWheelMps"],f.DRIVE["maximumWheelAccelMps2"]
        try:
            if mode=="nearSlow":
                f.DRIVE.update(maximumWheelMps=.35,maximumWheelAccelMps2=.45)
            auto.go(case["finish"][:2])
        finally:
            f.DRIVE.update(maximumWheelMps=speed,maximumWheelAccelMps2=acc)
        return f,[(0.,"near tape",auto)],{},auto.start
    nav=read(ROOT/"relieved-match/navigation.json")["cases"]
    with (ROOT/"packet-recovery/events.csv").open() as fh:
        events={p:{r["event"]:int(r["wall_ms"])/1000 for r in rows} for p,rows in
                group_by_piece([r for r in csv.DictReader(fh) if r["scenario"]==mode]).items()}
    legs=[(0.,"AUTO",auto)]
    for plan in plans:
        piece=plan["id"];ev=events.get(piece,{})
        if "carry" not in ev:break
        n=next(r for r in nav if r["pipe"]==piece and r["mode"]=="nominal")
        nominal,before,_=placed_route(piece,n["actualStationMmDeg"])
        start=18+plan["drive"]
        legs.append((start,piece+" transit",before))
        path=raw/(piece+"-nominal.csv")
        if sha(path)!=n["motionSHA256"]:raise ValueError("Wrong steering pose trace")
        a=np.loadtxt(path,delimiter=",",skiprows=1);a[:,0]/=1000;a[:,1:3]/=1000
        a[:,1]+=f.LIP[0]-.490;a[:,2]+=f.LIP[1];a[:,3]=np.unwrap(a[:,3])
        # The source stores local yaw modulo a whole turn. Preserve the previous physical winding.
        a[:,3]+=round((before.end[2]-a[0,3])/(2*math.pi))*2*math.pi
        legs.append((start+before.duration,piece+" visual",a))
        if "stow" not in ev:break
        parking=mode=="single" or piece=="coral-B"
        onward=route.park_route(f,a[-1,1:],plan["sector"]) if parking else route.return_route(f,a[-1,1:])
        legs.append((ev["stow"],"park" if parking else "return",onward))
        if parking:break
    return f,legs,events,auto.start

def group_by_piece(rows):
    result={}
    for r in rows:result.setdefault(r["piece"],[]).append(r)
    return result

def samples(raw,mode):
    f,legs,events,initial=registry(raw,mode)
    t=np.arange(31201)*DT
    pose=np.tile(initial,(len(t),1));labels=np.full(len(t),"idle",dtype=object)
    for start,name,obj in legs:
        if isinstance(obj,np.ndarray):
            duration=obj[-1,0];end=obj[-1,1:]
            lookup=lambda elapsed: np.column_stack([np.interp(elapsed,obj[:,0],col) for col in obj[:,1:].T])
        else:
            duration=obj.duration;end=obj.end
            lookup=lambda elapsed: np.asarray([obj.at(u)[0] for u in elapsed])
        if math.dist(pose[np.searchsorted(t,start),:2],(obj[0,1:3] if isinstance(obj,np.ndarray) else obj.start[:2]))>.008:
            raise ValueError("Discontinuous field leg")
        on=(t>=start)&(t<start+duration);pose[on]=lookup(t[on]-start);labels[on]=name
        pose[t>=start+duration]=end
    pose[:,2]=np.unwrap(pose[:,2])
    wheel=wheel_steps(pose)
    return t,pose,wheel,labels[:-1],events,f,legs

def sensitivity(t,poses,arcs,labels,legs,f,case):
    errors={}
    end=legs[0][2].duration
    first=min(np.searchsorted(t,end),len(t)-1)
    for name,scale,track,cal_r,cal_b in (
        ("nominal",(1,1),TRACK,(1,1),TRACK),("bothPlus02pct",(1.002,1.002),TRACK,(1,1),TRACK),
        ("leftSmallRightLarge",(.998,1.002),TRACK,(1,1),TRACK),
        ("trackPlus2mm",(1,1),.582,(1,1),TRACK),
        ("calibrated",(.998,1.002),.582,(.998,1.002),.582)):
        p=integrate(poses[0],arcs,scale,track,cal_r,cal_b)
        # Through opening AUTO the issued wheel inputs are independent of station/reef sensing.
        # Any later pitch/slide controller is conditional on first attaining the same dock.
        err=p[first]-poses[first]
        residual=1000*np.linalg.norm(p[:first+1,:2]-poses[:first+1,:2],axis=1)
        support=residual+1000*2*.425*math.sqrt(2)*abs(np.sin((p[:first+1,2]-poses[:first+1,2])/2))
        # Distance to the actual near station face from the front corner; normal faces into field.
        normal=f.NORMAL
        rel=f.rot((p[first,0]-f.DOCK[0],p[first,1]-f.DOCK[1]),-normal)
        angle=f.short(p[first,2]-poses[first,2])
        gap=10+1000*rel[0]-425*(abs(math.sin(angle))+abs(math.cos(angle))-1)
        errors[name]={"openingEndS":end,"XYerrorMm":(err[:2]*1000).tolist(),"yawErrorDeg":math.degrees(err[2]),
                      "openingMaxHullDisplacementMm":float(max(support)),"normalStationGapMm":gap if not case.startswith("near") else None}
    return errors
