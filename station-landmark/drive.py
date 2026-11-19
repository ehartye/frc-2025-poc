"""Staged, observed station approach. Distances in mm, time in seconds, heading in radians."""
import math
import sys

import numpy as np

from panel import ROOT,fit,acquire,render,track
sys.path.insert(0,str(ROOT/"selected-drive"))
import route

DT=.01
TRACK=580.


def path(raw, distance, waypoint_y=248, align_start=True,side=-310):
    f,legs,_,_=route.registry(raw,"nominal")
    original=legs[0][2]
    p=original.start
    intermediate=(440*f.IN,waypoint_y*f.IN)
    heading=math.atan2(intermediate[1]-p[1],intermediate[0]-p[0])
    r=f.Route((*p[:2],heading if align_start else p[2]))
    r.go(intermediate)
    dx,dy=f.rot((distance/1000,side/1000),f.NORMAL)
    dest=(f.DOCK[0]+dx,f.DOCK[1]+dy)
    r.go(dest)
    r.face(original.end[2]+.4*math.atan2(side,distance))
    return f,r


def propagate(q,ds,track_width):
    turn=(ds[1]-ds[0])/track_width
    increment=np.mean(ds)*np.sinc(turn/(2*math.pi))
    angle=q[2]+turn/2
    return q+[-increment*math.cos(angle),-increment*math.sin(angle),turn]


def approach(q,age,scale,track_width,depths,synthetic_bias,delay=.03,fault=None,side_bias=0,deadline=21.1):
    # An independent terminal stereo pose seeds a new field estimate; until that
    # measurement passes, the commanded wheel targets stay at zero.
    wheels=np.zeros(2)
    shaft=np.zeros(2)
    pred=None
    start=None
    confirm=[]
    pending=[]
    last=None
    seen=False
    stop=None
    arrived=False
    rows=[]
    max_speed=0.
    last_packet=None
    search_travel=0.
    for k in range(1601):
        t=round(k*DT,4)
        wall=age+t
        if wall>=deadline and stop is None:stop="feed deadline"
        if k%4==0:
            exposure=q.copy()
            # Perturb the relative camera-to-chassis registration without telling the controller.
            exposure[1]+=side_bias
            frames=render(exposure,depths,contrast=(fault!="noContrast"))
            mark=acquire(frames) if not seen else track(frames,pred,synthetic_bias)
            transit=.18 if fault=="late" and t>.4 else delay
            if fault=="interrupt" and t>.4:mark=None
            pending.append((t+transit,t,mark,shaft.copy()))
        while pending and pending[0][0] <= t+.00001:
            _,capture,mark,at_capture=pending.pop(0)
            if t-capture>.065:continue
            last_packet=capture
            if mark is None:continue
            # Last good estimator pose is a numerical starting guess only.
            base=pred if pred is not None else np.array([800.,0.,0.])
            pose,residual=fit(mark,synthetic_bias,base)
            if residual>1.0 or not np.isfinite(pose).all():continue
            if seen and (np.linalg.norm(pose[:2]-pred[:2])>80 or abs(pose[2]-pred[2])>.12):continue
            pred=propagate(pose,shaft-at_capture,TRACK)
            if not seen:
                start=pred.copy()
            seen=True;last=capture
            narrow=(abs(pred[0]-1.4)<=1.7 and abs(pred[1])<1.0 and abs(pred[2])<math.radians(.15))
            if narrow and max(abs(wheels))<9:arrived=True
            if narrow and arrived and max(abs(wheels))<1e-8 and np.isfinite(mark[:,:,0]).sum()>=6:
                confirm.append(capture)
            else:confirm.clear()
        if stop is None and not seen and last_packet is None and t>.12:stop="no fresh camera packet"
        if stop is None and not seen and (shaft.mean()>=195 or t>2.6):stop="search exhausted without unique stereo"
        if stop is None and seen and t-last>.28:stop="stale registered stereo"
        if stop is None and len(confirm)>=3:stop="observed stationary"
        wanted=np.zeros(2)
        # The official mode transition holds still until TELEOP opens at 18 s.
        if not seen and stop is None and last_packet is not None and t-last_packet<=.065 and not (15<=wall<18):
            # Before the first unique view, the encoder-bounded search may only
            # traverse the pre-screened final staging corridor.
            wanted[:]=100 if shaft.mean()<180 else 25
            search_travel=shaft.mean()
        if seen and not arrived and stop is None and t-last<=.095 and not (15<=wall<18):
            n,side,h=pred
            remaining=max(0,n)
            length=max(250,start[0])
            u=np.clip(1-remaining/length,0.,1.)
            y0=start[1];m0=-math.tan(start[2])
            ref=y0*(2*u**3-3*u*u+1)+length*m0*(u**3-2*u*u+u)
            slope=y0*(6*u*u-6*u)/length+m0*(3*u*u-4*u+1)
            curvature=-(y0*(12*u-6)/length**2+m0*(6*u-4)/length)/(1+slope*slope)**1.5
            desired=-math.atan(slope)+np.clip((side-ref)/170,-.16,.16)
            v=min(350,2.5*remaining,math.sqrt(2*190*max(0,remaining-2.)))
            if remaining<8:
                v=np.clip(1.5*n,-7,7)
                desired=0.
            omega=np.clip(v*curvature+5*(desired-h),-.7,.7)
            wanted=np.array([v-TRACK*omega/2,v+TRACK*omega/2])
        if seen and stop is None:
            # Per wheel, bound both one more scheduling interval at increasing
            # speed and a full finite stop. The sum bounds travel for the center
            # and for the farthest body point, even if the wheels differ in sign.
            gap=10+pred[0]-425*(abs(math.cos(pred[2]))+abs(math.sin(pred[2]))-1)
            allowance=gap-5-425*math.radians(.3)
            gain=1+2*math.hypot(425,425)/578
            safe_speed=max(0., math.sqrt(max(0,2*420*allowance/gain))-.025*420*2)
            max_want=max(abs(wanted))
            if max_want>safe_speed and max_want>0:wanted*=safe_speed/max_want
            if allowance<0:
                stop="wall guard";wanted[:]=0
        if wall<15:
            max_before_disable=max(0,420*(15-wall-.035))
            wanted=np.clip(wanted,-max_before_disable,max_before_disable)
        old=wheels.copy()
        wheels+=np.clip(wanted-wheels,-4.2,4.2)
        ds=.5*(old+wheels)*DT
        q=propagate(q,ds*scale,track_width)
        shaft+=ds
        if pred is not None:pred=propagate(pred,ds,TRACK)
        max_speed=max(max_speed,float(max(abs(wheels))))
        corner_gap=10+q[0]-425*(abs(math.cos(q[2]))+abs(math.sin(q[2]))-1)
        rows.append((wall,*q,corner_gap,float(max(abs(wheels)))))
        if stop is not None and max(abs(wheels))<1e-8:break
    return {"status":stop or "feed deadline","wallS":rows[-1][0],"finalStationMmDeg":[*q[:2],math.degrees(q[2])],
            "observedStationMmDeg":None if pred is None else [*pred[:2],math.degrees(pred[2])],
            "minimumStationPlaneMm":min(r[4] for r in rows),"peakWheelMmS":max_speed,
            "maxTransitionWheelMmS":max((r[5] for r in rows if 15<=r[0]<18),default=0.),
            "imagesConfirmed":len(confirm),"searchTravelMm":float(search_travel),"path":rows}


def entry(raw,scale,width,distance=925,side=-310):
    f,r=path(raw,distance,side=side)
    t=np.r_[np.arange(0,r.duration,DT),r.duration]
    poses=np.array([r.at(s)[0] for s in t])
    wheel=route.wheel_steps(poses)
    true=route.integrate(poses[0],wheel,scale,width/1000)
    actual=true[-1]
    x,y=f.rot(actual[:2]-f.DOCK[:2],-f.NORMAL)
    q=np.array([x*1000,y*1000,f.short(actual[2]-(f.NORMAL+math.pi))])
    return q,r.duration,f,r,t,true
