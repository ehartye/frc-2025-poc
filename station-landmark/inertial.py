"""Independent sampled yaw-rate feedback on the preplanned approach to visual range."""
import math

import numpy as np

from drive import TRACK,path
from route import wheel_steps


def staged(raw,scale,width,bias=.0,gain_error=.0,start_heading_deg=0,latency_ms=20,drop_s=None,
           stage_mm=420,side_mm=0,pace=1.):
    f,r=path(raw,stage_mm,side=side_mm)
    dt=.005
    duration=r.duration/pace
    t=np.arange(math.ceil((duration+.55)/dt)+1)*dt
    nominal=np.array([r.at(min(s*pace,r.duration))[0] for s in t])
    nominal[:,2]=np.unwrap(nominal[:,2])
    ff=wheel_steps(nominal)*1000/dt
    truth=np.array([*r.start[:2],r.start[2]+math.radians(start_heading_deg)])
    actual=[truth.copy()]
    spun=0.;observed_r=0.;heading=r.start[2]
    age=0.;queue=[];w=np.zeros(2);latch=False;limit=0.;received=False
    drift_log=[]
    shaft_speeds=[w.copy()]
    recovery={}
    for i,now in enumerate(t[:-1]):
        if i%2==0:
            if drop_s is None or now<drop_s:
                # This sensor packet comprises the time integral of the preceding
                # physical angular rates projected onto the calibrated chassis up axis.
                queue.append((now+latency_ms/1000,now,spun*(1+gain_error)+math.radians(bias)*now))
        while queue and queue[0][0]<=now+1e-9:
            _,age,observed_r=queue.pop(0)
            received=True
        if (i==0 and drop_s==0) or now-age>.06:latch=True
        estimated=r.start[2]+observed_r+(now-age)*(w[1]-w[0])/TRACK
        yaw_request=np.clip(6*(nominal[i,2]-estimated),-.45,.45)
        targets=ff[i]+np.array([-1.,1.])*TRACK*yaw_request/2
        targets=np.clip(targets,-1250,1250)
        if latch or not received or now>duration+.30:targets[:]=0
        old=w.copy()
        # Source wheel ramp includes straight acceleration up to 1.20 m/s²;
        # retain the slower 0.64 m/s² wheel ramp during in-place turns.
        a=640 if abs(ff[i].mean())<1 and not latch else 1200
        w+=np.clip(targets-w,-a*dt,a*dt)
        limit=max(limit,float(max(abs(w))))
        ds=(w+old)*dt*.5*scale
        dh=(ds[1]-ds[0])/width
        length=ds.mean()*.001*np.sinc(dh/(2*np.pi))
        mid=truth[2]+dh/2
        truth=truth+[length*math.cos(mid),length*math.sin(mid),dh]
        spun+=dh;actual.append(truth.copy());shaft_speeds.append(w.copy())
        drift_log.append(abs(estimated-truth[2]))
        if latch and max(abs(w))<1e-6:break
    if latch and drop_s is not None and drop_s>0 and max(abs(w))<.00001:
        # Stay within AUTO and retrace the travel measured by BOTH healthy shaft
        # encoders. Replay the inverse sequence after a complete two-wheel stop.
        # Challenge the return with an additional, opposing wheel-ground change.
        source=np.array(shaft_speeds)
        expected=np.zeros(2);measured=np.zeros(2)
        reverse_start=len(actual)-1
        return_scale=scale*np.array([1.002,.998])
        for j in range(len(source)-2,-1,-1):
            expected-=.5*(source[j]+source[j+1])*dt
            wanted=-source[j]+3*(expected-measured+.5*(source[j]+source[j+1])*dt)
            old=w.copy();w+=np.clip(wanted-w,-1200*dt,1200*dt)
            shaft_step=.5*(old+w)*dt;measured+=shaft_step
            ds=shaft_step*return_scale
            dh=(ds[1]-ds[0])/(width+2)
            length=ds.mean()*.001*np.sinc(dh/(2*np.pi))
            mid=truth[2]+dh/2
            truth=truth+[length*math.cos(mid),length*math.sin(mid),dh]
            actual.append(truth.copy())
        recovery={"source":"paired recorded shaft samples","startS":reverse_start*dt,
                  "stopS":(len(actual)-1)*dt,"returnScaleMultiplier":[1.002,.998],"returnTrackExtraMm":2,
                  "shaftResidualMm":(measured-expected).tolist(),"fieldReturnErrorMm":1000*math.dist(truth[:2],r.start[:2])}
    actual=np.array(actual)
    # Do not silently reset a moving chassis at the visual handoff.
    residual_speed=float(max(abs(w)))
    times=np.arange(len(actual))*dt
    x,y=f.rot(actual[-1,:2]-f.DOCK[:2],-f.NORMAL)
    q=np.array([x*1000,y*1000,f.short(actual[-1,2]-(f.NORMAL+math.pi))])
    summary={"latched":latch,"peakWheelMmS":limit,"residualWheelMmS":residual_speed,
             "headingResidualDeg":math.degrees(truth[2]-(r.start[2] if recovery else nominal[min(len(actual)-1,len(nominal)-1),2])),
             "gyroLossRecovery":recovery,
             "sensorErrorPlusLatencyPeakDeg":math.degrees(max(drift_log))}
    return q,times[-1],f,r,times,actual,summary
