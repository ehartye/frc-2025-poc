"""Differential drive docking replay: delayed pixel estimates, encoder transport and a finite interlock."""
import argparse
from collections import deque
import hashlib
import json
import math
from pathlib import Path
import sys
import numpy as np
from vision import station_frame, associate, estimate, interval, rotation
from scene import context,HERE

ROOT=HERE.parent
TRACK=580.
DT=.005
STRAIGHT=942.1634807291674
LEG_S=1.772158270517589
STOP_S=LEG_S+1.5
SECTORS={"coral-A":-180.,"coral-B":220.}
def integrate(p,left,right,track_mm=TRACK):
    turn=(right-left)/track_mm
    heading=p[2]+turn/2
    ds=(left+right)/2*np.sinc(turn/(2*np.pi))
    return p+np.array([ds*np.cos(heading),ds*np.sin(heading),turn])
def relative(a,b):
    return np.r_[rotation(-a[2])@(b[:2]-a[:2]),b[2]-a[2]]
def compose(a,delta):
    return np.r_[a[:2]+rotation(a[2])@delta[:2],a[2]+delta[2]]
def tracecase(pipe,initial,mode,ctx,cache,*,wheel_scale=None,physical_track_mm=TRACK,observer=station_frame,features=None):
    sector=SECTORS[pipe];goal=np.array([0.,sector,0.])
    actual=np.array([-STRAIGHT+initial[0],sector+initial[1],math.radians(initial[2])])
    odom=np.array([-STRAIGHT,sector,0.]);pred=odom.copy()
    wheels=np.zeros(2);distance=np.zeros(2);old_ticks=np.zeros(2)
    bias=np.array([.998,1.002]) if mode=="wheel-bias" else np.ones(2)
    if wheel_scale is not None:
        bias = np.asarray(wheel_scale, dtype=float)
    history={};packets=deque();path=[];assoc=False;last_seen=-10000
    source=None;bad=None;finished=None;strong=0;issued=0;rejected=0;optical_max=0.
    closest=None;last_fit=None;complete=[];max_error=0.;max_rate=0.;max_acc=0.;last_pair=None
    seen_tag=0;seen_heads=0;origin=None;last_gate_frame=-1;final_fit=None;admission_truth_error=False;terminal_pixel_max=0.;last_error=0.
    for tick in range(0,round(STOP_S*1000)+650,5):
        history[tick]=odom.copy()
        complete.append([tick,*actual])
        if tick%35==0 and tick<=STOP_S*1000:
            tag=11 if mode=="wrong-face" else 10
            duplicate=mode=="ambiguous" and tick>=1500
            hide=mode=="dropout" and tick>=1600
            image_pose=actual.copy()
            if mode=="shared-survey-shift": image_pose[1]-=14
            sample=observer(image_pose,ctx,tag,duplicate,hide)
            delay=75 if mode=="stale" and tick>=1500 else 15
            packets.append((tick+delay,tick,sample))
        while packets and packets[0][0]<=tick:
            _,capt,(tag,heads,truth,errors)=packets.popleft()
            if tick-capt>40:
                rejected+=1;continue
            if tag is not None:
                # Reject a decoded tag grossly inconsistent with the delayed encoder prediction.
                ref=compose(pred,relative(odom,history[capt]))
                if np.linalg.norm(tag[:2]-ref[:2])<100 and abs(tag[2]-ref[2])<math.radians(7):
                    assoc=True;seen_tag+=1;last_seen=capt
                    pred=compose(tag,relative(history[capt],odom));source="ID10"

            begin=compose(pred,relative(odom,history[capt]))
            heads_pair=None
            if assoc:
                heads_pair=associate(heads,ctx["bolts"],begin) if features is None else features.identify(heads,begin)
            if heads_pair is not None:
                # Only identified, ordered observations enter the estimate. Truth is used later for audit.
                raw=estimate(heads_pair,ctx["bolts"]) if features is None else features.pose(heads_pair,begin)
                if np.linalg.norm(raw[:2]-begin[:2])<45 and abs(raw[2]-begin[2])<math.radians(4):
                    pred=compose(raw,relative(history[capt],odom));source="paired heads";seen_heads+=1;last_seen=capt
                    last_pair=(capt,heads_pair)
                    if origin is None:origin=pred.copy()
                    last_error=float(np.max(abs(np.asarray(heads_pair)-truth))) if features is None else features.error(heads_pair,truth)
                    optical_max=max(optical_max,last_error)
                else:rejected+=1
            elif tag is None:rejected+=1
        if tick<=round(STOP_S*1000) and bad is None:
            if assoc and tick-last_seen>90:bad="lost registered field view"
            if not assoc and tick>=1250:bad="no unique ID10 registration"
        rx=-pred[0];dy=sector-pred[1]
        if bad is None and finished is None and tick<STOP_S*1000:
            # Blend steering to a shallow intercept at distance, and square the chassis as the reach closes.
            speed=np.clip(4.5*rx,0,620)
            speed=min(speed,math.sqrt(max(0,2*505*(rx-0.3))))
            heading=0.;curvature=0.
            if origin is not None:
                length=max(350.,-origin[0])
                u=float(np.clip((pred[0]-origin[0])/length,0,1))
                offset=origin[1]-sector
                slope=math.tan(origin[2])
                desired_y=sector+offset*(2*u**3-3*u*u+1)+length*slope*(u**3-2*u*u+u)
                first=offset*(6*u*u-6*u)/length+slope*(3*u*u-4*u+1)
                second=offset*(12*u-6)/length**2+slope*(6*u-4)/length
                heading=math.atan(first)+np.clip((desired_y-pred[1])/280.,-.08,.08)*np.clip(rx/90.,0,1)
                curvature=second/(1+first**2)**1.5
            yaw_rate=np.clip(speed*curvature+5.2*(heading-pred[2]),-.7,.7)
            if abs(rx)<8:
                speed=np.clip(4.5*rx,-16,16)
                yaw_rate=np.clip(-7*pred[2],-.5,.5)
            want=np.array([speed-TRACK*yaw_rate/2,speed+TRACK*yaw_rate/2])
        else:want=np.zeros(2)
        prior=wheels.copy()
        v0=wheels.mean();w0=(wheels[1]-wheels[0])/TRACK
        v_cmd=want.mean();w_cmd=(want[1]-want[0])/TRACK
        v0+=np.clip(v_cmd-v0,-850*DT,850*DT)
        w0+=np.clip(w_cmd-w0,-1.2*DT,1.2*DT)
        wheels=np.array([v0-TRACK*w0/2,v0+TRACK*w0/2])
        max_rate=max(max_rate,max(abs(wheels)));max_acc=max(max_acc,max(abs(wheels-prior))/DT)
        if tick%35==0 and tick>2000 and last_pair and tick-last_pair[0]<=55 and max(abs(wheels))<9 and bad is None:
            cap,pair=last_pair
            bounds=interval(pair,ctx["bolts"],goal[:2],epsilon=.18) if features is None else features.bounds(pair,goal[:2])
            if bounds is not None:
                lo,hi,radial=bounds
                # The stationary gate includes remaining odometry transport, braking and independent survey errors.
                ds=np.linalg.norm(odom[:2]-history[cap][:2])
                radius=radial+1.85+ds
                yaw=max(abs(lo[2]),abs(hi[2]))*180/np.pi+.14+abs(odom[2]-history[cap][2])*180/np.pi
                closest=[float(radius),float(yaw)]
                if radius<=10 and yaw<=1 and cap!=last_gate_frame:
                    last_gate_frame=cap;final_fit=closest
                    terminal_pixel_max=max(terminal_pixel_max,last_error)
                    if terminal_pixel_max>.18:raise ValueError("Terminal pixel fixture exceeds advertised budget")
                    if not (np.all(actual>=lo-np.r_[ds+1.85,ds+1.85,math.radians(.14)]) and np.all(actual<=hi+np.r_[ds+1.85,ds+1.85,math.radians(.14)])):
                        admission_truth_error=True
                        if mode!="shared-survey-shift":raise ValueError("Admitted observation misses its independent simulated pose")
                    strong+=1
                    if strong==3 and finished is None:finished=tick;last_fit=closest
                elif radius>10 or yaw>1:
                    strong=0
                    if finished is not None:bad="pose drift after initial admission"
        if tick>=STOP_S*1000 and bad is None:
            if finished is None or tick-last_gate_frame>65:bad="pose interval or stopping deadline"
        if abs(wheels).max()<.001 and tick>=STOP_S*1000 and (bad or finished is not None):break
        step=wheels*DT
        distance+=step
        ticks=np.rint(distance/.158)*.158
        nxt=integrate(odom,*(ticks-old_ticks));pred=compose(pred,relative(odom,nxt))
        old_ticks=ticks;odom=nxt
        actual=integrate(actual,*(step*bias),track_mm=physical_track_mm)
        if mode=="lateral-kick" and 760<=tick<880:
            phase=(tick-760)/120
            actual[1]+=6*(math.cos(math.pi*phase)-math.cos(math.pi*(phase+5/120)))
        if tick%50==0:path.append([tick,*np.round(actual[:2],3),round(math.degrees(actual[2]),3),source])
        max_error=max(max_error,np.linalg.norm(actual[:2]-pred[:2]))
    trace_path=cache/f"{pipe}-{mode}.csv"
    np.savetxt(trace_path,complete,fmt=["%d","%.9f","%.9f","%.12f"],delimiter=",",header="ms,x_mm,y_mm,yaw_rad",comments="")
    return {"motionSHA256":hashlib.sha256(trace_path.read_bytes()).hexdigest(),"pipe":pipe,"mode":mode,"initial":initial,"accepted":finished is not None and bad is None,"decisionMs":finished,"reason":bad,
            "finalBodyErrorMmDeg":[*np.round((actual-goal)[:2],5),round(math.degrees(actual[2]),5)],"acceptanceDiskMmYawDeg":last_fit,
            "latestInterval":closest,"latestQualifiedInterval":final_fit,"lastGateCaptureMs":last_gate_frame,"truthOutsideDeclaredCalibration":admission_truth_error,"decodedTagFrames":seen_tag,"pairedHeadFrames":seen_heads,"rejected":rejected,"maxHeadPixelError":optical_max,"terminalCheckPixelError":terminal_pixel_max,
            "peakWheelMmps":max_rate,"peakWheelMmps2":max_acc,"actualStopAfterStartMs":tick,"path":path}

def run(cache):
    ctx=context(cache)
    results=[]
    for p in SECTORS:
        for error,mode in [([0,0,0],"nominal"),([0,35,2],"positive"),([0,-35,-2],"negative"),([21,28,-1.5],"wheel-bias"),
                ([-21,-28,1.5],"lateral-kick"),([0,0,0],"wrong-face"),([0,0,0],"ambiguous"),([0,0,0],"dropout"),([0,0,0],"stale"),([0,0,0],"shared-survey-shift")]:
            r=tracecase(p,error,mode,ctx,cache);results.append(r)
            print(p,mode,r["accepted"],r["decisionMs"],r["finalBodyErrorMmDeg"],r["latestInterval"],r["maxHeadPixelError"],r["reason"],flush=True)
    (HERE/"results.json").write_text(json.dumps(results,indent=2)+"\n",encoding="utf8",newline="\n")
if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("cache",type=Path);run(p.parse_args().cache)
