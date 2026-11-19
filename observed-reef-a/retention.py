"""Carry the identified seated pipe with capped independent belt trims from fresh roof pixels."""
import argparse
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace
import hashlib
import json
import math
from pathlib import Path
import sys
import subprocess

import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
sys.path.insert(0,str(ROOT/"conditioned-match"))
from transport import nav,CompliantPipe,GUIDE,Shelf,feed,fictitious,momentum



class TrimPipe(CompliantPipe):
    def __init__(self,delta):
        super().__init__(delta,0,replace(GUIDE,sectors=128,radial_bins=4),Shelf(stations=16))
        self.speed=np.zeros(2)

    def belt_surface_speeds(self,targets,dt):
        self.speed+=np.clip(targets-self.speed,-feed.cfg.belt_accel*dt,feed.cfg.belt_accel*dt)
        return self.speed


def path_for(assets,side,variant):
    original=nav.read(ROOT/"conditioned-match/slow-navigation.json")[side]
    tr=assets/"conditioned-match"/side/"slow/A/transit.npz"
    if nav.sha(tr)!=original["transitSha256"]:
        raise ValueError("Different registered gyro approach")
    raw=np.load(tr)
    path=assets/"observed-reef-a"/side/("coral-A-"+variant+".csv")
    trial=next(x for x in nav.read(HERE/"field-trials.json") if x["wheelCase"]==side and x["mode"]==variant)
    if nav.sha(path)!=trial["motionSHA256"]:
        raise ValueError("Different field observation or wheel motion")
    q=np.loadtxt(path,delimiter=",",skiprows=1)
    q[:,1:3]/=1000
    q[:,1]+=nav.corrected().LIP[0]-.490
    q[:,2]+=nav.corrected().LIP[1]
    pose=raw["poses"]
    q[:,3]+=round((pose[-1,2]-q[0,3])/(2*math.pi))*2*math.pi
    if np.max(abs(pose[-1]-q[0,1:]))>1.e-7:
        raise ValueError("Discontinuous wheel pose")
    times=raw["times"]
    return np.r_[times,times[-1]+q[1:,0]/1000],np.vstack((pose,q[1:,1:])),trial


def run(job):
    assets,side,step,variant,servo=job
    with subprocess.Popen([sys.executable,str(HERE/"roof_trim.py"),str(assets)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True,bufsize=1) as camera:
        try:
            return propagate(assets,side,step,variant,servo,camera)
        finally:
            camera.stdin.close()


def propagate(assets,side,step,variant,servo,camera):
    feed.cfg=replace(feed.cfg,slices=10)
    row=next(x for x in nav.read(ROOT/"conditioned-match/feeds.json") if x["case"]==side and x["dtS"]==.00003125)
    path=assets/"conditioned-match/corrected-feeds"/(side+"-400000-4-31.25.npy")
    if nav.sha(path)!=row["trajectorySha256"]:
        raise ValueError("Different station pipe")
    source=np.load(path,mmap_mode="r")
    state=source[-1]
    start_java=int(round(state[0]));origin=21.571672635314284
    pipe=TrimPipe(row["sourceRobotMmDeg"])
    pipe.p=state[1:4].copy()/1000
    pipe.a=state[4:7].copy()
    pipe.v=state[7:10].copy()
    pipe.l=momentum(pipe.a,state[10:13])
    pipe.touch=["continued from the station pipe"]
    times,pose,field=path_for(assets,side,variant)
    wait=nav.read(ROOT/"relieved-match/first-feed.json")["scheduledDriveWallS"]-(origin+start_java/1000)
    times=np.r_[0.,wait+times];pose=np.vstack((pose[0],pose))
    # A constrained stop is a separate run. This trajectory is the planned wheel path,
    # and custody evidence ends the instant a fresh observation fails the inherited gate.
    yaw=np.unwrap(pose[:,2])
    rates=np.column_stack([np.gradient(x,times) for x in (pose[:,0],pose[:,1],yaw)])
    accel=np.column_stack([np.gradient(x,times) for x in rates.T])
    c,s=np.cos(yaw),np.sin(yaw)
    body=np.column_stack((c*accel[:,0]+s*accel[:,1],-s*accel[:,0]+c*accel[:,1],np.zeros(len(c))))
    pending=[]
    prior=np.array([np.interp(5457,source[:,0],v) for v in source[:,1:7].T])
    frames=[];last_good=5440;fault=None;targets=np.zeros(2);peak=0.
    seeks=0;motions=[];worst_yaw=0.;min_guide_mm=1e9
    def sample(capture,center,axis):
        camera.stdin.write(json.dumps([*center,*axis])+"\n");camera.stdin.flush()
        response=camera.stdout.readline()
        if not response:
            raise RuntimeError("The camera process ended before the requested exposure")
        fit,angle,key=json.loads(response)
        bounds=fit[1:]
        enclosed=fit[0]=="inside" and angle is not None and all(lo<=x<=hi for x,lo,hi in zip((center[0],center[1],center[2]-557.15),bounds[::2],bounds[1::2]))
        actual_yaw=math.atan2(axis[0],axis[1]);yaw_ok=angle is not None and angle[0]<=actual_yaw<=angle[1]
        if fit[0]=="inside" and (not enclosed or not yaw_ok):
            raise ValueError("Independent source truth outside the accepted image intervals")
        target=np.zeros(2)
        if fit[0]=="inside" and angle is not None and servo:
            common=np.clip(-.8*((bounds[0]+bounds[1])/2-75.)/1000,-.014,.014)
            trim=np.clip(.24*sum(angle)/2,-.018,.018)
            target=np.array([common+trim,common-trim])
        tick=5*math.ceil((capture+15)/5)
        pending.append((tick,capture,fit[0],target))
        frames.append((capture,tick,fit[0],key,enclosed,yaw_ok,target.tolist()))
    sample(5457,prior[:3],prior[3:])
    sampling=round(.001/step);drive_step=round(.005/step)
    for i in range(math.ceil(times[-1]/step)):
        t=i*step
        dt=min(step,times[-1]-t)
        java=start_java+round(1000*t)
        if i%sampling==0 and java%17==0:
            sample(java,1000*pipe.p,pipe.a)
        if i%drive_step==0:
            while pending and pending[0][0]<=java:
                _,stamp,status,command=pending.pop(0)
                if status!="inside":
                    fault=fault or [java,status]
                else:
                    last_good=stamp;targets=command
            if java-last_good>45:
                fault=fault or [java,"stale roof sample"]
            if fault:
                targets=np.zeros(2)
        while seeks+1<len(times)-1 and times[seeks+1]<=t:
            seeks+=1
        f=(t-times[seeks])/(times[seeks+1]-times[seeks])
        local=body[seeks]+f*(body[seeks+1]-body[seeks])
        turn=rates[seeks,2]+f*(rates[seeks+1,2]-rates[seeks,2])
        alpha=accel[seeks,2]+f*(accel[seeks+1,2]-accel[seeks,2])
        fictitious(pipe,local,turn,alpha,dt)
        pipe.moves(targets,dt)
        worst_yaw=max(worst_yaw,abs(math.degrees(math.atan2(pipe.a[0],pipe.a[1]))))
        min_guide_mm=min(min_guide_mm,1000*(.625-pipe.p[2]-feed.HALF*abs(pipe.a[2])-feed.R*math.sqrt(1-pipe.a[2]**2)))
        peak=max(peak,max(abs(pipe.speed)))
        if i%sampling==0 or pipe.hard or pipe.loss or i==math.ceil(times[-1]/step)-1:
            motions.append([t+dt,*pipe.p,*pipe.a,*pipe.v,*pipe.angular(pipe.l),*pipe.speed])
        if pipe.hard or pipe.loss or fault:
            break
    name=f"{side}-{variant}-{'active' if servo else 'passive'}-{int(round(step*1e9))}ns"
    cache=assets/"observed-reef-a";np.save(cache/(name+".npy"),np.asarray(motions));motion_path=cache/(name+".npy")
    frame_path=cache/(name+"-frames.json")
    frame_path.write_text(json.dumps(frames,separators=(",",":")),encoding="utf8")
    return {"id":name,"piece":"coral-A","wheelCase":side,"mode":variant,"stepS":step,"trimActive":servo,
            "sourceFeedSha256":row["trajectorySha256"],"sourceFieldPathSha256":field["motionSHA256"],
            "fieldPermitted":field["accepted"],"sourceStationWallS":origin+start_java/1000,
            "finalWallS":origin+start_java/1000+motions[-1][0],"physicalStop":pipe.hard,"lostPipe":pipe.loss,
            "lastObservedJavaMs":last_good,"firstFault":fault,"plannedEndpoint":not(pipe.hard or pipe.loss or fault),
            "roofFrames":len(frames),"peakSurfaceMps":peak,"peakPipeYawDeg":worst_yaw,"peakBeltCellForceN":pipe.peak_load,
            "minGuideGapMm":min_guide_mm,"maxCartridgeMm":pipe.peak,"lastPipe":motions[-1],
            "trajectorySha256":nav.sha(motion_path),"framesSha256":nav.sha(frame_path)}


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("assets",type=Path)
    p.add_argument("--fine",action="store_true");p.add_argument("--mode",default="nominal");p.add_argument("--passive",action="store_true")
    a=p.parse_args();jobs=[(a.assets,side,.0000625 if a.fine else .000125,a.mode,not a.passive) for side in ("leftSmall","rightSmall")]
    with ProcessPoolExecutor(max_workers=2) as pool:
        rows=list(pool.map(run,jobs))
    name="cargo-"+a.mode+("-passive" if a.passive else "-active")+("-fine" if a.fine else "")
    (HERE/(name+".json")).write_text(json.dumps(rows,indent=2)+"\n",encoding="utf8",newline="\n")
    for row in rows:
        print(row["id"],row["plannedEndpoint"],row["firstFault"],row["physicalStop"],row["finalWallS"],flush=True)
