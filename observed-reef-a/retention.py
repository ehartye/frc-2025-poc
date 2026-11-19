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
from belts import BeltDrive



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
    assets,side,step,variant,servo,elastic,head,quantized,stuck,limited=job
    camera_command=[sys.executable,str(HERE/"roof_trim.py"),str(assets)]
    if elastic:camera_command.extend([str(elastic),head])
    with subprocess.Popen(camera_command,stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True,bufsize=1) as camera:
        try:
            return propagate(assets,side,step,variant,servo,camera,elastic,head,quantized,stuck,limited)
        finally:
            camera.stdin.close()


def propagate(assets,side,step,variant,servo,camera,elastic,head,quantized,stuck,limited):
    feed.cfg=replace(feed.cfg,slices=10)
    if elastic:
        import importlib.util
        sys.path.insert(0,str(elastic))
        spec=importlib.util.spec_from_file_location("registered_elastic_drive",elastic/"drive.py")
        motion=importlib.util.module_from_spec(spec)
        # The two released project runners have unrelated modules named rehearse.
        # Limit the legacy module binding to loading the CAD/modal package.
        previous=sys.modules.pop("rehearse",None)
        try:
            spec.loader.exec_module(motion)
        finally:
            if previous is not None:
                sys.modules["rehearse"]=previous
        issued=next(r for r in json.loads((elastic/"departure.json").read_bytes())["cases"] if r["case"]==side)
        row=next(r for r in nav.read(ROOT/"conditioned-match/elastic.json")["cases"] if r["trajectorySha256"]==issued["stationTrajectorySha256"])
        path=assets/"conditioned-match/coupled-corrected"/(row["id"]+".npy")
        pipe=motion.restore(assets/"elastic-carry/checkpoints"/(side+"-state.npz"),issued)
        if type(pipe).belt_surface_speeds is not feed.Pipe.belt_surface_speeds:
            raise ValueError("Coupled receiver did not bind the active surface law")
        modal_max=np.max(np.abs(pipe.table),axis=(0,1))
    else:
        issued=None
        row=next(x for x in nav.read(ROOT/"conditioned-match/feeds.json") if x["case"]==side and x["dtS"]==.00003125)
        path=assets/"conditioned-match/corrected-feeds"/(side+"-400000-4-31.25.npy")
        pipe=CompliantPipe(row["sourceRobotMmDeg"],0,replace(GUIDE,sectors=128,radial_bins=4),Shelf(stations=16))
    if nav.sha(path)!=row["trajectorySha256"]:
        raise ValueError("Different station pipe")
    source=np.load(path,mmap_mode="r")
    state=source[-1]
    start_java=int(round(state[0]));origin=21.571672635314284
    if issued:
        if max(abs(pipe.p-state[1:4]/1000))>1.e-7 or pipe.belt!=0.:
            raise ValueError("Elastic checkpoint and final camera source disagree")
    else:
        pipe.p=state[1:4].copy()/1000
        pipe.a=state[4:7].copy()
        pipe.v=state[7:10].copy()
        pipe.l=momentum(pipe.a,state[10:13])
        pipe.touch=["continued from the station pipe"]
    acceleration=min(2.4,feed.cfg.belt_accel) if limited else feed.cfg.belt_accel
    actuator=BeltDrive(acceleration,quantized,stuck)
    pipe.belt_drive=actuator
    max_deflection=0.
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
        if actuator.follow_fault:
            fault=fault or [java,"idler failed to follow requested travel"]
        worst_yaw=max(worst_yaw,abs(math.degrees(math.atan2(pipe.a[0],pipe.a[1]))))
        min_guide_mm=min(min_guide_mm,1000*(.625-pipe.p[2]-feed.HALF*abs(pipe.a[2])-feed.R*math.sqrt(1-pipe.a[2]**2)))
        peak=max(peak,max(abs(actuator.speed)))
        if issued:
            max_deflection=max(max_deflection,1000*(abs(pipe.z)@modal_max[0]))
        if i%sampling==0 or pipe.hard or pipe.loss or i==math.ceil(times[-1]/step)-1:
            motions.append([t+dt,*pipe.p,*pipe.a,*pipe.v,*pipe.angular(pipe.l),*actuator.speed,*([*pipe.z,*pipe.zdot,*pipe.plunger] if issued else [])])
        if pipe.hard or pipe.loss or fault:
            break
    name=f"{side}-{variant}-{'active' if servo else 'passive'}-{int(round(step*1e9))}ns"
    qualifier=("machined-"+head+"-" if elastic else "")+("limited-" if limited else "")+("quantized-" if quantized else "")+("stuck-" if stuck else "")
    name=qualifier+name
    cache=assets/"observed-reef-a";np.save(cache/(name+".npy"),np.asarray(motions));motion_path=cache/(name+".npy")
    frame_path=cache/(name+"-frames.json")
    frame_path.write_text(json.dumps(frames,separators=(",",":")),encoding="utf8")
    return {"id":name,"piece":"coral-A","wheelCase":side,"mode":variant,"stepS":step,"trimActive":servo,
            "elasticStateSha256":issued["stateSha256"] if issued else None,"surfaceQuantized":quantized,"surfaceStuck":stuck,
            "actuatorAccelerationCapMps2":acceleration,"peakIdlerCommandErrorMm":1000*actuator.peak_position_error,"peakPlateDeflectionBoundMm":max_deflection if issued else None,
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
    p.add_argument("--elastic",type=Path);p.add_argument("--head",default="roof");p.add_argument("--quantized",action="store_true");p.add_argument("--stuck",action="store_true")
    p.add_argument("--limited",action="store_true",help="Use the 2.4 m/s² cap screened against reflected rotor inertia")
    a=p.parse_args();jobs=[(a.assets,side,.0000625 if a.fine else .000125,a.mode,not a.passive,a.elastic,a.head,a.quantized,a.stuck,a.limited) for side in ("leftSmall","rightSmall")]
    with ProcessPoolExecutor(max_workers=2) as pool:
        rows=list(pool.map(run,jobs))
    name="cargo-"+a.mode+("-passive" if a.passive else "-active")+("-fine" if a.fine else "")
    name=("machined-"+a.head+"-" if a.elastic else "")+("limited-" if a.limited else "")+("quantized-" if a.quantized else "")+("stuck-" if a.stuck else "")+name
    (HERE/(name+".json")).write_text(json.dumps(rows,indent=2)+"\n",encoding="utf8",newline="\n")
    for row in rows:
        print(row["id"],row["plannedEndpoint"],row["firstFault"],row["physicalStop"],row["finalWallS"],flush=True)
