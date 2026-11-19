"""Carry the same acquired pipe along the physically attained route until the reef stop."""
import argparse
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace
import json
import math
from pathlib import Path
import sys
import numpy as np
from views import load_navigation

nav = load_navigation()
HERE, ROOT = nav.HERE, nav.ROOT
sys.path.insert(0,str(ROOT/"shelf-contact"))
from experiment import CompliantPipe, Shelf, GUIDE, feed
from carry import fictitious, momentum

def trajectory(assets, side, mode):
    directory = assets/"conditioned-match"/side/mode/"A"
    first = np.load(directory/"transit.npz")
    t0,p0 = first["times"], first["poses"]
    reference = (nav.read(HERE/"slow-navigation.json")[side] if mode=="slow" else nav.read(HERE/"navigation.json")[side][mode])
    visual = directory/"coral-A-nominal.csv"
    if nav.sha(visual) != reference["motionSHA256"]:
        raise ValueError("Changed sector route")
    q = np.loadtxt(visual,delimiter=",",skiprows=1)
    q[:,0] /= 1000
    q[:,1:3] /= 1000
    q[:,1] += nav.corrected().LIP[0]-.490
    q[:,2] += nav.corrected().LIP[1]
    q[:,3] += round((p0[-1,2]-q[0,3])/(2*np.pi))*2*np.pi
    if np.max(abs(p0[-1]-q[0,1:])) > 1.e-7:
        raise ValueError("Chassis discontinuity on leaving gyro service")
    return np.r_[t0,t0[-1]+q[1:,0]], np.vstack((p0,q[1:,1:]))

def replay(job):
    assets,side,mode,dt = job
    revised = mode.startswith("corrected") or mode=="slow"
    if revised:
        source = next(r for r in nav.read(HERE/"feeds.json") if r["case"]==side and r["dtS"]==.00003125)
        if source["state"] != "IN_BAND":
            return {"pipe":"coral-A","wheelCase":side,"mode":mode,"dtS":dt,"outcome":"station interlock withheld"}
        cam = nav.read(HERE/"seat-images.json")[side]
        raw = assets/"conditioned-match/corrected-feeds"/(side+"-400000-4-31.25.npy")
        cam_sha = cam["trajectorySha256"]
    else:
        source = next(r for r in nav.read(ROOT/"shelf-contact/fine-ports.json")["results"]
                      if r["case"] == side and r["dtS"] == .000015625)
        cam = nav.read(ROOT/"shelf-contact/fine-custody.json")["runs"][side]
        raw = assets/"shelf-contact/fine-ports"/(side+"-400000-4-15.625.npy")
        cam_sha = cam["trajectorySha256"]
    if nav.sha(raw) != cam_sha or cam_sha != source["trajectorySha256"]:
        raise ValueError("Unregistered pickup")
    state = np.load(raw,mmap_mode="r")[-1]
    if abs(state[0]-5460) > 1.e-6 or source["portTransitions"][-1][-1] != 0:
        raise ValueError("Missing stationary zero-belt handoff")
    times,poses=trajectory(assets,side,mode)
    start=nav.read(ROOT/"relieved-match/first-feed.json")["scheduledDriveWallS"]
    origin=21.671672635314284-.100
    wait=start-(origin+state[0]/1000)
    times=np.r_[0.,wait+times];poses=np.vstack((poses[0],poses))
    feed.cfg=replace(feed.cfg,slices=10)
    pipe=CompliantPipe(source["sourceRobotMmDeg"],0,replace(GUIDE,sectors=128,radial_bins=4),Shelf(stations=16))
    pipe.p=state[1:4].copy()/1000
    pipe.a=state[4:7].copy()
    pipe.v=state[7:10].copy()
    pipe.l=momentum(pipe.a,state[10:13])
    pipe.belt=0.
    pipe.touch=["continued from last identified pipe state"]
    hs=np.unwrap(poses[:,2])
    # Differentiate the sampled chassis motion; the initial source and reached stop have zero velocity.
    rates=np.column_stack([np.gradient(x,times) for x in (poses[:,0],poses[:,1],hs)])
    acceleration=np.column_stack([np.gradient(x,times) for x in rates.T])
    c,s=np.cos(hs),np.sin(hs)
    body=np.column_stack((c*acceleration[:,0]+s*acceleration[:,1],-s*acceleration[:,0]+c*acceleration[:,1],np.zeros(len(c))))
    idx=0; sample=[]; endpoints=[pipe.p.copy(),pipe.p.copy()]
    min_pad=math.inf;min_guide=math.inf
    steps=math.ceil(times[-1]/dt)
    for i in range(steps):
        t=i*dt
        step=min(dt,times[-1]-t)
        while idx+1 < len(times)-1 and times[idx+1]<=t: idx+=1
        frac=(t-times[idx])/(times[idx+1]-times[idx])
        lin=body[idx]+frac*(body[idx+1]-body[idx])
        omega=rates[idx,2]+frac*(rates[idx+1,2]-rates[idx,2])
        alpha=acceleration[idx,2]+frac*(acceleration[idx+1,2]-acceleration[idx,2])
        fictitious(pipe,lin,omega,alpha,step)
        pipe.moves(0.,step)
        p,a=pipe.p,pipe.a
        endpoints=[np.minimum(endpoints[0],p),np.maximum(endpoints[1],p)]
        # Enclose the cylinder in the tube about its finite axis; pad covers start at X125, top Z500.8 mm.
        dx=max(0,.125-p[0]-feed.HALF*abs(a[0]))
        dz=max(0,p[2]-feed.HALF*abs(a[2])-.5008)
        min_pad=min(min_pad,math.hypot(dx,dz)-feed.R)
        min_guide=min(min_guide,.625-p[2]-feed.HALF*abs(a[2])-feed.R*math.sqrt(1-a[2]**2))
        if i%round(.001/dt)==0 or pipe.hard or pipe.loss or i==steps-1:
            sample.append([t+step,*p,*a,*pipe.v,*pipe.angular(pipe.l)])
        if pipe.hard or pipe.loss:break
    cache=assets/"conditioned-match"/side/mode/"A"
    path=cache/f"cargo-{round(dt*1e9)}ns.npy"
    np.save(path,np.asarray(sample))
    return {"pipe":"coral-A","wheelCase":side,"mode":mode,"dtS":dt,"feedTrajectorySha256":nav.sha(raw),
            "lastStationTickWallS":origin+state[0]/1000,"timeToDriveS":wait,"driveWallS":start,"endWallS":origin+state[0]/1000+sample[-1][0],
            "centerRangeMm":[(v*1000).tolist() for v in endpoints],"minPadGapMm":1000*min_pad,"minGuideGapMm":1000*min_guide,
            "stop":pipe.hard,"loss":pipe.loss,"peakCartridgeMm":pipe.peak,"relativePipeSha256":nav.sha(path),"lastState":sample[-1]}

if __name__ == "__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("assets",type=Path)
    p.add_argument("--fine",action="store_true");p.add_argument("--corrected",action="store_true");p.add_argument("--slow",action="store_true");args=p.parse_args()
    jobs=[(args.assets,side,"slow" if args.slow else "corrected" if args.corrected else "gyro",dt) for side in ("leftSmall","rightSmall") for dt in ((.000125,.0000625) if args.corrected or args.slow else (.0000625,.00003125) if args.fine else (.00025,.000125))]
    with ProcessPoolExecutor(max_workers=4) as pool:
        rows=list(pool.map(replay,jobs))
    (HERE/("slow-cargo.json" if args.slow else "corrected-cargo.json" if args.corrected else "fine-cargo.json" if args.fine else "cargo.json")).write_text(json.dumps(rows,indent=2)+"\n",encoding="utf8",newline="\n")
    for row in rows:print(row["wheelCase"],row["dtS"],row.get("endWallS"),row.get("loss"),row.get("stop"),flush=True)
