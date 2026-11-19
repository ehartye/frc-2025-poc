"""Render the two cameras on each 597-body path, from station custody to the first field contact."""
import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np

from sight import HERE, ROOT, optics, camera, silhouette, background, bridge_fit, bridge, sha


def read(p):
    return json.loads(p.read_bytes())


def state_at(piece,tick,feed,source,cargo,pickup,target=None):
    local_to_drive=source["scheduledDriveCameraMs"]
    local_to_edge=local_to_drive+1000*(cargo["wallS"]-cargo["driveWallS"])
    if tick<local_to_drive:
        xyz,axis=np.split(np.array([np.interp(tick,pickup[:,0],col) for col in pickup[:,1:7].T]),2)
        return xyz,axis/np.linalg.norm(axis),0.
    if tick<=local_to_edge:
        t=(tick-local_to_drive)/1000
        xyz,axis=np.split(np.array([np.interp(t,feed[:,0],col) for col in feed[:,1:7].T]),2)
        return 1000*xyz,axis/np.linalg.norm(axis),1000*np.interp(t,feed[:,0],feed[:,16])
    tau=(tick-local_to_edge)/1000
    angle=cargo["chassisHeadingRad"]
    c,s=math.cos(angle),math.sin(angle)
    rotation=np.array([[c,s,0],[-s,c,0],[0,0,1]])
    pos=np.array(cargo["positionM"])+tau*np.array(cargo["velocityMps"])-[0,0,.5*9.80665*tau*tau]
    a=np.array(cargo["axis"]);w=np.array(cargo["angularRadps"])
    # Axisymmetric free top: in flight the cylinder axis precesses about fixed world angular momentum.
    radii_squared=camera.R**2+50.8**2
    inertia_ratio=6*radii_squared/(3*radii_squared+4*camera.HALF**2)
    precession=w+(inertia_ratio-1)*a*(a@w)
    speed=np.linalg.norm(precession)
    if speed:
        n=precession/speed
        a=a*math.cos(speed*tau)+np.cross(n,a)*math.sin(speed*tau)+n*(n@a)*(1-math.cos(speed*tau))
    if target is not None:
        if tau<target[0,0] or tau>target[-1,0]:raise ValueError("Unobserved target time")
        v=np.array([np.interp(tau,target[:,0],col) for col in target[:,1:].T])
        pos=v[:3];a=v[3:]/np.linalg.norm(v[3:])
    local=1000*(rotation@(pos-np.r_[cargo["chassisAtReefXYm"],0.]))
    local[0]-=370
    return local,rotation@a,370.


def run(cache,piece,head):
    label="first" if piece=="coral-A" else "return"
    source=read(HERE/(label+"-feed.json"))["cases"][0]
    pickup=np.load(cache/(label+"-nominal.npy"))
    cargo=read(HERE/(piece+"-pads-edge-250.json"))
    raw=cache/(piece+"-pads-edge-250.npy")
    if sha(raw)!=cargo["rawSha256"] or sha(cache/(label+"-nominal.npy"))!=source["trajectorySHA256"]:
        raise ValueError("Trajectory identity mismatch")
    path=np.load(raw)
    release=source["scheduledDriveCameraMs"]+1000*(cargo["releaseRequestedWallS"]-cargo["driveWallS"])
    edge=source["scheduledDriveCameraMs"]+1000*(cargo["wallS"]-cargo["driveWallS"])
    # The first observed field contact is from the new finite-tube/target solver.
    with (HERE/"landing-windows.csv").open() as f:
        landing=next(r for r in csv.DictReader(f) if r["case"]==piece+"/250/115")
    touch=edge+1000*float(landing["first_floor_s"])
    with (cache/"landing-field/trace.csv").open() as f:
        selected=[r for r in csv.DictReader(f) if r["case"]==piece+"/250/extended"]
    target=np.array([[float(r[k]) for k in ("t_s","x_m","y_m","z_m","axis_x","axis_y","axis_z")] for r in selected])
    first=17*math.ceil((source["firstPortBandMs"]-51)/17) if head=="moving" else 17*math.ceil(release/17)
    if head=="moving":
        optics.ORIGIN=np.array([48.,0.,842.]);optics.configure()
        raw_bg=background(cache/"relieved-roof-nominal.npz")
        physical_backing=background(cache/"relieved-roof-backed.npz")
        ref=[(g,b) for (g,d),(_,b) in zip(raw_bg,physical_backing)]
    else:
        raw_bg=ref=background(cache/(piece+"-reef-background.npz"))
    link=bridge.mod("match_pair",ROOT/"moving-observer/handoff.py")
    status={}; maxerr=0.; memo={}
    # Identical geometry, material, camera clock and source trajectory can share a rendered
    # exposure. Verify the archived acquisition/carry state for every reused camera frame.
    previous={}
    old_csv=cache/"flat-control"/(piece+"-"+head+".csv")
    if old_csv.exists() and head=="moving":
        old_receipt=read(HERE/"flat-control"/(piece+"-"+head+".json"))
        old_case=read(HERE/(piece+"-coupled-250.json"))
        old_file=cache/(piece+"-coupled-250.npy")
        if sha(old_csv)!=old_receipt["rawFrameSha256"] or sha(old_file)!=old_case["rawSha256"]:
            raise ValueError("Changed earlier optical inputs")
        old_path=np.load(old_file)
        with old_csv.open() as f:
            for r in list(csv.reader(f))[1:]:
                previous[int(r[2])]=[r[3],*map(float,r[4:])]
    with (cache/(piece+"-"+head+".csv")).open("w",newline="") as f:
        writer=csv.writer(f,lineterminator="\n")
        writer.writerow(["piece","head","capture_ms","status","x_lo","x_hi","y_lo","y_hi","bottom_lo","bottom_hi"])
        for tick in range(first,int(touch+35)+1,17):
            center,axis,slide=state_at(piece,tick,path,source,cargo,pickup,target)
            if head=="fixed":center[0]+=slide
            key=tuple(np.round(center,6))+tuple(np.round(axis,9))
            if tick in previous:
                xyz,old_axis,old_slide=state_at(piece,tick,old_path,source,old_case,pickup)
                if head=="fixed":xyz[0]+=old_slide
                if np.array_equal(center,xyz) and np.array_equal(axis,old_axis):
                    memo[key]=previous[tick]
            if key not in memo:
                im=silhouette(raw_bg,np.asarray(key[:3]),np.asarray(key[3:]))
                if head=="moving":
                    fit=bridge_fit.fit(im,ref)
                    fields=("x_mm","y_mm","bottom_mm")
                else:
                    fit=link.read_pair(im,[g for g,d in ref],560,370)
                    fields=("u_interval_mm","y_interval_mm","bottom_relative_to_shelf_mm")
                memo[key]=[fit["status"],*[x for field in fields for x in fit.get(field,[])]]
            fit=memo[key];status[fit[0]]=status.get(fit[0],0)+1
            if len(fit)==7:
                true=(center[0]-(slide if head=="fixed" else 0),center[1],center[2]-557.15)
                maxerr=max(maxerr,max(max(fit[1+2*i]-x,x-fit[2+2*i]) for i,x in enumerate(true)))
            writer.writerow([piece,head,tick,*fit])
    result={"pipe":piece,"head":head,"frames":sum(status.values()),"counts":status,"maxTruthOutsideMm":maxerr,
            "releaseCameraMs":release,"edgeCameraMs":edge,"touchCameraMs":touch,"rawFrameSha256":sha(cache/(piece+"-"+head+".csv"))}
    (HERE/(piece+"-"+head+".json")).write_text(json.dumps(result,indent=2)+"\n",encoding="utf8",newline="\n")
    print(result,flush=True)


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("cache",type=Path)
    p.add_argument("piece",choices=["coral-A","coral-B"])
    p.add_argument("head",choices=["fixed","moving"])
    a=p.parse_args();run(a.cache,a.piece,a.head)
