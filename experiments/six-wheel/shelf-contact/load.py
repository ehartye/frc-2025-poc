"""Integrate the first normal reaction pulse on the actual finite shelf in the attained guide cases."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from experiment import source,GUIDE,CompliantPipe,Shelf,HERE,ROOT


def impact(name, dt, cache, stiffness=400000.):
    from dataclasses import replace
    delta,side=source(name)
    pipe=CompliantPipe(delta,side,replace(GUIDE,sectors=128,radial_bins=4),Shelf(stiffness_n_m=stiffness,stations=16))
    # Belt strip count controls load at first touchdown and is stated independently of guide quadrature.
    from contact import feed
    old=feed.cfg;feed.cfg=replace(old,slices=10)
    try:
        pipe.belt=.65;pipe.record_load=True
        if stiffness<400000.:pipe.distributed_history_s=.020
        while pipe.time<.280-1.e-11 and not pipe.hard and not pipe.loss:pipe.moves(.65,min(dt,.280-pipe.time))
    finally:
        feed.cfg=old
    rows=np.array(pipe.load_samples)
    peak=int(np.argmax(-rows[:,3]));threshold=-rows[peak,3]*.01
    start=peak;end=peak
    while start and -rows[start-1,3]>=threshold and rows[start,0]-rows[start-1,0]<1.5*dt:start-=1
    while end+1<len(rows) and -rows[end+1,3]>=threshold and rows[end+1,0]-rows[end,0]<1.5*dt:end+=1
    pulse=rows[start:end+1]
    cache.mkdir(parents=True,exist_ok=True)
    prefix=name if stiffness==400000. else f"{name}-{stiffness:.0f}"
    aggregate=cache/(prefix+"-shelf.csv")
    stations=cache/(prefix+"-stations.csv")
    np.savetxt(aggregate,rows,delimiter=",",fmt="%.12g",header="lip_s,force_x_n,force_y_n,force_z_n,cop_x_mm,cop_y_mm,cop_z_mm,active_length_mm",comments="")
    np.savetxt(stations,np.array(pipe.distributed_load_samples),delimiter=",",fmt="%.12g",
               header="lip_s,gauss_station,x_mm,y_mm,z_mm,force_x_n,force_y_n,force_z_n",comments="")
    return {"case":name,"dt":dt,"stiffnessNm":stiffness,"aggregateSha256":hashlib.sha256(aggregate.read_bytes()).hexdigest(),
            "stationsSha256":hashlib.sha256(stations.read_bytes()).hexdigest(),"startFromLipS":pulse[0,0],"endFromLipS":pulse[-1,0]+dt,
            "peakFromLipS":rows[peak,0],"peakRobotReactionXYZn":rows[peak,1:4].tolist(),
            "peakNormalCenterRobotMm":rows[peak,4:7].tolist(),"activeGeneratorLengthAtPeakMm":rows[peak,7],
            "pulseImpulseRobotXYZns":(pulse[:,1:4].sum(axis=0)*dt).tolist(),
            "firstImpactSpeedMps":pipe.first_impact_speed,"peakApproachMm":1000*pipe.peak_shelf_compression}


if __name__=="__main__":
    from concurrent.futures import ProcessPoolExecutor
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument("cache",type=Path);parser.add_argument("--soft",action="store_true");args=parser.parse_args()
    names=("nominal","leftSmall","rightSmall","shiftedHumanSource","badRegistration")
    ks=[400000.]*len(names)
    if args.soft:
        names=("leftSmall","rightSmall")*3
        ks=[20000.,20000.,40000.,40000.,80000.,80000.]
    with ProcessPoolExecutor(max_workers=4) as pool:
        rows=list(pool.map(impact,names,[.000015625]*len(names),[args.cache]*len(names),ks))
    (HERE/("soft-load.json" if args.soft else "load.json")).write_text(json.dumps(rows,indent=2)+"\n",encoding="utf8",newline="\n")
    for r in rows: print(r,flush=True)
