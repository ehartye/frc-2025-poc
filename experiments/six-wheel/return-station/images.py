"""Render both surveyed cameras at the attained station pose and measure the tilted physical pipe."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import sys
import numpy as np
from registration import HERE, ROOT, parts, mount, camera, poses


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def silhouette(bg,center,axis,intensity=220):
    # Each pixel ray intersects the finite tilted annulus in its actual three-dimensional pose.
    dx,dy,dz=camera.DX,camera.DY,camera.DZ
    ray_dot=dx*axis[0]+dy*axis[1]+dz*axis[2]
    length2=dx*dx+dy*dy+dz*dz
    approx=camera.R
    images=[]
    for eye,(gray,depth) in zip(camera.EYES,bg):
        origin=camera.ORIGIN+np.array([0,eye,0])-center
        along=float(origin@axis);origin2=float(origin@origin)
        ray_origin=dx*origin[0]+dy*origin[1]+dz*origin[2]
        b=ray_origin-ray_dot*along;k=length2-ray_dot**2
        disc=b*b-k*(origin2-along*along-approx**2)
        dist=(-b-np.sqrt(np.maximum(disc,0)))/np.maximum(k,1e-12)
        inside=(disc>=0)&(dist>0)&(np.abs(along+dist*ray_dot)<=camera.HALF)
        best=np.where(inside,dist,np.inf)
        for cap in (-camera.HALF,camera.HALF):
            with np.errstate(divide="ignore",invalid="ignore"):
                d=(cap-along)/ray_dot
                r2=origin2+2*d*ray_origin+d*d*length2-cap**2
            best=np.minimum(best,np.where((d>0)&(r2>=50.8**2)&(r2<=approx**2),d,np.inf))
        out=gray.copy();out[best<depth]=intensity;images.append(out)
    return images


def run(cache,step,probe=False,hold=False,records=HERE):
    feed=records/("hold-feed.json" if hold else "probe-feed.json" if probe else "feed.json")
    from observe import read_pair
    full=np.load(cache/("station-views-relief.npz" if probe else "station-views.npz"))
    robot=[(full[k],full[k+"_z"]) for k in ("left","right")]
    field=parts(step)
    mesh=[]
    for k,shape in sorted(field.items()):
        vertices,t=mount.original.triangulate(shape)
        vv=np.asarray(vertices,dtype=float).reshape(-1,3)/10
        color="#808080" if "/GE-" in k else "#ababab"
        mesh.append((k,color,vv,t))
    feeds=json.loads(feed.read_bytes())
    configs={r["name"]:r["actualReturn"] for r in feeds}
    data={}; observed={}
    for name,delta in configs.items():
        if name.endswith("-500"):continue
        key=tuple(delta)
        if key not in data:
            dx,dy,h=delta;c,s=math.cos(math.radians(h)),math.sin(math.radians(h));geometry=[]
            for k,color,station,t in mesh:
                # Station +X is robotward; robot +X and +Y oppose both in-plane station axes.
                x=444.525-dx-station[:,0];y=-dy-station[:,1]
                points=np.column_stack((x*c+y*s,-x*s+y*c,station[:,2]))
                geometry.append([k,"field",color,np.round(points*10).astype(int).reshape(-1).tolist(),t])
            station_views=camera.images_for_pose({"parts":geometry})
            data[key]=[(np.where(fz<rz,fg,rg),np.minimum(fz,rz)) for (rg,rz),(fg,fz) in zip(robot,station_views)]
        bg=data[key]
        ref=[g for g,_ in bg]
        rec=next(r for r in feeds if r["name"]==name)
        trace=np.load(cache/(name+".npy"))
        stamp=rec["firstPortBandMs"]
        if not len(trace) or stamp is None:
            observed[name]={"frames":[],"status":"port-gate-withheld"};continue
        # Render the five observed frames plus margin after the real port-band capture.
        frames=[]; fits={}
        for tm in range(17*((stamp-40)//17),min(int(trace[-1,0]),5540 if hold else stamp+220)+1,17):
            arr=np.array([np.interp(tm,trace[:,0],col) for col in trace[:,1:7].T])
            axis=arr[3:]/np.linalg.norm(arr[3:])
            key=(tuple(np.round(arr[:3],6)),tuple(np.round(axis,9)))
            if key not in fits:
                images=silhouette(bg,np.array(key[0]),np.array(key[1]))
                fits[key]=read_pair(images,ref,560,0)
            r=fits[key]
            bounds=[v for k in ("u_interval_mm","y_interval_mm","bottom_relative_to_shelf_mm") for v in r.get(k,[])]
            frames.append([tm,r["status"],*bounds])
        observed[name]={"frames":frames,"status":"rendered","referenceArraysSHA256":[hashlib.sha256(g.tobytes()+d.tobytes()).hexdigest() for g,d in bg]}
        print(name,[(s,len([r for r in frames if r[1]==s])) for s in sorted(set(r[1] for r in frames))],flush=True)
    # Quantify the end-dock silhouette in an opaque, uniformly painted field-panel rendering.
    # A rendered shadow/bolt contrast cannot be guaranteed on a transparent physical panel.
    reference=data.get((0,0,0),next(iter(data.values())));diff={}
    for key,bg in data.items():
        diff[str(key)]=[int(np.count_nonzero(np.abs(g.astype(int)-r.astype(int))>=30)) for (g,d),(r,_) in zip(bg,reference)]
    (records/("hold-images.json" if hold else "probe-images.json" if probe else "images.json")).write_text(json.dumps({"field":sha(step),"feed":sha(feed),"stationImageChangePixels":diff,"cases":observed},indent=2)+"\n",encoding="utf8",newline="\n")


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("cache",type=Path);p.add_argument("field",type=Path)
    p.add_argument("--probe",action="store_true");p.add_argument("--hold",action="store_true");a=p.parse_args();run(a.cache,a.field,a.probe or a.hold,a.hold)
