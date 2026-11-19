"""Reopen the 597-solid optics, removing unproven background rays throughout the rack stroke."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
sys.path.insert(0,str(ROOT/"return-station"))
from registration import mount, bridge, camera, sha
from receiver_relief import low_relief
from design import optics
sys.path.insert(0,str(ROOT/"design/receiver-drives"))
import receiver_drives


def stored(path, views):
    np.savez_compressed(path,left=views[0][0],left_z=views[0][1],right=views[1][0],right_z=views[1][1])
    return sha(path)


def mesh(parts,groups,colors):
    result=[]
    for name,shape in sorted(parts.items()):
        verts,triangles=mount.original.triangulate(shape)
        result.append([name,groups.get(name,"Mount"),colors.get(name,"#737d85"),verts,triangles])
    return {"parts":result}


def box_near(origin,direction,b):
    near=np.full(direction[0].shape,-np.inf)
    far=np.full(near.shape,np.inf)
    for i,ray in enumerate(direction):
        with np.errstate(divide="ignore",invalid="ignore"):
            r=(b[i]-origin[i])/ray
            s=(b[i+3]-origin[i])/ray
        near=np.maximum(near,np.minimum(r,s))
        far=np.minimum(far,np.maximum(r,s))
    return np.where((far>=near)&(far>0),np.maximum(0,near),np.inf)


def run(cache,lamp):
    cache.mkdir(exist_ok=True,parents=True)
    posed=[low_relief(lamp,s) for s in (0,370)]
    _,groups,colors,frames=mount.pose_layers(560,0,"red")
    if posed[0].keys()!=posed[1].keys():raise ValueError("Inconsistent configurations")
    fixed=camera.images_for_pose(mesh(posed[1],groups,colors))
    fixed_sha=stored(cache/"relieved-fixed-place.npz",fixed)
    optics.ORIGIN=np.array([48.,0.,842.])
    optics.configure()
    dynamic={}
    stable={}
    all_slide={}
    for name,part in posed[0].items():
        b0=np.array(mount.cad.bounds(part))
        b1=np.array(mount.cad.bounds(posed[1][name]))-np.array([370,0,0,370,0,0])
        if name=="camera-envelope":
            continue  # Sensor rays start at the two physical optical windows.
        moving=receiver_drives.translates_with_grip(name) or (name.startswith("grip-") and name.endswith("-spring"))
        if np.max(abs(b0-b1))<.001:
            all_slide[name]=part
            if not moving:
                stable[name]=part
                continue
        lo=np.minimum(b0[:3],b1[:3])
        hi=np.maximum(b0[3:],b1[3:])
        # Three rotating pinions lie inside their 11 mm radial envelope; enlarge the full
        # extrema to a diameter. The variable wire and ropes remain within their endpoints'
        # extrema at this fixed lift height. All remaining frames translate linearly.
        if "pinion" in name:
            lo-=22;hi+=22
        if moving:
            lo[1]-=1.4;hi[1]+=1.4
        dynamic[name]=np.r_[lo,hi]
    scene=camera.images_for_pose(mesh(stable,groups,colors))
    stable_sha=stored(cache/"relieved-roof-backed.npz",scene)
    nominal=camera.images_for_pose(mesh(all_slide,groups,colors))
    nominal_sha=stored(cache/"relieved-roof-nominal.npz",nominal)
    affected=[{},{}]
    result=[]
    for i,((gray,depth),eye) in enumerate(zip(scene,camera.EYES)):
        nearest=np.full(gray.shape,np.inf)
        direction=(camera.DX,camera.DY,camera.DZ)
        origin=optics.ORIGIN+[0,eye,0]
        for n,b in dynamic.items():
            if b[2]>=origin[2] or b[5]<200 or b[3]<-150 or b[0]>300 or b[1]>290 or b[4]<-290:
                continue
            d=box_near(origin,direction,b)
            mask=(d<depth)&(depth<3490)
            if mask.any():
                affected[i][n]=int(mask.sum())
                nearest=np.minimum(nearest,d)
        uncertain=nearest<depth
        result.append((np.where(uncertain,0,gray).astype(np.uint8),np.where(uncertain,3500,depth)))
    record={"nativeBodies":len(posed[0]),"fixedPlaceSha256":fixed_sha,"roofNominalSha256":nominal_sha,
            "roofGuaranteedSha256":stored(cache/"relieved-roof-guaranteed.npz",result),"coMovingBackingSha256":stable_sha,
            "coMovingBodies":len(all_slide),"stableCoMovingBodies":len(stable),"possibleOccluders":affected,
            "lamp":sha(lamp),"cheeks":sha(ROOT/"station-relief/cheeks.step"),"shims":sha(ROOT/"station-relief/shims.step"),
            "robotFrontMm":[max(mount.cad.bounds(shape)[3] for shape in parts.values()) for parts in posed]}
    (HERE/"camera-registration.json").write_text(json.dumps(record,indent=2)+"\n",encoding="utf8",newline="\n")
    print(record,flush=True)


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("cache",type=Path)
    p.add_argument("lamp",type=Path)
    a=p.parse_args()
    run(a.cache,a.lamp)
