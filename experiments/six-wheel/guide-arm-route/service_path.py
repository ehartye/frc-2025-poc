"""Check the stowed two-step removal with a conservative cover of the revised installed arm."""
import argparse
from pathlib import Path
import numpy as np
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common

from arm_route import ARM, ROOT, carriage, inputs, j


def occupied_cells(body):
    # Split the exact solid with planes normal to X and Z, then bound the clipped material.
    # Every point is in a closed cell; cells meet on shared boundaries.
    x0,y0,z0,x1,y1,z1=j.cad.bounds(body)
    xs=np.arange(np.floor(x0)-1,np.ceil(x1)+7,6)
    zs=np.arange(np.floor(z0)-1,np.ceil(z1)+7,6)
    cells=[]
    for xa,xb in zip(xs,xs[1:]):
        slab=BRepAlgoAPI_Common(body,j.box(xa,xb,y0-1,y1+1,z0-1,z1+1)).Shape()
        if not j.cad.solid_count(slab):
            continue
        _,_,low,_,_,high=j.cad.bounds(slab)
        for za,zb in zip(zs,zs[1:]):
            if za>high or zb<low:
                continue
            part=BRepAlgoAPI_Common(slab,j.box(xa-1,xb+1,y0-1,y1+1,za,zb)).Shape()
            if j.cad.solid_count(part):
                cells.append(j.cad.bounds(part))
    return np.array(cells)


def run(registry,folder):
    source=inputs()
    arm=carriage.selected(folder,"route.step","revision.json","stepSha256","volumesMm3")[ARM]
    scene,edges,reg=carriage.scene(registry,ROOT/"fixed-hoist")
    for k,box in reg["stowed"].items():
        if k in scene and np.max(np.abs(np.subtract(box,reg["bounds"][k])))>.001:
            scene[k]=carriage.bbox(box)
    braces=carriage.selected(ROOT/"carriage-support","carriage-support.step","fit.json","stepSha256","volumesMm3")
    scene.update({k:v for k,v in braces.items() if not k.startswith("support-machined-mast-")})
    scene.update({k:v for k,v in source.items() if k.startswith("guide-R-") or k.endswith(("machined-mast","-sleeve"))})
    bounds={k:j.cad.bounds(v) for k,v in scene.items()}
    inherited=j.read(ROOT/"guide-mounts/service.json")
    for action in ("inboard","forward"):
        if inherited["collisions"]["stowed/L/"+action]:
            raise ValueError("The inherited removal proof is obstructed")
    guide_sha=j.digest(ROOT/"guide-mounts/fit.json")
    if inherited["fitSha256"]!=guide_sha:
        raise ValueError("The inherited sweep belongs to different geometry")
    # The setting bores only remove material. Carry the proved original sweep for
    # shared material and cover every additional solid portion of the new arm.
    native_addition=j.cut(arm,source[ARM])
    cells=occupied_cells(native_addition)
    results={}
    for phase,offset in (("inward",(0,-65,0,0,0,0)),("forward",(0,-65,0,670,-65,0))):
        contacts={}
        swept=cells+offset
        for bb in swept:
            others=[k for k,ob in bounds.items() if j.lower_bound(bb,ob)<.001]
            cables=[(k,e,r) for k,e,b,r in edges if j.lower_bound(bb,b)<r+.001]
            if not others and not cables:continue
            x,y,z,X,Y,Z=bb
            shape=j.box(x,X,y,Y,z,Z)
            for k in others:
                v=j.cad.intersection(shape,scene[k])
                if v>.001:contacts[k]=max(contacts.get(k,0.),v)
            for k,e,r in cables:
                d=j.distance(shape,e)-r
                if d<-.001:contacts[k]=min(contacts.get(k,0.),d)
        results[phase]=contacts
        print(phase,contacts,flush=True)
    j.write(folder/"removal.json",{"stepSha256":j.digest(folder/"route.step"),"addedMaterialCells":len(cells),"newMaterialMm3":j.cad.volume(native_addition)[0],
        "inheritedServiceSha256":j.digest(ROOT/"guide-mounts/service.json"),
        "sourceRegistrySha256":j.digest(registry),"settingPins":"removed","rootFasteners":"removed, sleeves retained",
        "motionMm":[[0,-65,0],[670,0,0]],"overlappingCellBounds":results})


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("registry",type=Path)
    p.add_argument("folder",type=Path)
    args=p.parse_args()
    run(args.registry,args.folder)
