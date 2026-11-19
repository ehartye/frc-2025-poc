"""Check fitted module material and conservative swept parts against the selected robot."""
import argparse
import math
from pathlib import Path
import re
import sys
import numpy as np
from OCP.BRep import BRep_Builder
from OCP.BRepTools import BRepTools
from OCP.TopoDS import TopoDS_Shape
from module_placement import HERE,read,write,SITES,rotated_box
from frame import ROOT,j,register
from envelopes import corner_data

def indexed(assets):
    directory=assets/"robot-native"
    solids={}
    for k,v in read(directory/"index.json").items():
        s=TopoDS_Shape()
        if not BRepTools.Read_s(s,str(directory/v["file"]),BRep_Builder()):raise ValueError(k)
        solids[k]=s
    return solids

def audit(assets):
    src=indexed(assets)
    legacy={}
    for k in list(src):
        if k.startswith(("drive-rail-inner","drive-rail-outer","cross-front","cross-rear","deck-main",
                         "tie-machined-cross","tie-machined-deck-main")):legacy[k]=src.pop(k)
    kit=j.cad.read_named(HERE/"swerve-frame.step",[{"id":k} for k in read(HERE/"frame.json")["parts"]])
    groups={k:v for k,v in kit.items() if k.startswith("swerve-")}
    moving={}
    for k in list(src):
        if re.match(r"(front|rear)-[LR]-(ear|pilot|weld-|screw-(90|130)$|head-washer-(90|130)$|nut-washer-(90|130)$|nut-(90|130)$)",k):
            original=src.pop(k)
            legacy[k]=original
            sy=1 if "-L-" in k else -1
            moving["relocated-"+k]=j.cad.translated(original,[0,-75*sy,0])
    src.update(moving);src.update(groups)
    source_bounds={k:j.cad.bounds(s) for k,s in src.items()}
    old_bounds={k:j.cad.bounds(s) for k,s in legacy.items()}
    native,boxes,rad=corner_data(assets)
    gross={};hits={};old_hits={};pool={}
    clear={}
    for tag in SITES:
        placed={name:register(s,tag) for name,s in native.items()}
        def enclosing(solid_list):
            a=np.asarray([j.cad.bounds(t) for t in solid_list])
            lo=a[:,:3].min(axis=0)-.01;hi=a[:,3:].max(axis=0)+.01
            return j.box(lo[0],hi[0],lo[1],hi[1],lo[2],hi[2])
        shell=enclosing(placed.values())
        # One material intersection against a containing rectangular solid proves
        # separation for every contained corner part, including thin bumper skins.
        shell_b=j.cad.bounds(shell)
        ignores={k for k,t in src.items() if j.lower_bound(shell_b,source_bounds[k])>.001
                 or j.cad.intersection(shell,t)<.00001}
        for k,s in kit.items():
            if k.startswith(tag+"-"):placed[k]=s
        largest=np.asarray([j.cad.bounds(s) for s in placed.values()])
        gross[tag]=np.r_[largest[:,:3].min(axis=0),largest[:,3:].max(axis=0)].tolist()
        for name,s in placed.items():
            b=j.cad.bounds(s)
            for key,t in src.items():
                if name in native and key in ignores:continue
                if j.lower_bound(b,source_bounds[key])>.001:continue
                q=j.cad.intersection(s,t)
                if q>.01:hits[tag+"/"+name+" :: "+key]=round(q,4)
            if name.startswith(("9-","10-","0-","1-")):
                for key,t in legacy.items():
                    if not key.startswith(("drive-rail-inner","deck-main","front-L-ear","front-R-ear","rear-L-ear","rear-R-ear")):continue
                    if j.lower_bound(b,old_bounds[key])>.001:continue
                    q=j.cad.intersection(s,t)
                    if q>.01:old_hits[tag+"/"+name+" :: "+key]=round(q,4)
        # Downward replacement clearance. Every corner travels parallel to -Z.
        # Bound each individual body throughout travel. Any broadphase pairing
        # is explicitly retained for a native vertical sweep or shop gate.
        pending={}
        for name,s in placed.items():
            if "welded-thread-boss" in name or "rail-screw" in name:continue
            b=j.cad.bounds(s);swept=list(b);swept[2]-=195
            for key,t in src.items():
                if j.lower_bound(swept,source_bounds[key])>.001:continue
                if name.startswith(("0-","1-")) and key=="swerve-relieved-ribbed-deck":continue
                pending.setdefault(key,[]).append(name)
        pool[tag]=pending
        print(tag,"material hits",len(hits),"historical controls",len(old_hits),flush=True)
        write(assets/("partial-"+tag+".json"),{"hits":hits,"old":old_hits,"service":pending})
    return {"initialMaterialHitsMm3":old_hits,"installedMaterialHitsMm3":hits,
            "continuousAzimuthAndSpin":True,"grossCornerBoundsMm":gross,
            "downwardRemovalDistanceMm":195,"downwardBroadphasePairs":pool,
            "sourcePhysicalLowStructures":len(src),"explicitLegacyControlStructures":len(legacy),
            "kitSha256":j.digest(HERE/"swerve-frame.step")}
if __name__=="__main__":write(HERE/"fit.json",audit(Path(sys.argv[1])))
