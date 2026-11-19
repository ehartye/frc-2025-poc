"""Check straight tool insertion, rail screw withdrawal and the adjacent deck hook."""
from pathlib import Path
import sys
import numpy as np
from frame import HERE,ROOT,SITES,j,cylinder,transform,register
from module_placement import read,write
from envelopes import load_raw
from fit import indexed

def run(folder):
    src=indexed(folder); meta=read(HERE/"frame.json");rear=read(HERE/"rear-electronics.json")
    frame=j.cad.read_named(HERE/"swerve-frame.step",[{"id":k} for k in meta["parts"]])
    fixed=j.cad.read_named(HERE/"rear-electronics.step",[{"id":k} for k in rear["partsMm3"]])
    for k,s in src.items():
        if k.startswith(("bumper-","deck-tie","deck-main","tie-machined-","cross-","drive-rail-inner",
                         "drive-rail-outer","observer-pi","electrical-backplate","front-","rear-")):continue
        fixed[k]=s
    fixed.update(frame)
    fixed.pop("swerve-relieved-ribbed-deck")
    rad=read(HERE/"envelopes.json")["movingRadiusMm"]
    raw=load_raw(folder)
    hits={};who={};control={}
    for tag in SITES:
        parts=dict(fixed)
        for name,s,b in raw:
            shape=cylinder([0,b[1],0],[0,1,0],rad[name],b[4]-b[1]) if name in rad else s
            parts["supplier/"+name]=register(shape,tag)
        boxes={k:j.cad.bounds(s) for k,s in parts.items()}
        for mount,mat in (("A",np.eye(3)),("B",np.array([[0,0,1],[0,1,0],[1,0,0.]]))):
            project=lambda s:register(transform(s,mat,[0,0,0]),tag)
            for y in (46,67):
                screw=tag+"-"+mount+"-rail-screw-"+str(y)
                driver=project(cylinder([-17.49,y,69.85],[1,0,0],3,62))
                shank=project(cylinder([-23.49,y,69.85],[1,0,0],5.05,45))
                for label,tool in (("6mm-key",driver),("withdrawn-head",shank)):
                    envelope=j.cad.bounds(tool)
                    for name,s in parts.items():
                        if name==screw or j.lower_bound(envelope,boxes[name])>.001:continue
                        v=j.cad.intersection(tool,s)
                        if v>.01:hits[screw+"/"+label+" :: "+name]=round(v,4)
            who[tag+"-"+mount]=2
        anchor=next(k for k in read(HERE/"service.json") if k["corner"]==tag)["matchingTemporaryDeckAnchorRemoval"][0]
        for mount in ("A","B"):
            shape=frame[tag+"-"+mount+"-tab-saddle"]
            for depth in range(4,97,4):
                s=j.cad.translated(shape,[0,0,-depth])
                if j.lower_bound(j.cad.bounds(s),j.cad.bounds(src[anchor]))>.001:continue
                v=j.cad.intersection(s,src[anchor])
                if v>.01:
                    control[tag]={"anchor":anchor,"saddle":mount,"downMm":depth,"intersectionMm3":round(v,5)}
                    break
    write(HERE/"driver-access.json",{"allHorizontalScrews":sum(who.values()),"continuousScrewPullMm":40,
          "toolInsertionMm":62,"straightKeyOdMm":6,"materialHitsMm3":hits,
          "retainedHookControls":control,"passScope":"Straight hex key and major-diameter head. No hand or torque-wrench envelope."})
if __name__=="__main__":run(Path(sys.argv[1]))
