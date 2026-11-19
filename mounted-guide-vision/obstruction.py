"""Identify the actual foreground article that covers a projected surveyed field fastener."""
import argparse
from collections import Counter
import json
from pathlib import Path
import sys
import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
sys.path.insert(0,str(ROOT/"observed-reef-a"))
from pipe_scene import nav
from vision import project,EYES,F,C,S

def run(assets):
    previous=nav.checked_scene(assets)
    refs=nav.read(ROOT/"sector-alignment/scene.json")
    names=sorted(refs["boltFacesMm"])
    out=[]
    for installation,study in nav.read(HERE/"station.json").items():
        register=study["register"]
        folder=assets/"mounted-guide-vision"
        raw=folder/(installation+"-labels.npz")
        if nav.sha(raw)!=register["bodyLabelSha256"]:raise ValueError("Wrong optical part labels")
        data=np.load(raw)
        new=nav.checked_scene(assets,use_guide=False,registered_background=(folder/(installation+"-fixed.npz"),register["fixedBackgroundSha256"]))
        for wheel in ("leftSmall","rightSmall"):
            for run_label,src in (("new-actual",folder/installation/wheel/"coral-A-nominal.csv"),("earlier-controller",assets/"observed-reef-a"/wheel/"coral-A-nominal.csv")):
                path=np.loadtxt(src,delimiter=",",skiprows=1)
                breakdown={}
                examples=[]
                occupied={}
                for t,x,y,h in path:
                    if t%35 or t>3272:continue
                    for key,eye in zip(("left","right"),EYES):
                        for fid,face in zip(names,previous["faces"]):
                            poly,depth=project(face,(x,y,h),eye)
                            low=np.floor(poly.min(axis=0)).astype(int)-1
                            high=np.ceil(poly.max(axis=0)).astype(int)+2
                            if any(low<0) or high[0]>640 or high[1]>480:continue
                            region=np.s_[low[1]:high[1],low[0]:high[0]]
                            was=np.min(previous[key+"_z"][region])>depth.max()+12
                            now=np.min(new[key+"_z"][region])>depth.max()+12
                            k=key+"/"+fid
                            summary=breakdown.setdefault(k,{"oldComplete":0,"newComplete":0,"newlyOccluded":0,"parts":Counter()})
                            summary["oldComplete"]+=int(was);summary["newComplete"]+=int(now)
                            if not was or now:continue
                            hit=(data[key+"_z"][region]<depth.max()+12)&(data[key+"_z"][region]<=new[key+"_z"][region]+.05)
                            labels=data[key][region][hit]
                            seen=sorted({register["bodyPerGray"].get(str(v),str(v)) for v in np.unique(labels)})
                            summary["parts"].update(seen);summary["newlyOccluded"]+=1
                            vv,uu=np.nonzero(hit)
                            uu=uu+low[0];vv=vv+low[1]
                            z=data[key+"_z"][vv,uu]
                            up=(240-(vv+.5))/F
                            points=np.column_stack((-315+(C+up*S)*z,eye+(320-(uu+.5))*z/F,970+(-S+up*C)*z))
                            for code in np.unique(labels):
                                name=register["bodyPerGray"].get(str(code),str(code))
                                box=occupied.setdefault(name,[])
                                box.extend(points[labels==code].tolist())
                            if not any(e["channel"]==k and e["parts"]==seen for e in examples):
                                examples.append({"ms":t,"channel":k,"pixelsLtrb":[*low.tolist(),*high.tolist()],"parts":seen})
                out.append({"installation":installation,"wheelCase":wheel,"path":run_label,"pathSha256":nav.sha(src),
                            "frames":breakdown,"differentBlockers":examples,"interceptedSurfaceBoxesRobotMm":{k:[*np.min(q,axis=0).tolist(),*np.max(q,axis=0).tolist()] for k,q in occupied.items()}})
    return out
if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("assets",type=Path)
    a=p.parse_args();(HERE/"obstruction.json").write_text(json.dumps(run(a.assets),indent=2)+"\n",encoding="utf8",newline="\n")
