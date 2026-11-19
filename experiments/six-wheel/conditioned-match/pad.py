"""Find known material points shared by an attained pipe and each as-drawn sensor laminate."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from OCP.BRepClass3d import BRepClass3d_SolidClassifier
from OCP.gp import gp_Pnt
from OCP.STEPControl import STEPControl_Reader
from OCP.IFSelect import IFSelect_RetDone
from OCP.TopAbs import TopAbs_IN

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent

def signed_interiors(assets):
    reg=json.loads((ROOT/"release-observer/mount-check.json").read_bytes())
    step=ROOT/"release-observer/stereo-mount.step"
    if hashlib.sha256(step.read_bytes()).hexdigest()!=reg["mount_sha256"]:raise ValueError("Reopened STEP does not match inventory")
    reader=STEPControl_Reader()
    if reader.ReadFile(str(step))!=IFSelect_RetDone or reader.TransferRoots()!=len(reg["part_order"]):raise ValueError("Native part mismatch")
    out={}
    for side in ("leftSmall","rightSmall"):
        raw=assets/"shelf-contact/fine-ports"/f"{side}-400000-4-15.625.npy"
        source=np.load(raw,mmap_mode="r");source=source[source[:,0]<500]
        rows=[]
        for i,y in enumerate((-70.,0.,70.)):
            name="moving-pad-"+str(i)
            shape=reader.Shape(1+reg["part_order"].index(name))
            point=np.array([125.1,y,500.65])
            # Native pad STEP is at 370 mm extension; chassis matching stowed pad is translated back.
            native=point+[370,0,0]
            if BRepClass3d_SolidClassifier(shape,gp_Pnt(*native),1.e-7).State()!=TopAbs_IN:raise ValueError("Witness outside actual laminate")
            offset=point-source[:,1:4];a=source[:,4:7]
            along=(offset*a).sum(axis=1)
            radius=np.linalg.norm(offset-along[:,None]*a,axis=1)
            margin=np.minimum.reduce((57.15-radius,radius-50.8,150.8125-abs(along)))
            hits=np.flatnonzero(margin>0)
            k=np.argmax(margin)
            rows.append({"nativeId":name,"stowedPointMm":point.tolist(),"firstJavaMs":float(source[hits[0],0]),
                         "lastJavaMs":float(source[hits[-1],0]),"maximumInsideMm":float(margin[k]),"peakAtJavaMs":float(source[k,0])})
        out[side]=rows
    return {"stepSha256":reg["mount_sha256"],"frame":"stowed, undeformed laminate","cases":out}

if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("assets",type=Path);a=p.parse_args()
    result=signed_interiors(a.assets)
    (HERE/"pad.json").write_text(json.dumps(result,indent=2)+"\n",encoding="utf8",newline="\n")
