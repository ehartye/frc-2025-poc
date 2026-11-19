"""Observe the original and revised station pipe using the actual stowed roof view plus both guides."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
sys.path.insert(0,str(ROOT/"selected-optics"))
from exposures_check import optics,camera,background,silhouette,fit_row
from cameras import mount

def run(assets,carry=False):
    reg=json.loads((ROOT/"selected-optics/mount.json").read_bytes())
    scenes=assets/"integrated-slide/issue/marked"
    info=json.loads((scenes/"scenes.json").read_bytes())
    def load(name):
        f=scenes/(name+".npz")
        if hashlib.sha256(f.read_bytes()).hexdigest()!=info["backgroundSha256"][name]:raise ValueError("Camera source changed")
        return background(f)
    full=load("barrel-moving-0")
    backed=load("barrel-moving-reference")
    optics.ORIGIN=np.array(reg["registrations"]["roof"]["assumedRectifiedOriginMm"])
    optics.configure()
    ids=[{"id":"guide-positive-y"},{"id":"guide-negative-y"}]
    shapes=mount.cad.read_named(ROOT/"station-guide/narrow-face.step",ids)
    mesh=[]
    for key,shape in shapes.items():
        vertices,faces=mount.original.triangulate(shape)
        mesh.append([key,"guide","#a9a49c",vertices,faces])
    projection=camera.images_for_pose({"parts":mesh})
    actual=[];reference=[];pixels=[]
    for (g,z),(_,guaranteed),(added,dist) in zip(full,backed,projection):
        visible=dist<z
        gray=np.where(visible,added,g)
        actual.append((gray,np.minimum(z,dist)))
        reference.append((gray,np.minimum(guaranteed,dist)))
        pixels.append(int(visible.sum()))
    output={}
    inputs=json.loads((HERE/("corrected-cargo.json" if carry else "feeds.json")).read_bytes())
    for r in inputs:
        if carry:
            if r["dtS"]!=.0000625 or "relativePipeSha256" not in r:continue
            side=r["wheelCase"]
            raw=assets/"conditioned-match"/side/"corrected/A/cargo-62500ns.npy"
            source_hash=r["relativePipeSha256"]
            source=np.load(raw,mmap_mode="r")
            # Carry time starts on Java 5460, preserving the fixed 17 ms capture phase.
            clock=5460+1000*source[:,0]
            positions=source[:,1:4]*1000
            axes=source[:,4:7]
            ticks=range(17*int(np.ceil(clock[0]/17)),int(clock[-1])+1,17)
        else:
            if r["state"]!="IN_BAND" or r["dtS"]!=.00003125:continue
            side=r["case"]
            raw=assets/"conditioned-match/corrected-feeds"/f"{side}-400000-4-31.25.npy"
            source_hash=r["trajectorySha256"]
            data=np.load(raw,mmap_mode="r")
            clock,positions,axes=data[:,0],data[:,1:4],data[:,4:7]
            ticks=range(17*int(np.ceil(r["firstBandMs"]/17)),5441,17)
        if hashlib.sha256(raw.read_bytes()).hexdigest()!=source_hash:raise ValueError("Changed reached pipe")
        counts={};lost=[];memo={};frames=[]
        for tm in ticks:
            q=np.array([np.interp(tm,clock,v) for v in (*positions.T,*axes.T)])
            images=silhouette(actual,q[:3],q[3:]/np.linalg.norm(q[3:]))
            digest=hashlib.sha256(b"".join(img.tobytes() for img in images)).hexdigest()
            if digest not in memo:memo[digest]=fit_row(images,reference,True,0)
            status,*edges=memo[digest]
            inside=status=="inside" and all(lo<=v<=hi for v,lo,hi in zip((q[0],q[1],q[2]-557.15),edges[::2],edges[1::2]))
            counts[status]=counts.get(status,0)+1
            if not inside:lost.append(tm)
            frames.append([tm,digest,inside])
            if carry and not inside:break
        output[side]={"pipe":"coral-A","sourceSha256":source_hash,"stowedGuideVisiblePixels":pixels,
                      "frameCount":len(frames),"uniqueImages":len(memo),"counts":counts,"firstFailedJavaMs":lost[:1],
                      "lastUsableJavaMs":frames[-1][0]+15 if frames and frames[-1][2] else None,"frames":frames,
                      "empty":fit_row([g for g,_ in actual],reference,True,0)[0]}
        print(side,counts,lost[:2],flush=True)
    (HERE/("carry-roof.json" if carry else "roof.json")).write_text(json.dumps(output,indent=2)+"\n",encoding="utf8",newline="\n")

if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("assets",type=Path);p.add_argument("--carry",action="store_true");a=p.parse_args();run(a.assets,a.carry)
