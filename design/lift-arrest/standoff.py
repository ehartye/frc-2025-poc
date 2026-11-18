"""Find height loss producing positive-volume contact on one independent robot STEP."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent/"station-l1"))
import build_assembly as cad
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common


def digest(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def candidates(shapes,frames,depth):
    boxes={k:cad.bounds(v) for k,v in shapes.items()}
    result=[]
    for m,frame in frames.items():
        if frame not in ("lift","middle","slide"): continue
        a=boxes[m]
        for s,anchor in frames.items():
            if anchor!="base": continue
            b=boxes[s]
            if min(a[3],b[3])-max(a[0],b[0])<.01 or min(a[4],b[4])-max(a[1],b[1])<.01: continue
            start=max(0,a[2]-b[5])
            end=min(depth,a[5]-b[2])
            if end>start+.001: result.append((start,end,m,s))
    return sorted(result)


def shared(a,b):
    common=BRepAlgoAPI_Common(a,b)
    if not common.IsDone(): raise ValueError("native interference calculation failed")
    return abs(cad.volume(common.Shape())[0])


def review(folder,pose,step_mm=5,maximum=100):
    frames=json.loads((folder/"part-frames.json").read_bytes())
    src=folder/(pose+".step")
    result=json.loads((folder/"review.json").read_bytes())[pose]
    sha=digest(src)
    if sha!=result["export_sha256"]: raise ValueError("pose bytes differ")
    shapes=cad.read_named(src,[{"id":k} for k in frames])
    pairs=candidates(shapes,frames,maximum)
    hits=[]
    for drop in range(step_mm,maximum+1,step_mm):
        moved={}
        for start,end,m,s in pairs:
            if start>=drop: break
            if end<=drop: continue
            if m not in moved: moved[m]=cad.translated(shapes[m],(0,0,-drop))
            v=shared(moved[m],shapes[s])
            if v>.01: hits.append((m,s,v))
        if hits:
            break
    strikes=[]
    for m,s,witness in hits:
        # Refine this witnessed overlap locally; later disjoint contacts remain possible.
        a,b=drop-step_mm,drop
        for _ in range(12):
            mid=(a+b)/2
            if shared(cad.translated(shapes[m],(0,0,-mid)),shapes[s])>.01: b=mid
            else: a=mid
        strikes.append({"moving":m,"fixed":s,"firstLocalOverlapBracketMm":[a,b],
                        "sampleDropMm":drop,"overlapAtSampleMm3":witness})
    return {"source":sha,"rigidBodyCount":len(frames),"frameDigest":digest(folder/"part-frames.json"),
            "searchStepMm":step_mm,"maxSearchedMm":maximum,"pairCount":len(pairs),
            "witnesses":sorted(strikes,key=lambda r:r["firstLocalOverlapBracketMm"][1]),
            "creditForSafeTravelMm":None}


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("folder",type=Path)
    p.add_argument("pose")
    p.add_argument("output",type=Path)
    args=p.parse_args()
    args.output.write_text(json.dumps(review(args.folder,args.pose),indent=2)+"\n",encoding="utf-8",newline="\n")
