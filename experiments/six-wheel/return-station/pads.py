"""Refine native distances for the three excluded volumes left by the rounded sweep at touchdown."""
import argparse
import json
import math
from pathlib import Path
import numpy as np
from registration import HERE,mount,sha
from receiver_relief import low_relief
from rigid import annulus
from OCP.BRepExtrema import BRepExtrema_DistShapeShape


def run(cache,lamp):
    shapes=low_relief(lamp);rows=json.loads((HERE/"probe-feed.json").read_bytes())
    reference=json.loads((HERE/"probe-native.json").read_bytes())
    overlap=[b for b in reference["bins"] if any(k.startswith("observer-moving-pad") for k in b["outerEnvelopeOverlapMm3"])]
    result=[]
    for r in rows:
        if "bias-" not in r["name"]:continue
        tr=np.load(cache/(r["name"]+".npy"));source=tr[:,0]
        for b in overlap:
            pads={k:shapes[k] for k in b["outerEnvelopeOverlapMm3"] if k.startswith("observer-moving-pad")}
            lo,hi=b["fromMs"],b["toMs"]
            times=np.unique(np.r_[lo,source[(source>lo)&(source<hi)],hi])
            min_gap=100.;min_lower=100.;intersect=None;count=0
            stack=list(zip(times[:-1],times[1:]))
            def state(t):
                p=np.array([np.interp(t,source,col) for col in tr[:,1:7].T]);p[3:]/=np.linalg.norm(p[3:]);return p
            while stack:
                a,c=stack.pop();mid=(a+c)/2;one=state(a);center=state(mid);two=state(c)
                tool=annulus(center[:3],center[3:]);count+=1
                reach=max(np.linalg.norm(v[:3]-center[:3])+162*np.linalg.norm(v[3:]-center[3:]) for v in (one,two))
                values=[]
                for k,shape in pads.items():
                    query=BRepExtrema_DistShapeShape(tool,shape)
                    if not query.IsDone():raise ValueError(k)
                    values.append((query.Value(),k))
                d,k=min(values);min_gap=min(min_gap,d)
                if d<1e-6 and mount.cad.intersection(tool,pads[k])>1e-6:
                    intersect={"ms":mid,"id":k,"volumeMm3":mount.cad.intersection(tool,pads[k])};break
                if d>reach+.0001 or c-a<.0005:min_lower=min(min_lower,d-reach)
                else:stack.extend(((a,mid),(mid,c)))
            result.append({"case":r["name"],"fromMs":lo,"toMs":hi,"sampledMinimumMm":min_gap,"continuousLowerMm":None if intersect else min_lower,
                           "materialContact":intersect,"evaluatedIntervals":count})
    (HERE/"pads.json").write_text(json.dumps({"source":sha(HERE/"probe-native.json"),"results":result},indent=2)+"\n",encoding="utf8",newline="\n")
    print([(r["case"],r["continuousLowerMm"],r["materialContact"]) for r in result])


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("cache",type=Path);p.add_argument("lamp",type=Path)
    a=p.parse_args();run(a.cache,a.lamp)
