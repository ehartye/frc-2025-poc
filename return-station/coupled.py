"""Use only mutually compatible loaded cartridge and tube positions for previously inconclusive bins."""
import argparse
import json
from pathlib import Path
import sys
import numpy as np
from registration import HERE,ROOT,mount,sha
from receiver_relief import low_relief
from feed import Pipe,HALF,R


def run(cache,lamp):
    robot=low_relief(lamp)
    rows=json.loads((HERE/"probe-feed.json").read_bytes())
    initial=json.loads((HERE/"probe-native.json").read_bytes())
    results=[]
    # This is a separate supplier/bench allowance. A witness inside real working
    # end-face material requires the flat belt to retreat to at least this depth.
    for interval in initial["bins"]:
        hits=interval["outerEnvelopeOverlapMm3"]
        for name in hits:
            if not name.startswith("grip-"):continue
            side=1 if name.startswith("grip-L-") else -1
            bounds=mount.cad.bounds(robot[name])
            inner=side*(bounds[1] if side==1 else bounds[4])
            for row in rows:
                if "bias-" not in row["name"]:continue
                tr=np.load(cache/(row["name"]+".npy"))
                mask=(tr[:,0]>=interval["fromMs"]-.51)&(tr[:,0]<=interval["toMs"]+.51)
                segment=tr[mask];gaps=[];required=[]
                p=Pipe(row["actualReturn"],relief=True)
                for sample in segment:
                    p.p=sample[1:4]/1000;p.a=sample[4:7]/np.linalg.norm(sample[4:7])
                    points=p.footprint(side)
                    demand=max((1000*(side*(p.p[1]+r[1])-HALF) for r,w in points),default=-5)
                    reach=side*sample[2]+1000*(HALF*abs(p.a[1])+R*np.sqrt(max(0,1-p.a[1]**2)))
                    required.append(demand);gaps.append((inner,demand,reach))
                # Native contact run permits independent inboard reach through .40 mm.
                # A simultaneous belt witness caps inward travel. A separate .05 mm reserve
                # covers change in cap-side projection between consecutive 0.25/0.50ms samples.
                def lower(indentation):
                    return min(face+max(-.40,req-indentation)-pipe for face,req,pipe in gaps)-.05
                results.append({"fromMs":interval["fromMs"],"toMs":interval["toMs"],"case":row["name"],"part":name,"sideInnerFaceMm":inner,
                    "minimumRequiredOutwardMm":min(required),"atZeroIndentationMm":lower(0),"at025mmIndentationMm":lower(.25),"at050mmIndentationMm":lower(.5)})
    (HERE/"coupled-clearance.json").write_text(json.dumps({"source":sha(HERE/"probe-native.json"),"interstepAllowanceMm":.05,"cases":results},indent=2)+"\n",encoding="utf8",newline="\n")
    print([(k,min(r[k] for r in results)) for k in ("atZeroIndentationMm","at025mmIndentationMm","at050mmIndentationMm")])


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("cache",type=Path);p.add_argument("lamp",type=Path)
    a=p.parse_args();run(a.cache,a.lamp)
