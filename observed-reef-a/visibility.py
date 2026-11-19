"""Separate shop/guide self-occlusion from the carried pipe under the original approach."""
import argparse
import json
import math
from pathlib import Path
import numpy as np
from pipe_scene import HERE,ROOT,nav,robot_scene,carried
from vision import station_frame

def run(assets):
    optical=robot_scene(assets)
    original=nav.checked_scene(assets)
    rows={}
    for side in ("leftSmall","rightSmall"):
        record=next(r for r in nav.read(ROOT/"conditioned-match/slow-cargo.json") if r["dtS"]==.0000625 and r["wheelCase"]==side)
        path=assets/"conditioned-match"/side/"slow/A/cargo-62500ns.npy"
        if nav.sha(path)!=record["relativePipeSha256"]:
            raise ValueError("Changed achieved pipe")
        # These are independent stationary visibility comparisons. A local pose after
        # cargo is physically lost cannot be used to authorize the moving mission.
        q=np.load(path,mmap_mode="r")[-1]
        actual=carried(optical,1000*q[1:4],q[4:7].copy())
        out=[]
        for x in (-950.,-700.,-450.,-200.,-75.,0.):
            pose=np.array([x,-180.,0.])
            for name,ctx in (("no-pipe",optical),("finite-pipe-at-last-sample",actual),("full-storage-enclosure",original)):
                tag,heads,proj,_=station_frame(pose,ctx)
                errors=[[float(min([np.linalg.norm(px-u) for px in found],default=1e3)) for u in truth]
                        for found,truth in zip(heads,proj)]
                out.append({"normalMm":x,"shape":name,"bothEyesDecodeTag":tag is not None,
                    "detectedCandidates":[len(i) for i in heads],"minimumCentroidErrorPx":errors})
        rows[side]={"tubeSource":record["relativePipeSha256"],"tubeFinalSourceWallS":record["endWallS"],
                    "sourceLostPipe":record["loss"],"comparisons":out}
    return rows

if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("assets",type=Path);a=p.parse_args()
    (HERE/"visibility.json").write_text(json.dumps(run(a.assets),indent=2)+"\n",encoding="utf8",newline="\n")
