"""Check the actual pre-departure pipe with the fixed stereo at each powered station pose."""
import argparse
import json
from pathlib import Path
import sys
import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
sys.path.insert(0,str(ROOT/"shelf-contact"))
from custody import field_mesh,scene,read_pair,silhouette,sha

def run(assets,field_step):
    baseline=json.loads((ROOT/"shelf-contact/fine-custody.json").read_bytes())
    robot_file=assets/"station-guide/selected-background/stowed-robot-fixed.npz"
    if sha(field_step)!=baseline["fieldSha256"] or sha(robot_file)!=baseline["shopDepthSha256"]:
        raise ValueError("Changed station/native camera")
    field=field_mesh(field_step)
    views=np.load(robot_file)
    robot=[(views[k],views[k+"_z"]) for k in ("left","right")]
    out={}
    for r in json.loads((HERE/"feeds.json").read_bytes()):
        if r["dtS"]!=.00003125 or r["state"]!="IN_BAND":continue
        side=r["case"]
        bg=scene(field,r["sourceRobotMmDeg"],robot)
        empty=[g for g,_ in bg]
        raw=assets/"conditioned-match/corrected-feeds"/f"{side}-400000-4-31.25.npy"
        if sha(raw)!=r["trajectorySha256"]:raise ValueError("Unregistered pipe state")
        tr=np.load(raw,mmap_mode="r")
        frames=[]
        for tick in (5372,5389,5406,5423,5440):
            q=np.array([np.interp(tick,tr[:,0],v) for v in tr[:,1:7].T])
            fit=read_pair(silhouette(bg,q[:3],q[3:]/np.linalg.norm(q[3:])),empty,560,0)
            bounds=(fit.get("u_interval_mm"),fit.get("y_interval_mm"),fit.get("bottom_relative_to_shelf_mm"))
            enclosed=all(b is not None and b[0]<=v<=b[1] for v,b in zip((q[0],q[1],q[2]-557.15),bounds))
            frames.append({"captureJavaMs":tick,"receivedJavaMs":tick+15,"status":fit["status"],"bounds":bounds,"containsSimulatedCenter":enclosed})
        out[side]={"pipe":"coral-A","trajectorySha256":sha(raw),"frames":frames,"empty":read_pair(empty,empty,560,0)["status"]}
    (HERE/"seat-images.json").write_text(json.dumps(out,indent=2)+"\n",encoding="utf8",newline="\n")

if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("assets",type=Path);p.add_argument("field",type=Path);a=p.parse_args();run(a.assets,a.field)
