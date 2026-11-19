"""Screen independent fixed-eye custody on the two actually attained machined-shelf pipe states."""
import argparse
import math
import json
from pathlib import Path
import sys
import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent

def run(assets,field_step,structure):
    sys.path.insert(0,str(structure))
    from custody_check import guard
    from visibility import register
    from custody import field_mesh,scene,read_pair,silhouette,sha
    baseline=json.loads((ROOT/"shelf-contact/fine-custody.json").read_bytes())
    camera_file=assets/"shelf-structure/native-background/stowed-robot-fixed.npz"
    if sha(camera_file)!="dc3898f7a3f27bdd8707ef47280e20a766b52e7980f135478b911a2015132fd7" or sha(field_step)!=baseline["fieldSha256"]:raise ValueError("Changed station camera")
    changes=structure/"stow-receiver.step"
    if sha(changes)!="69960175cc8845b4eef4fed67538908091450369ede93adbfd0e1dc85b042b01":raise ValueError("Changed machined native material")
    data=np.load(camera_file)
    robot=[(data[k],data[k+"_z"]) for k in ("left","right")]
    route=assets/"fixed-camera-route/scene.json"
    if sha(route)!="291d68c43e21f20c33885064e6bbe8c0e47b684d13a8a06e8af61f9fd9db5886":raise ValueError("Changed fixed harness geometry")
    fixed_lead=register(json.loads(route.read_bytes())["stowed"])
    masked,masks=guard(robot,2.,fixed_lead)
    field=field_mesh(field_step)
    folder=assets/"conditioned-match/coupled-corrected"
    result={}
    for r in json.loads((folder/"machined.json").read_bytes()):
        if r["stepS"]!=.00003125:continue
        side=r["case"];entry={"portState":r["lastState"],"nativeSha256":sha(changes),"nativeStowedBackgroundSha256":sha(camera_file),"fixedLeadSceneSha256":sha(route),"fixedLeadEnclosures":len(fixed_lead),"trajectorySha256":r["trajectorySha256"],"finishes":{}}
        result[side]=entry
        if r["lastState"]!="IN_BAND" or r["lastJavaMs"]!=5460:continue
        raw=folder/(r["id"]+".npy")
        if sha(raw)!=r["trajectorySha256"]:raise ValueError("Mismatched machined pipe")
        tr=np.load(raw,mmap_mode="r")
        bg=scene(field,r["sourceRobotMmDeg"],masked)
        nominal=scene(field,r["sourceRobotMmDeg"],robot)
        for label,finish in (("native",None),("dark",0),("pipe-intensity",220)):
            pose=nominal if finish is None else [(np.where(m & (z==mz),finish,g).astype(np.uint8),z) for (g,z),(_,mz),m in zip(bg,masked,masks)]
            empty=[g for g,_ in pose];rows=[]
            control=read_pair(empty,empty,560,0)["status"]
            for t in (5372,5389,5406,5423,5440):
                q=np.array([np.interp(t,tr[:,0],v) for v in tr[:,1:7].T])
                pair=silhouette(pose,q[:3],q[3:]/np.linalg.norm(q[3:]))
                fit=read_pair(pair,empty,560,0)
                b=[fit.get(k) for k in ("u_interval_mm","y_interval_mm","bottom_relative_to_shelf_mm")]
                inside=all(v is not None and v[0]<=x<=v[1] for x,v in zip((q[0],q[1],q[2]-557.15),b))
                rows.append({"captureJavaMs":t,"firstServiceMs":5*math.ceil((t+15)/5),"status":fit["status"],"containsSimulatedCenter":inside,"oneEyeBlank":read_pair([empty[0],pair[1]],empty,560,0)["status"]})
            entry["finishes"][label]={"empty":control,"frames":rows}
        print(side,entry["finishes"],flush=True)
    (HERE/"machined-images.json").write_text(json.dumps(result,indent=2)+"\n",encoding="utf8",newline="\n")

if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    for name in ("assets","field","structure"):p.add_argument(name,type=Path)
    a=p.parse_args();run(a.assets,a.field,a.structure)
