"""Cross the stationary fixed-eye and moving roof exposures on the actual machined pipe."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent

def run(assets,structure,field):
    sys.path.insert(0,str(structure))
    import carry_images as sensor
    from custody import field_mesh,scene,read_pair,silhouette
    from shelf_marks import apply
    from paths import O,F,U,L
    from visibility import depths
    from custody_check import cable_boxes
    native=assets/"shelf-structure/native-background/stowed-robot-fixed.npz"
    register=json.loads((ROOT/"conditioned-match/machined-images.json").read_bytes())
    if sensor.digest(native)!=register["leftSmall"]["nativeStowedBackgroundSha256"]:
        raise ValueError("Different stationary receiver")
    if sensor.digest(field)!=json.loads((ROOT/"full-match/installed-stock.json").read_bytes())["field_step_sha256"]:
        raise ValueError("Different field")
    full=sensor.background(native)
    marked,_=apply(full,O,F,U,0)
    cable=assets/"fixed-camera-route/scene.json"
    if sensor.digest(cable)!=register["leftSmall"]["fixedLeadSceneSha256"]:
        raise ValueError("Different fixed harness")
    bounds=cable_boxes(json.loads(cable.read_bytes())["stowed"])
    masked=[]
    for off,(gray,z) in zip(sensor.camera.EYES,marked):
        near=depths(bounds,O+off*L,F,U)
        occupied=(near<=z+.2)&np.isfinite(near)
        masked.append((np.where(occupied,0,gray).astype(np.uint8),np.minimum(z,near)))
    sector=assets/"observed-reef-a/machined-fixed.npz"
    np.savez_compressed(sector,left=masked[0][0],left_z=masked[0][1],right=masked[1][0],right_z=masked[1][1])
    field_shape=field_mesh(field)
    result={}
    dep=json.loads((structure/"departure.json").read_bytes())["cases"]
    for rec in dep:
        side=rec["case"]
        arrival=next(r for r in json.loads((ROOT/"conditioned-match/elastic.json").read_bytes())["cases"] if r["trajectorySha256"]==rec["stationTrajectorySha256"])
        f=assets/"conditioned-match/coupled-corrected"/(arrival["id"]+".npy")
        if sensor.digest(f)!=rec["stationTrajectorySha256"]:raise ValueError("Different pipe")
        states=np.load(f,mmap_mode="r")
        sensor.scenes.camera=sensor.camera
        sensor.scenes.fixed_pose(O)
        bg=scene(field_shape,arrival["sourceRobotMmDeg"],masked)
        empty=[g for g,_ in bg]
        fixed=[]
        for tm in (5372,5389,5406,5423,5440):
            q=np.array([np.interp(tm,states[:,0],c) for c in states[:,1:7].T])
            images=silhouette(bg,q[:3],q[3:]/np.linalg.norm(q[3:]))
            fit=read_pair(images,empty,560,0)
            bounds_now=[fit.get(key) for key in ("u_interval_mm","y_interval_mm","bottom_relative_to_shelf_mm")]
            enclosed=all(z is not None and z[0]<=v<=z[1] for z,v in zip(bounds_now,(q[0],q[1],q[2]-557.15)))
            fixed.append([tm,fit["status"],enclosed])
        cases={}
        for mode in ("roof","roof-050","roof-same"):
            opt,ref=sensor.camera_views(assets,assets/"elastic-carry",mode)
            rows=[]
            for tm in (5372,5389,5406,5423,5440,5457):
                q=np.array([np.interp(tm,states[:,0],c) for c in states[:,1:7].T])
                fit=sensor.fit_row(sensor.silhouette(opt,q[:3],q[3:]/np.linalg.norm(q[3:])),ref,True,0)
                rows.append([tm,fit[0],sensor.outcome(fit,(q[0],q[1],q[2]-557.15))])
            cases[mode]=rows
        result[side]={"pipeSha256":sensor.digest(f),"nativeStowedSha256":sensor.digest(native),"fixedLeadSha256":sensor.digest(cable),"machinedFieldBackgroundSha256":sensor.digest(sector),"fixedMarked":fixed,"roof":cases}
    return result

if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    for key in ("assets","elastic","field"):p.add_argument(key,type=Path)
    a=p.parse_args()
    (HERE/"station-bridge.json").write_text(json.dumps(run(a.assets,a.elastic,a.field),indent=2)+"\n",encoding="utf8",newline="\n")
