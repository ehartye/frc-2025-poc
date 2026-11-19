"""Reopen the issued stereo mounting and check optical rays at registered and intermediate receiver poses."""
import csv
import hashlib
import json
import math
import sys
from collections import Counter
from imaging import HERE,ROOT,RED,R,HALF,ORIGIN,EYES,stereo
sys.path.insert(0,str(ROOT/"design/competition-package/shop-duplex"))
import assembly
sys.path.insert(0,str(ROOT/"release-observer"))
from build_mount import hits


def box_hit(box,origin,end):
    near,far=0,1
    for i in range(3):
        v=end[i]-origin[i]
        if abs(v)<1e-12:
            if not box[i]-1e-6<=origin[i]<=box[i+3]+1e-6:return False
        else:
            a,b=sorted(((box[i]-origin[i])/v,(box[i+3]-origin[i])/v))
            near,far=max(near,a),min(far,b)
    return near<=far


def run():
    frames=json.loads((RED/"part-frames.json").read_bytes())
    issued=json.loads((ROOT/"axial-spin/study.json").read_bytes())["robot_pose_sha256"]
    cases=[("receive",940,240),("half-retracted",940,120),("stowed",940,0),
           ("half-lowered",805,0),("transport",670,0),("place",560,370)]
    records=[]
    for name,height,slide in cases:
        if name in issued:
            path=RED/(name+".step")
            sha=hashlib.sha256(path.read_bytes()).hexdigest()
            if sha!=issued[name]: raise ValueError("wrong source")
            shapes=assembly.cad.read_named(path,[{"id":k} for k in frames])
        else:
            sha=None
            shapes=assembly.pose_shapes(height,slide,"red")
        if set(shapes)!=set(frames):raise ValueError("wrong component inventory")
        boxes={k:assembly.cad.bounds(p) for k,p in shapes.items() if k!="observer-camera-proxy"}
        obstacles=Counter()
        clipped=0
        tested=0
        for u in (72.,80.,88.):
            for a in (-30,0,30,55):
                theta=math.radians(a)
                for y in (-HALF,-115,0,115,HALF):
                    target=(slide+u-R*math.cos(theta),y,height-60+R+R*math.sin(theta))
                    for eye in EYES:
                        origin=tuple(ORIGIN+[0,eye,0])
                        px,py=stereo.project(target,eye)
                        if not (0<=px<640 and 0<=py<480): clipped+=1
                        local={k:shapes[k] for k,b in boxes.items() if box_hit(b,origin,target)}
                        obstacles.update(hits(local,origin,target))
                        tested+=1
        records.append({"name":name,"lift_mm":height,"slide_mm":slide,"step_sha256":sha,
                        "bodies":len(shapes),"rays":tested,"out_of_frame":clipped,
                        "positive_obstacles":dict(sorted(obstacles.items())),
                        "camera_bounds_mm":assembly.cad.bounds(shapes["observer-camera-proxy"]),
                        "center_pad_bounds_mm":assembly.cad.bounds(shapes["observer-moving-pad-1"])})
    with (ROOT/"design/installed-procurement/installed.csv").open(encoding="utf-8") as stream:
        inventory=[r for r in csv.reader(stream) if r[0].startswith("observer-")]
    return {"poses":records,"existing_observer_bodies":inventory,"added_bodies":0,"added_mass_kg":0,
            "pc_and_camera_frame":frames["observer-camera-proxy"],"fsr_frame":frames["observer-moving-pad-1"],
            "sourced_pipe_kg":.816466266,"static_pipe_weight_N":.816466266*9.80665,
            "min_gap_disk_to_seated_pipe_projection_mm":155-3.6-(87.69514446832102+R)}


if __name__=="__main__":
    (HERE/"native.json").write_text(json.dumps(run(),indent=2,allow_nan=False)+"\n",encoding="utf-8",newline="\n")
