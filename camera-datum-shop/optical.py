"""Render the replacement support and check the tested calibration/pose envelope."""
import argparse,json,sys
from pathlib import Path
import numpy as np
from camera_parts import HERE,ROOT,SHELF,read,sha,mount
from fit import assembly
from replay_candidate import model,mod
import optics
from bridge_fit import fit as read_moving
from calibration import scenario,matrix
from observe import read_pair

def run(lamp,scratch):
    source,kit,full=assembly(lamp)
    _,group,color,frame=mount.pose_layers(560,370,"red")
    cad=mount.cad
    old=model(lamp)
    def mesh(shapes,roof):
        parts=[]
        for k,v in sorted(shapes.items()):
            if roof and (not (k.startswith("roof-") or k==SHELF or frame.get(k)=="slide") or k=="roof-camera-envelope"):continue
            if roof:v=cad.translated(v,[-370,0,0])
            xyz,tri=mount.original.triangulate(v)
            parts.append([k,group.get(k,"Mount"),color.get(k,"#737d85"),xyz,tri])
        return {"parts":parts}
    camera=optics.camera
    fixed_results={}
    selected=[("prior",old),("shop",full)]
    backgrounds={}
    for key,robot in selected:
        backgrounds[key]=camera.images_for_pose(mesh(robot,False))
    optics.ORIGIN=np.array([48.,0.,842.])
    # Keep the fixed imager's native rays for the release test.
    static=read(ROOT/"low-observation/results.json")
    fixed=backgrounds["shop"];prior=backgrounds["prior"]
    finite_pieces=0
    statuses={}
    delta_bounds=0.
    for piece,record in static["pieces"].items():
        tr=np.load(scratch/f"{piece}-trajectory.npy")
        for t in range(int(np.ceil(record["truth_ms"]["place"])),int(np.floor(record["truth_ms"]["paired-trough-first-contact"]))+1):
            x,y,z,a=[np.interp(t,tr[:,0],v) for v in tr[:,1:5].T]
            obtained=[]
            for bg in (prior,fixed):
                obtained.append(read_pair(camera.exposure(bg,[x,y,z],a),[g for g,_ in bg],560,370))
            left,right=obtained
            statuses[left["status"]+"/"+right["status"]]=statuses.get(left["status"]+"/"+right["status"],0)+1
            for k in ("u_interval_mm","y_interval_mm","bottom_relative_to_shelf_mm"):
                if (k in left)!=(k in right):raise ValueError("Lost fixed feature")
                if k in left:delta_bounds=max(delta_bounds,float(np.max(np.abs(np.asarray(left[k])-right[k]))))
            finite_pieces+=1
    def differences(left,right):
        return [{"gray_changed":int(np.count_nonzero(a[0]!=b[0])),"depth_changed_over_point1mm":int(np.count_nonzero(abs(a[1]-b[1])>.1))} for a,b in zip(left,right)]
    fixed_results={"empty_native_pixels":differences(prior,fixed),"exposure_count":finite_pieces,
                   "status_pairs":statuses,"max_interval_change_mm":delta_bounds}
    optics.configure()
    roof=mesh(full,True)
    bg=camera.images_for_pose(roof)
    old_bg=camera.images_for_pose(mesh(old,True))
    compare=differences(old_bg,bg)
    samples=[(68.58,0.,557.15,-1.15),(80.,0.,557.15,0.),(91.42,0.,557.15,1.15)]
    source_cases=read(ROOT/"bridge-tolerance/calibration.json")["cases"]
    cases={}
    for entry in source_cases:
        cases[entry["case"]]=scenario(roof,np.array(entry["translation"]),matrix(entry["axis"],entry["degrees"]),bg,samples)
    normal={};cache={}
    for key in static["pieces"]:
        src=read(scratch/"source.json")["pieces"][key]
        tr=np.load(scratch/f"{key}-trajectory.npy")
        first=17*(int(src["first_in_band_ms"])//17)
        issue={};error=0.;count=0
        for t in range(first,int(src["truth"]["place"])+17,17):
            x,y,z,a,s=[np.interp(t,tr[:,0],v) for v in tr[:,1:].T]
            p=tuple(round(v,6) for v in (x-s,y,z,a))
            if p not in cache:cache[p]=read_moving(camera.exposure(bg,p[:3],p[3]),bg)
            r=cache[p]
            if r["status"]!="inside":issue[t]=r["status"]
            else:
                for k,v in zip(("x_mm","y_mm","bottom_mm"),(p[0],p[1],p[2]-500-camera.R)):
                    low,high=r[k];error=max(error,low-v,v-high)
            count+=1
        normal[key]={"exposures":count,"misses":issue,"maximum_truth_outside_mm":error}
    out={"camera_step_sha256":sha(HERE/"camera-shelf.step"),"host_source":sha(ROOT/"bridge-tolerance/native-place.json"),"robot_body_count":len(full),
        "fixed_view":fixed_results,"moving_empty_native_pixels":compare,"nominal_moving":normal,"registration_cases":cases}
    (HERE/"optical.json").write_text(json.dumps(out,indent=2)+"\n",encoding="utf8",newline="\n")
if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("--lamp",type=Path,required=True);p.add_argument("--scratch",type=Path,required=True)
    a=p.parse_args();run(a.lamp,a.scratch)
