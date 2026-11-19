"""Generate independent pixels from the issued acquisition trajectories, then replay the position witness."""
import argparse
import csv
import hashlib
import io
import json
import math
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
from imaging import HERE,ROOT,RED,R,exposure,read_mesh,images_for_pose
from observe import read_pair
from clock import CustodyClock,RECEIVE_SHA
import sys
sys.path.insert(0,str(ROOT/"observable-acquisition"))
import study as acquisition


def digest(path): return hashlib.sha256(path.read_bytes()).hexdigest()


def plant(case,rows,reference):
    conditions=acquisition.Case(**reference["conditions"][case])
    history=[]
    def note(contact,u,v,yaw,phase,time,zrel=0):
        history.append((time*1000,u*1000,acquisition.route.SHELF*1000+zrel*1000))
        return original(contact,u,v,yaw,phase,time,zrel)
    original=acquisition.AxialContact.normals
    sequence=["{state},{active},{belt_mps},{reason}".format(**r) for r in rows]
    console=SimpleNamespace(stdin=io.StringIO(),stdout=io.StringIO("\n".join(sequence)+"\n"))
    with patch.object(acquisition.AxialContact,"normals",note):
        trial=acquisition.run(console,conditions,.0005,[])
    if json.loads(json.dumps(trial))!=reference["cases"][case]["0.0005"]:
        raise ValueError("acquisition source changed")
    history=np.array(history)
    def position(ms):
        if ms<history[0,0] or ms>history[-1,0]: return None
        return [240+np.interp(ms,history[:,0],history[:,1]),0,float(np.interp(ms,history[:,0],history[:,2]))]
    return position


def picture(background, pipe, height, stroke, **options):
    pixels=exposure(background,pipe,options.get("yaw",0),options.get("intensity",220))
    if options.get("one_eye"): pixels[1]=background[1][0]
    if options.get("clutter"):
        for img in pixels: img[42:132,180:432]=232
    if options.get("lighting"):
        pixels=[(im.astype(float)*.94+8).astype(np.uint8) for im in pixels]
    return read_pair(pixels,[image for image,_ in background],height,stroke,pixel_error=options.get("error",1.0))


def make_frames(background,position,last,phase=0,first=0,cache=None):
    result=[]
    if cache is None: cache={}
    for seq,ms in enumerate(range(phase,last+1,17)):
        if ms<first: continue
        xyz=position(ms)
        key=None if xyz is None else tuple(round(v,6) for v in xyz)
        if key not in cache:
            cache[key]=picture(background,xyz,940,240)
        result.append({"sequence":seq,"capture_ms":ms,"arrived_ms":ms+15,"epoch":"trial","pose_sha256":RECEIVE_SHA,
                       "observation":cache[key]})
    return result


APPLICATION=[]
CAMERA=[]


def execute(rows,frames,*,name=None,calibrated=True,moving_from=None,revoke_ms=None,wrong_epoch=False):
    clock=CustodyClock()
    positive=[]
    first_revoked=None
    previous=False
    next_frame=0
    for r in rows:
        now=int(r["ms"])
        pending=[]
        while next_frame<len(frames) and frames[next_frame]["arrived_ms"]<=now:
            packet=frames[next_frame]
            next_frame+=1
            if revoke_ms is None or now<revoke_ms: pending.append(packet)
        ok=clock.tick(now,r["state"]=="IN_BAND",pending,calibrated=calibrated,
                      stationary=moving_from is None or now<moving_from,epoch="other" if wrong_epoch else "trial")
        if name:
            APPLICATION.append([name,now,*[r[k] for k in ("sample_ms","valid","calibrated","docked","enabled","entry","seat")],
                                int(calibrated),int(moving_from is None or now<moving_from),"other" if wrong_epoch else "trial",
                                r["state"],int(ok)])
            for p in pending:
                o=p["observation"]
                CAMERA.append([name,now,p["sequence"],p["capture_ms"],p["arrived_ms"],p["epoch"],o["status"],
                               *o.get("u_interval_mm",["",""]),*o.get("y_interval_mm",["",""]),
                               *o.get("bottom_relative_to_shelf_mm",["",""]),p["pose_sha256"]])
        if ok: positive.append(now)
        if previous and not ok and first_revoked is None: first_revoked=now
        previous=ok
    return {"first_corroborated_ms":positive[0] if positive else None,"last_corroborated_ms":positive[-1] if positive else None,
            "positive_ticks":len(positive),"first_revoked_ms":first_revoked,"last_status":clock.last_result}


def draw_example(background):
    from PIL import Image
    canvases=[("Empty registered receiver",background[0][0]),
              ("Seated pipe · left mono",exposure(background,[315.67,0,937.15])[0]),
              ("Seated pipe · right mono",exposure(background,[315.67,0,937.15])[1])]
    out=['<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 1000 279"><rect width="1000" height="279" fill="#12212c"/>']
    for i,(title,img) in enumerate(canvases):
        x=12+330*i
        out.append(f'<text x="{x}" y="20" fill="white" font-family="sans-serif" font-size="14">{title}</text>')
        small=np.array(Image.fromarray(img).resize((320,240),Image.Resampling.BOX))//16
        paths={}
        for row,line in enumerate(small):
            breaks=np.flatnonzero(np.diff(line.astype(int)))+1
            for a,b in zip(np.r_[0,breaks],np.r_[breaks,320]):
                paths.setdefault(int(line[a]),[]).append(f"M{a},{row}h{b-a}")
        out.append(f'<g transform="translate({x},30)" stroke-width="1.05">')
        for level,commands in paths.items():
            gray=level*16+8
            out.append(f'<path stroke="rgb({gray},{gray},{gray})" d="{"".join(commands)}"/>')
        out.append("</g>")
    (HERE/"stereo-views.svg").write_text("".join(out)+"</svg>\n",encoding="utf-8",newline="\n")


def generate():
    APPLICATION.clear()
    CAMERA.clear()
    ref=json.loads((ROOT/"observable-acquisition/results.json").read_bytes())
    with (ROOT/"observable-acquisition/ticks.csv").open() as file: ticks=list(csv.DictReader(file))
    rows={k:[r for r in ticks if r["case"]==k and r["step_us"]=="500"] for k in ("nominal","low-grip")}
    routes={k:plant(k,rows[k],ref) for k in rows}
    meshes=read_mesh()
    source=images_for_pose(meshes["receive"])
    draw_example(source)
    clock0={}
    for k in rows:
        clock0[k]=make_frames(source,routes[k],int(rows["nominal"][-1]["ms"]))
    last=int(rows["nominal"][-1]["ms"])
    empty=make_frames(source,lambda t:None,last)
    cache={}
    phases=[execute(rows["nominal"],make_frames(source,routes["nominal"],last,p,first=1240,cache=cache),name=f"phase-{p}")
            for p in range(17)]
    failed=ref["cases"]["low-grip"]["0.0005"]["physical_loss"]["u_mm"]
    edge=make_frames(source,lambda t:[240+failed,0,937.15],last)
    repeated=[dict(f,sequence=1,capture_ms=1275) for f in clock0["nominal"] if f["capture_ms"]>=1275]
    other_pose=[dict(f,pose_sha256="different-pose") for f in clock0["nominal"]]
    result={"nominal":execute(rows["nominal"],clock0["nominal"],name="nominal"),
            "different-pose":execute(rows["nominal"],other_pose,name="different-pose"),
            "replayed-ports-empty-shelf":execute(rows["nominal"],empty,name="empty"),
            "replayed-ports-weak-grip":execute(rows["nominal"],clock0["low-grip"],name="weak-replay"),
            "replayed-ports-pipe-at-rear-edge":execute(rows["nominal"],edge,name="rear-edge"),
            "repeated-camera-capture":execute(rows["nominal"],repeated,name="repeated"),
            "true-ports-weak-grip":execute(rows["low-grip"],clock0["low-grip"],name="true-weak"),
            "uncalibrated":execute(rows["nominal"],clock0["nominal"],name="uncalibrated",calibrated=False),
            "moving":execute(rows["nominal"],clock0["nominal"],name="moving",moving_from=1420),
            "camera-disconnected":execute(rows["nominal"],clock0["nominal"],name="disconnect",revoke_ms=1420),
            "old-attempt":execute(rows["nominal"],clock0["nominal"],name="old-epoch",wrong_epoch=True)}
    once={}
    for label,xyz,kwargs in [
        ("seated",[315.67,0,937.15],{}),("wrong-forward",[350,0,937.15],{}),
        ("behind-seat",[270,0,937.15],{}),("airborne",[315.67,0,972.15],{}),
        ("lateral",[315.67,30,937.15],{}),("missing-eye",[315.67,0,937.15],{"one_eye":True}),
        ("lit-empty",None,{"lighting":True}),("flat-clutter",None,{"clutter":True}),
        ("contrast-lost",[315.67,0,937.15],{"intensity":35}),("yaw-8",[315.67,0,937.15],{"yaw":8}),
        ("loose-errors",[315.67,0,937.15],{"error":3})]:
        once[label]=picture(source,xyz,940,240,**kwargs)
    for pose in ("stowed","transport","place"):
        model=meshes[pose]
        bg=images_for_pose(model)
        once[pose]=picture(bg,[model["stroke"]+75.67,0,model["height"]-60+R],model["height"],model["stroke"])
    example=clock0["nominal"][-1]
    if not example["observation"]["status"]=="position-witness":
        example=next(f for f in reversed(clock0["nominal"]) if f["observation"]["status"]=="position-witness")
    pose_sha=json.loads((ROOT/"axial-spin/study.json").read_bytes())["robot_pose_sha256"]
    for name,sha in pose_sha.items():
        if digest(RED/(name+".step"))!=sha: raise ValueError("545 pose changed")
    for filename,head,observations in (
        ("application.csv","case,ms,sample_ms,valid,ports_calibrated,docked,enabled,entry,seat,camera_calibrated,stationary,epoch,port_state,corroborated",APPLICATION),
        ("camera.csv","case,application_ms,sequence,capture_ms,arrived_ms,epoch,measurement,u_min,u_max,y_min,y_max,z_min,z_max,pose_sha256",CAMERA)):
        with (HERE/filename).open("w",encoding="ascii",newline="") as out:
            writer=csv.writer(out,lineterminator="\n")
            writer.writerow(head.split(","))
            writer.writerows(observations)
    files=[HERE/k for k in ("studies.py","observe.py","clock.py","imaging.py","application.csv","camera.csv")]
    files.extend([RED/"robot.html",ROOT/"observable-acquisition/ticks.csv",ROOT/"observable-acquisition/results.json",
                  ROOT/"observable-acquisition/study.py",ROOT/"release-observer/stereo.py",ROOT/"design/installed-procurement/installed.csv"])
    data={"schema":"station-independent-position/1","pose_sha256":pose_sha,"source_sha256":{
              p.relative_to(ROOT).as_posix():digest(p) for p in files},"schedule":result,"frames":once,
          "example_packet":example,"all_integer_stereo_phases":phases,
          "observations_nominal":[[f["capture_ms"],f["observation"]["status"]] for f in clock0["nominal"]],
          "real_calibration":False,"physical_load_measured":False,"carry_permission":False}
    (HERE/"results.json").write_text(json.dumps(data,indent=2,allow_nan=False)+"\n",encoding="utf-8",newline="\n")


if __name__=="__main__": generate()
