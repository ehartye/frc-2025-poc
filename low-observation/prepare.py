"""Materialize source-revision-bound frame backgrounds and the two existing fixed-height feeder paths."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from source import ROOT,HERE,inputs,LOW
from geometry import render_pose


def source_pins():
    files=["design/fixed-height/"+name for name in ("low-locks.step","native.py","handoff.py","handoff.json","ports.py","ports-065.json")]
    files += ["design/competition-package/shop-duplex/assembly.py","design/competition-package/shop-duplex/red/source-register.json",
              "observable-acquisition/study.py","stereo-custody/imaging.py","stereo-custody/observe.py","scripts/station_l1_settle.py",
              "low-observation/source.py","low-observation/geometry.py","low-observation/prepare.py"]
    return {f:hashlib.sha256((ROOT/f).read_bytes()).hexdigest() for f in files}


def prepare(scratch,jdk):
    scratch.mkdir(parents=True,exist_ok=True)
    data={"sources":source_pins(),"poses":{},"pieces":{}}
    for stroke in (0,370):
        pairs,meta=render_pose(stroke)
        data["poses"][str(stroke)]=meta
        np.savez_compressed(scratch/f"background-{stroke}.npz",left=pairs[0][0],right=pairs[1][0],
                            left_z=pairs[0][1],right_z=pairs[1][1])
    for key,record in inputs(jdk).items():
        times=record["truth"]
        tr=record["trajectory"]
        np.save(scratch/f"{key}-trajectory.npy",tr)
        response=record["post"]
        contact=times["paired-trough-first-contact"]
        # Settlement uses target X relative to the lip and the robot center is 490 mm back.
        post=np.array([[contact+1000*t,490+1000*x,0,1000*z,vx,vz,omega] for t,x,z,vx,vz,omega in response["trace"]])
        if np.linalg.norm(post[0,1:4]-tr[-1,1:4])>3:raise ValueError("target/robot coordinate discontinuity")
        np.save(scratch/f"{key}-settled.npy",post)
        last_pad=None
        for row in record["source_run"]["trace"]:
            if row["state"]!="supported" or row["phase"]!="place":continue
            x,y,z=row["xyz_m"]
            cx,cy,a=row["chassis_xy_heading"]
            u=1000*((x-cx)*np.cos(a)+(y-cy)*np.sin(a)-row["slide_m"])
            if abs(u-155)<=2.8:last_pad=record["origin"]+1000*row["time_s"]
        data["pieces"][key]={"truth":times,"lip_ms":record["origin"],"first_in_band_ms":record["first"],
            "acquisition_ticks":record["acquisition_ticks"],"theoretical_last_pad_ms":last_pad,
            "contact_input":response["initial_xz_vxvz"],"post_status":response["stop"],
            "fixed_slide_outcome":record["post_fixed"]["stop"],"target_contact_teleop_s":record["report"]["after-separation"]["targetContactTeleopS"]}
    (scratch/"source.json").write_text(json.dumps(data,indent=2,allow_nan=False)+"\n",encoding="utf-8",newline="\n")


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--scratch",type=Path,required=True)
    p.add_argument("--java-home",type=Path,required=True)
    args=p.parse_args()
    prepare(args.scratch,args.java_home)
