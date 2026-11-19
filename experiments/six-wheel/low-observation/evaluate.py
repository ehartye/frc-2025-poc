"""Apply the issued grayscale estimator and clock to the fixed-height fork."""
import argparse
import csv
from functools import lru_cache
import json
import math
from pathlib import Path
from unittest.mock import patch
import numpy as np
from prepare import HERE, ROOT, source_pins
import clock
import studies
from imaging import exposure, R, HALF
from observe import read_pair


def evaluate(scratch, output):
    output.mkdir(parents=True,exist_ok=True)
    source=json.loads((scratch/"source.json").read_bytes())
    if source["sources"]!=source_pins():raise ValueError("regenerate cached geometry and plant inputs")
    backgrounds={}
    for stroke in (0,370):
        with np.load(scratch/f"background-{stroke}.npz") as arrays:
            backgrounds[stroke]=[(arrays[k],arrays[k+"_z"]) for k in ("left","right")]
    result={"inputs":source["sources"],"poses":source["poses"],"pieces":{}}
    studies.APPLICATION.clear()
    studies.CAMERA.clear()
    screening=[]
    for key,record in source["pieces"].items():
        trajectory=np.load(scratch/f"{key}-trajectory.npy")
        events=record["truth"]
        register=source["poses"]["0"]["mesh_sha256"]
        front=source["poses"]["370"]["front_mm"]
        empty_pose=lambda s:[g for g,d in backgrounds[s]]
        def at(ms):
            return tuple(np.interp(ms,trajectory[:,0],col) for col in trajectory[:,1:].T)
        @lru_cache(maxsize=1024)
        def image_at(stroke,coords,rigid):
            bg=backgrounds[stroke]
            if coords is None:images=exposure(bg,None)
            else:images=exposure(bg,coords[:3],coords[3])
            if rigid:
                for (gray,depth),image in zip(bg,images):
                    diff=np.abs(image.astype(int)-gray.astype(int))>36
                    for row in np.flatnonzero(diff.any(axis=1)):
                        cols=np.flatnonzero(diff[row])
                        if np.any(depth[row,max(0,cols[0]-2):cols[-1]+3]>=3490):
                            image[row]=gray[row]
            return read_pair(images,empty_pose(stroke),560,stroke)
        def position(ms,stroke,rigid=False):
            coords=tuple(round(float(v),6) for v in at(ms)[:4]) if ms>=trajectory[0,0] else None
            return image_at(stroke,coords,rigid)
        rows=[dict(zip(("ms","sample_ms","valid","calibrated","docked","enabled","entry","seat","state"),row[2:11]))
              for row in record["acquisition_ticks"]]
        packets=[dict(sequence=i,capture_ms=t,arrived_ms=t+15,epoch="trial",pose_sha256=register,observation=position(t,0))
                 for i,t in enumerate(range(0,rows[-1]["ms"]+1,17))]
        with patch.object(clock,"RECEIVE_SHA",register):
            checks={}
            for name,frames,kwargs in [
                ("nominal",packets,{}),("wrong-epoch",packets,{"wrong_epoch":True}),
                ("empty",[dict(f,observation=image_at(0,None,False)) for f in packets],{}),
                ("wrong-pose",[dict(f,pose_sha256=studies.RECEIVE_SHA) for f in packets],{}),
                ("uncalibrated",packets,{"calibrated":False}),
                ("stale",packets,{"revoke_ms":record["first_in_band_ms"]+140})]:
                checks[name]=studies.execute(rows,frames,name=key+"-"+name,**kwargs)
        drive=events["drive-and-turn"]
        stop=events["lower-at-target"]
        extend=events["extend-at-target"]
        release=events["place"]
        first_contact=events["paired-trough-first-contact"]
        last_pad=record["theoretical_last_pad_ms"]
        samples=[(int(ms),position(ms,0,True)) for ms in np.arange(drive,stop,250)]
        counts={}
        for ms,obs in samples:counts[obs["status"]]=counts.get(obs["status"],0)+1
        controls={"station_after_ports":position(record["first_in_band_ms"],0),
                  "last_stationary_before_drive":position(drive-.5,0),
                  "first_transit_rigid_backdrop":samples[0][1],
                  "last_stationary_before_extension":position(extend-.5,0),
                  "first_stationary_at_release":position(release,370),
                  "last_powered_shelf_contact":position(events["left-powered-shelf-with-same-piece"],370),
                  "first_shelf_1mm_clear":position(events["finite-pipe-envelope-clear-of-shelf-by-1mm"],370),
                  "at_first_trough_contact":position(first_contact,370)}
        # All phases use pixels formed before the two post-contact continuations diverge.
        # A still-valid observation establishes an upper crossing time. Earlier contact must be independently attributed.
        start=int(events["finite-pipe-envelope-clear-of-shelf-by-1mm"])
        phases=[]
        for phase in range(17):
            exposures=range(start+(phase-start)%17,math.floor(first_contact)+1,17)
            earliest=None
            for t in exposures:
                o=position(t,370)
                u=o.get("u_interval_mm")
                yaw=math.radians(1)
                forward_half=(R+.5)*math.cos(yaw)+(HALF+.25)*math.sin(yaw)
                if u and 370+u[0]-forward_half-(front+1)>=1:
                    earliest=t
                    break
            # Optimistic bound: zero actuator delay, contemporaneous prior load, no scene/identity loss.
            # A 2 ms scan + 1 ms timestamp error encloses the lower prior, +1 ms the clearance upper bound.
            lower=math.floor(last_pad-3)
            app=None if earliest is None else 20*math.ceil(max(earliest+15,earliest+1+50)/20)
            timely=app is not None and app-lower<=150
            phases.append(dict(phase_ms=phase,first_proven_forward_ms=earliest,first_eligible_tick_ms=app,
                               earliest_prior_ms=lower,latest_tick_ms=20*((lower+150)//20),timely=timely))
            screening.append([key,phase,earliest,lower,app,int(timely)])
        teleop_offset=record["target_contact_teleop_s"]-first_contact/1000
        result["pieces"][key]={"truth_ms":events,"teleop_minus_local_seconds":teleop_offset,
            "station":checks,"blind_durations_ms":{"moving_field_unregistered":stop-drive,"moving_slide_unregistered":release-extend},
            "rigid_backdrop":{"interval_ms":250,"sample_status_counts":counts},"selected_exposures":controls,
            "phases":phases,"ideal_prior_pad_ms":last_pad,"unwithdrawn_plant":record["fixed_slide_outcome"],
            "hypothetical_110ms_plant":record["post_status"],"first_usable_station_teleop_s":
             teleop_offset+checks["nominal"]["first_corroborated_ms"]/1000}
    for name,head,records in [
        ("application.csv","case,ms,sample_ms,valid,ports_calibrated,docked,enabled,entry,seat,camera_calibrated,stationary,epoch,port_state,corroborated",studies.APPLICATION),
        ("camera.csv","case,application_ms,sequence,capture_ms,arrived_ms,epoch,measurement,u_min,u_max,y_min,y_max,z_min,z_max,pose_sha256",studies.CAMERA),
        ("withdrawal.csv","piece,phase,capture_ms,prior_lower_ms,first_eligible_tick_ms,timely",screening)]:
        with (output/name).open("w",encoding="ascii",newline="") as file:
            writer=csv.writer(file,lineterminator="\n")
            writer.writerow(head.split(","))
            writer.writerows(records)
    (output/"results.json").write_text(json.dumps(result,indent=2,allow_nan=False)+"\n",encoding="utf-8",newline="\n")


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--scratch",type=Path,required=True)
    p.add_argument("--output",type=Path,default=HERE)
    args=p.parse_args()
    evaluate(args.scratch,args.output)
