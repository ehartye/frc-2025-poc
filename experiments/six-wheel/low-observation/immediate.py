"""Test the earlier whole-front clock and ensuing retraction against all fixed-height robot bodies."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
from unittest.mock import patch
import numpy as np
from source import HERE, ROOT, inputs, handoff
from geometry import hardware,pose_layers,original

PIPE_HALF_X=57.65*math.cos(math.radians(1))+151.0625*math.sin(math.radians(1))
ACTUATION_BOUND=10


def forward_planes():
    layers,_,_,frames=pose_layers(560,0,"red")
    parts=original.flatten(layers)
    changed,added=hardware(parts,height=560)
    parts.update(changed)
    parts.update(added)
    frames.update({name:"base" for name in added})
    if len(parts)!=577 or set(parts)!=set(frames):raise ValueError("fixed configuration inventory changed")
    turns=original.rotation_bounds()
    scale={"base":0,"lift":0,"slide":1,"middle":.5,"variable-rope":0,"variable-draw-wire":1}
    winners={}
    for name,shape in sorted(parts.items()):
        front=original.native.bounds(shape)[3]
        if name in turns:front=max(front,turns[name]["bounds_mm"][3])
        rate=scale[frames[name]]
        if rate not in winners or front>winners[rate]["unextended_front_mm"]:
            winners[rate]={"unextended_front_mm":front,"governing_body":name}
    return winners


def run(jdk):
    result=json.loads((HERE/"results.json").read_bytes())
    planes=forward_planes()
    def front(stroke):
        return max(group["unextended_front_mm"]+rate*stroke for rate,group in planes.items())
    for s in (0,370):
        if abs(front(s)-result["poses"][str(s)]["front_mm"])>.001:raise ValueError("end registration mismatch")
    receipts={"native_front_planes_by_stroke_factor":planes,"pieces":{}}
    rows=[]
    for piece,source in inputs(jdk).items():
        primary=result["pieces"][piece]
        phases=[]
        motion={}
        for row in primary["phases"]:
            capture=row["first_proven_forward_ms"]
            prior=row["earliest_prior_ms"]
            tick=None if capture is None else 20*math.ceil((capture+15)/20)
            good=tick is not None and 7<=tick-capture<=29 and tick-prior+ACTUATION_BOUND<=150
            rows.append([piece,row["phase_ms"],capture,prior,tick,int(good)])
            phases.append({"phase_ms":row["phase_ms"],"capture_ms":capture,"tick_ms":tick,
                           "time_window_only":good})
            if not good:continue
            for lag in range(ACTUATION_BOUND+1):
                onset=tick+lag
                if onset in motion:continue
                world_contact=source["truth"]["paired-trough-first-contact"]
                clear=source["truth"]["finite-pipe-envelope-clear-of-shelf-by-1mm"]
                withdraw=onset-clear
                kw=dict(source["settle_kwargs"],wait_after_true_separation_s=withdraw/1000)
                original_gap=handoff.spin.settle.shelf_gap
                margin=[]
                # Piece center and tip are in registered target coordinates. Converting back to robot X adds 490 mm.
                # The scalar front bound includes all bodies at each stroke and the same absolute-error reserve.
                def gap(x,z,tip,top):
                    stroke=490+1000*tip-165
                    margin.append(490+1000*x-PIPE_HALF_X-front(stroke)-1)
                    return original_gap(x,z,tip,top)
                with patch.object(handoff.spin.settle,"shelf_gap",gap):
                    out=handoff.spin.settle.continue_contact(**kw)
                tr=source["trajectory"]
                if onset<world_contact:
                    for t in np.r_[onset,tr[(tr[:,0]>onset)&(tr[:,0]<world_contact),0]]:
                        x=np.interp(t,tr[:,0],tr[:,1])
                        margin.append(x-PIPE_HALF_X-front(370-min(370,max(0,t-onset)*.15))-1)
                motion[onset]={"motion_after_true_clear_ms":withdraw,"first_target_contact_to_end_s":out["elapsed_s"],
                               "outcome":out["stop"],"minimum_whole_front_gap_mm":min(margin),
                               "minimum_shelf_gap_mm":out["min_shelf_clearance_mm"],"contact_dt_s":out["dt_s"]}
        receipts["pieces"][piece]={"phases":phases,"onsets_ms":motion,
                                  "physically_calibrated":False,"continuous_track_established":False}
    receipts["primary_sha256"]=hashlib.sha256((HERE/"results.json").read_bytes()).hexdigest()
    (HERE/"immediate.json").write_text(json.dumps(receipts,indent=2,allow_nan=False)+"\n",encoding="utf-8",newline="\n")
    with (HERE/"immediate.csv").open("w",newline="",encoding="ascii") as file:
        writer=csv.writer(file,lineterminator="\n")
        writer.writerow("piece,phase,capture_ms,prior_lower_ms,first_eligible_tick_ms,timely".split(","))
        writer.writerows(rows)


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--java-home",type=Path,required=True)
    run(p.parse_args().java_home)
