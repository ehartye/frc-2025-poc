"""Reopen the exact installed receiver, field station and robot bodies on the coupled candidate."""
import argparse
import copy
import json
import math
import sys

from spin_cases import CANDIDATE
from pipe_spin_route import HERE,ROOT,DATA,SECTORS,SHELF,U_ORIGIN,trial,digest,vision
from pipe_axial_contact import K
from replay_source_match import optics,feeding,corrected
from mission.review_combined_departure import POSES,SuppliedRobot,recorded_departure,early_contacts,prepare_extreme
from mission import moving_traffic
from match import trial as match_trial
sys.path.insert(0,str(ROOT/"station-supply"))
from transfer import corridor,hits,parts,cad,sha,FIRST_SHA


def intake(field,poses,path,summary):
    if sha(field)!=FIRST_SHA:raise ValueError("Unexpected January field")
    shapes=parts(field)
    boxes={k:cad.bounds(v) for k,v in shapes.items()}
    ids=json.loads((poses/"part-frames.json").read_bytes())
    robot=cad.read_named(poses/"receive.step",[{"id":k} for k in ids])
    robo={k:cad.bounds(v) for k,v in robot.items()}
    impact=summary["impact"]["time_s"]
    air=[r for r in path if r[0]<impact]
    terminal=(impact,summary["impact"]["local_u_m"],SHELF,summary["impact"]["spin_before_rad_s"])
    air.append(terminal)
    # Acceleration is bounded by gravity and both capped 10 N/side friction forces.
    # Expand each block chord by a measured max center deviation plus a conservative
    # substep allowance. First/last tangencies remain visible in the outer enclosure.
    spans=[]
    for i in range(0,len(air)-1,12):
        chunk=air[i:min(i+13,len(air))]
        ta,u0,z0,_=chunk[0];tb,u1,z1,_=chunk[-1]
        if tb<=ta:continue
        bend=max(math.hypot(u-u0-(u1-u0)*(t-ta)/(tb-ta),z-z0-(z1-z0)*(t-ta)/(tb-ta))
                 for t,u,z,_ in chunk)
        h=max(b[0]-a[0] for a,b in zip(chunk,chunk[1:]))
        grow=1000*(bend+20*h*h/8)+.00001
        r0=((u0+.240)*1000,z0*1000);r1=((u1+.240)*1000,z1*1000)
        s0=((U_ORIGIN-u0)*1000,z0*1000);s1=((U_ORIGIN-u1)*1000,z1*1000)
        spans.append({"time_s":[ta,tb],"extra_radius_mm":grow,"station":corridor(shapes,boxes,s0,s1,grow),
                      "robot":corridor(robot,robo,r0,r1,grow)})
    low,high=summary["capture_travel_m"]
    travel=corridor(robot,robo,((low+.240)*1000,SHELF*1000),((high+.240)*1000,SHELF*1000))
    positive={}
    unexpected=set()
    for s in spans:
        for k in s["robot"]:
            if k not in {"receiver-R6-ported","grip-L-420-3M-15","grip-R-420-3M-15"}:unexpected.add(k)
        for k in s["station"]:
            if not k.endswith("/GE-25010"):unexpected.add(k)
    if unexpected or travel:
        for t,u,z,_ in path[::4]:
            active=hits(robot,robo,(u+.240)*1000,z*1000)
            for k,vol in active.items():positive[k]=max(positive.get(k,0),vol)
    return {"flight_spans":spans,"supported_center_interval_m":[low,high],"supported_nominal_enclosure_mm3":travel,
            "unexpected_envelope_names":sorted(unexpected),"positive_sampled_pipe_wall_material_mm3":positive}


def check(field,poses):
    register=json.loads((ROOT/"sourced-match/robot-models.json").read_bytes())["shop-545"]
    for name,key in (("part-frames.json","inventory_sha256"),("rotation-bounds.json","rotation_sha256")):
        if digest(poses/name)!=register[key]:raise ValueError("Motion register changed")
    robot=SuppliedRobot(poses/"part-frames.json",{k:poses/(k+".step") for k in POSES},hashes=register["poses"],
        radial=json.loads((poses/"rotation-bounds.json").read_bytes()),outer_error=15,grip_error=1.5)
    prepare_extreme(robot,ROOT/"design/receiver-drives/receiver_drives.py")
    result={"contact_source_sha256":digest(HERE/"pipe_axial_contact.py"),"transport_source_sha256":digest(HERE/"pipe_spin_route.py"),"field_sha256":FIRST_SHA,"robot_poses_sha256":robot.hashes,"departures":{}}
    scene=copy.deepcopy(moving_traffic.INPUT)
    records=[]
    for entry,trip in zip(DATA["identified_transfers"],scene["trips"],strict=True):
        row,up,path=trial(CANDIDATE,entry,.0005)
        key=entry["id"];sector=SECTORS[key]
        if key=="coral-A":result["intake"]=intake(field,poses,path,row["pickup"])
        if row["status"]!="sustained-paired-floor-and-lip":raise ValueError("Unexpected target outcome")
        start=up["trace"][0]["time_s"]
        camera=dict(up,trace=[r for r in up["trace"] if abs((r["time_s"]-start)*1000-round((r["time_s"]-start)*1000))<.0001])
        front,budgets=optics(robot,camera)
        exposures=budgets["0.1"]["0"]["exposures"]
        lo=min(e["onset_after_shelf_ms"][0] for e in exposures)
        hi=max(e["onset_after_shelf_ms"][1] for e in exposures)
        dep=recorded_departure(robot,(sector,key,up),(hi+lo)/2000,(hi-lo)/2000,
                               initial_roll=row["release_spin_rad_s"],inertia_ratio=K)
        native=early_contacts(robot,dep,lo/1000,up)
        shelf=dep["first_shelf_witness_s"]
        last=(dep["last_possible_contact_s"]-shelf)*1000
        clear=sum(bool(e["within_150ms"] and not e["nominal_ray_hits"] and e["onset_after_shelf_ms"][0]>last) for e in exposures)
        result["departures"][key]={"front_bound_mm":front,"tight_17_phases_clear":clear,"onset_after_shelf_ms":[lo,hi],
               "last_possible_envelope_ms":last,"min_tail_gap_mm":dep["min_tail_gap_mm"],
               "next_sector_mm":dep["checks_next"][1]["min_swept_bound_gap_mm"],"native_material_samples":native}
        events={e["event"]:e["time_s"] for e in up["events"]}
        support=row["target"]["end_teleop_s"]-row["lip_teleop_s"]
        recovery=max(0,.370-max(0,support-shelf-hi/1000)*.15)/.15+1
        final=row["target"]["final"][:2]
        trip["support_confirmation_s"]=trip["feed_s"]+support
        trip["departure"]["settled_target_xz_m"]=final
        records.append(dict(piece_id=key,sector_m=sector,seat_seconds=events["drive-and-turn"],support_from_feed_s=support,
            empty_retract_and_lift_s=recovery,settled_target_xz_m=final))
    # The same declared two human feeds may not both be available after changing the cycle.
    outcome,_=match_trial(corrected(),scene,records,station_feeder=feeding(DATA))
    result["issued_human_schedule"]={k:outcome[k] for k in ("scheduled_station_lip_arrivals_s","qualified_supports","rules","first_pair","dynamic_min_gap_m")}
    return result


if __name__=="__main__":
    from pathlib import Path
    parser=argparse.ArgumentParser()
    parser.add_argument("--field",required=True,type=Path)
    parser.add_argument("--poses",type=Path,default=ROOT/"design/competition-package/shop-duplex/red")
    parser.add_argument("--output",type=Path,default=HERE/"geometry.json")
    a=parser.parse_args()
    a.output.write_text(json.dumps(check(a.field,a.poses),indent=2,allow_nan=False)+"\n",encoding="utf8",newline="\n")
