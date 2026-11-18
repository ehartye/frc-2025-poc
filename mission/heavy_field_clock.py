"""Replay the higher-mass differential drive and finite station under the existing other-robot programs."""
import argparse
import copy
import importlib.util
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from mission import moving_traffic as traffic
from scripts import station_l1_mission as mission, station_l1_settle as settle


def revised_motion():
    # The registered model's design constants are module-local. Isolate the focal
    # candidate from the five independent robot programs and the historical replay.
    spec=importlib.util.spec_from_file_location("scripts.heavy_registered_transit",ROOT/"scripts/registered_transit.py")
    motion=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(motion)
    motion.DRIVE=dict(motion.DRIVE,turnWheelAccelMps2=.64)
    return motion


def run(contact_root=None):
    motion=revised_motion()
    producer=mission
    cartridge=None
    if contact_root:
        from mission.review_combined_departure import load_source
        producer=load_source("scripts.profiled_station_l1",contact_root/"scripts/station_l1_mission.py")
        cartridge=load_source("profiled_traction",contact_root/"design/receiver-drives/size_drives.py").Cartridge
    template=copy.deepcopy(traffic.INPUT)
    cases=[]
    for p in template["trips"]:
        sector=p["sector_m"]
        key=p["piece_id"]
        route=motion.RegisteredMission(sector)
        args={"grip_law":cartridge()} if cartridge else {}
        up=producer.run(dt=.001,target_y=sector,trajectory=route,sample_every_step=True,piece_id=key,**args)
        if up["outcome"]!="paired-trough-first-contact":
            raise ValueError("Upstream custody or delivery failed for "+key+": "+up["outcome"])
        events={e["event"]:e["time_s"] for e in up["events"]}
        post=settle.continue_contact(sector=sector,upstream=up,piece_id=key,withdrawal="after-separation",
                                     wait_after_true_separation_s=.127)
        if post["stop"]!="sustained-paired-floor-and-lip":
            raise ValueError("No supported state for "+key)
        obs=post["start_from_station_s"]+post["elapsed_s"]
        p["support_confirmation_s"]=p["feed_s"]+obs
        p["departure"]["settled_target_xz_m"]=post["last_xz_vxvz_spin"][:2]
        rows=template["events"]
        idx=next(i for i,e in enumerate(rows) if e["event"]=="loaded-"+key)
        rows[idx-1]["elapsed_s"]=events["drive-and-turn"]
        idx=next(i for i,e in enumerate(rows) if e["event"]=="empty-return-"+key)
        withdrew=max(0,obs-post["clear_witness_from_station_s"]-.127)*.15
        rows[idx-2]["elapsed_s"]=max(0,.370-withdrew)/.15
        cases.append({"piece":key,"loaded_travel_s":route.carry.duration,"empty_return_s":route.return_leg.duration,
                      "seconds_after_lip_until_support":obs,"settled_target_xz_m":p["departure"]["settled_target_xz_m"],
                      "recovery_until_transport_s":rows[idx-2]["elapsed_s"]+rows[idx-1]["elapsed_s"]})
    outcomes={}
    for key,(_,hold,stock) in traffic.SCENARIOS.items():
        result=traffic.analyze(hold,stock,mission_record=template,focal_routes=motion.routes)
        outcomes[key]={k:result[k] for k in ("hold_before_start_s","stock_ready_s","first_sampled_overlap",
            "min_separating_bound_m","other_robot_to_target_min_bound_m","support_observations",
            "planned_empty_finish_s","empty_finish_available")}
    return {"schema":"registered-67kg-cycle-clock/1","design_mass_kg":67,"focal_drive":motion.DRIVE,
            "other_motion":traffic.f.DRIVE,"trips":cases,"scenarios":outcomes}


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output",type=Path,required=True)
    p.add_argument("--contact-root",type=Path,help="Project with the corresponding passive receiver contact law")
    a=p.parse_args()
    a.output.write_text(json.dumps(run(a.contact_root),indent=2,allow_nan=False)+"\n",encoding="utf-8",newline="\n")
