"""Replay complete-robot contact bounds across the eligible loose optical onset intervals."""
import argparse
import json
from pathlib import Path

from match import ROOT, corrected, load_source, mission, settle
from qualify import write
from mission.review_combined_departure import POSES, SuppliedRobot, recorded_departure, digest


def run(poses):
    here=Path(__file__).resolve().parent
    clock=json.loads((here/"red-withdrawal-clock.json").read_text())
    field=json.loads((here/"field-trial.json").read_text())
    assembly=json.loads((here/"red-assembly.json").read_text())
    robot=SuppliedRobot(poses/"part-frames.json",{p:poses/(p+".step") for p in POSES},
                        hashes=assembly["pose_step_sha256"],radial=json.loads((poses/"rotation-bounds.json").read_text()),
                        grip_error=1.5,outer_error=15)
    drive=load_source("cartridge_full_match",ROOT/"design/receiver-drives/size_drives.py")
    output=[]
    feed_times=field["hold_variants"]["0"]["scheduled_station_lip_arrivals_s"]
    for row in clock["runs"]:
        key,sector,mu=row["piece"],row["sector_m"],row["shelf_coast_mu"]
        up=mission.run(target_y=sector,trajectory=corrected().RegisteredMission(sector),piece_id=key,
                       dt=.001,sample_every_step=True,grip_law=drive.Cartridge(shelf_mu=mu))
        eligible=[p for p in row["budgets"]["0.25"]["0"]["exposures"] if p["within_150ms"]]
        lo=min(p["onset_after_shelf_ms"][0] for p in eligible)
        hi=max(p["onset_after_shelf_ms"][1] for p in eligible)
        sweep=recorded_departure(robot,(sector,key,up),(lo+hi)/2000,(hi-lo)/2000)
        result={"piece":key,"sector_m":sector,"shelf_coast_mu":mu,"pixel_bound":.25,"wait_ms":0,
                "eligible_phases":[p["source_phase_mod17"] for p in eligible],
                "modeled_onset_interval_after_shelf_ms":[lo,hi],
                "last_possible_contact_after_shelf_ms":round(1000*(sweep["last_possible_contact_s"]-sweep["first_shelf_witness_s"]),3),
                "min_tail_separation_mm":sweep["min_tail_gap_mm"],
                "minimum_neighbor_sector_mm":sweep["checks_next"][1]["min_swept_bound_gap_mm"] if key=="coral-A" else None}
        if mu==.1:
            feed=feed_times[0 if key=="coral-A" else 1]
            duration=138-feed-sweep["floor_time_s"]
            post=settle.continue_contact(upstream=up,sector=sector,piece_id=key,withdrawal="after-separation",
                                         wait_after_true_separation_s=.110,duration=duration,dt=.0002)
            result["static_target_retention"]={"from_teleop_s":feed+sweep["floor_time_s"],"through_teleop_s":138,
               "stop":post["stop"],"still_paired_contact_s":post["ending_still_double_contact_s"],
               "final_pose_and_velocity":post["last_xz_vxvz_spin"],"min_shelf_clearance_mm":post["min_shelf_clearance_mm"],
               "commanded_motion_after_buzzer":False,"assumption":"Fixed target and no later external impacts."}
        output.append(result)
    return {"schema":"assembled-loose-observer-and-retention/1","clock_sha256":digest(here/"red-withdrawal-clock.json"),
            "assembly_sha256":digest(here/"red-assembly.json"),"field_trial_sha256":digest(here/"field-trial.json"),
            "pose_step_sha256":assembly["pose_step_sha256"],"runs":output}


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--poses",type=Path,required=True)
    parser.add_argument("--output",type=Path,required=True)
    args=parser.parse_args()
    write(args.output,run(args.poses))
