"""Carry the separately attained station bodies forward without reinitializing a pipe at the receiver."""
import argparse
import copy
import json
import math
import sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
sys.path.insert(0,str(ROOT/"full-match"))
sys.path.insert(0,str(ROOT/"design/receiver-drives"))
from match import corrected,trial
from size_drives import Cartridge
from scripts import assess_coral,station_l1_mission as mission,station_l1_settle as settle
from mission import moving_traffic as traffic, short_vision_clock as vision
from mission.review_combined_departure import POSES,SuppliedRobot,digest,recorded_departure,early_contacts,prepare_extreme

SOURCE=ROOT/"station-supply/transfer.json"
PINS={"source":"19c14207324a2ee9d0b7b0ab48095f206fa1b1c4fcc1fe5f1275d6b0213e67e7",
      "match":"5cf57314c6cd699d56e711737c85df21fef78d39ff978a9f2b04c19b9b416350"}
SECTORS={"coral-A":-.18,"coral-B":.22}


def checkpoint(data,transfer,route):
    if transfer["id"] not in SECTORS or transfer["available_teleop_s"]>transfer["human_release_teleop_s"]:
        raise ValueError("Unknown source identity or release precedes stock")
    feed=data["feed"]
    rate=round(feed["lip_mps"],6)
    seat=data["rates"][str(rate)]["centered_flat_model"]
    t=seat["simulated_s"]
    phase,cx,cy,h,z,stroke=route.pose(t)
    if phase!="retract" or abs(stroke-.24)>1e-8:
        raise ValueError("Station source ends at a different machine state")
    u=stroke+seat["last_local_x_m"]
    du=seat["last_relative_speed_mps"]
    first=seat["impact"]["time_s"]
    lip=transfer["ramp_lip_teleop_s"]
    actual=transfer["ideal_seat_check_teleop_s"]
    if abs(lip+first+t-actual)>1e-8:
        raise ValueError("Source clock discontinuity")
    return {"piece_id":transfer["id"],"status":seat["result"],"mass_kg":mission.M["pipeMass_kg"],
            "lip_teleop_s":lip,"seat_teleop_s":actual,"first_contact_from_lip_s":first,"seat_elapsed_s":t,
            "xyz_m":[cx+u*math.cos(h),cy+u*math.sin(h),z+mission.R],
            "vxyz_mps":[du*math.cos(h),du*math.sin(h),0],"yaw_rad":h,"pitch_rad":0,"yaw_rate_rad_s":0,
            "tube_axial_phase":"unobserved; circular section","axial_spin_at_seat_rad_s":None,
            "chute_lip_rolling_speed_magnitude_rad_s":feed["lip_mps"]/mission.R}


def transfer_model(data,entry,f,coast=.1):
    key=entry["id"]
    sector=SECTORS[key]
    route=f.RegisteredMission(sector)
    state=checkpoint(data,entry,route)
    run=mission.resume_from_seat(state,.001,route,Cartridge(shelf_mu=coast),sector,key)
    first=run["trace"][0]
    if run["events"][0]["event"]!="resume-from-source-seat" or math.dist(first["xyz_m"],state["xyz_m"])>1e-8:
        raise ValueError("Seat was reinitialized instead of continued")
    if math.dist(first["vx_vy_mps"],state["vxyz_mps"][:2])>1e-8 or abs(first["yaw_rad"]-state["yaw_rad"])>1e-8:
        raise ValueError("Momentum or orientation discontinuity")
    if len({sample["piece_id"] for sample in run["trace"]})!=1 or run["outcome"]!="paired-trough-first-contact":
        raise ValueError("Same-piece mission failed: "+run["outcome"])
    return state,run


def feeding(data):
    entries={t["id"]:t for t in data["identified_transfers"]}
    def feeder(ready,stock,previous,key,consumed):
        if stock is None or key in consumed:
            return None
        source=entries[key]
        release=source["human_release_teleop_s"]
        if max(ready,stock,previous)+1.1>release+.001:
            return None
        return {"piece_id":key,"stock_available_s":stock,"insertion_started_s":release-1.1,
                "chute_roll_start_s":release+data["feed"]["free_drop_s"],"lip_arrival_s":source["ramp_lip_teleop_s"]}
    return feeder


def optics(robot,up):
    from mission import end_cartridge_vision as v
    front=robot.hulls["place"][3]+min(2,robot.errors["outer_slide_x_mm"])
    schedules,_=vision.schedule(up,front)
    cloud={k:shape for k,shape in robot.shapes.items() if k!="observer-camera-proxy"}
    at={round(r["time_s"]*1000,4):r for r in up["trace"]}
    geometry={}
    for budget in schedules.values():
        for e in budget["0"]["exposures"]:
            time=round(e["capture_from_lip_ms"],4)
            if time in geometry:continue
            ends=v.stereo.endpoints(at[time])
            hits=set()
            for eye in (-v.stereo.BASELINE/2,v.stereo.BASELINE/2):
                origin=(v.stereo.ORIGIN[0],eye,v.stereo.ORIGIN[2])
                for end in ends:
                    for height in (0,v.stereo.RADIUS):
                        target=(end[0],end[1],end[2]+height)
                        shortlist={k:shape for k,shape in cloud.items() if v.ray_box(origin,target,robot.boxes[k])}
                        hits.update(v.hits(shortlist,origin,target))
            geometry[time]=sorted(hits)
    for budgets in schedules.values():
        for budget in budgets.values():
            for e in budget["exposures"]:
                e["nominal_ray_hits"]=geometry[round(e["capture_from_lip_ms"],4)]
    return front,schedules


def witness_paths(robot,state,up,sector,entry):
    key=entry["id"]
    lip=entry["ramp_lip_teleop_s"]
    front=robot.hulls["place"][3]+min(2,robot.errors["outer_slide_x_mm"])
    budgets,_=vision.schedule(up,front)
    # Lowest and highest permitted arrivals bound the continuous chassis-and-slide sweep.
    eligible=[e for options in budgets.values() for e in options["0"]["exposures"] if e["within_150ms"]]
    low=min(e["onset_after_shelf_ms"][0] for e in eligible);high=max(e["onset_after_shelf_ms"][1] for e in eligible)
    dep=recorded_departure(robot,(sector,key,up),(low+high)/2000,(high-low)/2000)
    front,budgets=optics(robot,up)
    native=early_contacts(robot,dep,low/1000,up)
    if not native or any(row["forward_endpoint_inward_grip_overlap_mm3"] or
                         any(body["overlap_mm3"] for body in row["bodies"].values()) for row in native):
        raise ValueError("Native wall material interferes before the observer permits withdrawal")
    last=round((dep["last_possible_contact_s"]-dep["first_shelf_witness_s"])*1000,3)
    slotted={}
    for px,options in budgets.items():
        wait=options["0"]
        slotted[px]={}
        for e in wait["exposures"]:
            ok=e["within_150ms"] and not e["nominal_ray_hits"] and e["onset_after_shelf_ms"][0]>last
            slotted[px][e["source_phase_mod17"]]={**e,"eligible":ok}
    # Use the worst tight-pixel zero-wait onset for the conservative motion case.
    tight=[e for e in slotted["0.1"].values() if e["eligible"]]
    if len(tight)!=17:
        raise ValueError("Not all nominal tight-pixel phases qualify")
    delay=max(e["onset_after_shelf_ms"][1] for e in tight)/1000
    case=dict(sector=sector,upstream=up,piece_id=key,withdrawal="after-separation",dt=.0002)
    post=settle.continue_contact(**case,wait_after_true_separation_s=delay)
    if post["stop"]!="sustained-paired-floor-and-lip":
        raise ValueError("No sustained paired contact in no-spin design case")
    supported=post["start_from_station_s"]+post["elapsed_s"]
    if lip+supported>135:
        raise ValueError("Offered source cannot deliver on time")
    end=settle.continue_contact(**case,wait_after_true_separation_s=delay,duration=138-lip-post["start_from_station_s"])
    if end["stop"]!="sustained-paired-floor-and-lip":
        raise ValueError("No static-target retention at assessment")
    events={e["event"]:e["time_s"] for e in up["events"]}
    recovered=max(0,.370-max(0,supported-dep["first_shelf_witness_s"]-delay)*.15)/.15+1
    record={"piece_id":key,"sector_m":sector,"seat_seconds":events["drive-and-turn"],"support_from_feed_s":supported,
            "empty_retract_and_lift_s":recovered,"settled_target_xz_m":end["last_xz_vxvz_spin"][:2]}
    spin={}
    for speed in (-state["chute_lip_rolling_speed_magnitude_rad_s"],0,state["chute_lip_rolling_speed_magnitude_rad_s"]):
        alt=settle.continue_contact(**case,wait_after_true_separation_s=delay,initial_roll=speed)
        spin[str(round(speed,6))]={"stop":alt["stop"],"elapsed_s":alt["elapsed_s"],"min_shelf_gap_mm":alt["min_shelf_clearance_mm"]}
    late=settle.continue_contact(**case,wait_after_true_separation_s=.6)
    return record,{"state":state,"station_source":entry,"resume_first_sample":up["trace"][0],
            "events_teleop_s":{k:lip+v for k,v in events.items()},
            "release_trajectory_outcome":up["outcome"],"actuator_onset_after_shelf_ms":[low,high],
            "native_clearance_samples":native,"front_bound_mm":front,"last_possible_contact_ms":last,
            "min_tail_gap_mm":dep["min_tail_gap_mm"],"neighbor":dep["checks_next"][1],
            "images":slotted,"number_50ms_wait_eligible":{px:options["50"]["phases_within_150ms_prior_contact_bracket"] for px,options in budgets.items()},
            "zero_spin":{"selected_worst_tight_onset_ms":round(delay*1000),"support_teleop_s":lip+supported,
                         "settled_xz_m":end["last_xz_vxvz_spin"][:2],"through_teleop_s":138,"last_paired_still_s":end["ending_still_double_contact_s"]},
            "axial_spin_at_target_sensitivity_rad_s":spin,"delayed_600ms":{"stop":late["stop"],"minimum_shelf_gap_mm":late["min_shelf_clearance_mm"]}}


def main(model,poses,source_path):
    if digest(SOURCE)!=PINS["source"] or digest(ROOT/"full-match/field-trial.json")!=PINS["match"]:
        raise ValueError("Issued source or field configuration changed")
    expected=json.loads((Path(__file__).parent/"robot-models.json").read_bytes())[model]
    data=json.loads(source_path.read_bytes())
    issued=json.loads(SOURCE.read_bytes())
    if data["receive_step_sha256"]!=expected["poses"]["receive"] or data["field_step_sha256"]!=issued["field_step_sha256"]:
        raise ValueError("Supplied pickup source does not match the selected receiving pose and field")
    for path,key in ((source_path,"pickup_sha256"),(poses/"part-frames.json","inventory_sha256"),
                     (poses/"rotation-bounds.json","rotation_sha256")):
        if digest(path)!=expected[key]:
            raise ValueError("Source differs from selected robot contract: "+key)
    robot=SuppliedRobot(poses/"part-frames.json",{k:poses/(k+".step") for k in POSES},
        hashes=expected["poses"],radial=json.loads((poses/"rotation-bounds.json").read_bytes()),outer_error=15,grip_error=1.5)
    if len(robot.boxes)!=expected["body_count"]:
        raise ValueError("Incomplete body inventory")
    prepare_extreme(robot,ROOT/"design/receiver-drives/receiver_drives.py")
    f=corrected()
    records=[];cases=[]
    scene=copy.deepcopy(traffic.INPUT)
    for t,e in zip(scene["trips"],data["identified_transfers"],strict=True):
        key=e["id"];sector=SECTORS[key]
        state,up=transfer_model(data,e,f)
        src,result=witness_paths(robot,state,up,sector,e)
        t["support_confirmation_s"]=t["feed_s"]+src["support_from_feed_s"]
        t["departure"]["settled_target_xz_m"]=src["settled_target_xz_m"]
        records.append(src);cases.append(result)
    feeder=feeding(data)
    outcomes={}
    schedules=(("centered",{}),("partner-holds-start",{"vacate_lane":False}),("return-to-station",{"park":False}),
               ("second-stock-late",{"stock":(0,111),"park":False}))
    for name,options in schedules:
        result,team=trial(f,scene,records,station_feeder=feeder,**options)
        if name=="centered":
            primary_team=team
        outcomes[name]={"support":result["qualified_supports"],"lip_arrivals":result["scheduled_station_lip_arrivals_s"],
                        "scheduled_end_s":result["rules"]["teleop_motion_completion_s"],"park_overlap_m":result["rules"]["red_barge_park_overlap_m"],
                        "first_body_pair":result["first_pair"],"min_interrobot_gap_m":result["dynamic_min_gap_m"],
                        "model_conditional_point_ceiling":result["potential_points_after_other_acceptance"]}
    primary=outcomes["centered"]
    if [p["id"] for p in primary["support"]]!=list(SECTORS) or primary["park_overlap_m"]<=0:
        raise ValueError("Lost a source identity or station handoff")
    safety=traffic.analyze(0,(0,35),mission_record=scene,prepared=(primary_team,[],primary["support"]))
    # The new trajectories reach the same finite branch-exclusion region; the
    # moving roster check supplies a continuous other-robot exclusion there.
    if safety["other_robot_to_target_min_bound_m"]<=0:
        raise ValueError("A moving robot can disturb the stored pipe")
    witnesses=[{"id":p["piece_id"],"observedMs":138000,"touchesOwnTrough":True,"touchesAlliedRobot":False,
                "touchesOpponentRobot":False,"scoredOnAnotherReefLevel":False,"fullySupportedByRobot":False,
                "stuckToRobot":False,"intentionalHerding":False,"pushesScoredL1WhilePlacing":False} for p in records]
    assessment={e:assess_coral.assess({"schema":"reefscape-coral-observations/1","phase":"TELEOP",
              "basis":"Centered no-rebound source, zero target spin, native robot clearance, branch exclusions and scheduled six-robot traffic.",
              "samples":[{"timeMs":138000,"inventoryComplete":True,"robotCommandActive":False,"pieces":witnesses}]},e)
             for e in assess_coral.EDITIONS}
    return {"schema":"station-sourced-match/1","model":model,"robot_bodies":len(robot.boxes),"pose_sha256":robot.hashes,
            "robot_inventory_sha256":digest(poses/"part-frames.json"),"source_receipt_sha256":digest(source_path),
            "issued_predecessor_sha256":PINS,"hulls_mm":robot.hulls,"case_studies":cases,"scenarios":outcomes,
            "end_assessment":assessment,"min_other_robot_to_settled_piece_m":safety["other_robot_to_target_min_bound_m"],
            "rejected_upstream_bounce":data["feed"]["first_rebound_alternatives"],"public_hal_authorization":False,
            "physical_mission":"pending","limits":["No measurement of axial spin after seating or at target, bounce, tracking, human transfer, or friction.",
              "Drive schedule uses the reference 67 kg limits and level carpet. No current official score or as-built qualification.",
              "Camera end-rim extraction, physical pressure and actuation remain simulated. Existing HAL cases have distinct source identities."]}


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model",required=True,choices=("reference-401","shop-545"))
    for key in ("poses","source","output"):
        parser.add_argument("--"+key,type=Path,required=True)
    args=parser.parse_args()
    args.output.write_text(json.dumps(main(args.model,args.poses,args.source),indent=2,allow_nan=False)+"\n",encoding="utf8",newline="\n")
