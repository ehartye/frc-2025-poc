"""Check a complete competition-field trial, including initial stock, AUTO transition and actual ending pose."""
import argparse
import copy
import json
import math
import sys
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from mission import heavy_field_clock as heavy, moving_traffic as traffic
from mission.replay_registered_field import others,rect
from mission.review_combined_departure import load_source
from mission import end_cartridge_vision as vision
from scripts import station_l1_mission as mission, station_l1_settle as settle

FACTOR=25.4/1000
HULL=(-425,-425,0,425,425,1059)
COORDS=[(x,y) for x in (48,690.875-48) for y in (158.5-72,158.5,158.5+72)]
MARKS=[(f"floor-stack-{i+1}",rect(x*FACTOR,y*FACTOR,14.7*FACTOR,14.7*FACTOR)) for i,(x,y) in enumerate(COORDS)]
STANDOFF=.65


def corrected():
    f=heavy.revised_motion()
    f.STAGE=f.add(f.STATION,f.rot((STANDOFF,0),f.NORMAL))
    f.NORTH=(440*f.IN,230*f.IN)
    return f


def build_park(f,start):
    route=f.Route(start)
    route.go((425*f.IN,start[1]),reverse=True)
    route.go((425*f.IN,95.5625*f.IN))
    route.go((355*f.IN,95.5625*f.IN))
    return route


def basis(f):
    record=copy.deepcopy(traffic.INPUT)
    law=load_source("declared_receiver_model",ROOT/"design/receiver-drives/size_drives.py")
    sources=[]
    for trip in record["trips"]:
        key,y=trip["piece_id"],trip["sector_m"]
        route=f.RegisteredMission(y)
        up=mission.run(target_y=y,dt=.001,trajectory=route,sample_every_step=True,piece_id=key,grip_law=law.Cartridge())
        if up["outcome"]!="paired-trough-first-contact":
            raise ValueError("Custody or transfer failed for "+key)
        events={e["event"]:e["time_s"] for e in up["events"]}
        shelf=events["finite-pipe-envelope-clear-of-shelf-by-1mm"]
        frames=[(r["time_s"],vision.plane_gap(vision.stereo.image_pairs(vision.stereo.endpoints(r)),537,.1))
                for r in up["trace"] if r["time_s"]>=events["left-powered-shelf-with-same-piece"]]
        captures=[next(t for t,g in frames if round(t*1000)%17==p and g>=1) for p in range(17)]
        delay=max(captures)-shelf+.039
        post=settle.continue_contact(sector=y,upstream=up,piece_id=key,withdrawal="after-separation",
                                      wait_after_true_separation_s=delay)
        if post["stop"]!="sustained-paired-floor-and-lip":
            raise ValueError("Missing sustained support: "+key)
        supported=post["start_from_station_s"]+post["elapsed_s"]
        trip["support_confirmation_s"]=trip["feed_s"]+supported
        trip["departure"]["settled_target_xz_m"]=post["last_xz_vxvz_spin"][:2]
        retract=max(0,.370-max(0,supported-shelf-delay)*.15)/.15
        sources.append({"piece_id":key,"sector_m":y,"seat_seconds":events["drive-and-turn"],
                        "support_from_feed_s":supported,"empty_retract_and_lift_s":retract+1,
                        "worst_onset_after_shelf_s":delay,"settled_target_xz_m":post["last_xz_vxvz_spin"][:2]})
    return record,sources


def trial(f,record,sources,hold=0,park=True,stock=(0,35),vacate_lane=True,station_feeder=traffic.feeder):
    team,_,_=traffic.programs()
    focal=traffic.Program(2,"red")
    auto=f.routes(-.18)[0]
    focal.initial=focal.pose=auto.end
    team["red2"]=focal
    red1=team["red1"]
    vacate=f.Route(red1.pose)
    vacate.go((485*f.IN,65*f.IN),reverse=True)
    if vacate_lane:
        red1.append(vacate.duration,"clear own endgame lane",vacate)
    focal.append(hold,"wait for field traffic")
    feeds=[]
    tubes=[]
    prior={}
    legs=[]
    entered=[]
    for i,s in enumerate(sources):
        key=s["piece_id"]
        y=s["sector_m"]
        _,loaded,empty=f.routes(y)
        if math.dist(focal.pose[:2],loaded.start[:2])>1e-7 or abs(f.short(focal.pose[2]-loaded.start[2]))>1e-7:
            raise ValueError("Discontinuous station handoff")
        focal.append(2,"extend empty receiver")
        feed=station_feeder(focal.clock,stock[i],feeds[-1]["lip_arrival_s"] if feeds else 0,key,prior)
        if not feed:
            break
        feeds.append(feed)
        prior[key]=True
        focal.append(feed["lip_arrival_s"]-focal.clock,"single file station feed")
        focal.append(s["seat_seconds"],"seat, retract and lower")
        focal.append(loaded.duration,"loaded "+key,loaded)
        legs.append(("loaded-"+key,loaded,entered.copy()))
        arrival=feeds[-1]["lip_arrival_s"]+s["support_from_feed_s"]
        focal.append(arrival-focal.clock,"place, withdraw and observe "+key)
        tubes.append({"id":key,"support_s":focal.clock,"sector_m":y})
        x,z=s["settled_target_xz_m"]
        entered.append((key,rect(f.LIP[0]+x,f.LIP[1]+y,.05725,.1509125)))
        focal.append(s["empty_retract_and_lift_s"],"empty mechanism recovery")
        finish=build_park(f,focal.pose) if i==1 and park else empty
        focal.append(finish.duration,"park at red barge" if i==1 and park else "return to station",finish)
        legs.append(("after-"+key,finish,entered.copy()))
        if i==0 or not park:
            focal.append(2,"raise empty station receiver")
    dynamic=traffic.analyze(hold,stock,mission_record=record,focal_routes=f.routes,prepared=(team,feeds,tubes))
    field=[]
    for name,path,occupied in [("autonomous-approach",auto,[])]+legs:
        extra=MARKS+occupied+(others() if name=="autonomous-approach" else [])
        row=min(f.audit(path,HULL,extra),key=lambda g:g["gap_m"])
        field.append({"leg":name,"minimum_gap_m":row["gap_m"],"closest":row["closest"],"drive_time_s":path.duration})
    for key in ("red1","red3","blue3"):
        for _,_,name,path in team[key].intervals:
            if path:
                row=min(f.audit(path,HULL,MARKS),key=lambda g:g["gap_m"])
                field.append({"leg":key+" "+name,"minimum_gap_m":row["gap_m"],"closest":row["closest"],"drive_time_s":path.duration})
    line=(690.875-144-65.5-88)*f.IN
    # Both start and target orientations have long side-cover strips spanning the
    # line. At park, the front/rear panels run continuously across the center.
    start_center=auto.start[0]
    parked=focal.pose
    zone_edge=(690.875/2+23)*f.IN
    in_red_y=(0<parked[1]-.425 and parked[1]+.425<146.5*f.IN)
    park_overlap=max(0,min(parked[0]+.425,zone_edge)-max(parked[0]-.425,zone_edge-46*f.IN)) if in_red_y else 0
    legal={"start_robot_center_at_red_tape_edge_m":abs(start_center-line),"start_center_tape_edges_in":[line/f.IN-2,line/f.IN],
           "auto_end_bumper_x_gap_from_tape_m":auto.end[0]-.425*math.sqrt(2)-line,
           "auto_arrival_s":auto.duration,"auto_coast_to_deadline_s":15-auto.duration,
           "transition_disabled_s":3,"teleop_start_pose":focal.initial,
           "teleop_end_pose":focal.at(135)[0],"teleop_motion_completion_s":focal.clock,
           "red_barge_park_overlap_m":park_overlap,"reef_zone_bumper_overlap_at_release_m":.425-.49+14*f.IN}
    clear=dynamic["separated_for_full_clock"] and all(s["minimum_gap_m"]>0 for s in field)
    return {"field":field,"rules":legal,"preflight_route_clear":clear,"robot_pairs":15,
            "dynamic_min_gap_m":dynamic["min_separating_bound_m"],"first_pair":dynamic["first_sampled_overlap"],
            "scheduled_station_lip_arrivals_s":[p["lip_arrival_s"] for p in feeds],
            "qualified_supports":dynamic["support_observations"] if clear else [],"on_time":focal.clock<=135,
            "potential_points_after_other_acceptance":3+2*len(dynamic["support_observations"])+2*bool(park_overlap)
                if clear and focal.clock<=135 and auto.duration<=15 else None},team


def main():
    f=corrected()
    rec,sources=basis(f)
    results={}
    for h in (0,16):
        r,_=trial(f,rec,sources,h)
        results[str(h)]=r
    results["partner-holds-start"]=trial(f,rec,sources,vacate_lane=False)[0]
    results["return-to-station"]=trial(f,rec,sources,park=False)[0]
    results["second-inventory-absent"]=trial(f,rec,sources,park=False,stock=(0,None))[0]
    old=heavy.revised_motion()
    old_path=old.routes(-.18)[0]
    old_check=min(old.audit(old_path,HULL,MARKS+others()),key=lambda g:g["gap_m"])
    return {"schema":"reefs-2025-match-onset/1","inches_to_m":FACTOR,"field_floor_stacks_centers_in":COORDS,
            "stack_occupied_half_extent_in":14.7,"prior_alignment_m":1.25,"prior_result":old_check,
            "revised_alignment_m":STANDOFF,"north_waypoint_in":[440,230],
            "archive_rules":["5.2","6.3.3","6.3.4","6.4","6.5.1","6.5.2","6.5.4","G303","G412","G431","G433"],
            "staging":{"total_coral":126,"floor":6,"all_robot_preloads":0,"red_station_total":60,"blue_station_total":60,
                       "algae_on_floor_coral":6,"algae_on_reefs":12,"chosen_receipts":["coral-A","coral-B"]},
            "open_acceptance":["location and ball/robot contact with all 12 reef-staged ALGAE",
                "measured empty/start and as-built inspection","physical legal handoff, segmentation and controller calibration",
                "measured traction, mass distribution, partner routes and referee assessment"],
            "sources":sources,"hold_variants":results}


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output",type=Path,required=True)
    a=p.parse_args()
    a.output.write_text(json.dumps(main(),indent=2,allow_nan=False)+"\n",encoding="utf-8",newline="\n")
