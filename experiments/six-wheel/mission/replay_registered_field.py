"""Run an offline finite six-robot field rehearsal from source station through two qualified sectors."""
import argparse
import csv
import hashlib
import json
import math
import sys
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from mission.field_body_clearance import CAD,departure
from scripts import registered_transit as field, station_l1_mission as mission, station_l1_settle as settle, assess_coral


def rect(cx,cy,half_x,half_y):
    return [(cx-half_x,cy-half_y),(cx+half_x,cy-half_y),(cx+half_x,cy+half_y),(cx-half_x,cy+half_y)]


def others():
    result=[]
    for side in ("blue","red"):
        for i,(x,y) in enumerate(field.PROFILE["rehearsal"]["starts"],1):
            if side=="red" and i==field.REG["startSlot"]["slot"]:
                continue
            cx=x if side=="blue" else field.PROFILE["field"]["length"]-x
            result.append((side+" "+str(i),rect(cx*field.IN,y*field.IN,.425,.425)))
    return result


def provenance():
    paths=("mission/field-registration.json","mission/candidate.json","mission/target.json",
           "kickoff/season-profile.json","scripts/registered_transit.py","scripts/kickoff_route_clearance.py",
           "scripts/station_l1_mission.py","scripts/station_l1_settle.py","scripts/assess_coral.py",
           "mission/field_body_clearance.py","mission/replay_registered_field.py","design/station-l1/review.json")
    return {p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in paths}


def scene(seconds=135, second=.22, blocked=False, hold_end_s=135, delay=.127):
    if not 0<seconds<=135 or not seconds<=hold_end_s<=135:
        raise ValueError("Use a positive active-phase deadline and a finite hold horizon")
    geometry=CAD()
    parked=others()
    if blocked:
        parked[-1]=(parked[-1][0],rect(*field.NORTH,.425,.425))
    occupied=[]
    clock=0.0
    events=[]
    tracks=[]
    observations=[]
    results=[]
    pose=field.routes(-.18)[0].start
    def event(name,**fields):
        events.append({"seconds":round(clock,7),"event":name,**fields})
    def stopped(why):
        phase="AUTO" if seconds<=15 else "TELEOP"
        scores={}
        if observations:
            for edition in assess_coral.EDITIONS:
                scores[edition]=assess_coral.assess({"schema":"reefscape-coral-observations/1","phase":phase,
                    "basis":"Six registered model robots, finite-support witnesses and bounded hulls; referee and other scoring levels unobserved.",
                    "samples":observations},edition)
        return {"schema":"registered-station-l1-field/1","source_sha256":provenance(),"phase":phase,"deadline_s":seconds,
                "elapsed_s":clock,"stop":why,"events":events,"trips":results,"scores":scores,"track":tracks,
                "parked":parked,"transport_step_sha256":geometry.transport_sha,"place_step_sha256":geometry.sha}
    def add_pose(t,p,phase,wheels=(0,0)):
        tracks.append([round(t,5),*[round(v,7) for v in p],phase,*[round(v,6) for v in wheels]])
    def wait(name,duration):
        nonlocal clock
        limit=min(duration,seconds-clock)
        add_pose(clock,pose,name)
        clock+=limit
        add_pose(clock,pose,name)
        event(name,elapsed_s=limit)
        return limit>=duration-1e-9
    def travel(name,route,extra):
        nonlocal clock,pose
        if math.dist(route.start[:2],pose[:2])>1e-6 or abs(field.short(route.start[2]-pose[2]))>1e-6:
            raise ValueError("Route starts at a different chassis pose")
        screen=field.audit(route,geometry.pose_bounds,parked+extra)
        gap=min(screen,key=lambda item:item["gap_m"])
        if gap["gap_m"]<=0:
            event("swept-route-blocked",stage=name,obstacle=gap["closest"])
            return False
        limit=min(route.duration,seconds-clock)
        for i in range(math.ceil(limit/.02)):
            t=i*.02
            p,w=route.at(t)
            add_pose(clock+t,p,name,w)
        pose,w=route.at(limit)
        clock+=limit
        add_pose(clock,pose,name,w)
        event(name,elapsed_s=limit,closest=gap,centerline_m=sum(s["wheel"] for s in route.segments if s["kind"]=="drive"))
        return limit>=route.duration-1e-9
    if not travel("empty-first-approach",field.routes(-.18)[0],[]):
        return stopped("clock-or-occupied-route")
    for n,sector in enumerate((-.18,second)):
        data=field.REG["supply"][n]
        key=data["id"]
        route=field.RegisteredMission(sector)
        if n:
            prior=results[-1]
            x,z=prior["departure"]["settled_target_xz_m"]
            screen=geometry.next_sector(x,prior["sector_m"],z,sector)
            if not screen["clear_all_modeled_poses"]:
                event("prior-pipe-blocks-next-candidate",screen=screen)
                return stopped("next-sector-blocked")
            event("next-sector-swept-clear",gap_mm=screen["min_swept_bound_gap_mm"])
        if not wait("extend-empty-receiver",2) or not wait("wait-for-distinct-feed",max(0,data["readyS"]-clock)):
            return stopped("clock")
        launch=clock
        event("identified-station-feed",piece_id=key,sector_m=sector)
        source=mission.run(target_y=sector,dt=.001,trajectory=route,piece_id=key,sample_every_step=True)
        e={row["event"]:row["time_s"] for row in source["events"]}
        start=e.get("drive-and-turn")
        if start is None or not wait("acquire-retract-lower",start):
            return stopped("custody-or-clock")
        if not travel("loaded-"+key,route.carry,occupied):
            return stopped("clock-or-occupied-route")
        if source["outcome"]!="paired-trough-first-contact":
            return stopped(source["outcome"])
        floor=e[source["outcome"]]
        if not wait("lower-extend-release-first-contact",floor-(clock-launch)):
            return stopped("clock")
        dep=departure(geometry,sector,delay,piece_id=key)
        # Use the exact same release and end state for a persistence check through
        # the horizon. The prior tube remains present during all subsequent trips.
        horizon=hold_end_s-launch-dep["floor_time_s"]
        if horizon<2 or not wait("observe-floor-lip-and-retract",2+dep["floor_time_s"]-floor):
            return stopped("clock")
        hold=settle.continue_contact(sector=sector,upstream=source,piece_id=key,withdrawal="after-separation",
                                     wait_after_true_separation_s=delay,duration=horizon,dt=.0002)
        if hold["stop"]!="sustained-paired-floor-and-lip":
            return stopped(hold["stop"])
        xx,zz=dep["settled_target_xz_m"]
        max_drift=max(math.hypot(x-xx,z-zz) for t,x,z,*_ in hold["trace"] if t>=1.99)
        if max_drift>.0001 or dep["last_possible_contact_s"]>=dep["post_observation_s"] or dep["min_tail_gap_mm"]<=0:
            return stopped("own-clearance-or-persistence-unresolved")
        results.append({"piece_id":key,"sector_m":sector,"feed_s":launch,"sampled_floor_s":launch+floor,
                        "support_confirmation_s":clock,"departure":dep,"hold_to_s":hold_end_s,
                        "hold_max_drift_mm":1000*max_drift,"source_peak":source["peak_supported"]})
        ms=round(clock*1000)
        # All five other modeled robots are stationary well outside this sector.
        # Check the observed finite pipe against their occupied XY geometries.
        px=field.LIP[0]+xx
        py=field.LIP[1]+sector
        obstacle=rect(px,py,mission.R+.0001,mission.M["pipeLength_m"]/2+.0001)
        for _,poly in parked:
            contact,_=field.sweep.sweep((0,0),(0,0),obstacle,poly)
            if contact is not None:
                return stopped("another-robot-at-observed-pipe")
        occupied.append((key,obstacle))
        observations.append({"timeMs":ms,"inventoryComplete":False,"robotCommandActive":True,
              "pieces":[{"id":key,"observedMs":ms,"touchesOwnTrough":True,"touchesAlliedRobot":False,
                         "touchesOpponentRobot":False}]})
        event("modeled-whole-robot-and-sector-observations-qualified",piece_id=key)
        since_clear=clock-launch-dep["first_shelf_witness_s"]
        remaining=max(0,.370-max(0,since_clear-delay)*.15)
        if not wait("finish-slide-retraction",remaining/.15) or not wait("raise-empty-to-travel",1):
            return stopped("clock")
        if not travel("empty-return-"+key,route.return_leg,occupied) or not wait("raise-station-receiver",2):
            return stopped("clock-or-occupied-route")
    return stopped("allocated-supply-exhausted")


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--seconds",type=float,default=135)
    p.add_argument("--next-sector",type=float,default=.22)
    p.add_argument("--blocked",action="store_true")
    p.add_argument("--wheel-csv",type=Path)
    p.add_argument("--output",type=Path)
    a=p.parse_args()
    result=scene(a.seconds,a.next_sector,a.blocked)
    if a.wheel_csv:
        with a.wheel_csv.open("w",newline="",encoding="utf-8") as out:
            writer=csv.writer(out)
            writer.writerow(["clock_s","field_x_m","field_y_m","heading_rad","phase","left_mps","right_mps"])
            writer.writerows(result["track"])
    result.pop("track")
    raw=json.dumps(result,indent=2,allow_nan=False)+"\n"
    if a.output:
        a.output.write_text(raw,encoding="utf-8",newline="\n")
    else:
        print(raw,end="")
