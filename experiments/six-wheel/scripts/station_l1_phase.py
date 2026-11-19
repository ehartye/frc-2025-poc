"""Run a finite-phase rehearsal with explicit custody evidence and separately timed empty legs."""
import argparse
import json
import math

if __package__:
    from . import assess_coral, station_l1_mission as mission
else:
    import assess_coral
    import station_l1_mission as mission


def rehearse(seconds=135, witnessed_clear=False, phase="TELEOP", supplies=None, targets=(-.18,.18),
             trough_continuation=False):
    if not math.isfinite(seconds) or seconds <= 0 or phase not in {"AUTO","TELEOP"} or seconds > {"AUTO":15,"TELEOP":135}[phase]:
        raise ValueError("Clock exceeds the declared active period")
    stock = [("coral-A",0),("coral-B",35)] if supplies is None else supplies
    ids = [key for key,_ in stock]
    if len(set(ids))!=len(ids) or any(not isinstance(key,str) or not key.strip() or not math.isfinite(ready)
                                     or ready<0 for key,ready in stock):
        raise ValueError("Supply requires distinct IDs and nonnegative ready times")
    if not all(math.isfinite(target) for target in targets):
        raise ValueError("Target positions must be finite")
    now = 0.0
    # Candidate empty chassis begins clear of the station, facing the feed.
    pose = [1.20,0.0,180.0,940.0,0.0] # x/y (m), heading (degrees), lift/slide (mm)
    dock = mission.M["chassisStationX_m"]
    events = []
    pieces = {}
    observed = []
    def event(name, **data):
        events.append({"time_s":round(now,8),"event":name,**data})
    def move(label,duration,end, *, linear=False):
        nonlocal now,pose
        if duration <= 0:
            return True
        elapsed = min(duration,seconds-now)
        ratio = elapsed/duration if linear else mission.smooth(elapsed/duration)
        start = list(pose)
        pose = [a+(b-a)*ratio for a,b in zip(start,end)]
        now += elapsed
        event(label if elapsed>=duration else "clock-stopped-"+label,
              pose_from=start,pose_to=list(pose),elapsed_s=round(elapsed,8))
        return elapsed>=duration
    def report(reason):
        return {"schema":"station-l1-phase/1","phase":phase,"deadline_s":seconds,
                "trough_continuation":trough_continuation,
                "synthetic_empty_and_sector_clear_observations":witnessed_clear,
                "elapsed_s":round(now,8),"stop":reason,"pieces":pieces,"events":events,
                "assessments":{edition:assess_coral.assess(
                    {"schema":"reefscape-coral-observations/1","phase":phase,
                     "basis":"Model first contact; remaining robot and level observations stay unknown.",
                     "samples":observed},edition) for edition in assess_coral.EDITIONS} if observed else {}}
    if not move("empty-first-station-approach",2,[dock,0,180,940,0]):
        return report("clock")
    for index,(piece,ready) in enumerate(stock):
        if index>=len(targets):
            return report("no-fresh-sector-confirmation")
        sector = targets[index]
        if any(abs(sec["sector_y_m"]-sector)<mission.M["pipeLength_m"] for sec in pieces.values()):
            return report("target-overlaps-previous-piece-envelope")
        if not move("extend-empty-receiver",2,[dock,0,180,940,240]):
            return report("clock")
        if not move("wait-for-assigned-feed",max(0,ready-now),pose):
            return report("clock")
        launch = now
        result = mission.run(target_y=sector,piece_id=piece)
        pieces[piece] = {"sector_y_m":sector,"release_time_s":round(now,8),"state":"in-progress"}
        event("distinct-pipe-at-station-lip",piece_id=piece)
        # This interval starts with the empty receiver already extended and ends at
        # the physical first-contact boundary. Later phases cannot bypass an earlier clock.
        for obs in result["events"]:
            t = launch+obs["time_s"]
            if t>seconds:
                now=seconds
                event("clock-stopped-active-mission",piece_id=piece)
                return report("clock")
            now=t
            event(obs["event"],piece_id=piece,xyz_m=obs["xyz_m"])
        pieces[piece]["state"] = result["outcome"]
        pieces[piece]["first_contact_time_s"] = round(now,8)
        if result["outcome"]!="paired-trough-first-contact":
            return report("unresolved-mission-contact")
        pose = [mission.M["chassisAtTargetX_m"],sector,0,560,370]
        ms = round(now*1000)
        observed.append({"timeMs":ms,"inventoryComplete":False,"robotCommandActive":True,
                         "pieces":[{"id":piece,"observedMs":ms,"touchesOwnTrough":True}]})
        if trough_continuation:
            if __package__:
                from .station_l1_settle import continue_contact
            else:
                from station_l1_settle import continue_contact
            follow=continue_contact(sector=sector,piece_id=piece,withdrawal="after-separation",upstream=result)
            # The solver reconstructs exact floor tangency from the same upstream
            # release. Use that clock internally and map duration to this timestamp.
            def stroke_at(t):
                return max(0,370-150*max(0,t-follow["clear_witness_from_station_s"]-.05))
            appointment=launch+follow["start_from_station_s"]+follow["elapsed_s"]
            elapsed=min(seconds-now,appointment-now)
            pose[-1]=stroke_at(now+elapsed-launch)
            now+=elapsed
            event("observe-identified-pipe-and-withdraw",piece_id=piece,slide_mm=pose[-1])
            if now<appointment-1e-8:
                return report("clock")
            pieces[piece]["post_contact"]=follow["stop"]
            pieces[piece]["supported_observation_time_s"]=round(now,8)
            if follow["stop"]!="sustained-paired-floor-and-lip":
                return report("unresolved-post-contact")
            event("dual-target-contact-and-shelf-gap-witnessed",piece_id=piece)
            ms=round(now*1000)
            observed.append({"timeMs":ms,"inventoryComplete":False,"robotCommandActive":True,
                             "pieces":[{"id":piece,"observedMs":ms,"touchesOwnTrough":True}]})
        if not witnessed_clear:
            return report("await-empty-robot-and-separate-target-observations")
        if not trough_continuation and not move("wait-for-clear-witnesses",1.2,pose):
            return report("clock")
        event("synthetic-clear-to-withdraw",piece_id=piece)
        remaining_withdraw_s=pose[-1]/150 if trough_continuation else 3
        for name,duration,end in (
            ("withdraw-empty-slide",remaining_withdraw_s,[pose[0],sector,0,560,0]),
            ("raise-to-empty-transit",1,[pose[0],sector,0,670,0]),
            ("return-empty-to-station",5,[dock,0,180,670,0]),
            ("raise-station-receiver",2,[dock,0,180,940,0])):
            if not move(name,duration,end,linear=trough_continuation and name=="withdraw-empty-slide"):
                return report("clock")
    return report("allocated-supply-exhausted")


if __name__=="__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--phase",choices=["AUTO","TELEOP"],default="TELEOP")
    p.add_argument("--seconds",type=float)
    p.add_argument("--assume-clear-witnesses",action="store_true")
    p.add_argument("--trough-continuation",action="store_true")
    a=p.parse_args()
    seconds=a.seconds if a.seconds is not None else {"AUTO":15,"TELEOP":135}[a.phase]
    print(json.dumps(rehearse(seconds,a.assume_clear_witnesses,a.phase,trough_continuation=a.trough_continuation),
                     indent=2,allow_nan=False))
