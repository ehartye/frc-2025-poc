"""Serialize the same wheel programs as the study for an offline student replay."""
import json
import math
from rehearse import HERE, ROOT, STOCK, TAPE, ZONE, candidate, corrected, digest, prepare


def auto_frames(route):
    return [[round(t,5),*[round(v,6) for v in route.at(t)[0]]]
            for t in sorted({0,15,route.duration,*[i/10 for i in range(math.ceil(route.duration*10))]})]


def keyframes(program):
    frames = []
    times = {0, 135, *program.ends}
    for index, (start, _, _, route) in enumerate(program.intervals):
        if route:
            end = program.ends[index]
            times.update(start + t for t in route.times)
            times.update(start + i/10 for i in range(math.ceil((end-start)*10)))
    for t in sorted(times):
        pose, _, _ = program.at(t)
        frames.append([round(t,5),*[round(v,6) for v in pose]])
    return frames


def main():
    f = corrected()
    leave,park = candidate(f)
    study = json.loads((HERE/"study.json").read_bytes())
    programs = {}
    cases = {}
    native=json.loads((HERE/"native-park.json").read_bytes())
    motion=json.loads((HERE/"cage-motion.json").read_bytes())
    near=json.loads((HERE/"near-tape.json").read_bytes())
    if motion["route_sha256"] != digest(HERE/"native-park.json") or motion["near_tape_sha256"] != digest(HERE/"near-tape.json"):
        raise ValueError("Rebuild the cage motion study for this route revision")
    def track_refs(team,case):
        refs={}
        for key,program in team.items():
            data=keyframes(program)
            ref=next((r for r,v in programs.items() if v==data),case+"-"+key)
            programs[ref]=data
            refs[key]=ref
        return refs
    for name,row in study["scenarios"].items():
        team=prepare(f,leave,park,row["lane_reserved"],row["decision_teleop_s"],row["attempt"])
        cases[name]=dict(row,robots=track_refs(team,name))
    from straight_role import team_at_teleop
    for name,trial in (("direct","issued"),("shorter","shorter"),("near-tape","near-tape"),("shared","shared-105")):
        row=(near if trial=="near-tape" else native)["cases"][trial]
        direct=f.Route(row["start"])
        direct.go(row["finish"][:2])
        peers=team_at_teleop(f,direct,move_partner=not row["red1_keeps_south"])
        cases[name]={"robots":track_refs(peers,name),"leave_points":3,"park_points":2,"auto":auto_frames(direct),
                     "attempt":True,"decision_teleop_s":0,"first_sampled_overlap":None,
                     "swing":motion["cases"][trial],
                     "tapeReserve":row["one_percent_wheel_case"]["remaining_end_auto_tape_clearance_m"]*1000,
                     "staticGap":row["static_field_lower_bound_mm"],"planningGap":row["planning_field"]["gap_m"]*1000,
                     "partnerGap":min(row["auto_focal_to_each_peer_m"].values())*1000}
    hardware=[]
    for extent in native["local_hardware_native_boxes_mm"].values():
        u,v,w,uu,vv,ww=extent
        left,bottom=f.field_from_native(u/25.4,ww/25.4)
        right,top=f.field_from_native(uu/25.4,w/25.4)
        hardware.append([left,bottom,right,top])
    auto=auto_frames(leave)
    stock=json.loads(STOCK.read_bytes())
    pieces=[]
    for part in stock["stock"].values():
        x,_,z=part["native_center_mm"]
        pieces.append([part["group"],*f.field_from_native(x/25.4,z/25.4)])
    obj={"auto":auto,"hardware":hardware,"cases":cases,"programs":programs,"leave":study["auto_tape_clearance_m"],
         "cages":{name:station["axis_mm"] for name,station in motion["stations"].items()},
         "cageRadius":motion["radius_mm"],"eyeHeight":motion["upper_height_mm"],
         "sharedTiltVolume":motion["tilt_trial"]["overlap_mm3_by_degrees"]["6"],
         "lockedBodies":motion["high_locked_robot"]["installed_bodies"],
         "zone":[v*f.IN for v in ZONE],"tape":[v*f.IN for v in TAPE],
         "field":[f.PROFILE["field"][k]*f.IN for k in ("length","width")],
         "obstacles":[[n,poly] for n,poly in f.obstacles()],"pieces":pieces}
    (HERE/"tracks.js").write_bytes(("const REHEARSAL="+json.dumps(obj,separators=(",",":"),allow_nan=False)+";\n").encode())


if __name__=="__main__":
    main()
