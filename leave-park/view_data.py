"""Serialize the same wheel programs as the study for an offline student replay."""
import json
import math
from rehearse import HERE, ROOT, STOCK, TAPE, ZONE, candidate, corrected, prepare


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
    for name,row in study["scenarios"].items():
        team = prepare(f,leave,park,row["lane_reserved"],row["decision_teleop_s"],row["attempt"])
        refs = {}
        for key,program in team.items():
            data = keyframes(program)
            ref = next((r for r,v in programs.items() if v==data),name+"-"+key)
            programs[ref] = data
            refs[key]=ref
        cases[name] = dict(row, robots=refs)
    from straight_role import path,team_at_teleop
    direct=path(f)
    straight=study["preassigned_straight"]
    peers=team_at_teleop(f,direct)
    refs={}
    for key,program in peers.items():
        data=keyframes(program)
        ref=next((r for r,v in programs.items() if v==data),"direct-"+key)
        programs[ref]=data
        refs[key]=ref
    cases["direct"]={"robots":refs,"leave_points":3,"park_points":straight["potential_points"]-3,
                     "attempt":True,"decision_teleop_s":0,"first_sampled_overlap":None}
    auto=auto_frames(leave)
    stock=json.loads(STOCK.read_bytes())
    pieces=[]
    for part in stock["stock"].values():
        x,_,z=part["native_center_mm"]
        pieces.append([part["group"],*f.field_from_native(x/25.4,z/25.4)])
    obj={"auto":auto,"directAuto":auto_frames(direct),"cases":cases,"programs":programs,"leave":study["auto_tape_clearance_m"],
         "zone":[v*f.IN for v in ZONE],"tape":[v*f.IN for v in TAPE],
         "field":[f.PROFILE["field"][k]*f.IN for k in ("length","width")],
         "obstacles":[[n,poly] for n,poly in f.obstacles()],"pieces":pieces}
    (HERE/"tracks.js").write_text("const REHEARSAL="+json.dumps(obj,separators=(",",":"),allow_nan=False)+";\n",encoding="utf8")


if __name__=="__main__":
    main()
