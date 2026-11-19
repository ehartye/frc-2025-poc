"""Test a station-independent match using the stowed robot and the occupied field."""
import hashlib
import json
import math
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT / "full-match"))
from match import corrected, others, traffic
from installed_stock import check as check_stock, GROUPS, FIRST_SHA, stationary
sys.path.insert(0, str(ROOT / "design/competition-package/shop-duplex"))
from assembly import cad
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.gp import gp_Pnt

KIT = ROOT / "design/competition-package/shop-duplex/red"
STOCK = ROOT / "full-match/installed-stock.json"
TAPE = (690.875 - 144 - 65.5 - 88 - 2, 690.875 - 144 - 65.5 - 88)
ZONE = (690.875/2 - 23, 0, 690.875/2 + 23, 146.5)
TELEOP = 135
DECISION = 118


def digest(file):
    return hashlib.sha256(file.read_bytes()).hexdigest()


def candidate(f):
    start = f.routes(-.18)[0].start
    leave = f.Route(start)
    leave.go((425*f.IN, start[1]), reverse=True)
    park = f.Route(leave.end)
    park.go((leave.end[0], 95.5625*f.IN))
    park.go((355*f.IN, 95.5625*f.IN))
    return leave, park


def overlap(f, shapes, pose, region):
    # Both assessment poses have the chassis aligned with the field axes.
    if abs(math.sin(pose[2])) > 1e-8:
        raise ValueError("Expected axis-aligned assessment pose")
    x0,y0,x1,y1 = region
    corners = [f.rot((x*f.IN-pose[0], y*f.IN-pose[1]), -pose[2])
               for x in (x0,x1) for y in (y0,y1)]
    x,y = zip(*corners)
    prism = BRepPrimAPI_MakeBox(gp_Pnt(min(x)*1000,min(y)*1000,-10),
                                  gp_Pnt(max(x)*1000,max(y)*1000,1200)).Shape()
    return {key:cad.volume(BRepAlgoAPI_Common(shape,prism).Shape())[0]
            for key,shape in shapes.items() if key.startswith("bumper-cover-")}


def prepare(f, leave, park, clear, depart, attempt):
    team,_,_ = traffic.programs(stock=(None,None))
    focal = traffic.Program(2,"red")
    focal.pose = focal.initial = leave.end
    team["red2"] = focal
    if clear:
        partner = team["red1"]
        vacate = f.Route(partner.pose)
        vacate.go((485*f.IN,65*f.IN),reverse=True)
        partner.append(vacate.duration,"clear south lane",vacate)
    focal.append(depart,"wait on own side")
    if attempt:
        focal.append(park.duration,"park",park)
    return team


def scenario(f, leave, park, clear, depart, force=False):
    attempt = force or (clear and depart + park.duration <= TELEOP)
    team = prepare(f,leave,park,clear,depart,attempt)
    screen = traffic.analyze(stock=(None,None),prepared=(team,[],[]))
    return team, {
        "lane_reserved":clear, "decision_teleop_s":depart, "attempt":attempt,
        "motion_finish_s":depart+park.duration if attempt else leave.duration,
        "motion_finish_phase":"TELEOP" if attempt else "AUTO",
        "min_robot_pair_bound_m":screen["min_separating_bound_m"], "limiting_pair":screen["worst"],
        "first_sampled_overlap":screen["first_sampled_overlap"],
        "traffic_clear":screen["separated_for_full_clock"],
        "at_buzzer":team["red2"].at(TELEOP)[0], "zero_wheel_speed_at_buzzer":team["red2"].at(TELEOP)[1]==(0,0),
    }


def build():
    f = corrected()
    leave,park = candidate(f)
    step = KIT/"stowed.step"
    ids = json.loads((KIT/"part-frames.json").read_bytes())
    pinned = json.loads((KIT/"review.json").read_bytes())["stowed"]
    if digest(step)!=pinned["export_sha256"] or pinned["bodies"]!=len(ids):
        raise ValueError("Stowed assembly differs from its issued review")
    shapes = cad.read_named(step,[{"id":n} for n in ids])
    if set(ids)!=set(shapes):
        raise ValueError("Incomplete reopened assembly")
    boxes = {k:cad.bounds(s) for k,s in shapes.items()}
    hull = [min(b[i] for b in boxes.values()) for i in range(3)]+[max(b[i] for b in boxes.values()) for i in range(3,6)]
    if hull[:2]!=[-425,-425] or hull[3:5]!=[425,425]:
        raise ValueError("Traffic footprint differs from issued assembly")
    overhead = next(o for o in f.PROFILE["rehearsal"]["obstacles"] if o["kind"]=="overlay")
    if hull[5] >= overhead["lower"]*f.IN*1000:
        raise ValueError("Stowed robot reaches modeled overhead exclusion")
    inventory = json.loads(STOCK.read_bytes())
    stock = inventory["stock"]
    if inventory["field_step_sha256"] != FIRST_SHA or {g:sum(r["group"]==g for r in stock.values()) for g in GROUPS}!=GROUPS:
        raise ValueError("Unregistered stock")
    tape = (TAPE[0],0,TAPE[1],f.PROFILE["field"]["width"])
    start = overlap(f,shapes,leave.start,tape)
    auto = overlap(f,shapes,leave.end,tape)
    parked = overlap(f,shapes,park.end,ZONE)
    held = overlap(f,shapes,leave.end,ZONE)
    assess = {"start_tape_mm3":start,"end_auto_tape_mm3":auto,"park_zone_mm3":parked,"hold_zone_mm3":held}
    legs = {}
    for name,path in (("leave",leave),("park",park)):
        field = min(f.audit(path,hull,others() if name=="leave" else []),key=lambda row:row["gap_m"])
        legs[name]={"time_s":path.duration,"start":path.start,"end":path.end,"field":field,
                    "stock":check_stock(f,stock,boxes,path.segments)}
    def clear_leg(key):
        leg=legs[key]
        return leg["field"]["gap_m"]>0 and all(x["lower_bound_mm"]>0 for x in leg["stock"].values())
    leave_ok = sum(start.values())>0 and sum(auto.values())==0 and leave.duration<=15 and clear_leg("leave")
    other_legs = {}
    for key, program in prepare(f,leave,park,True,DECISION,True).items():
        if key == "red2":
            continue
        routes = [route for _,_,_,route in program.intervals if route]
        segments = [segment for route in routes for segment in route.segments] or stationary(program.initial)
        stock_gaps = check_stock(f,stock,{"other robot envelope":hull},segments)
        field_rows = [row for route in routes for row in f.audit(route,hull)]
        if not field_rows:
            fixed = f.Route(program.initial)
            fixed.segments = [dict(segments[0],end=program.initial,dt=0)]
            field_rows = f.audit(fixed,hull)
        other_legs[key] = {"field":min(field_rows,key=lambda row:row["gap_m"]), "stock":stock_gaps}
    if any(item["field"]["gap_m"] <= 0 or min(r["lower_bound_mm"] for r in item["stock"].values())<=0
           for item in other_legs.values()):
        raise ValueError("Another robot has an obstructed route")
    outcomes={}
    programs={}
    for key,clear,depart,force in (("reserved",True,DECISION,False),("occupied",False,DECISION,False),
                                  ("late",True,129,False),("ignore-reservation",False,DECISION,True)):
        team,row=scenario(f,leave,park,clear,depart,force)
        park_ok = (row["attempt"] and clear_leg("park") and row["traffic_clear"] and
                   row["motion_finish_s"]<=TELEOP and row["zero_wheel_speed_at_buzzer"] and sum(parked.values())>0)
        row["leave_points"]=3 if leave_ok else 0
        row["park_points"]=2 if park_ok else 0
        row["route_usable"]=leave_ok and row["traffic_clear"] and (not row["attempt"] or park_ok)
        outcomes[key]=row
        programs[key]=team
    clearance = legs["park"]["field"]["gap_m"]
    radius = math.hypot(hull[3],hull[4])/1000
    tolerance = {}
    for mm in (10,20):
        error = mm/1000 + 2*radius*math.sin(math.radians(1)/2)
        eroded = (ZONE[0]+error/f.IN,ZONE[1]+error/f.IN,ZONE[2]-error/f.IN,ZONE[3]-error/f.IN)
        tolerance[str(mm)] = {"translation_mm":mm,"heading_deg":1,"footprint_error_m":error,
                              "field_margin_m":clearance-error,
                              "two_robot_margin_m":outcomes["reserved"]["min_robot_pair_bound_m"]-2*error,
                              "park_cover_in_eroded_zone_mm3":sum(overlap(f,shapes,park.end,eroded).values())}
    displaced = f.Route(leave.end)
    displaced.go((leave.end[0],park.end[1]+.030))
    displaced.go((park.end[0],park.end[1]+.030))
    missed = min(f.audit(displaced,hull),key=lambda r:r["gap_m"])
    from straight_role import analyze as straight_role
    straight = straight_role(f,leave,park,shapes,boxes,hull,stock,tape,ZONE,overlap)
    return ({"preassigned_straight":straight,"stowed_step_sha256":digest(step),"body_count":len(boxes),"hull_mm":hull,
             "field_step_sha256":FIRST_SHA,"stock_receipt_sha256":digest(STOCK),
             "other_robots":other_legs,"stock_counts":GROUPS,"tracking_cases":tolerance,"shift_30mm_toward_cage":missed,"assessment":assess,"legs":legs,
             "auto_tape_clearance_m":leave.end[0]-hull[3]/1000-TAPE[1]*f.IN,
             "latest_park_start_teleop_s":TELEOP-park.duration,"scenarios":outcomes},programs)


if __name__=="__main__":
    result,_=build()
    (HERE/"study.json").write_text(json.dumps(result,indent=2,allow_nan=False)+"\n",encoding="utf8")
