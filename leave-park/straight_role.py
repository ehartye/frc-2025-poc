"""Compare a preassigned straight AUTO corridor with the middle-slot turn program."""
import json
import math
import sys
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"design/drivetrain"))
from route_check import assess
from mission import moving_traffic as traffic
from mission.replay_registered_field import rect
from installed_stock import check as check_stock


def path(f):
    x,_,h=f.routes(-.18)[0].start
    start=(x,95.5625*f.IN,h)
    run=f.Route(start)
    run.go((365*f.IN,start[1]))
    return run


def team_at_teleop(f,route,move_partner=True):
    team,_,_=traffic.programs(stock=(None,None))
    if move_partner:
        team["red1"]=traffic.Program(2,"red")
    team["red2"]=traffic.Program(2,"red")
    team["red2"].initial=team["red2"].pose=route.end
    return team


def wheel_load(paths):
    mass_file=ROOT/"design/competition-package/shop-duplex/mass.json"
    data=json.loads(mass_file.read_bytes())["poses"]["stowed"]["empty"]
    params={"inertiaKgM2":data["yaw_inertia_upper_kg_m2"],"cgOffsetM":data["radial_cg_upper_m"],
            "minSideWeightFraction":data["load_transfer"]["turn-side"]["opposite_edge_equivalent_load_fraction"],
            "outerAxleWeightFraction":1.0}
    rows=[assess(dict(params,massKg=m),{"segments":[seg for p in paths for seg in p.segments]})
          for m in data["mass_kg"]]
    return {"mass_kg":data["mass_kg"],"yaw_inertia_upper_kg_m2":params["inertiaKgM2"],
            "min_side_fraction":params["minSideWeightFraction"],
            "friction_required":max(r["extrema"]["frictionRequired"] for r in rows),
            "peak_drive_phase_A":max(r["extrema"]["phaseA"] for r in rows),
            "peak_branch_A":max(r["extrema"]["branchA"] for r in rows),
            "lowest_bus_V":min(r["extrema"]["busV"] for r in rows),
            "min_voltage_headroom_V":min(r["extrema"]["voltageMarginV"] for r in rows),
            "assumed_friction":rows[0]["parameters"]["longitudinalFriction"],
            "parameter_cases_clear":all(r["parameterCaseClears"] for r in rows)}


def wheel_error_budget(f,direct,hull,tape,zone,shapes,overlap,field_gap,fraction):
    distance=direct.segments[0]["demand"]
    radius=math.hypot(hull[3],hull[4])/1000
    heading=math.radians(.4)
    placement=.005
    # At distance s the differential wheel scale mismatch is bounded by 2 e s / track.
    # Integrating this ramp bounds center displacement throughout the straight.
    wheel_yaw=2*fraction*distance/f.DRIVE["trackM"]
    center=placement+fraction*distance+distance*(1+fraction)*(heading+wheel_yaw/2)
    body=center+2*radius*math.sin((heading+wheel_yaw)/2)
    initial=placement+2*radius*math.sin(heading/2)
    def eroded(region,error):
        x0,y0,x1,y1=region
        return (x0+error/f.IN,y0+error/f.IN,x1-error/f.IN,y1-error/f.IN)
    start_cover=sum(overlap(f,shapes,direct.start,eroded(tape,initial)).values())
    parked_cover=sum(overlap(f,shapes,direct.end,eroded(zone,body)).values())
    tape_gap=(tape[0]*f.IN-direct.end[0]-hull[3]/1000)-body
    return {"wheel_scale_fraction_each":fraction,"initial_position_radius_m":placement,
            "initial_heading_deg":math.degrees(heading),"center_error_bound_m":center,
            "max_heading_deg":math.degrees(heading+wheel_yaw),"footprint_error_bound_m":body,
            "remaining_field_margin_m":field_gap-body,"remaining_end_auto_tape_clearance_m":tape_gap,
            "start_cover_in_eroded_tape_mm3":start_cover,"end_cover_in_eroded_zone_mm3":parked_cover}


def analyze(f,leave,park,shapes,boxes,hull,stock,tape,zone,overlap):
    direct=path(f)
    peers=team_at_teleop(f,direct)
    occupied=[(k,rect(*robot.initial[:2],.425,.425)) for k,robot in peers.items() if k!="red2"]
    field=min(f.audit(direct,hull,occupied),key=lambda r:r["gap_m"])
    roster=traffic.analyze(stock=(None,None),prepared=(peers,[],[]))
    collision=traffic.separation(direct.start,traffic.Program(1,"red").initial)
    stock_gaps=check_stock(f,stock,boxes,direct.segments)
    material=[sum(overlap(f,shapes,pose,region).values()) for pose,region in
              [(direct.start,tape),(direct.end,tape),(direct.end,zone)]]
    load=wheel_load([direct])
    geometry=(direct.duration<=15 and material[0]>0 and material[1]==0 and material[2]>0
              and min(r["lower_bound_mm"] for r in stock_gaps.values())>0 and field["gap_m"]>0
              and roster["separated_for_full_clock"] and all(v==0 for v in direct.at(15)[1]))
    if len(direct.segments)!=1 or direct.segments[0]["kind"]!="drive" or not collision<0:
        raise ValueError("Starting slot or straight-drive assumptions changed")
    return {"start":direct.start,"end":direct.end,"travel_m":direct.segments[0]["demand"],
            "time_s":direct.duration,"hold_from_auto_s":direct.duration,"start_tape_mm3":material[0],
            "end_auto_tape_mm3":material[1],"park_zone_mm3":material[2],"field":field,"stock":stock_gaps,
            "min_teleop_pair_gap_m":roster["min_separating_bound_m"],
            "start_overlap_if_partner_keeps_south_slot_m":-collision,
            "minimum_centerline_clearance_m":direct.end[0]-.425-f.PROFILE["field"]["length"]*.5*f.IN,
            "wheel_error_cases":[wheel_error_budget(f,direct,hull,tape,zone,shapes,overlap,field["gap_m"],e)
                                 for e in (.0025,.005)],
            "drive":load,"middle_slot_drive":wheel_load([leave,park]),
            "potential_points":5 if geometry and load["parameter_cases_clear"] else None}
