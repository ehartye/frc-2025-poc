"""Bound incoming pipe registration in the plane of the station and compare cheek clearance."""
import argparse
import json
import math
from pathlib import Path
import sys

from entry_envelope import projected_section
from dock_tolerance import BASE, HERE, RECEIVE_SHA, moved, separation
from transfer import G, H, R, ST, cad, sha, tube
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace, BRepBuilderAPI_MakePolygon
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
from OCP.BRepPrimAPI import BRepPrimAPI_MakePrism
from OCP.gp import gp_Pnt, gp_Vec

ROOT = HERE.parent
ASSEMBLY = ROOT / "design/competition-package/shop-duplex"
sys.path.insert(0, str(ASSEMBLY))
import assembly
import receiver_drives


def flight_hull(p,q,landing_x,normal,lateral,yaw,travel,sag):
    profile, half = projected_section(p,q,landing_x,normal,lateral,yaw,travel,sag,R,H)
    contour = BRepBuilderAPI_MakePolygon()
    for x,z in profile:
        contour.Add(gp_Pnt(x,-half,z))
    contour.Close()
    return BRepPrimAPI_MakePrism(BRepBuilderAPI_MakeFace(contour.Wire()).Face(),gp_Vec(0,2*half,0)).Shape()


def clearance(shapes, speed, errors, inward_limit):
    normal,lateral,yaw = errors
    boxes = {k:cad.bounds(s) for k,s in shapes.items()}
    state, duration, _ = ST.path_to_shelf(speed)
    dt = duration/8
    centers = [tuple(1000*v for v in ST.flight_center(state,i*dt)) for i in range(9)]
    belts = {"grip-L-420-3M-15","grip-R-420-3M-15"}
    closest = (float("inf"), "")
    approach = {}
    for i, (p,q) in enumerate(zip(centers,centers[1:]),1):
        envelopes = {}
        for travel in (0, inward_limit):
            shape = flight_hull(p,q,centers[-1][0],normal,lateral,yaw,travel,1000*G*dt**2/8)
            envelopes[travel] = shape,cad.bounds(shape)
        for key, body in shapes.items():
            if key == "receiver-R6-ported" or key in belts:
                continue
            movable = receiver_drives.translates_with_grip(key) or key.endswith("-spring")
            if movable and boxes[key][1] * boxes[key][4] <= 0:
                raise ValueError("Moving cartridge body straddles pipe centerline: "+key)
            shape, box = envelopes[inward_limit if movable else 0]
            lower = separation(box,boxes[key])
            if lower > min(2.5,closest[0]) and lower > .00001:
                continue
            distance = BRepExtrema_DistShapeShape(shape,body)
            if not distance.IsDone():
                raise ValueError("Unresolved envelope distance for "+key)
            gap = distance.Value()
            closest = min(closest,(gap,key))
            if gap < 1e-6 and key not in approach:
                amount = cad.intersection(shape,body)
                if amount > .001:
                    approach[key] = {"chord":i,"envelope_overlap_mm3":round(amount,4)}
    angle = math.radians(yaw)
    belt_motion = lateral+(normal+abs(centers[-1][0]-centers[0][0])+R)*math.sin(angle)
    return {"normal_mm":normal,"lateral_mm":lateral,"yaw_deg":yaw,
            "hardware_lower_mm":min(2.5, closest[0]),"closest_body":closest[1],
            "unresolved_material":approach,"max_belt_excursion_mm":belt_motion}


def registered_robot():
    source = ASSEMBLY/"red/source-register.json"
    if sha(ASSEMBLY/"red/receive.step") != RECEIVE_SHA or assembly.source_register() != json.loads(source.read_bytes()):
        raise ValueError("Changed registered robot")
    robot = assembly.pose_shapes(940,270,"red")
    if set(robot) != set(json.loads((ASSEMBLY/"red/part-frames.json").read_bytes())):
        raise ValueError("Changed receiving pose identities")
    return robot


def offset_witness(robot, speed, inward, side_error=.5):
    """Native intersections on one assumed ballistic path with both unloaded grips inward."""
    station = moved(robot,BASE+40,side_error,0)
    bodies = {}
    for key,shape in station.items():
        if receiver_drives.translates_with_grip(key) and not key.endswith("420-3M-15"):
            direction = 1 if key.startswith("grip-L-") else -1
            bodies[key] = cad.translated(shape,(0,direction*inward,0))
    boxes = {key:cad.bounds(body) for key,body in bodies.items()}
    state,duration,_ = ST.path_to_shelf(speed)
    for index in range(1,101):
        time = duration*index/100
        pipe = tube(*(1000*v for v in ST.flight_center(state,time)))
        box = cad.bounds(pipe)
        hits = {}
        for key,body in bodies.items():
            if separation(box,boxes[key]) <= 1e-6:
                overlap = cad.intersection(pipe,body)
                if overlap > .001:
                    hits[key] = round(overlap,5)
        if hits:
            return {"fraction":index/100,"seconds":time,"material_mm3":hits}
    return None


def stop_parts(robot, step):
    # Move the issued 2 mm stop outward toward its lug. Reuse its full section.
    from duplex import finalize_step
    stops = {}
    for side in "LR":
        key = "grip-"+side+"-inboard-hardstop"
        x,y,z,xx,yy,zz = cad.bounds(robot[key])
        if max(abs(expected-actual) for expected,actual in zip((4,2,16),(xx-x,yy-y,zz-z))) > 1e-5:
            raise ValueError("Changed issued stop size")
        stops[key] = cad.translated(robot[key],(0,1 if side=="L" else -1,0))
    labels = [{"id":key,"group":"stop"} for key in stops]
    cad.write_named(step,list(stops.values()),labels,{"stop":"#a66739"})
    finalize_step(step)
    reopened = cad.read_named(step,labels)
    report = {}
    for side in "LR":
        key = "grip-"+side+"-inboard-hardstop"
        candidate = reopened[key]
        sign = 1 if side=="L" else -1
        mate = "grip-"+side+"-moving-stop-lug"
        original_volume = cad.volume(robot[key])[0]
        restored = cad.translated(candidate,(0,-sign,0))
        if abs(cad.intersection(restored,robot[key])-original_volume) > .001:
            raise ValueError("Reopened relocated stop does not match the issued body")
        x,y,z,xx,yy,zz = cad.bounds(candidate)
        datum = receiver_drives.block(x,y,z-.1,xx-x,yy-y,.1)
        foot = robot["grip-"+side+"-front-support-foot"]
        footprint = cad.intersection(datum,foot)/.1
        witnesses = {}
        for offset in (-.75,-.5,0,1.5):
            lug = cad.translated(robot[mate],(0,sign*offset,0))
            q = BRepExtrema_DistShapeShape(lug,candidate)
            if not q.IsDone():
                raise ValueError("Unresolved stop face")
            witnesses[str(offset)] = {"distance_mm":q.Value(),"overlap_mm3":cad.intersection(lug,candidate)}
        other = {}
        sbox = cad.bounds(candidate)
        for name,body in robot.items():
            if name in (key,mate):
                continue
            b = list(cad.bounds(body))
            if receiver_drives.translates_with_grip(name) or name.endswith("-spring"):
                b[1] -= 1.5
                b[4] += 1.5
            if separation(b,sbox) > 1e-5:
                continue
            # Retain any swept box that could enter the stop interior. Fixed
            # mounting surfaces can share a face without sharing material.
            if all(min(sbox[i+3],b[i+3])-max(sbox[i],b[i]) > 1e-5 for i in range(3)):
                other[name] = b
        report[side] = {"added_mm3":cad.volume(candidate)[0]-original_volume,
                        "supported_footprint_mm2":footprint,
                        "lug_positions":witnesses,"unresolved_other_body_boxes":other}
    return reopened,report


def study(step):
    robot = registered_robot()
    speed = json.loads((HERE/"transfer.json").read_bytes())["feed"]["lip_mps"]
    cases = [(5,.35,.15),(5,.5,.25),(5,1,.25),(5,2,.5)]
    placed = moved(robot,BASE+40,0,0)
    issued = clearance(placed,speed,cases[1],1.5)
    original_witness = offset_witness(robot,speed,1.5)
    stops,check = stop_parts(robot,step)
    robot.update(stops)
    revised = moved(robot,BASE+40,0,0)
    return {"source_step":RECEIVE_SHA,"source_register":sha(ASSEMBLY/"red/source-register.json"),
            "source_transfer":sha(HERE/"transfer.json"),"compensated_pose":sha(HERE/"compensated-dock.json"),
            "reopened_stop_step":sha(step),"body_count":len(robot),"stop_checks":check,
            "inward_travel_nominal_mm":.5,"stop_position_allowance_mm":.1,
            "outward_travel_nominal_mm":1.5,"issued":issued,
            "at_half_mm_side_zero_yaw":{"issued_inward_1_5":original_witness,
                                        "revised_inward_0_6":offset_witness(robot,speed,.6)},
            "revised":[clearance(revised,speed,errors,.6) for errors in cases]}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--step",type=Path,required=True,help="Destination for the two revised stops at the receiving pose")
    args = parser.parse_args()
    args.output.write_text(json.dumps(study(args.step),indent=2,allow_nan=False)+"\n",encoding="utf8",newline="\n")
