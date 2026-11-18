"""Check station docking error and incoming pipe contact on the registered red receiver."""
import argparse
import json
import math
from pathlib import Path

from native_station import FIRST_SHA, ROOT, cad, parts
from transfer import G, ST, corridor, hits, seat, sha
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
from OCP.gp import gp_Trsf

HERE = Path(__file__).resolve().parent
RECEIVE_SHA = "6389d766ad3d4d65b885bccb615a23807cadd8ef4f14d846214cea8a4dd96f73"
POSITION_ERROR = 10.0
ANGLE_ERROR = 3.0
BASE = 434.525  # Station-frame chassis center when its aligned cover reaches the lower wall.


def moved(robot, center, side, yaw_deg):
    angle = math.pi + math.radians(yaw_deg)
    c,s = math.cos(angle),math.sin(angle)
    change = gp_Trsf()
    change.SetValues(c,-s,0,center,s,c,0,side,0,0,1,0)
    return {key:BRepBuilderAPI_Transform(body,change,False).Shape() for key,body in robot.items()}


def separation(a,b):
    return math.hypot(*(max(0, a[i]-b[i+3], b[i]-a[i+3]) for i in range(3)))


def sweep_box(box,center,angle_error,normal_error,lateral_error):
    # Each robot-frame corner sweeps a circular arc. Include coordinate extrema
    # at the cardinal angles inside this arc, then translate by the full error box.
    points = []
    for x in (box[0],box[3]):
        for y in (box[1],box[4]):
            angle = math.atan2(y,x) + math.pi
            lo,hi = angle-math.radians(angle_error),angle+math.radians(angle_error)
            turn = math.pi/2
            angles = [lo,hi]+[i*turn for i in range(math.ceil(lo/turn), math.floor(hi/turn)+1)]
            radius = math.hypot(x,y)
            points.extend((radius*math.cos(h),radius*math.sin(h)) for h in angles)
    xs,ys = zip(*points)
    return (center+min(xs)-normal_error,min(ys)-lateral_error,box[2],
            center+max(xs)+normal_error,max(ys)+lateral_error,box[5])


def bounded_dock(boxes,field,offset):
    distances = ((separation(sweep,sbox),key,sk)
                 for key,box in boxes.items()
                 for sweep in [sweep_box(box,BASE+offset,ANGLE_ERROR,POSITION_ERROR,POSITION_ERROR)]
                 for sk,sbox in field.items())
    gap,key,sk = min(distances)
    return {"gap_lower_bound_mm":gap,"body":key,"station_occurrence":sk}


def native_dock(robot,field,field_boxes,offset,yaw,side=0):
    current = moved(robot,BASE+offset,side,yaw)
    boxes = {key:cad.bounds(s) for key,s in current.items()}
    pairs = sorted((separation(box,fb),key,sk) for key,box in boxes.items() for sk,fb in field_boxes.items())
    best = (math.inf,"","")
    overlaps = {}
    for lower,key,sk in pairs:
        if lower > best[0]+1e-5:
            break
        query = BRepExtrema_DistShapeShape(current[key],field[sk])
        if not query.IsDone():
            raise ValueError("Native distance unresolved: "+key+" / "+sk)
        best = min(best,(query.Value(),key,sk))
        if lower<1.e-5:
            volume = cad.intersection(current[key],field[sk])
            if volume>.05:
                overlaps[key+" / "+sk] = volume
    return {"station_standoff_mm":offset,"yaw_deg":yaw,"side_mm":side,"minimum_native_mm":best[0],
            "closest_body":best[1],"closest_station_occurrence":best[2],"intersections_mm3":overlaps}


def arrival(robot, speed, offset, side, yaw):
    state,flight,_ = ST.path_to_shelf(speed)
    current = moved(robot,BASE+offset,side,yaw)
    boxes = {k:cad.bounds(s) for k,s in current.items()}
    first_positive = None
    chord_candidates = {}
    prev = [v*1000 for v in ST.flight_center(state,0)]
    for n in range(1,9):
        dt = flight/8
        point = [v*1000 for v in ST.flight_center(state,n*dt)]
        enclosed = corridor(current,boxes,prev,point,1000*G*dt*dt/8)
        for key in enclosed:
            chord_candidates.setdefault(key,n)
        prev = point
    # Near the shelf a raised pad or jaw can be the first surface. A material
    # intersection is a counterexample to uninterrupted flight; the chord only
    # localizes possible contacts and intentionally overbounds the hollow pipe.
    for i in range(1,101):
        t = flight*i/100
        x,z = (v*1000 for v in ST.flight_center(state,t))
        material = hits(current,boxes,x,z)
        if material:
            first_positive = {"flight_fraction":i/100,"seconds_from_lip":t,"overlaps_mm3":material}
            break
    return {"station_standoff_mm":offset,"side_mm":side,"yaw_deg":yaw,"bare_shelf_height_time_s":flight,
            "chord_count":8,"earliest_candidate_chord_by_body":chord_candidates,
            "first_positive_material_sample":first_positive}


def run(field_step,poses):
    path = poses/"receive.step"
    source = json.loads((HERE/"transfer.json").read_bytes())
    if sha(path)!=RECEIVE_SHA or source["field_step_sha256"]!=FIRST_SHA:
        raise ValueError("Unexpected receiving assembly or transfer source")
    robot = cad.read_named(path,[{"id":k} for k in json.loads((poses/"part-frames.json").read_bytes())])
    if len(robot)!=545:
        raise ValueError("Missing successor body")
    field = parts(field_step)
    field_boxes = {k:cad.bounds(s) for k,s in field.items()}
    boxes = {k:cad.bounds(s) for k,s in robot.items()}
    speed = source["feed"]["lip_mps"]
    physical = [native_dock(robot,field,field_boxes,offset,yaw) for offset,yaw in ((10,0),(5,1),(10,2),(30,3))]
    bounds = {str(offset):bounded_dock(boxes,field_boxes,offset) for offset in (10,25,30,35,40,50)}
    dry = []
    for offset in (10,25,30,35,40,45,50,60,80,100):
        result = seat(speed,extra_standoff_mm=offset)
        dry.append({"station_standoff_mm":offset,"surface":result["impact"]["surface"],"result":result["result"],
                    "local_seat_m":result.get("last_local_x_m"),"local_velocity_mps":result.get("last_relative_speed_mps")})
    poses_to_check = [(10,0,0),(30,0,0),(35,0,0),(40,0,0),(50,0,0),
                      (30,-10,-3),(30,10,3),(50,-10,-3),(50,10,3)]
    arrivals = [arrival(robot,speed,offset,side,yaw) for offset,side,yaw in poses_to_check]
    return {"schema":"station-docking-tolerance/1","field_step_sha256":FIRST_SHA,"receive_step_sha256":RECEIVE_SHA,
            "source_transfer_sha256":sha(HERE/"transfer.json"),"body_count":len(robot),"station_occurrences":len(field),
            "normal_and_lateral_error_mm":POSITION_ERROR,"yaw_error_deg":ANGLE_ERROR,"receiver_extension_mm":240,
            "conditional_nonrebounding_lip_speed_mps":speed,"native_dock":physical,"continuous_dock_bounds":bounds,
            "centered_bare_shelf_seating":dry,"finite_pipe_flight":arrivals}


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    for arg in ("field","poses","output"):
        p.add_argument("--"+arg,type=Path,required=True)
    a = p.parse_args()
    a.output.write_text(json.dumps(run(a.field,a.poses),indent=2,allow_nan=False)+"\n",encoding="utf8",newline="\n")
