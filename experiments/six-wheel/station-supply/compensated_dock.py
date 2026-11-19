"""Evaluate a compensated station approach using the authored successor assembly."""
import argparse
import importlib.util
import json
import math
import sys
from pathlib import Path

from dock_tolerance import BASE, FIRST_SHA, HERE, RECEIVE_SHA, arrival, bounded_dock, moved, separation
from transfer import G, ST, cad, chord_solids, parts, seat, sha, tube
from OCP.BRepExtrema import BRepExtrema_DistShapeShape


def contact_margin(robot, speed, offset, follows_cartridge):
    """Bound free flight relative to all solid robot bodies except the intended shelf."""
    station_shapes = moved(robot,BASE+offset,0,0)
    shelf = station_shapes.pop("receiver-R6-ported")
    boxes = {k:cad.bounds(v) for k,v in station_shapes.items()}
    state,flight,_ = ST.path_to_shelf(speed)
    dt = flight/8
    start = [v*1000 for v in ST.flight_center(state,0)]
    margins = []
    belts = {"grip-L-420-3M-15","grip-R-420-3M-15"}
    for i in range(1,9):
        end = [v*1000 for v in ST.flight_center(state,i*dt)]
        chord = chord_solids(start,end,1000*G*dt*dt/8)
        closest = {"belt":(math.inf,""),"hardware":(math.inf,"")}
        for tool in chord:
            tool_box = cad.bounds(tool)
            for key,box in boxes.items():
                group = "belt" if key in belts else "hardware"
                # The spring-loaded carriage can move through the specified
                # +/-1.5 mm. Include the entire displacement on its bodies.
                allowance = 1.5 if group=="hardware" and (follows_cartridge(key) or key.endswith("-spring")) else 0
                coarse = separation(tool_box,box)-allowance
                if coarse >= closest[group][0]:
                    continue
                distance = BRepExtrema_DistShapeShape(tool,station_shapes[key])
                if not distance.IsDone():
                    raise ValueError("Unresolved flight clearance: "+key)
                closest[group] = min(closest[group],(distance.Value()-allowance,key))
        margins.append({"through_s":i*dt,"belt_gap_mm":closest["belt"][0],"belt":closest["belt"][1],
                        "hardware_gap_mm":closest["hardware"][0],"hardware":closest["hardware"][1]})
        start = end
    point = [v*1000 for v in ST.flight_center(state,flight)]
    query = BRepExtrema_DistShapeShape(tube(*point),shelf)
    if not query.IsDone():
        raise ValueError("Unresolved shelf touchdown")
    # Rotating the complete swept pipe about the chassis bounds relative pose error.
    # Its original radius includes half length, outside radius and the entire X run.
    trajectory_x = [v[0]*1000 for v in [ST.flight_center(state,0),ST.flight_center(state,flight)]]
    radius = math.hypot(max(abs(BASE+offset-x) for x in trajectory_x)+57.15,150.8125)
    lower = min(r["hardware_gap_mm"] for r in margins)
    # Registering the receiver at the pipe's landing center changes the yaw
    # lever arm. The tube can assume any position along the entire flight.
    x_sweep = abs(trajectory_x[1]-trajectory_x[0])
    mouth_radius = math.hypot(x_sweep+57.15,150.8125)
    budgets = []
    for pivot,lever in (("chassis",radius),("landing-center",mouth_radius)):
        for axial,lateral,yaw in ((.25,.25,.1),(1,.5,.25),(1,1,.25),(2,2,.5),(10,10,3)):
            angle = math.radians(yaw)
            displacement = math.hypot(axial,lateral)+2*lever*math.sin(angle/2)
            max_x = x_sweep if pivot=="landing-center" else max(abs(BASE+offset-x) for x in trajectory_x)
            # Bounding axial-end projection into either nominal belt plane.
            # Lateral body-frame center shift includes the full feed path.
            end_shift = lateral+(max_x+axial)*math.sin(angle)+57.15*math.sin(angle)
            budgets.append({"pivot":pivot,"normal_mm":axial,"lateral_mm":lateral,"yaw_deg":yaw,
                            "possible_relative_motion_mm":displacement,"hardware_bound_mm":lower-displacement,
                            "belt_travel_upper_bound_mm":end_shift,"belt_travel_available_mm":1.5})
    return {"nominal_chords":margins,"shelf_surface_gap_mm":query.Value(),"radius_about_chassis_mm":radius,
            "radius_about_landing_center_mm":mouth_radius,"landing_station_x_mm":point[0],
            "registration_budgets":budgets}


def run(step,directory):
    directory = directory.resolve()
    registry = directory/"red/source-register.json"
    sources = json.loads(registry.read_bytes())
    if sha(directory/"red/receive.step")!=RECEIVE_SHA:
        raise ValueError("Different reference receiver")
    root = directory.parents[2]
    for path,expected in sources.items():
        if sha(root/path)!=expected:
            raise ValueError("Changed assembly generator or input: "+path)
    sys.path.insert(0,str(directory))
    import assembly
    if assembly.source_register()!=sources:
        raise ValueError("Incomplete generator source inventory")
    import motion
    import receiver_drives
    poses = assembly.pose_shapes(940,270,"red")
    nominal_ids = json.loads((directory/"red/part-frames.json").read_bytes())
    if set(poses)!=set(nominal_ids) or len(poses)!=545:
        raise ValueError("Different set of bodies at the new slide stroke")

    # The historical checker exposes three named legs. Load it as a private
    # module so the fourth leg uses precisely the same sweep algorithm.
    travel_path = directory.parent/"travel.py"
    spec = importlib.util.spec_from_file_location("dock_extension_travel",travel_path)
    travel = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(travel)
    travel.paths = lambda: {"station-to-compensated":((940,0),(940,270))}
    travel_result = travel.screen(lambda h,s:assembly.pose_layers(h,s,"red"),motion.supplemental)["station-to-compensated"]

    field_boxes = {k:cad.bounds(v) for k,v in parts(step).items()}
    body_boxes = {k:cad.bounds(v) for k,v in poses.items()}
    transfer = json.loads((HERE/"transfer.json").read_bytes())
    speed = transfer["feed"]["lip_mps"]
    flight = contact_margin(poses,speed,40,receiver_drives.translates_with_grip)
    state = seat(speed,extra_standoff_mm=40,extension_mm=270)
    rows = [arrival(poses,speed,off,side,yaw) for off,side,yaw in
            [(40,0,0),(40,-10,-3),(40,10,3),(40,1,.25),(40,-1,-.25)]]
    return {"schema":"compensated-station-pose/1","field_step_sha256":FIRST_SHA,"reference_step_sha256":RECEIVE_SHA,
            "assembly_source_register_sha256":sha(registry),"transfer_sha256":sha(HERE/"transfer.json"),
            "body_count":len(poses),"standoff_mm":40,"extension_mm":270,"source_lip_mps":speed,
            "high_extension_self_sweep":travel_result,"structural_error_box":bounded_dock(body_boxes,field_boxes,40),
            "free_flight":flight,"centered_bare_shelf_seat":state,"finite_pipe_samples":rows}


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    for key in ("field","assembly","output"):
        p.add_argument("--"+key,type=Path,required=True)
    args = p.parse_args()
    args.output.write_text(json.dumps(run(args.field,args.assembly),indent=2,allow_nan=False)+"\n",encoding="utf8",newline="\n")
