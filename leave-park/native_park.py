"""Measure the stowed straight-drive corridor in the team's registered official STEP."""
import argparse
import json
import math
from pathlib import Path

from rehearse import ROOT, HERE, KIT, corrected, cad, digest, overlap, ZONE, TAPE, traffic
from straight_role import team_at_teleop, wheel_error_budget
from mission.check_branch_exclusion import direct_children, name, FIRST_SHA
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
from OCP.TopLoc import TopLoc_Location
from OCP.gp import gp_Pnt

# The first field half, tape at the barge boundary and starting tape support the wheels.
SUPPORTS = {"NAUO2:FE-2024-01","NAUO16:tape_Truss1_Red","NAUO31:tape_AutoLine"}
SEARCH_MM = 350
TRIALS = {"issued":(365,95.5625),"shorter":(370,95.5625),
          "shared-100":(370,100),"shared-105":(370,105),"shared-110":(370,110)}
INCH = 25.4


def bounds(shape):
    box=Bnd_Box()
    BRepBndLib.Add_s(shape,box,False)
    if box.IsVoid(): return None
    return (*box.CornerMin().Coord(),*box.CornerMax().Coord())


def box_distance(a,b):
    return math.sqrt(sum(max(a[i]-b[i+3],b[i]-a[i+3],0)**2 for i in range(3)))


def field_tree(field, search):
    if digest(field)!=FIRST_SHA: raise ValueError("Unexpected field revision")
    doc,table=cad.xcaf_doc()
    rd=cad.STEPCAFControl_Reader()
    if rd.ReadFile(str(field))!=cad.IFSelect_RetDone or not rd.Transfer(doc):
        raise ValueError("Failed to reopen field")
    roots=cad.Sequence_TDF_Label()
    table.GetFreeShapes(roots)
    if roots.Length()!=1: raise ValueError("Expected one field assembly")
    leaves={}
    supports={}
    def visit(parent,location,prefix):
        for occ,definition in direct_children(table,parent):
            loc=location.Multiplied(table.GetLocation_s(occ))
            key=prefix+name(occ)+":"+name(definition)
            shape=table.GetShape_s(definition).Moved(loc)
            extent=bounds(shape)
            if extent is None or box_distance(extent,search)>SEARCH_MM: continue
            if key in SUPPORTS:
                if extent[4]>5.2: raise ValueError("Unexpected support surface height")
                supports[key]=extent
            elif table.IsAssembly_s(definition):
                visit(definition,loc,key+"/")
            else:
                leaves[key]=(shape,extent)
    visit(roots.Value(1),TopLoc_Location(),"")
    if set(supports)!=SUPPORTS or not leaves: raise ValueError("Incomplete local field register")
    return leaves,supports


def prism(r):
    return BRepPrimAPI_MakeBox(gp_Pnt(*r[:3]),gp_Pnt(*r[3:])).Shape()


def native_box(f,route,hull):
    originX=f.PROFILE["field"]["length"]*INCH/2
    originZ=f.PROFILE["field"]["width"]*INCH/2
    if any(abs(p[2]-math.pi)>1e-8 for p in (route.start,route.end)):raise ValueError("Expected straight red drive")
    # World X increases with native X; world Y decreases with native Z.
    return [route.end[0]*1000-originX-hull[3],hull[2],originZ-route.start[1]*1000+hull[1],
            route.start[0]*1000-originX-hull[0],hull[5],originZ-route.start[1]*1000+hull[4]]


def distance(shape,other):
    calc=BRepExtrema_DistShapeShape(shape,other)
    if not calc.IsDone():raise ValueError("Native shape distance did not converge")
    return calc.Value()


def run(field):
    f=corrected()
    prior=json.loads((HERE/"study.json").read_bytes())
    if digest(KIT/"stowed.step")!=prior["stowed_step_sha256"]: raise ValueError("Unexpected robot")
    ids=json.loads((KIT/"part-frames.json").read_bytes())
    robot=cad.read_named(KIT/"stowed.step",[{"id":key} for key in ids])
    rb=[cad.bounds(s) for s in robot.values()]
    hull=[min(b[i] for b in rb) for i in range(3)]+[max(b[i] for b in rb) for i in range(3,6)]
    if hull!=prior["hull_mm"] or len(robot)!=prior["body_count"]: raise ValueError("Stowed robot changed")
    startX=f.routes(-.18)[0].start[0]
    routes={}
    for key,(x,y) in TRIALS.items():
        route=f.Route((startX,y*f.IN,math.pi))
        route.go((x*f.IN,y*f.IN))
        routes[key]=route
    boxes={k:native_box(f,r,hull) for k,r in routes.items()}
    search=[min(b[i] for b in boxes.values()) for i in range(3)]+[max(b[i] for b in boxes.values()) for i in range(3,6)]
    leaves,supports=field_tree(field,search)
    cases={}
    for key,route in routes.items():
        envelope=prism(boxes[key])
        contacts={n:distance(envelope,s) for n,(s,b) in leaves.items() if box_distance(boxes[key],b)<=SEARCH_MM}
        closest=min(contacts,key=contacts.get)
        if contacts[closest]>=SEARCH_MM:raise ValueError("Search radius cannot establish minimum")
        tape=(TAPE[0],0,TAPE[1],f.PROFILE["field"]["width"])
        original_south=traffic.Program(1,"red")
        partner_gap=abs(route.start[1]-original_south.initial[1])-.85
        # Test the conventional southern partner as well as the preassigned handoff.
        keep_south=key.startswith("shared")
        roster=team_at_teleop(f,route,move_partner=not keep_south)
        tele=traffic.analyze(stock=(None,None),prepared=(roster,[],[]))
        partner=roster["red1"].initial
        peer_rect=(partner[0]-.425,partner[1]-.425,partner[0]+.425,partner[1]+.425)
        south=original_south.initial
        adverse_rect=(south[0]-.425,south[1]-.425,south[0]+.425,south[1]+.425)
        adverse=overlap(f,robot,route.start,tuple(v/f.IN for v in adverse_rect))
        static_peer_min=min(abs(route.start[1]-r.initial[1])-.85 for n,r in roster.items()
                            if n.startswith("red") and n!="red2")
        native_gap=contacts[closest]/1000
        wheel=wheel_error_budget(f,route,hull,tape,ZONE,robot,overlap,native_gap,.01)
        # In AUTO the other five hold axis-aligned starting footprints.
        aligned_gaps={}
        for peer_id,other in roster.items():
            if peer_id=="red2":continue
            x,y,_=other.initial
            dx=max(route.end[0]-x,x-route.start[0],0)-.85
            dy=abs(route.start[1]-y)-.85
            aligned_gaps[peer_id]=max(dx,dy)
        cases[key]={"start":route.start,"finish":route.end,"red1_keeps_south":keep_south,
            "teleop_pair_bound_m":tele["min_separating_bound_m"],"teleop_clear":tele["separated_for_full_clock"],
            "auto_red_partner_lateral_bound_m":static_peer_min,"initial_overlap_against_south_partner_mm3":adverse,
            "reserved_red1_world_xy":peer_rect,"south_red1_world_xy":adverse_rect,"drive_s":route.duration,"one_percent_wheel_case":wheel,"auto_focal_to_each_peer_m":aligned_gaps,
            "native_swept_box_mm":boxes[key],"static_field_lower_bound_mm":contacts[closest],
            "limiting_occurrence":closest,"nearest_occurrences_mm":dict(sorted(contacts.items(),key=lambda item:item[1])[:3]),
            "tape_clearance_mm":(TAPE[0]*f.IN-route.end[0])*1000-hull[3],
            "start_cover_mm3":overlap(f,robot,route.start,tape),"end_tape_cover_mm3":overlap(f,robot,route.end,tape),
            "end_park_cover_mm3":overlap(f,robot,route.end,ZONE),"south_partner_gap_mm":partner_gap*1000,
            "planning_field":min(f.audit(route,hull),key=lambda row:row["gap_m"])}
    return {"robot_step_sha256":prior["stowed_step_sha256"],"field_step_sha256":FIRST_SHA,"body_count":len(ids),
            "search_radius_mm":SEARCH_MM,"local_hardware_leaf_count":len(leaves),"support_surfaces":supports,
            "local_hardware_native_boxes_mm":{k:b for k,(_,b) in leaves.items()},"cases":cases}


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("field",type=Path)
    parser.add_argument("--output",type=Path,default=HERE/"native-park.json")
    args=parser.parse_args()
    args.output.write_bytes((json.dumps(run(args.field),indent=2,allow_nan=False)+"\n").encode())
