"""Replay the registered field routes and pipe departure against an explicit named robot inventory."""
import argparse
import hashlib
import importlib.util
import json
import math
import sys
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from mission import field_body_clearance as clearance
from mission.replay_registered_field import others, rect
from mission import end_cartridge_vision
from scripts import registered_transit as field
from scripts import station_l1_mission as mission, station_l1_settle as settle
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut

POSES={"stowed":(940,0),"receive":(940,240),"transport":(670,0),"place":(560,370)}


def unique_object(pairs):
    obj=dict(pairs)
    if len(pairs)!=len(obj):
        raise ValueError("Duplicate key in supplied inventory")
    return obj


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def enclosing(boxes):
    return tuple((min if i<3 else max)(b[i] for b in boxes) for i in range(6))


def translated(box,dx,dz):
    return tuple(v+(dx if i%3==0 else dz if i%3==2 else 0) for i,v in enumerate(box))


def contains(outer,inner,tol=.025):
    return all(outer[i]-tol<=inner[i] and inner[i+3]<=outer[i+3]+tol for i in range(3))


def read_inventory(path):
    raw=json.loads(path.read_text(encoding="utf-8"),object_pairs_hook=unique_object)
    if not isinstance(raw,dict) or not raw:
        raise ValueError("Expected a nonempty named motion map")
    entries={}
    for key,desc in raw.items():
        if not isinstance(key,str) or not key:
            raise ValueError("Missing part identity")
        entry={"frame":desc} if isinstance(desc,str) else desc
        if not isinstance(entry,dict) or entry.get("frame") not in {*clearance.FACTORS,"variable-rope","variable-draw-wire"}:
            raise ValueError("Unsupported motion declaration for "+key)
        entries[key]=entry
    return entries


class SuppliedRobot(clearance.CAD):
    """Supply the existing sweep with known placement bodies and conservative continuous envelopes."""
    def __init__(self,inventory,paths,rotors=(),hashes=None,radial=None,outer_error=0,grip_error=0):
        parts=read_inventory(inventory)
        radial=radial or {}
        if any(not math.isfinite(e) or e<0 for e in (outer_error,grip_error)):
            raise ValueError("Use finite nonnegative motion uncertainties")
        self.errors={"outer_slide_x_mm":outer_error,"grip_independent_y_mm":grip_error}
        if set(paths)!=set(POSES):
            raise ValueError("All four registered pose STEP files are required")
        if not (set(rotors)|set(radial))<=set(parts):
            raise ValueError("A declared rotating body is absent")
        self.hashes={n:digest(p) for n,p in paths.items()}
        if hashes is not None and self.hashes!=hashes:
            raise ValueError("A supplied STEP differs from the expected hashes")
        self.sha=self.hashes["place"]
        self.transport_sha=self.hashes["transport"]
        self.inventory_sha=digest(inventory)
        self.frames={}
        self.boxes={}
        self.exceptions={}
        self.hulls={}
        masks={}
        named=[{"id":k} for k in parts]
        for pose,path in paths.items():
            shapes=clearance.cad.read_named(path,named)
            masks[pose]={k:clearance.cad.bounds(s) for k,s in shapes.items()}
            self.hulls[pose]=enclosing(list(masks[pose].values()))
            if pose=="place":
                self.shapes=shapes
        self.pose_bounds=self.hulls["transport"]
        for name,entry in parts.items():
            frame=entry["frame"]
            if frame=="variable-draw-wire":
                if name!="slide-position-line":
                    raise ValueError("Unrecognized draw wire "+name)
                frame="lift"
            placed=masks["place"][name]
            if frame=="variable-rope":
                # The project declares a straight vertical return line with its fixed top
                # above a translating lower end. The place pose has the longest length.
                if any(not contains(placed,p[name]) or any(abs(p[name][i]-placed[i])>.025
                    for i in (0,1,3,4,5)) or abs(p[name][2]-placed[2]-(POSES[n][0]-560))>.025
                    for n,p in masks.items()):
                    raise ValueError("Return-line endpoints differ from the declared sweep: "+name)
                self.frames[name]="base"
                self.boxes[name]=placed
                self.exceptions[name]="straight fixed-top return line, bounded by longest STEP pose"
                continue
            factor=clearance.FACTORS[frame]
            local=[translated(masks[p][name],(370-s)*factor,560-h if frame!="base" else 0)
                   for p,(h,s) in POSES.items()]
            rotor=name in rotors or name in radial or entry.get("rotatesWithinAabbPivot") is True
            if name=="slide-position-line":
                # Lift-mounted straight draw wire pays out by the exact outer stroke.
                for b,(_,s) in zip(local,POSES.values(),strict=True):
                    predicted=list(placed)
                    predicted[3]+=s-370
                    if not contains(b,predicted) or not contains(predicted,b):
                        raise ValueError("Draw-wire stroke disagrees with the registered two endpoints")
                envelope=placed
                self.exceptions[name]="retained lift-frame box of the longest draw-wire extension"
            elif name in radial:
                spec=radial[name]
                if spec["frame"]!=frame:
                    raise ValueError("Rotary frame mismatch: "+name)
                bounds=spec["bounds_mm"]
                r=spec["outer_radius_mm"]
                length=spec["axial_width_mm"]
                if (spec["axis"]!=[0,1,0] or len(bounds)!=6 or not all(math.isfinite(k) for k in (*bounds,r,length))
                        or min(r,length)<=0 or not all(abs(bounds[i+3]-bounds[i]-s)<.001 for i,s in enumerate((2*r,length,2*r)))):
                    raise ValueError("Invalid radial motion envelope: "+name)
                dz=0 if frame=="base" else 560
                envelope=translated(bounds,370*factor,dz)
                self.exceptions[name]="full swept +Y-axis cylinder from supplied registered radial dimensions"
            elif rotor:
                # A full turn about a pivot inside the part's reference box lies within
                # a box expanded by that part's diagonal. This also covers intermediate teeth.
                diameter=math.dist(placed[:3],placed[3:])
                envelope=tuple(v+(-diameter if i<3 else diameter) for i,v in enumerate(placed))
                self.exceptions[name]="full rotation about an internal pivot, bounded by its box diagonal"
            else:
                envelope=placed
            if any(not contains(envelope,b) or (name!="slide-position-line" and not rotor and not contains(b,envelope)) for b in local):
                raise ValueError("Pose geometry disagrees with declared motion: "+name)
            ex=outer_error*(1 if name=="slide-position-line" else factor)
            ey=grip_error if name.startswith(("grip-L-","grip-R-")) else 0
            envelope=tuple(v+d for v,d in zip(envelope,(-ex,-ey,0,ex,ey,0),strict=True))
            self.frames[name]=frame
            self.boxes[name]=envelope


def load_source(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    module=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def recorded_departure(robot,case,delay,onset_half_range=0):
    sector,key,up=case
    post=settle.continue_contact(sector=sector,withdrawal="after-separation",wait_after_true_separation_s=delay,
                                 upstream=up,piece_id=key)
    if post["stop"]!="sustained-paired-floor-and-lip":
        raise ValueError("Revised contact experiment did not sustain support: "+key)
    first=post["clear_witness_from_station_s"]
    floor=post["start_from_station_s"]
    points=[(r["time_s"],r["target_xyz_m"]) for r in up["trace"] if first<=r["time_s"]<floor]
    points.extend((floor+t,(x,sector,z)) for t,x,z,*_ in post["trace"])
    last={}
    gap_tail=math.inf
    for (ta,a),(tb,b) in zip(points,points[1:]):
        pad=.025+500*(tb-ta)*(post["peak_planar_speed_mps"] if tb>floor else 1.3)
        lo=max(0,min(370,(ta-first-delay)*150))
        hi=max(0,min(370,(tb-first-delay)*150))
        for name,body in robot.boxes.items():
            k=clearance.FACTORS[robot.frames[name]]
            if onset_half_range and tb>=first+delay-onset_half_range:
                uncertainty=150*k*onset_half_range
                body=(body[0]-uncertainty,*body[1:3],body[3]+uncertainty,*body[4:])
            p=((a[0]+.49)*1000+lo*k,(a[1]-sector)*1000,a[2]*1000)
            q=((b[0]+.49)*1000+hi*k,(b[1]-sector)*1000,b[2]*1000)
            gap=clearance.cylinder_sweep(p,q,body,pad)
            if gap<=0:
                last[name]=tb
            if ta>=floor+.25:
                gap_tail=min(gap_tail,gap)
    x,z=post["last_xz_vxvz_spin"][:2]
    return {"piece_id":key,"sector_m":sector,"floor_time_s":floor,"first_shelf_witness_s":first,
            "body_count":len(robot.boxes),"last_possible_contact_s":max(last.values(),default=first),
            "last_possible_by_body":last,"min_tail_gap_mm":gap_tail,"post_observation_s":floor+post["elapsed_s"],
            "settled_target_xz_m":[x,z],"checks_next":[robot.next_sector(x,sector,z,s) for s in (.18,.22)]}


def early_contacts(robot,dep,delay,result):
    """Compare pipe wall material with each broad-phase candidate at fixed registered place pose."""
    first=dep["first_shelf_witness_s"]
    last=dep["last_possible_contact_s"]
    end=min(last,first+delay,dep["floor_time_s"])
    if end<=first or not dep["last_possible_by_body"]:
        return []
    sector=dep["sector_m"]
    selected=[r for r in result["trace"] if first<=r["time_s"]<=end+1e-9]
    if not selected:
        return []
    rows=[]
    for row in selected[::5]+([selected[-1]] if selected[-1] not in selected[::5] else []):
        x,y,z=row["target_xyz_m"]
        yaw=row["robot_relative_yaw_rad"]
        dx,dy=-math.sin(yaw),math.cos(yaw)
        axis=clearance.gp_Ax2(clearance.gp_Pnt((x+.49)*1000-dx*clearance.HALF,
            (y-sector)*1000-dy*clearance.HALF,z*1000),clearance.gp_Dir(dx,dy,0))
        outer=clearance.BRepPrimAPI_MakeCylinder(axis,clearance.RAD,2*clearance.HALF).Shape()
        bore=clearance.BRepPrimAPI_MakeCylinder(axis,50.8,2*clearance.HALF).Shape()
        cut=BRepAlgoAPI_Cut(outer,bore)
        if not cut.IsDone():
            raise ValueError("Could not form the observed finite CORAL wall")
        pipe=cut.Shape()
        hits={}
        extremes={}
        candidates=set(dep["last_possible_by_body"])
        candidates.update(k for k in robot.shapes if k.startswith(("grip-L-","grip-R-"))
                          and k.endswith(("-3M-15","-front-riser")))
        for key in sorted(candidates):
            dist=clearance.BRepExtrema_DistShapeShape(pipe,robot.shapes[key])
            if not dist.IsDone():
                raise ValueError("Unresolved native distance: "+key)
            amount=clearance.cad.intersection(pipe,robot.shapes[key]) if dist.Value()<1e-5 else 0
            hits[key]={"gap_mm":round(dist.Value(),6),"overlap_mm3":round(amount,6)}
            if robot.extreme_shapes is not None:
                dx=clearance.FACTORS[robot.frames[key]]*min(2,robot.errors["outer_slide_x_mm"])
                # The authored receiver model resolves compliance, fixed risers,
                # spring length, and the stroke-dependent pinions and draw wire.
                altered=robot.extreme_shapes.get(key)
                if altered is None:
                    altered=clearance.cad.translated(robot.shapes[key],(dx,0,0))
                d=clearance.BRepExtrema_DistShapeShape(pipe,altered)
                if not d.IsDone():
                    raise ValueError("Unresolved displaced shape "+key)
                volume=clearance.cad.intersection(pipe,altered) if d.Value()<1e-5 else 0
                if volume>1e-6:
                    extremes[key]=round(volume,6)
        rows.append({"after_shelf_ms":round(1000*(row["time_s"]-first),4),"bodies":hits,
                     "forward_endpoint_inward_grip_overlap_mm3":extremes})
    return rows


def prepare_extreme(robot,source):
    robot.extreme_shapes=None
    robot.extreme_source_sha=None
    if source is None:
        return
    module=load_source("candidate_receiver_drives",source)
    dy=robot.errors["grip_independent_y_mm"]
    if not 0<=dy<=1.5:
        raise ValueError("Lateral compliance exceeds the modeled cartridge stops")
    nominal=module.added_shapes({"height":560,"stroke":370})
    if not set(nominal)<=set(robot.shapes):
        raise ValueError("Receiver model and supplied STEP list different parts")
    for k,shape in nominal.items():
        a,b=clearance.cad.bounds(shape),clearance.cad.bounds(robot.shapes[k])
        if not contains(a,b) or not contains(b,a):
            raise ValueError("Receiver model disagrees with reopened nominal STEP "+k)
    robot.extreme_shapes=module.added_shapes({"height":560,"stroke":370+min(2,robot.errors["outer_slide_x_mm"])},
                                             {"L":-dy,"R":-dy})
    robot.extreme_source_sha=digest(source)


def review(robot,delay=.127,contact_root=None):
    if not math.isfinite(delay) or not 0<=delay<=.6:
        raise ValueError("Use a finite delay from 0 through 0.6 s")
    source=ROOT/"mission/registered-field-receipt.json"
    reference=json.loads(source.read_text())
    if len(reference["trips"])!=2:
        raise ValueError("The two registered specimen trajectories are required")
    receiver="receiver-R6-ported"
    baseline=clearance.CAD()
    if receiver not in robot.shapes:
        raise ValueError("The receiver which establishes the registered release surface is absent")
    a=baseline.shapes[receiver]
    b=robot.shapes[receiver]
    common=clearance.BRepAlgoAPI_Common(a,b)
    if not common.IsDone():
        raise ValueError("Unable to compare registered receiver geometry")
    shared=clearance.cad.volume(common.Shape())[0]
    diff=[abs(clearance.cad.volume(s)[0]-shared) for s in (a,b)]
    if max(diff)>.05:
        raise ValueError("Supplied shelf differs from the geometry underlying the pipe trajectory")
    parts=list(robot.boxes)
    targets=[]
    routes=[]
    deposits=[]
    parked=others()
    producer=mission
    cartridge=None
    if contact_root:
        producer=load_source("scripts.declared_station_producer",contact_root/"scripts/station_l1_mission.py")
        cartridge=load_source("declared_contact_law",contact_root/"design/receiver-drives/size_drives.py").Cartridge
    def route_check(leg,route,pose,extra):
        hull=robot.hulls[pose]
        overhead=[]
        for obs in field.PROFILE["rehearsal"]["obstacles"]:
            if obs["kind"]!="overlay":
                continue
            lower=obs["lower"]*field.IN
            if 1.020<lower<=hull[5]/1000:
                overhead.append((obs["name"],[(x*field.IN,y*field.IN) for x,y in obs["poly"]]))
        clearance_rows=field.audit(route,hull,parked+extra+overhead)
        routes.append({"leg":leg,"pose":pose,"extra_height_obstacles":[k for k,_ in overhead],
                       "min_clearance":min(clearance_rows,key=lambda v:v["gap_m"])})
    for idx,specimen in enumerate(reference["trips"]):
        sector=specimen["sector_m"]
        paths=field.routes(sector)
        if idx==0:
            route_check("starting-line-to-station",paths[0],"stowed",[])
        route_check("loaded-"+specimen["piece_id"],paths[1],"transport",targets)
        key=specimen["piece_id"]
        args={"grip_law":cartridge()} if cartridge else {}
        up=producer.run(dt=.001,target_y=sector,trajectory=field.RegisteredMission(sector),sample_every_step=True,
                        piece_id=key,**args)
        dep=recorded_departure(robot,(sector,key,up),delay)
        dep["last_possible_after_shelf_ms"]=1000*(dep["last_possible_contact_s"]-dep["first_shelf_witness_s"])
        dep["clear_at_support"]=dep["min_tail_gap_mm"]>0 and dep["last_possible_contact_s"]<dep["post_observation_s"]
        dep["early_native_pipe_wall_snapshots"]=early_contacts(robot,dep,delay,up)
        dep["independent_front_plane_stereo"]=end_cartridge_vision.study(robot,up,dep)
        deposits.append(dep)
        x,z=dep["settled_target_xz_m"]
        targets.append((specimen["piece_id"],rect(field.LIP[0]+x,field.LIP[1]+sector,.05725,.1509125)))
        route_check("return-"+specimen["piece_id"],paths[2],"transport",targets)
    occupied=deposits[0]["checks_next"]
    chosen=next(v for v in occupied if v["sector_m"]==reference["trips"][1]["sector_m"])
    roster_bounds={n:contains((-535 if n=="place" else -425,-425,-math.inf,
                              535 if n=="place" else 425,425,math.inf),b,1e-5)
                   for n,b in robot.hulls.items()}
    return {"schema":"supplied-robot-registered-clearance/1","source_trajectory_sha256":digest(source),
            "pose_step_sha256":robot.hashes,"inventory_sha256":robot.inventory_sha,
            "body_count":len(parts),"part_ids":sorted(parts),"pose_bounds_mm":robot.hulls,
            "registered_receiver_difference_volumes_mm3":diff,"clearance_error_allowances_mm":robot.errors,
            "exact_displaced_receiver_source_sha256":robot.extreme_source_sha,
            "grip_law_sha256":digest(contact_root/"design/receiver-drives/size_drives.py") if contact_root else None,
            "continuous_envelope_exceptions":robot.exceptions,"retreat_delay_after_true_separation_s":delay,
            "fits_previously_tested_roster_rectangles_by_pose":roster_bounds,
            "route_checks":routes,"departures":deposits,
            "automatic_release_at_shelf_witness_qualified":False,
            "qualifies_registered_departure_and_travel":all(r["min_clearance"]["gap_m"]>0 for r in routes)
                and all(d["clear_at_support"] for d in deposits) and chosen["clear_all_modeled_poses"]}


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--inventory",required=True,type=Path,help="Part name to motion-frame JSON")
    p.add_argument("--poses",required=True,type=Path,help="Directory of four named project STEP files")
    for name in POSES:
        p.add_argument("--"+name,type=Path,help="Override this individual registered STEP pose")
    p.add_argument("--rotor",action="append",default=[],help="Body rotating about a pivot inside its own bounds")
    p.add_argument("--radial",type=Path,help="Map from part names to local swept rotary cylinder dimensions")
    p.add_argument("--outer-error-mm",type=float,default=0)
    p.add_argument("--grip-error-mm",type=float,default=0)
    p.add_argument("--receiver-source",type=Path,help="Trusted project receiver_drives.py for displaced native material checks")
    p.add_argument("--contact-root",type=Path,help="Project source containing the corresponding passive grip law and trajectory solver")
    p.add_argument("--hashes",type=Path,help="Optional JSON map of expected SHA-256 by pose")
    p.add_argument("--delay",type=float,default=.127)
    p.add_argument("--output",required=True,type=Path)
    a=p.parse_args()
    paths={n:getattr(a,n) or a.poses/(n+".step") for n in POSES}
    expected=json.loads(a.hashes.read_text(),object_pairs_hook=unique_object) if a.hashes else None
    radial=json.loads(a.radial.read_text(),object_pairs_hook=unique_object) if a.radial else None
    robot=SuppliedRobot(a.inventory,paths,a.rotor,expected,radial,a.outer_error_mm,a.grip_error_mm)
    prepare_extreme(robot,a.receiver_source)
    result=review(robot,a.delay,a.contact_root)
    a.output.write_text(json.dumps(result,indent=2,allow_nan=False)+"\n",encoding="utf-8",newline="\n")
    print("Bodies:",result["body_count"],"registered clearance:",result["qualifies_registered_departure_and_travel"])
