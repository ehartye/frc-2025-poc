"""Bound two identified station introductions and test their first receiver contact on the 401-part robot."""
import argparse
import hashlib
import json
import math
import sys
from pathlib import Path
from native_station import parts, ROOT, FIRST_SHA, cad

sys.path.insert(0,str(ROOT/"full-match"))
sys.path.insert(0,str(ROOT/"design/receiver-drives"))
from match import corrected, basis, trial
from size_drives import Cartridge
from mission import field_body_clearance as clearance, moving_traffic as traffic
from scripts import station_l1_mission as mission, station_receiver_impact as impact
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace,BRepBuilderAPI_MakePolygon
from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder,BRepPrimAPI_MakePrism
from OCP.gp import gp_Ax2,gp_Dir,gp_Pnt,gp_Vec

R,H=57.15,150.8125
ST=impact.station
G=9.80665
A=math.radians(35)
K=.5*(1+(50.8/57.15)**2)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def tube(x,z,yaw=0,y=0):
    dx,dy=math.sin(math.radians(yaw)),math.cos(math.radians(yaw))
    axis=gp_Ax2(gp_Pnt(x-H*dx,y-H*dy,z),gp_Dir(dx,dy,0))
    shell=BRepAlgoAPI_Cut(BRepPrimAPI_MakeCylinder(axis,R,2*H).Shape(),BRepPrimAPI_MakeCylinder(axis,50.8,2*H).Shape())
    if not shell.IsDone():
        raise ValueError("Pipe bore cut failed")
    return shell.Shape()


def hits(shapes,boxes,x,z,yaw=0,y=0):
    piece=tube(x,z,yaw,y)
    p=cad.bounds(piece)
    results={}
    for name,box in boxes.items():
        if cad.occupied(p,box):
            material=cad.intersection(piece,shapes[name])
            if material>.05:
                results[name]=round(material,4)
    return results


def corridor(shapes,boxes,p0,p1,extra_radius=0):
    """A transverse filled cylinder swept along a chord is two caps and a prism."""
    x,z=p0
    ex,ez=p1
    dx,dz=ex-x,ez-z
    length=math.hypot(dx,dz)
    radius=R+extra_radius
    nx,nz=-dz*radius/length,dx*radius/length
    polygon=BRepBuilderAPI_MakePolygon()
    for xx,zz in ((x+nx,z+nz),(ex+nx,ez+nz),(ex-nx,ez-nz),(x-nx,z-nz)):
        polygon.Add(gp_Pnt(xx,-H,zz))
    polygon.Close()
    prism=BRepPrimAPI_MakePrism(BRepBuilderAPI_MakeFace(polygon.Wire()).Face(),gp_Vec(0,2*H,0)).Shape()
    solids=[prism]+[BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(xx,-H,zz),gp_Dir(0,1,0)),radius,2*H).Shape()
                    for xx,zz in (p0,p1)]
    overlaps={}
    for solid in solids:
        bounds=cad.bounds(solid)
        for key,box in boxes.items():
            if cad.occupied(bounds,box):
                volume=cad.intersection(solid,shapes[key])
                if volume>.05:
                    overlaps[key]=max(overlaps.get(key,0),round(volume,5))
    return overlaps


def rolling(start_x,zrelease):
    lipx,lipz=ST.LIP_X,ST.LIP_Z
    # Pipe center when the transverse outer surface is tangent to the 35 degree ramp.
    center_z=lipz+(lipx-start_x)*math.tan(A)+R/1000/math.cos(A)
    run=(lipx+R/1000*math.sin(A)-start_x)/math.cos(A)
    if run<=0 or zrelease<=center_z:
        raise ValueError("Release must be above a positive length of ramp")
    drop=math.sqrt(2*G*(zrelease-center_z))
    # No rebound or initial spin. Tangential sticking impulse establishes rolling.
    v0=drop*math.sin(A)/(1+K)
    mu=K*v0/(drop*math.cos(A))
    acc=G*math.sin(A)/(1+K)
    vlip=math.sqrt(v0*v0+2*acc*run)
    return dict(free_drop_s=drop/G,first_ramp_center_xz_m=[start_x,center_z],fall_speed_mps=drop,
                run_m=run,initial_rolling_speed_mps=v0,ramp_accel_mps2=acc,lip_mps=vlip,
                ramp_time_s=(vlip-v0)/acc,minimum_slide_to_roll_friction=mu,
                minimum_static_rolling_friction=K/(1+K)*math.tan(A))


def seat(speed,dt=.001):
    first=impact.first_impact(speed,10,240,6,json.loads(impact.DESIGN.read_bytes()))
    if first["surface"]!="top":
        return {"result":"hold-first-contact-unmodeled","impact":first}
    u=.444525-first["center_m"][0]-.240
    velocity=-first["velocity_in_mps"][0]
    law=Cartridge()
    mass=mission.M["pipeMass_kg"]
    minimum=u
    for i in range(round(1.1/dt)):
        normals=law(u,0,0,"seat",i*dt)
        a,b,mu,_,drag=normals
        request=mass*(mission.M["servoGain_s2"]*(.080-u)-mission.M["servoDamping_s"]*velocity)
        force=mission.clip(request,mu*(a+b))
        if a+b==0:
            force-=mission.clip(mass*velocity/dt,mass*G*drag)
        accel=force/mass
        u+=velocity*dt+.5*accel*dt**2
        velocity+=accel*dt
        minimum=min(u,minimum)
        if not -.100<=u<=.165:
            break
    confirmed=abs(u-.08)<=.008 and abs(velocity)<.04 and i+1==round(1.1/dt)
    return {"result":"within-modeled-seat-window" if confirmed else "recover","impact":first,
            "minimum_local_x_m":minimum,"last_local_x_m":u,"last_relative_speed_mps":velocity,
            "simulated_s":(i+1)*dt,"peak_grip_travel_mm":law.max_travel}


def remote_robot_gap(team,start,end,f,source_x_range):
    # All transfer poses are contained in a fixed box. Project the pipe's actual
    # transverse half length, end-of-seat position and outer radius into a disk.
    center_x=(source_x_range[0]+source_x_range[1])/2
    r=math.hypot((source_x_range[1]-source_x_range[0])/2+.05715,.1508125)
    center=f.add(f.STATION,f.rot((center_x,0),f.NORMAL))
    min_gap=1e9
    nearest=None
    n=math.ceil((end-start)/.02)
    # The maximum speed on all five declared programs is <= 1.25 m/s at the
    # chassis center, including in-place turns whose center stays still.
    for i in range(n):
        t=start+(i+.5)*(end-start)/n
        for name,p in team.items():
            if name=="red2":
                continue
            pose=p.at(t)[0]
            gap=math.dist(center,pose[:2])-r-traffic.RADIUS-1.25*(end-start)/(2*n)
            if gap<min_gap:
                min_gap,nearest=gap,name
    return {"lower_bound_m":min_gap,"nearest_program":nearest,"other_robot_count":5,"max_interval_s":.02}


def run(step,poses):
    shape=parts(step)
    boxes={k:cad.bounds(v) for k,v in shape.items()}
    rear=next(k for k in boxes if k.endswith("/GE-25020"))
    front=next(k for k in boxes if k.startswith("NAUO1496/"))
    low=boxes[rear][3]
    high=boxes[front][0]
    center=(low+high)/2
    slot=high-low
    yaw=math.degrees(math.asin(slot/math.hypot(2*R,2*H))-math.atan2(2*R,2*H))
    source=rolling(center/1000,1.550)
    rkey=ROOT/"full-match/red-kit/manifest.json"
    expected=json.loads(rkey.read_bytes())["receive"]["red_sha256"]
    robot=cad.read_named(poses/"receive.step",[{"id":k} for k in json.loads((poses/"part-frames.json").read_bytes())])
    if len(robot)!=401 or sha(poses/"receive.step")!=expected:
        raise ValueError("Robot inventory or STEP differs")
    robotbox={k:cad.bounds(v) for k,v in robot.items()}
    negatives={}
    for name,dx,angle in (("center",0,0),("shifted-toward-field",10,0),("yaw-three-degrees",0,3)):
        negatives[name]=hits(shape,boxes,center+dx,1340,angle)
    zstart=source["first_ramp_center_xz_m"][1]*1000
    rampfeed=[]
    for frac in (0,.25,.5,.75,1):
        x=center+1000*source["run_m"]*frac*math.cos(A)
        z=zstart-1000*source["run_m"]*frac*math.sin(A)
        rampfeed.append({"fraction":frac,"center_mm":[x,z],"material_mm3":hits(shape,boxes,x,z)})
    ramp_start=[center,zstart]
    ramp_end=rampfeed[-1]["center_mm"]
    sweeps={"vertical-drop":corridor(shape,boxes,[center,1550],ramp_start),
            "tangent-roll":corridor(shape,boxes,ramp_start,ramp_end),"flight":[]}
    for label,origin,end in (("vertical-drop", [center,1550],ramp_start),("tangent-roll",ramp_start,ramp_end)):
        sweeps[label+"-receiving-robot"]=corridor(robot,robotbox,*[(444.525-x,z) for x,z in (origin,end)])
    state,flight,_=ST.path_to_shelf(source["lip_mps"])
    previous=ramp_end
    for i in range(1,9):
        endpoint=[v*1000 for v in ST.flight_center(state,flight*i/8)]
        # Maximum distance between a ballistic parabola and its chord.
        grow=1000*G*(flight/8)**2/8
        robot_points=[(444.525-x,z) for x,z in (previous,endpoint)]
        sweeps["flight"].append({"until_s":flight*i/8,"chord_expansion_mm":grow,
            "station":corridor(shape,boxes,previous,endpoint,grow),
            "robot":corridor(robot,robotbox,*robot_points,grow)})
        previous=endpoint
    # After the same first ramp impact allow one normal rebound. Keep the same
    # sticking tangential impulse. Stop at the first resolved material sample.
    rebound=[]
    for restitution in (.2,.4,.6):
        normal=restitution*source["fall_speed_mps"]*math.cos(A)
        tangent=source["initial_rolling_speed_mps"]
        vx=tangent*math.cos(A)+normal*math.sin(A)
        vz=-tangent*math.sin(A)+normal*math.cos(A)
        end=2*normal/(G*math.cos(A))
        first=None
        for i in range(1,math.ceil(end/.005)):
            t=i*.005
            px=center+1000*vx*t
            pz=zstart+1000*(vz*t-G*t*t/2)
            # The ramp is the intended landing surface at t=0 and the return.
            candidates={k:v for k,v in boxes.items() if not k.endswith("/GE-25010")}
            material=hits(shape,candidates,px,pz)
            if material:
                first={"elapsed_after_impact_s":t,"center_mm":[px,pz],"material_mm3":material}
                break
        rebound.append({"assumed_normal_restitution":restitution,"sampled_before_return_s":end,"first_material_witness":first})
    rates={}
    for speed in (.5,source["lip_mps"],2.5):
        stage=seat(speed)
        if "impact" not in stage:
            raise ValueError("Failed to observe first impact")
        point=stage["impact"]["center_m"]
        x,z=(.444525-point[0])*1000,point[1]*1000
        state,flight,curve=ST.path_to_shelf(speed)
        snapshots=[]
        for phase in (.90,.98,.99,.995,1):
            sx,sz=ST.flight_center(state,flight*phase)
            material=hits(robot,robotbox,(.444525-sx)*1000,sz*1000)
            snapshots.append({"flight_fraction":phase,"time_from_lip_s":flight*phase,"material_mm3":material})
        catch_to_low=corridor(robot,robotbox,(x,z),((.240+stage["minimum_local_x_m"])*1000,z))
        rates[str(round(speed,6))]={"centered_flat_model":stage,"native_arrival_samples":snapshots,
            "supported_motion_occupied_enclosure":catch_to_low,
            "supported_motion_basis":"Filled finite pipe along the centerline to maximum penetration. Nominal fixed cartridge geometry; positive enclosures need material/compliance interpretation."}
    f=corrected()
    rec,sources=basis(f)
    field,team=trial(f,rec,sources)
    issued=json.loads((ROOT/"full-match/field-trial.json").read_bytes())
    feeds=field["scheduled_station_lip_arrivals_s"]
    two=[]
    old_roll=.5/source["ramp_accel_mps2"]
    for idx,(key,stock,lip) in enumerate(zip(("coral-A","coral-B"),(0,35),feeds,strict=True)):
        start=lip-old_roll+(two[0]["cumulative_extra_s"] if idx else 0)
        seconds=source["free_drop_s"]+source["ramp_time_s"]
        new_lip=start+seconds
        take=seat(source["lip_mps"])
        seat_clock=new_lip+take["impact"]["time_s"]+1.1
        extra=new_lip-lip+take["impact"]["time_s"]-.06831018862283218
        two.append({"id":key,"available_teleop_s":stock,"human_release_teleop_s":start,"ramp_lip_teleop_s":new_lip,
                    "ideal_seat_check_teleop_s":seat_clock,"cumulative_extra_s":extra,
                    "alliance_stock_remaining_if_taken":59-idx,
                    "five_other_programs":remote_robot_gap(team,start,seat_clock,f,[-.286,.35])})
    return {"schema":"station-source-contact/1","field_step_sha256":FIRST_SHA,"receive_step_sha256":expected,
            "issued_match_sha256":sha(ROOT/"full-match/field-trial.json"),"opening_sections":{"rear":rear,"front":front,"center_x_mm":center,"human_side_face_x_mm":low,"field_side_face_x_mm":high,"slot_mm":slot,
            "centered_side_margin_mm":(slot-2*R)/2,"outer_hull_yaw_limit_deg":yaw,"test_z_mm":1340,"native_controls_mm3":negatives},
            "feed":{"release_z_m":1.55,"original_lip_mps":.5,"old_rolling_distance_m":.5**2/(2*source["ramp_accel_mps2"]),
                    "source_assumptions":"Transverse centered, zero spin/velocity, perfectly inelastic ramp landing, no slipping thereafter, no hand model.",
                    **source,"first_rebound_alternatives":rebound,"sampled_ramp_material":rampfeed,"continuous_occupied_hulls":sweeps},"rates":rates,"identified_transfers":two,
            "issued_park_s":issued["hold_variants"]["0"]["rules"]["teleop_motion_completion_s"],
            "whole_match_using_new_source":"pending","remaining":["No observed lawful human transfer or hand tracking.",
              "Sweeps cover centered entry down the nominal ramp through first-catch height. Bounce, actual friction, contact stiffness and sensing are unqualified.",
              "Full custody and release clock for the changed source require a new downstream qualification."]}

if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    for k in ("field","poses","output"):
        p.add_argument("--"+k,type=Path,required=True)
    args=p.parse_args()
    args.output.write_text(json.dumps(run(args.field,args.poses),indent=2,allow_nan=False)+"\n",encoding="utf8",newline="\n")
