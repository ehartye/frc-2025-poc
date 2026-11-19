"""Continue the source-registered L1 contact using a bounded planar compliant model."""
import argparse
import json
import math
from pathlib import Path

if __package__:
    from . import station_l1_mission as mission
else:
    import station_l1_mission as mission


def incoming(case, sector, piece_id, upstream=None):
    result = mission.run(case,.001,target_y=sector,piece_id=piece_id) if upstream is None else upstream
    if result["piece_id"]!=piece_id or result["case"]!=case or result["target_offset_y_m"]!=sector:
        raise ValueError("Upstream identity, configuration or target does not match")
    if result["outcome"]!="paired-trough-first-contact":
        raise ValueError("The upstream case lacks registered paired trough contact")
    edge = next(e for e in result["events"] if e["event"]=="left-powered-shelf-with-same-piece")
    x0,y,z0 = edge["xyz_m"]
    last=result["trace"][-1]
    vx,vy = last.get("target_vxvy_mps",last["vx_vy_mps"])
    if abs(vy)>1e-5 or abs(last.get("robot_relative_yaw_rad",last["yaw_rad"]))>1e-3:
        raise ValueError("Transverse source geometry or trajectory is outside this 2-D continuation")
    source = mission.TARGET
    slope = source["floor_slope"]
    if "target_xyz_m" in edge:
        x0,y,z0=edge["target_xyz_m"]
    else:
        x0 -= mission.M["targetLipX_m"]
    gap = z0 - source["floor_intercept_m"] - slope*x0 - mission.R*math.hypot(1,slope)
    # Positive root at first exact gravity/circle-plane tangency, vz starts at 0 at release.
    turn = (math.sqrt((slope*vx)**2+2*mission.G*gap) - slope*vx)/mission.G
    x = x0 + vx*turn
    z = z0 - .5*mission.G*turn**2
    foot = x+mission.R*slope/math.hypot(1,slope)
    if not all(mission.panel_support(foot,y+d) for d in (-.1,.1)):
        raise ValueError("Exact analytical initial contact lies outside paired support")
    separated = next(e for e in result["events"] if e["event"]=="finite-pipe-envelope-clear-of-shelf-by-1mm")
    return result["piece_id"],result["mass_kg"],edge["time_s"]+turn,[x,z,vx,-mission.G*turn],y,separated["time_s"]


def rectangle(x,z,xmin,zmin,xmax,zmax):
    dx = x-min(max(x,xmin),xmax)
    dz = z-min(max(z,zmin),zmax)
    distance = math.hypot(dx,dz)
    if distance < 1e-12:
        raise ValueError("Tube center entered solid proxy")
    return distance-mission.R, dx/distance,dz/distance


def shelf_gap(x,z,front,top):
    radius=.006
    if x>=front-radius and z>=top-radius:
        return math.hypot(x-(front-radius),z-(top-radius))-radius-mission.R
    return rectangle(x,z,front-.265,top-.015,front,top)[0]


def continue_contact(case="reference", sector=-.18, mu=.35, damping_ratio=.6, dt=.0001, duration=2.0,
                     withdrawal="fixed", initial_roll=0.0, piece_id="station-coral-001", upstream=None,
                     wait_after_true_separation_s=.05, inertia_ratio=1.0):
    if not all(math.isfinite(v) for v in (mu,damping_ratio,dt,duration,initial_roll)) or abs(initial_roll)>40:
        raise ValueError("Contact and time inputs must be finite")
    if not math.isfinite(inertia_ratio) or not .5<=inertia_ratio<=1:
        raise ValueError("Require a finite cylinder polar inertia ratio from 0.5 through 1")
    if not 0<=mu<=1 or not 0<=damping_ratio<=1 or not .000025<=dt<=.00025 or not 0<duration<=135:
        raise ValueError("Outside the explored contact or integration range")
    source = mission.TARGET
    if withdrawal not in ("fixed","late","after-separation"):
        raise ValueError("Unknown withdrawal schedule")
    if not math.isfinite(wait_after_true_separation_s) or wait_after_true_separation_s<0:
        raise ValueError("Separation-to-motion delay must be finite and nonnegative")
    key,mass,clock,state,y,clear_clock = incoming(case,sector,piece_id,upstream)
    x,z,vx,vz = state
    omega=initial_roll # Axial roll was unobserved in the upstream mission.
    spin_start=omega
    slope=source["floor_slope"]
    normal=(-slope/math.hypot(1,slope),1/math.hypot(1,slope))
    r=mission.R
    # Thin-ring estimate. Actual CORAL wall/material properties are unmeasured.
    inertia=mass*r*r*inertia_ratio
    base_energy=.5*mass*(vx*vx+vz*vz)+.5*inertia*omega**2+mass*mission.G*z
    peak_energy_gain=0.0
    peak_speed=math.hypot(vx,vz)
    stiffness=60000.0
    damping=2*damping_ratio*math.sqrt(stiffness*mass)
    friction_speed=.01
    shelf_front = mission.M["chassisAtTargetX_m"]+.535 - mission.M["targetLipX_m"]
    shelf_top = .5
    panel_hit_time=0.0
    first_lip=None
    peak_overlap=0.0
    min_shelf=math.inf
    stop="time-window-ended"
    steps=round(duration/dt)
    trace=[]
    nsteps=0
    still_dwell=0.0
    for i in range(steps+1):
        t=i*dt
        nsteps=i
        floor_gap=(z-source["floor_intercept_m"]-slope*x)*normal[1]-r
        contacts=[]
        if floor_gap<=0:
            foot=x-r*normal[0]
            if not all(mission.panel_support(foot,y+dy) for dy in (-.1,.1)):
                stop="outside-finite-paired-panel"
                break
            contacts.append(("floor",floor_gap,*normal))
        lip_gap,nx,nz=rectangle(x,z,0,.003175,source["lip_depth_m"],source["lip_top_m"])
        if lip_gap<=0:
            contacts.append(("lip",lip_gap,nx,nz))
            if first_lip is None:
                first_lip=t
        energy=.5*mass*(vx*vx+vz*vz)+.5*inertia*omega**2+mass*mission.G*z
        energy+=sum(.5*stiffness*g*g for _,g,_,_ in contacts)
        peak_energy_gain=max(peak_energy_gain,energy-base_energy)
        for a,b,c,d,e,f in source["riser_bounds_m"]:
            if y+r+mission.M["pipeLength_m"]/2<b or y-r-mission.M["pipeLength_m"]/2>e:
                continue
            gap,_,_=rectangle(x,z,a,c,d,f)
            if gap<=0:
                stop="inner-riser-bound"
                break
        if stop!="time-window-ended":
            break
        start_retract = {"fixed":math.inf,"late":clock+1.2,
                         "after-separation":clear_clock+wait_after_true_separation_s}[withdrawal]
        stroke = min(.37, max(0,clock+t-start_retract)*.15)
        gap_shelf = shelf_gap(x,z,shelf_front-stroke,shelf_top)
        min_shelf=min(min_shelf,gap_shelf)
        if gap_shelf<0:
            stop="receiver-recontact"
            break
        if i%max(1,round(.01/dt))==0:
            trace.append([round(t,6),round(x,8),round(z,8),round(vx,8),round(vz,8),round(omega,8)])
        if i==steps:
            break
        fx=torque=0.0
        fz=-mass*mission.G
        active_floor=False
        active_lip=False
        for name,gap,nx,nz in contacts:
            vn=vx*nx+vz*nz
            # Parallel spring-damper normal support with zero tensile force.
            fn=max(0,-stiffness*gap-damping*vn)
            tx,tz=nz,-nx
            # omega positive means anticlockwise in the XZ drawing. Contact
            # point velocity along the chosen tangent gains omega*radius.
            slip=vx*tx+vz*tz+omega*r
            ft=-mu*fn*math.tanh(slip/friction_speed)
            fx+=fn*nx+ft*tx
            fz+=fn*nz+ft*tz
            torque+=r*ft
            peak_overlap=max(peak_overlap,-gap)
            active_floor |= name=="floor" and fn>0
            active_lip |= name=="lip" and fn>0
        if active_floor:
            panel_hit_time+=dt
        if active_floor and active_lip and math.hypot(vx,vz)<.01 and abs(omega)<.2 and gap_shelf>.001:
            still_dwell+=dt
        else:
            still_dwell=0.0
        vx+=fx/mass*dt
        vz+=fz/mass*dt
        peak_speed=max(peak_speed,math.hypot(vx,vz))
        omega+=torque/inertia*dt
        x+=vx*dt
        z+=vz*dt
    if stop=="time-window-ended" and still_dwell>=.25:
        stop="sustained-paired-floor-and-lip"
    return {"piece_id":key,"input_case":case,"sector_m":y,"start_from_station_s":clock,
            "elapsed_s":nsteps*dt,"stop":stop,"last_xz_vxvz_spin":[x,z,vx,vz,omega],"initial_xz_vxvz":state,
            "initial_roll_rad_s":spin_start,"friction":mu,"damping_ratio":damping_ratio,
            "spring_N_per_m":stiffness,"dt_s":dt,"support_duration_s":panel_hit_time,
            "first_lip_after_s":first_lip,"max_compression_mm":1000*peak_overlap,
            "withdrawal":withdrawal,"clear_witness_from_station_s":clear_clock,
            "ending_still_double_contact_s":still_dwell,"peak_total_energy_gain_J":peak_energy_gain,
            "peak_planar_speed_mps":peak_speed,
            "min_shelf_clearance_mm":1000*min_shelf if math.isfinite(min_shelf) else None,
            "trace":trace,"official_score":None}


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--mu",type=float,default=.35)
    p.add_argument("--damping-ratio",type=float,default=.6)
    p.add_argument("--dt",type=float,default=.0001)
    p.add_argument("--duration",type=float,default=2)
    p.add_argument("--sector",type=float,default=-.18)
    p.add_argument("--withdrawal",choices=["fixed","late","after-separation"],default="fixed")
    p.add_argument("--initial-roll",type=float,default=0)
    p.add_argument("--wait-after-separation-ms",type=float,default=50)
    p.add_argument("--full",action="store_true")
    a=p.parse_args()
    out=continue_contact(mu=a.mu,damping_ratio=a.damping_ratio,dt=a.dt,duration=a.duration,
                         sector=a.sector,withdrawal=a.withdrawal,initial_roll=a.initial_roll,
                         wait_after_true_separation_s=a.wait_after_separation_ms/1000)
    if not a.full:
        out.pop("trace")
    print(json.dumps(out,indent=2,allow_nan=False))
