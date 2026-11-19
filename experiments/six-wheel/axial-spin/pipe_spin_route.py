"""Transport tube rotation and translation from one release through the selected two registered routes."""
import argparse
import json
import math
import sys
from dataclasses import asdict,replace
from pathlib import Path

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
sys.path.insert(0,str(ROOT/"sourced-match"))
from replay_source_match import corrected,mission,settle,SECTORS,digest
from scripts import receiver_contact as station
from mission import short_vision_clock as vision
from pipe_axial_contact import AxialContact,Contact,G,I,K,M,R,clip,ramp

SOURCE=ROOT/"sourced-match/source-545.json"
assert digest(SOURCE)=="25a342eec25913ea422a734e649c4b55f25b591c432a2350d17579877e617796"
DATA=json.loads(SOURCE.read_bytes())
SHELF=station.SHELF_TOP+R
U_ORIGIN=.434525+.010-.240


def acquire(cfg,dt):
    initial=ramp(DATA["feed"],cfg.ramp_mu)
    departure=station.departure(initial["speed"])
    if departure["mode"]!="immediate-separation":
        raise ValueError("The tube remains supported around the lip")
    source=AxialContact(cfg,initial["spin"])
    source.enter_phase("free-flight",0)
    x,z=station.flight_center(departure,0)
    u=U_ORIGIN-x
    speed=-departure["velocity_x_mps"]
    vertical=departure["velocity_z_mps"]
    t=0.
    impact=None
    path=[]
    first_belt=None
    min_u=1.
    max_u=-1.
    while t<2:
        step=min(dt,impact["time_s"]+1.1-t) if impact else dt
        if step<1e-11:break
        supported=impact is not None
        if not supported and vertical<0:
            # Resolve the first top-plane interception within the next nominal step.
            step=min(step,max(1e-10,(z-SHELF)/(-vertical)))
        n_a,n_b,mu,limit,_=source.normals(u,0,0,"seat",t,z-SHELF)
        request=M*(100*(.080-u)-20*speed)/2
        if not supported:
            # Both belts are stationary while waiting for initial shelf contact.
            request=0
            limit=0
        fx, floor, vertical_belt=source.forces(speed,vertical,(n_a,n_b),(request,request),limit,step,M*G if supported else None)
        acc=(sum(fx)+floor)/M
        az=0. if supported else vertical_belt/M-G
        path.append((t,u,z,source.spin))
        if first_belt is None and n_a+n_b>0:first_belt=t
        u+=speed*step+.5*acc*step**2
        speed+=acc*step
        z+=vertical*step+.5*az*step**2
        vertical+=az*step
        t+=step
        if not supported and z<=SHELF+1e-10:
            if not -.1 <= u <=.159:
                return {"status":"flat-touchdown-outside-receiver","u":u,"ramp":initial},None,path
            impulse=clip(-(speed+source.spin*R)/(1/M+R*R/I),cfg.shelf_mu*M*max(0,-vertical))
            impact={"time_s":t,"local_u_m":u,"spin_before_rad_s":source.spin,"vx_before_mps":speed,
                    "vz_before_mps":vertical,"floor_tangent_impulse_Ns":impulse}
            speed+=impulse/M
            source.spin+=R*impulse/I
            source.angular_impulse+=R*impulse
            source.enter_phase("capture",t)
            z=SHELF
            vertical=0.
        if impact:
            min_u=min(u,min_u)
            max_u=max(u,max_u)
            if not -.1<=u<=.165:break
    confirmed=bool(impact and abs(t-impact["time_s"]-1.1)<1e-8 and abs(u-.08)<=.008 and abs(speed)<.04 and not source.lifted)
    return {"status":"seated" if confirmed else "hold-unconfirmed-seat","ramp":initial,"first_belt_after_lip_s":first_belt,
            "impact":impact,"seat_elapsed_from_lip_s":t,"seat_u_m":u,"seat_speed_mps":speed,"seat_spin_rad_s":source.spin,
            "capture_travel_m":[min_u,max_u],"minimum_shelf_normal_N":source.min_shelf_normal},source,path


def trial(cfg,piece,dt=.0005,post_dt=.0002,through_end=False):
    key=piece["id"];sector=SECTORS[key]
    pick,contact,path=acquire(cfg,dt)
    result={"id":key,"settings":asdict(cfg),"dt_s":dt,"pickup":pick}
    if pick["status"]!="seated":
        result["status"]=pick["status"]
        return result,None,path
    route=corrected().RegisteredMission(sector)
    _,cx,cy,yaw,top,stroke=route.pose(1.1)
    xlocal=pick["seat_u_m"]+stroke
    vx=pick["seat_speed_mps"]
    lip=piece["human_release_teleop_s"]+DATA["feed"]["free_drop_s"]+pick["ramp"]["time"]
    state=dict(piece_id=key,status="within-modeled-seat-window",mass_kg=M,seat_elapsed_s=1.1,
               first_contact_from_lip_s=pick["impact"]["time_s"],xyz_m=(cx+math.cos(yaw)*xlocal,cy+math.sin(yaw)*xlocal,top+R),
               vxyz_mps=(math.cos(yaw)*vx,math.sin(yaw)*vx,0),yaw_rad=yaw,yaw_rate_rad_s=0,pitch_rad=0)
    contact.previous=None
    run=mission.resume_from_seat(state,dt,route,contact.normals,sector,key,axial_contact=contact)
    result["release_trace"]=[r for r in contact.samples if r[0]-contact.start<.4]
    from size_drives import motor,R_GRIP,UP5
    result["belt_motor_envelope"]=motor(contact.max_belt_load+.4,contact.max_belt_speed,R_GRIP,UP5,.80,
                                       contact.max_belt_accel,3e-6,current_limit=8)
    result.update(status=run["outcome"],phase_entry_spin_rad_s=contact.phases,release_spin_rad_s=contact.spin,
                  min_shelf_normal_N=contact.min_shelf_normal,force_fraction=contact.max_force_fraction,
                  dissipated_J=contact.dissipation,motor_work_J=contact.belt_work,
                  angular_momentum_residual_Nms=I*(contact.spin-pick["ramp"]["spin"])-contact.angular_impulse)
    if contact.lifted:result["status"]="unilateral-shelf-liftoff";return result,run,path
    if run["outcome"]!="paired-trough-first-contact":return result,run,path
    # At the red reef the outward chassis X points along registered +target X.
    p=route.target_velocity(math.cos(run["trace"][-1]["yaw_rad"]),math.sin(run["trace"][-1]["yaw_rad"]))
    if abs(p[0]-1)>.0001 or abs(p[1])>.001:raise ValueError("Positive axial sign transform changed")
    start=run["trace"][0]["time_s"]
    camera=dict(run,trace=[r for r in run["trace"] if abs((r["time_s"]-start)*1000-round((r["time_s"]-start)*1000))<.0001])
    schedules,_=vision.schedule(camera,537.)
    if "0.1" not in schedules:
        result["status"]="observer-clock-withheld";return result,run,path
    eligible=schedules["0.1"]["0"]["exposures"]
    if len(eligible)!=17 or any(not e["within_150ms"] for e in eligible):
        result["status"]="observer-clock-withheld"
        return result,run,path
    delay=max(e["onset_after_shelf_ms"][1] for e in eligible)/1000
    kwargs=dict(sector=sector,piece_id=key,upstream=run,withdrawal="after-separation",initial_roll=contact.spin,
                dt=post_dt,inertia_ratio=K,wait_after_true_separation_s=delay)
    result["lip_teleop_s"]=lip
    post=settle.continue_contact(**kwargs)
    result["target"]={"stop":post["stop"],"impact_teleop_s":lip+post["start_from_station_s"],
            "end_teleop_s":lip+post["start_from_station_s"]+post["elapsed_s"],"contact_input":post["initial_xz_vxvz"],
            "input_spin_rad_s":post["initial_roll_rad_s"],"delay_after_shelf_ms":1000*delay,
            "min_shelf_gap_mm":post["min_shelf_clearance_mm"],"final":post["last_xz_vxvz_spin"]}
    result["status"]=post["stop"]
    if through_end and result["status"]=="sustained-paired-floor-and-lip":
        end=settle.continue_contact(**kwargs,duration=138-result["target"]["impact_teleop_s"])
        result["target"]["buzzer_assessment"]={"at_teleop_s":138,"status":end["stop"],"still_double_contact_s":end["ending_still_double_contact_s"]}
    return result,run,path


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case",default="nominal")
    parser.add_argument("--step",type=float,default=.0005)
    args=parser.parse_args()
    cfg=Contact()
    cases={"nominal":cfg,"low-ramp":replace(cfg,ramp_mu=.15),"low-grip":replace(cfg,belt_mu=.08,preload=2),
           "slick-shelf":replace(cfg,shelf_mu=.02),"high-shelf":replace(cfg,shelf_mu=.35)}
    rows=[trial(cases[args.case],piece,args.step)[0] for piece in DATA["identified_transfers"]]
    print(json.dumps(rows,indent=2,allow_nan=False))
