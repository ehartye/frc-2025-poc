"""Offline incoming spin and offset capture in the receiver frame. SI units internally."""
import argparse
import hashlib
import itertools
import json
import math
from dataclasses import replace
from pathlib import Path
import sys
HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
sys.path.insert(0,str(ROOT/"axial-spin"))
from pipe_axial_contact import Contact, G, HALF, I, K, M, R, RI, clip, patches, ramp
from pipe_spin_route import DATA, SHELF, acquire, station

CANDIDATE=Contact(drive="velocity-command",release_limit=.90)
INERTIA_YAW = M*((2*HALF)**2/12+(R*R+RI*RI)/4)
ORIGIN = .434525+.040-.270  # The same receiver-local landing X as the 10 mm / 240 mm case.
LANDING = json.loads((ROOT/"station-supply/compensated-dock.json").read_bytes())["free_flight"]["landing_station_x_mm"]/1000


def patch_load(u,y,z,yaw,vy,omega,side,cfg,reach,gap_mm):
    """First-order transverse cap projection on exact annular strips, at the current lateral reach."""
    sn,cs=math.sin(yaw),math.cos(yaw)
    # Unrotate the finite straight belt interval at each cap. The small-yaw model treats the
    # belt width in longitudinal projection as nominal. Record and bound this error separately.
    points=patches(u-side*HALF*sn,z-SHELF,cfg)
    loaded=[]
    for xr,zr,area in points:
        relx=xr*cs-side*HALF*sn
        rely=xr*sn+side*HALF*cs
        displacement=1000*(side*(y+rely)-HALF)-gap_mm
        if displacement < -reach:
            continue
        rate=side*(vy+omega*relx)
        pressure=max(0,cfg.preload+2*displacement+37.52*rate)
        loaded.append((relx,rely,xr,zr,area,displacement,pressure))
    area=sum(p[2] for p in points)
    denom=max(cfg.area_for_preload,area)
    return [(x,y,radial_x,radial_z,area/denom*pressure,travel)
            for x,y,radial_x,radial_z,area,travel,pressure in loaded]


def simulate(dx,dy,heading,dt=.0005,cfg=CANDIDATE,reach=.40,outward=1.40, path=None,gap_mm=0.):
    initial=ramp(DATA["feed"],cfg.ramp_mu)
    lip=station.departure(initial["speed"])
    if lip["mode"]!="immediate-separation": raise ValueError("Source remains on lip")
    x,z=station.flight_center(lip,0)
    yaw=math.radians(heading)
    # Heading is the incoming pipe yaw relative to the robot. The chassis station yaw
    # has the opposite sign. Rotate BOTH independent station-frame errors at the landing.
    longitudinal=LANDING-x+dx/1000
    u=(ORIGIN-LANDING)+longitudinal*math.cos(yaw)-dy/1000*math.sin(yaw)
    y=longitudinal*math.sin(yaw)+dy/1000*math.cos(yaw)
    vx=-lip["velocity_x_mps"]*math.cos(yaw)
    vy=-lip["velocity_x_mps"]*math.sin(yaw)
    vz=lip["velocity_z_mps"]
    spin=initial["spin"]
    omega=0.
    belts=[0.,0.]
    t=0.
    touchdown=None
    first=[None,None]
    peak=0.
    max_yaw=abs(yaw); max_side=abs(y); min_u=u; max_force=0.; max_speed=0.
    state="hold-unconfirmed-seat"
    travel_exceeded=False
    shelf_lifted=False
    while t<2:
        step=min(dt,touchdown["timeS"]+1.1-t) if touchdown else dt
        if step<1e-10:break
        if touchdown is None and vz<0:
            step=min(step,max(1e-10,(z-SHELF)/(-vz)))
        ax_force=fy=fz=yaw_moment=axial_moment=0.
        torque=INERTIA_YAW*(-100*yaw-20*omega)
        for index,side in enumerate((-1,1)):
            nodes=patch_load(u,y,z,yaw,vy,omega,side,cfg,reach,gap_mm)
            normal=sum(p[4] for p in nodes)
            capacity=cfg.belt_mu*normal
            if normal>0 and first[index] is None:first[index]=t
            request=(M*(100*(.080-u)-20*vx)-side*torque/HALF)/2 if touchdown else 0.
            increment=2*request/(M*20.) if cfg.drive=="velocity-command" else cfg.slip_scale*request/capacity if capacity else 0.
            desired=clip(vx+increment,1.05) if touchdown and capacity else 0.
            belts[index]+=clip(desired-belts[index],cfg.belt_accel*step)
            max_speed=max(max_speed,abs(belts[index]))
            individual_fx=0.
            for rx,ry,xr,zr,n,position in nodes:
                peak=max(peak,position)
                travel_exceeded |= position>outward
                slip_x=vx-omega*ry-spin*zr*math.cos(yaw)-belts[index]
                slip_z=vz+spin*xr
                friction=cfg.belt_mu*n/max(cfg.slip_scale,math.hypot(slip_x,slip_z))
                fx,vertical=-slip_x*friction,-slip_z*friction
                normal_y=-side*n
                ax_force+=fx; individual_fx+=fx
                fy+=normal_y; fz+=vertical
                yaw_moment+=rx*normal_y-ry*fx
                axial_moment+=xr*vertical-zr*(fx*math.cos(yaw)+normal_y*math.sin(yaw))
            max_force=max(max_force,abs(individual_fx))
        az=fz/M-G
        if touchdown:
            support=M*G-fz
            if support<0:shelf_lifted=True
            slip_x=vx+spin*R*math.cos(yaw)
            slip_y=vy+spin*R*math.sin(yaw)
            scale=cfg.shelf_mu*max(0,support)/max(cfg.slip_scale,math.hypot(slip_x,slip_y))
            floor_x,floor_y=-scale*slip_x,-scale*slip_y
            ax_force+=floor_x; fy+=floor_y
            axial_moment+=R*(floor_x*math.cos(yaw)+floor_y*math.sin(yaw))
            az=0.
        u+=vx*step+.5*ax_force/M*step**2
        y+=vy*step+.5*fy/M*step**2
        yaw+=omega*step+.5*yaw_moment/INERTIA_YAW*step**2
        z+=vz*step+.5*az*step**2
        vx+=ax_force/M*step; vy+=fy/M*step
        omega+=yaw_moment/INERTIA_YAW*step; spin+=axial_moment/I*step
        vz+=az*step; t+=step
        if path is not None: path.append((t,1000*(u+.270),1000*y,1000*z,math.degrees(yaw)))
        if touchdown is None and z<=SHELF+1e-10:
            if not -.1<=u<=.159:
                state="miss-flat-shelf";break
            # Coulomb-capped coupled proximal impact along rolling and transverse directions.
            sn,cs=math.sin(yaw),math.cos(yaw)
            roll_impulse=-(vx*cs+vy*sn+spin*R)/(1/M+R*R/I)
            cross_impulse=-M*(-vx*sn+vy*cs)
            scale=min(1.,cfg.shelf_mu*M*max(0,-vz)/max(1e-16,math.hypot(roll_impulse,cross_impulse)))
            j,k=scale*roll_impulse,scale*cross_impulse
            touchdown={"timeS":t,"uMm":1000*u,"yMm":1000*y,"incomingSpinRadS":spin,"incomingVerticalMps":vz}
            vx+=(j*cs-k*sn)/M; vy+=(j*sn+k*cs)/M; spin+=j*R/I; z=SHELF; vz=0.
        min_u=min(min_u,u); max_yaw=max(max_yaw,abs(yaw)); max_side=max(max_side,abs(y))
        if touchdown and not -.100<=u<=.165:
            state="lost-shelf-edge";break
    if touchdown and abs(t-touchdown["timeS"]-1.1)<1e-7 and abs(u-.08)<=.008 and math.hypot(vx,vy)<.04:
        state="conditional-seat"
    if shelf_lifted:state="hold-unloaded-shelf"
    if travel_exceeded:state="hold-outboard-stop"
    return {"dxMm":dx,"dyMm":dy,"headingDeg":heading,"dtS":dt,"status":state,"touchdown":touchdown,
        "firstContactS":first,"largestRequiredOutwardMm":peak,"minimumUMm":1000*min_u,
        "largestSideMm":1000*max_side,"largestYawDeg":math.degrees(max_yaw),"terminalUMm":1000*u,
        "terminalSideMm":1000*y,"terminalYawDeg":math.degrees(yaw),"terminalXYSpeedMps":math.hypot(vx,vy),
        "terminalSpinRadS":spin,"terminalYawRateRadS":omega,"maxBeltForceN":max_force,"maxBeltSpeedMps":max_speed}


def run():
    cfg=CANDIDATE
    cases={}
    for lateral,yaw in ((.5,.25),(.75,.25),(.80,.25),(.90,.20),(1.,.15),(1.15,.1),(1.25,.15),(2,.5)):
        cases[f"{lateral}/{yaw}"]=[simulate(x,y,a) for x,y,a in itertools.product((-5.,5.),(-lateral,lateral),(-yaw,yaw))]
    controls={name:simulate(5,.8,.25,**kw) for name,kw in (("fine",dict(dt=.00025)),("coarse",dict(dt=.001)),
        ("lowFrictionPreload",dict(cfg=replace(cfg,belt_mu=.08,preload=2.))),
        ("blockedOutward",dict(outward=.3)),("forceFollowing",dict(cfg=Contact())))}
    spring_cases={}
    for key,(side,angle) in {"0.8/0.25":(.8,.25),"1.0/0.15":(1.,.15)}.items():
        spring_cases[key]=[simulate(x,y,h,cfg=replace(cfg,preload=11.5)) for x,y,h in itertools.product((-5,5),(-side,side),(-angle,angle))]
    for preload in (10.5,11,11.5,12):
        controls["preload-"+str(preload)]=simulate(5,.8,.25,cfg=replace(cfg,preload=preload))
    controls["largeMiss"]=simulate(30,5,3)
    controls["widenBothUnchangedBelts1mm"]=simulate(0,0,0,gap_mm=1.)
    controls["lowRampFriction"]=simulate(5,.8,.25,cfg=replace(cfg,ramp_mu=.15))
    reference=acquire(cfg,.0005)[0]
    return {"schema":"station-offset-capture/1","sourceSha256":{p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest()
        for p in (ROOT/"axial-spin/pipe_axial_contact.py",ROOT/"axial-spin/pipe_spin_route.py",ROOT/"sourced-match/source-545.json",
                  ROOT/"station-supply/entry-window.json",HERE/"geometry.json")},
        "centered":simulate(0,0,0),"issuedCentered":reference,"corners":cases,"shimmedCorners":spring_cases,"controls":controls,
        "physicalCapture":False}


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument("--output",type=Path,required=True)
    args=parser.parse_args()
    args.output.write_text(json.dumps(run(),indent=2,allow_nan=False)+"\n",encoding="utf-8",newline="\n")
