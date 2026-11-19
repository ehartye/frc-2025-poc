"""Body-supported cradle study. Robot longitudinal/vertical cross-section in metres."""
from dataclasses import dataclass
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "axial-spin"))
from scripts import receiver_contact as station
from scripts import station_l1_mission as mission

G = 9.80665
SOURCE = json.loads((ROOT/"sourced-match/source-545.json").read_bytes())["feed"]
TARGET = mission.TARGET
PIVOT = (.142, .018)
# Closed rear wall, two faces of the pocket and open front edge.
V_PROFILE = ((-.090,.095),(-.090,.040),(.018,.004),(.155,.038))
BACK_PROFILE = ((-.090,.110),(-.090,.004),(.155,.048))
FLAT_PROFILE = ((-.100,.004),(.165,.004))


@dataclass(frozen=True)
class Case:
    length: float = .301625
    diameter: float = .1143
    mass: float = .816466266
    wall: float = .00635
    normal: float = 0.
    lateral: float = 0.
    yaw: float = 0.
    rail_lean: float = 0.
    rail_height: float = 0.
    rail_offset_y: float = 0.
    dump_angle: float = 27.
    rise_time: float = .35
    ramp_mu: float = .40
    rail_mu: float = .35
    rolling: float = .10
    station_pipe_yaw: float = 0.
    damping: float = .45
    acceleration: float = 1.5
    target_gap: float = 0.
    withdrawal: float = 0.
    dt: float = .0005
    sector: float = -.18
    face: str = "V"
    rail_slice: float = 0.


def rotate(point,angle):
    dx,dz=point[0]-PIVOT[0],point[1]-PIVOT[1]
    c,s=math.cos(angle),math.sin(angle)
    return PIVOT[0]+c*dx+s*dz, PIVOT[1]-s*dx+c*dz


def segment(x,z,a,b):
    dx,dz=b[0]-a[0],b[1]-a[1]
    f=max(0.,min(1.,((x-a[0])*dx+(z-a[1])*dz)/(dx*dx+dz*dz)))
    qx,qz=a[0]+f*dx,a[1]+f*dz
    d=math.hypot(x-qx,z-qz)
    return d,(x-qx)/max(1e-12,d),(z-qz)/max(1e-12,d),qx,qz


def support_margin(case):
    # Chassis axes, independent signs. Include pipe silhouette and mounting uncertainty.
    y=abs(case.lateral)+abs(case.rail_offset_y)
    a=abs(math.radians(case.yaw))
    cfg_yaw=case.station_pipe_yaw
    half=case.length/2
    r=case.diameter/2
    end=y+half*math.cos(a)+r*math.sin(a)
    inner=half*math.cos(a)-r*math.sin(a)-y
    return {"sideGuardMm":1000*(.180-end),"bothRailCoverageMm":1000*(inner-.100),
            "nativeChuteSlotHalfMm":1000*(.12199054775091216/2-r*math.cos(math.radians(cfg_yaw))-half*abs(math.sin(math.radians(cfg_yaw))))}


def run(case, trace=False):
    cfg=case
    r=cfg.diameter/2
    ratio=.5*(1+((r-cfg.wall)/r)**2)
    inertia=ratio*cfg.mass*r*r
    # Reuse the issued ramp calculation at nominal radial inertia; integrate its equations for
    # each supplied diameter/wall. The tube rolls without slipping once tangential slip is zero.
    angle=math.radians(35)
    drop=SOURCE["fall_speed_mps"]
    v=drop*math.sin(angle)
    impulse=min(cfg.ramp_mu*drop*math.cos(angle),v*ratio/(1+ratio))
    v-=impulse; rim=impulse/ratio
    a=G*(math.sin(angle)-cfg.ramp_mu*math.cos(angle))
    angular=cfg.ramp_mu*G*math.cos(angle)/ratio
    slip=v-rim
    to_roll=0. if abs(slip)<1e-10 and cfg.ramp_mu>=ratio/(1+ratio)*math.tan(angle) else max(0.,slip/(angular-a)) if angular>a else math.inf
    distance=SOURCE["run_m"]
    slide_dist=v*to_roll+.5*a*to_roll**2 if math.isfinite(to_roll) else math.inf
    if slide_dist>=distance:
        ts=2*distance/(v+math.sqrt(v*v+2*a*distance))
        entry={"speed":v+a*ts,"spin":(rim+angular*ts)/r}
    else:
        speed=math.sqrt((v+a*to_roll)**2+2*G*math.sin(angle)/(1+ratio)*(distance-slide_dist))
        entry={"speed":speed,"spin":speed/r}
    lip=station.departure(entry["speed"])
    if lip["mode"]!="immediate-separation": raise ValueError("Unhandled station lip contact")
    sx,sz=station.flight_center(lip,0)
    # The published lip state uses the nominal radius. Shift its center from the lip
    # normal to the actual radius before starting a perturbed free flight.
    n=math.radians(lip["separation_angle_deg_from_up"])
    sx+=(r-station.RADIUS_M)*math.sin(n)
    sz+=(r-station.RADIUS_M)*math.cos(n)
    yaw=math.radians(cfg.yaw)
    landing=.09852895414329949
    x=.204525-landing+(landing-sx+cfg.normal)*math.cos(yaw)-cfg.lateral*math.sin(yaw)
    # Independent body slices bracket the different fore/aft arrivals at the two longitudinal rails.
    x+=cfg.rail_slice*math.tan(yaw)
    z=sz-.88
    vx=-lip["velocity_x_mps"]*math.cos(yaw)
    vz=lip["velocity_z_mps"]
    omega=entry["spin"]
    profile={"V":V_PROFILE,"flat":FLAT_PROFILE,"backstop":BACK_PROFILE}[cfg.face]
    seat_x=-.090+r if cfg.face=="backstop" else .020
    t=0.
    dt=cfg.dt
    first=None; received=None; carry=None; exit_state=None; first_floor=None
    last_touch=None; peak=0.; max_pen=0.; dwell=0.; log=[]; min_x=x
    shelf_clear_since=None; retract=0.; max_torque=0.; max_speed=0.; peak_impulse=0.; capture_range=[0.,0.]
    seat_dwell=0.; seat_at_deadline=None; first_settled=None; dump_torque=0.; lowest_clearance=None
    gx=mission.M["chassisAtTargetX_m"]+.370-mission.M["targetLipX_m"]+cfg.target_gap
    stop="time-limit"
    for i in range(round(8/dt)):
        t=i*dt
        if received is None and t>=1.5:
            received=(x,z,vx,vz,omega)
        if carry is None and t>=3.5:carry=(x,z,vx,vz,omega)
        a0=math.radians(cfg.rail_lean)
        fraction=min(1.,max(0.,(t-3.5)/cfg.rise_time))
        # Smoothstep with zero angular velocity at both ends.
        a=a0+math.radians(cfg.dump_angle)*(3*fraction**2-2*fraction**3)
        adot=math.radians(cfg.dump_angle)*6*fraction*(1-fraction)/cfg.rise_time
        max_speed=max(max_speed,adot)
        if shelf_clear_since is not None and t>=shelf_clear_since+.10:
            retract=min(.370,retract+cfg.withdrawal*dt)
        shift_v=-cfg.withdrawal if 0<retract<.370 else 0.
        pts=[rotate(q,a) for q in profile]
        pts=[(px-retract,pz+cfg.rail_height) for px,pz in pts]
        # Transport test in the accelerating cradle frame; rectangular longitudinal acceleration pulses.
        ax=cfg.acceleration if 1.5<t<1.95 else -cfg.acceleration if 2.05<t<2.50 else 0.
        fx,fz,tau=-cfg.mass*ax,-cfg.mass*G,0.
        hit=[]
        distances=[]
        cradle_torque=0.
        contacts=[]
        for j,(p,q) in enumerate(zip(pts,pts[1:])):
            d,nx,nz,qx,qz=segment(x,z,p,q)
            distances.append(d-r)
            if d<r:
                svx=adot*(qz-PIVOT[1])+shift_v
                svz=-adot*(qx+retract-PIVOT[0])
                contacts.append(("rail-"+str(j),d-r,nx,nz,svx,svz,qx,qz))
        # The fixed front bearing posts span two finite strips within the tube width.
        # Guard against spindle/post contact on the way into and out of the pocket.
        support_top=(.133-retract,.028+cfg.rail_height),(.151-retract,.028+cfg.rail_height)
        d,nx,nz,qx,qz=segment(x,z,*support_top)
        distances.append(d-r)
        if d<r:
            contacts.append(("bearing-post",d-r,nx,nz,shift_v,0.,qx,qz))
        if t>=3.5:
            cx,cz=x+gx,z+.5
            slope=TARGET["floor_slope"]
            sn=math.hypot(1,slope)
            floor_gap=(cz-TARGET["floor_intercept_m"]-slope*cx)/sn-r
            if floor_gap<=0:
                foot=cx+r*slope/sn
                support=all(mission.panel_support(foot,cfg.sector+dy) for dy in (-.100,.100))
                if support:
                    contacts.append(("target-floor",floor_gap,-slope/sn,1/sn,0.,0.,0.,0.))
                    if first_floor is None:first_floor=(t,cx,cz,vx,vz,omega)
            # Vertical and top edges of the trough lip, with all distances to finite segments.
            left,right,top=0.,TARGET["lip_depth_m"],TARGET["lip_top_m"]
            for p,q in (((right,.003175),(right,top)),((right,top),(left,top)),((left,top),(left,.003175))):
                d,nx,nz,_,_=segment(cx,cz,p,q)
                if d<r:
                    contacts.append(("target-lip",d-r,nx,nz,0.,0.,0.,0.))
                    break
            for x1,y1,z1,x2,y2,z2 in TARGET["riser_bounds_m"]:
                if cfg.sector+cfg.length/2<y1 or cfg.sector-cfg.length/2>y2:continue
                if math.hypot(cx-min(x2,max(x1,cx)),cz-min(z2,max(z1,cz)))<=r:
                    stop="inner-riser-envelope";break
            if stop=="inner-riser-envelope":break
        # Existing flat base shelf with a 6 mm rounded front, still present after removing the grips.
        x_edge=.165-retract
        base_pts=[(-.1-retract,0.),(x_edge-.006,0.)]
        for j in range(1,13):
            alpha=math.pi/2*(1-j/12)
            base_pts.append((x_edge-.006+.006*math.cos(alpha),-.006+.006*math.sin(alpha)))
        base_pts.append((x_edge,-.015))
        for p,q in zip(base_pts,base_pts[1:]):
            d,nx,nz,qx,qz=segment(x,z,p,q)
            distances.append(d-r)
            if d<r:
                contacts.append(("base-shelf",d-r,nx,nz,shift_v,0.,qx,qz))
        # The two physical rails share the total normal in this symmetric section. Merge duplicate
        # normal contacts at a common polyline corner so one physical corner has one spring.
        normals=[]
        for name,gap,nx,nz,svx,svz,qx,qz in contacts:
            if any(name.startswith("rail") and old.startswith("rail") and nx*ox+nz*oz>.999999 for old,ox,oz in normals):continue
            normals.append((name,nx,nz))
            vn=(vx-svx)*nx+(vz-svz)*nz
            stiff=60000. if name.startswith("target") else 40000.
            damping=.6 if name.startswith("target") else cfg.damping
            normal=max(0.,-stiff*gap-2*damping*math.sqrt(stiff*cfg.mass)*vn)
            if normal<=0: continue
            slip=(vx-svx)*nz-(vz-svz)*nx+omega*r
            mu=.35 if name.startswith("target") else cfg.rail_mu
            tangent=-mu*normal*max(-1.,min(1.,slip/.015))
            cx,cz=normal*nx+tangent*nz,normal*nz-tangent*nx
            fx+=cx; fz+=cz; tau+=r*tangent
            if name.startswith("rail"):
                tau-=cfg.rolling*r*normal*max(-1.,min(1.,(omega+adot)/.15))
            if name.startswith("rail"):
                cradle_torque+=(qx+retract-PIVOT[0])*cz-(qz-PIVOT[1])*cx
            peak=max(peak,normal);max_pen=max(max_pen,-gap);hit.append(name)
        max_torque=max(max_torque,abs(cradle_torque))
        if t<1.5:
            capture_range=[min(capture_range[0],cradle_torque),max(capture_range[1],cradle_torque)]
        if t>=3.5:dump_torque=max(dump_torque,abs(cradle_torque))
        peak_impulse=max(peak_impulse,abs(cradle_torque) if t<.5 else 0.)
        if any(h.startswith("rail") or h in ("base-shelf","bearing-post") for h in hit):
            if first is None:first=(t,x,z,vx,vz,omega)
            if shelf_clear_since is not None:stop="recontact-after-cradle-clear";break
            last_touch=t
        seat_now=abs(x-seat_x)<.010 and math.hypot(vx,vz)<.04 and abs(omega*r)<.04 and any(h.startswith("rail") for h in hit)
        if first is not None and received is None:
            seat_dwell=seat_dwell+dt if seat_now else 0.
            if seat_dwell>=.06 and first_settled is None:first_settled=t
            if seat_at_deadline is None and t>=first[0]+1.1:seat_at_deadline=seat_dwell>=.06
        if t>=3.5 and shelf_clear_since is None and x>.155-retract and min(distances)>.001:
            shelf_clear_since=t;exit_state=(t,x,z,vx,vz,omega)
        if set(hit)>={"target-floor","target-lip"} and not any(k.startswith("rail") for k in hit) and math.hypot(vx,vz)<.015:
            dwell+=dt
            if dwell>=1.2:stop="sustained-paired-target";break
        else:dwell=0.
        if t<3.5 and (x<-.16 or x>.24 or z<-.02):stop="escaped-before-dump";break
        if t>=3.5 and (z<-.45 or x>.75):stop="missed-target";break
        if trace and i%max(1,round(.005/dt))==0:log.append([t,x,z,vx,vz,omega,a,retract])
        vx+=fx/cfg.mass*dt; vz+=fz/cfg.mass*dt; omega+=tau/inertia*dt
        x+=vx*dt; z+=vz*dt; min_x=min(min_x,x)
    return {"outcome":stop,"firstContact":first,"received":received,"afterCarry":carry,"exit":exit_state,
            "firstFloor":first_floor,"seatAtExistingTime":seat_at_deadline,"firstSeatAfterLipS":first_settled,"dumpTorqueNm":dump_torque,"endTimeS":t,"minXm":min_x,"peakNormalN":peak,"maxPenetrationMm":1000*max_pen,
            "captureTorqueRangeNm":capture_range,"pivotPeakNm":max_torque,"capturePeakNm":peak_impulse,"peakPivotRadS":max_speed,
            "terminal":[x,z,vx,vz,omega],"support":support_margin(cfg),"trace":log}
