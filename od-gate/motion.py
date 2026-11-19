"""Low receiver with opposed OD drive and laterally separating bottom supports."""
from dataclasses import dataclass,replace
from pathlib import Path
import math
import sys
ROOT=Path(__file__).resolve().parents[1]
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT))
sys.path.insert(0,str(ROOT/"axial-spin"))
from pipe_axial_contact import ramp
from pipe_spin_route import DATA,station
from scripts import station_l1_mission as mission
def unequal_belt_length(c,r1,r2):
    d=r2-r1
    return 2*math.sqrt(c*c-d*d)+math.pi*(r1+r2)+2*d*math.asin(d/c)
lo,hi=80.,110.
for _ in range(50):
    mid=(lo+hi)/2
    if unequal_belt_length(mid,108/(2*math.pi),150/(2*math.pi))<315:lo=mid
    else:hi=mid
ARM=(lo+hi)/2000


@dataclass(frozen=True)
class Case:
    length: float=.301625
    diameter: float=.1143
    mass: float=.816466266
    normal: float=0.
    lateral: float=0.
    yaw: float=0.
    slice_y: float=0.
    station_yaw: float=0.
    sector: float=-.18
    target_error: float=0.
    mu: float=.55
    damping: float=.38
    head_x: float=0.
    head_z: float=.714
    head_r: float=.0381
    back_x: float=-.054
    back_top: float=.688
    belt_front: float=-.070+.2235*math.cos(math.radians(6))
    belt_rear: float=-.070
    bottom: float=.566
    incline: float=6.
    hood_raise: float=.04
    close_time: float=.4
    close_duration: float=.24
    belt_r: float=.09/(2*math.pi)+.0012+.012
    initial_surface: float=-.38
    acquire_until: float=.78
    release: float=.80
    gate_duration: float=.28
    gate_delay: float=0.
    travel: float=.112
    lag: float=0.
    belt_halfwidth: float=.0075
    center: float=.085
    hold: float=1.2
    withdraw: float=.15
    dt: float=.0005
    finite_drive: bool=True
    ratio: float=(68/13)*(84/29)
    drive_current: float=8.
    inertia: float=3e-6
    surface_mass: float=.40
    tread_stop: float=.009
    stop_rate: float=650000.
    servo_gain: float=130.

def smooth(x):
    x=max(0.,min(1.,x))
    return x*x*(3-2*x)

def gap_segment(x,z,p,q):
    px,pz=p;dx,dz=q[0]-px,q[1]-pz
    f=max(0.,min(1.,((x-px)*dx+(z-pz)*dz)/(dx*dx+dz*dz)))
    dx=x-(px+f*dx);dz=z-(pz+f*dz)
    h=math.hypot(dx,dz)
    return h,dx/max(h,1e-15),dz/max(h,1e-15)

def run(c,trace=False):
    dt=c.dt;r=c.diameter/2
    I=c.mass*(r*r+(r-.00635)**2)/2
    a=ramp(DATA["feed"],.4)
    lip=station.departure(a["speed"])
    xx,zz=station.flight_center(lip,0)
    angle=math.radians(c.yaw)
    x=.444525-xx+c.normal+c.slice_y*math.tan(angle)
    z=zz+(r-.05715)*math.cos(math.radians(35))
    vx=-lip["velocity_x_mps"]*math.cos(angle);vz=lip["velocity_z_mps"];w=a["spin"]
    origin=3+.370-3.49+c.target_error
    target=mission.TARGET
    gate_at=3.5+c.gate_delay
    release_at=3.5
    retract=0.; ww=0.;topv=0.;lowv=0.
    times={};peak={};sample=[];outcome="time-limit";seat=0.;dwell=0.;seated=None
    grounded=0.;lost_support=False;side_margin=1.;last_robot=None;first_touch=None;observed=None
    max_motor=0.;peak_amps=0.;peak_voltage=0.;max_drive_error=0.;max_drive_speed=0.;floor_clear=None;had_separation=False
    for i in range(round(8/dt)):
        t=i*dt
        fl=smooth((t-gate_at)/c.gate_duration)
        fr=smooth((t-gate_at-c.lag)/c.gate_duration)
        left=c.center+c.travel*fl;right=-c.center-c.travel*fr
        # A slice is only supported if it overlaps a length of the finite tube.
        outer=c.length*.5*math.cos(angle)+r*abs(math.sin(angle))
        def fraction(y):
            return max(0.,min(y+c.belt_halfwidth,c.lateral+outer)-max(y-c.belt_halfwidth,c.lateral-outer))/(2*c.belt_halfwidth)
        fractions=(fraction(left),fraction(right))
        weight=sum(fractions)/2
        if min(fractions)<.01 and max(fractions)>.99 and t>3.5:lost_support=True
        if t>gate_at+c.gate_duration+c.lag and weight>0:side_margin=min(side_margin,0.)
        target_speed=c.release if t>=release_at else c.initial_surface if t<c.acquire_until else 0.
        setpoint=target_speed
        if not c.finite_drive:
            lowv+=max(-18*dt,min(18*dt,target_speed-lowv))
        topv=lowv
        drive_load=0.
        contacts=[];robot_dist=[];field_contacts=[]
        if times.get("exit") is not None and t>times["exit"]+.1:retract=min(.370,retract+c.withdraw*dt)
        xshift=-c.withdraw if 0<retract<.370 else 0.
        # Capsule top is bottom datum; actuator blocks translate laterally before departure.
        if weight>1e-7:
            d,nx,nz=gap_segment(x,z,(c.belt_rear-retract,c.bottom-c.belt_r),(c.belt_front-retract,c.bottom-c.belt_r+math.tan(math.radians(c.incline))*(c.belt_front-c.belt_rear)))
            g=d-r-c.belt_r
            robot_dist.append(g)
            if g<0:contacts.append(("belt",g,nx,nz,lowv+xshift*nz,42000*weight,c.damping,c.mu))
        # Two compliant OD rollers share one head shaft. Their centered sections represent total force.
        head_fraction=max(0.,min(1.,(t-c.close_time)/c.close_duration))
        hz=c.head_z+c.hood_raise*(1-smooth(head_fraction))
        hv=-c.hood_raise*6*head_fraction*(1-head_fraction)/c.close_duration
        rise=hz-c.head_z
        hx=c.head_x-ARM+math.sqrt(ARM**2-rise**2)
        hvx=-rise*hv/max(1e-12,hx-c.head_x+ARM)
        d=math.hypot(x-(hx-retract),z-hz);nx=(x-hx+retract)/d;nz=(z-hz)/d
        g=d-r-c.head_r;robot_dist.append(g)
        if g<0:contacts.append(("hood",g,nx,nz,-topv+(xshift+hvx)*nz-hv*nx,5500,c.damping,c.mu))
        d,nx,nz=gap_segment(x,z,(c.back_x-retract,c.bottom+.002),(c.back_x-retract,c.back_top))
        g=d-r;robot_dist.append(g)
        if g<0:contacts.append(("rear",g,nx,nz,xshift*nz,40000,c.damping,.35))
        if t>=release_at:
            tx=x+origin
            f=target["floor_slope"];norm=math.hypot(1,f)
            gap=(z-target["floor_intercept_m"]-f*tx)/norm-r
            if gap<0 and all(mission.panel_support(tx+r*f/norm,c.sector+c.lateral+y) for y in (-.1,.1)):
                contacts.append(("floor",gap,-f/norm,1/norm,0,60000,.60,.35))
            edge=target["lip_depth_m"];height=target["lip_top_m"]
            for p,q in (((edge,.003175),(edge,height)),((edge,height),(0,height)),((0,height),(0,.003175))):
                d,nx,nz=gap_segment(tx,z,p,q)
                if d<r:
                    contacts.append(("lip",d-r,nx,nz,0,60000,.60,.35));break
            if any(c.sector+c.lateral+outer>=y1 and c.sector+c.lateral-outer<=y2 and math.hypot(tx-min(x2,max(x1,tx)),z-min(z2,max(z1,z)))<r
                   for x1,y1,z1,x2,y2,z2 in target["riser_bounds_m"]):
                outcome="inner-riser";break
        ax=1.5 if 1.5<t<2 else -1.5 if 2.25<t<2.75 else 0.
        fx=-c.mass*ax;fz=-c.mass*9.80665;tau=0.;hits=set()
        for name,gap,nx,nz,sv,k,dam,mu in contacts:
            normal=max(0.,-k*gap-2*dam*math.sqrt(c.mass*k)*(vx*nx+vz*nz-(xshift*nx if name in ("rear","hood","belt") else 0)-(hv*nz+hvx*nx if name=="hood" else 0)))
            if name=="belt" and -gap>c.tread_stop:
                normal+=c.stop_rate*weight*(-gap-c.tread_stop)
            if not normal:continue
            slip=vx*nz-vz*nx+w*r-sv
            shear=-mu*normal*max(-1.,min(1.,slip/.022))
            fx+=normal*nx+shear*nz;fz+=normal*nz-shear*nx;tau+=r*shear
            hits.add(name);peak[name]=max(peak.get(name,0),normal)
            if name in ("belt","hood"):
                max_motor=max(max_motor,abs(shear))
                drive_load+=shear*(1 if name=="belt" else -1)
        if c.finite_drive:
            # Power-limited DC motor and reflected roller inertia share the impact impulse.
            # Effective drag/loss and speed-loop gain are uncalibrated project assumptions.
            radius=c.belt_r
            shaft_speed=lowv*c.ratio/radius
            emf=shaft_speed/(11000*2*math.pi/60)*(12-.12*1.40)
            demand=c.servo_gain*(setpoint-lowv)
            torque=demand*radius/(c.ratio*.78)
            amps=max(-c.drive_current,min(c.drive_current,torque*(100-1.4)/.97))
            amps=max((-9.5-emf)/.12,min((9.5-emf)/.12,amps))
            amps=max(-c.drive_current,min(c.drive_current,amps))
            force=amps*.97/(100-1.4)*c.ratio*.78/radius
            drag=.4+1.4*.97/(100-1.4)*c.ratio*.78/radius
            lowv+=dt*(force-drive_load-drag*math.tanh(lowv/.005))/(c.surface_mass+c.inertia*(c.ratio/radius)**2)
            peak_amps=max(peak_amps,abs(amps))
            peak_voltage=max(peak_voltage,abs(emf+amps*.12))
            max_drive_speed=max(max_drive_speed,abs(lowv))
            max_drive_error=max(max_drive_error,abs(setpoint-lowv))

        if hits & {"belt","hood","rear"}:
            if had_separation:outcome="robot-recontact";break
            if first_touch is None:first_touch=t
            last_robot=t
        if t>=release_at and last_robot is not None and min(robot_dist)>.001 and x>.04 and not had_separation:
            had_separation=True;times["exit"]=t
        if abs(x-(c.back_x+r))<.04 and abs(vx)<.04 and abs(vz)<.04 and abs(w*r)<.06 and {"belt","hood"}<=hits:
            seat+=dt
            if seat>=.06 and seated is None:seated=t
        else:seat=0.
        if .95<t<3.45 and not {"belt","hood"}<=hits:grounded+=dt
        if t>release_at and {"lip","floor"}<=hits and math.hypot(vx,vz)<.015 and not hits & {"belt","hood","rear"}:
            dwell+=dt
            if dwell>=c.hold:outcome="sustained-paired-target";break
        else:dwell=0.
        if "floor" in hits:times.setdefault("floor",t)
        if t<release_at and (x<-.16 or x>.30 and t>.4 or z<.43):
            outcome="lost-before-carry";break
        if t>release_at and (x>.7 or z<.26 or x<-.17):
            outcome="target-miss";break
        if trace and i%max(1,round(.002/dt))==0:sample.append([t,x,z,vx,vz,w,left,right,retract])
        vx+=dt*fx/c.mass;vz+=dt*fz/c.mass;w+=dt*tau/I
        x+=dt*vx;z+=dt*vz
        if abs(t-3.4)<dt/2:observed=[x,z,vx,vz,w]
    return {"outcome":outcome,"firstContactS":first_touch,"seatedS":seated,"eventsS":times,"endS":t,
            "preRelease":observed,"peakNormalN":peak,"peakSingleSurfaceTractionN":max_motor,"peakDrivePhaseA":peak_amps,"peakTerminalV":peak_voltage,"peakDriveSpeedMps":max_drive_speed,"maxTrackingErrorMps":max_drive_error,
            "incompleteNipDuringCarryS":grounded,"oneSidedSupport":lost_support,
            "terminal":[x,z,vx,vz,w],"trace":sample}
