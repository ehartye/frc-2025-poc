"""Single identified CORAL through a conditional station intake, carry and bounded target approach."""
import argparse
import json
import math
from pathlib import Path

if __package__:
    from . import station_receiver_impact as rx
else:
    import station_receiver_impact as rx

ROOT = Path(__file__).resolve().parents[1]
CONFIG = json.loads((ROOT/"mission/candidate.json").read_text())
TARGET = json.loads((ROOT/"mission/target.json").read_text())
M = CONFIG["motion"]
R = M["pipeRadius_m"]
G = rx.station.GRAVITY_MPS2
PHASES = (("seat",1.1),("retract",2.0),("lower",2.0),("drive-and-turn",5.0),
          ("lower-at-target",1.5),("extend-at-target",3.0),("place",1.5))


def smooth(t):
    u = min(1,max(0,t))
    return u**3 * (10+u*(-15+6*u))


def clip(value, limit):
    return max(-limit,min(limit,value))


def angle_delta(target, actual):
    return math.atan2(math.sin(target-actual),math.cos(target-actual))


def mount(t):
    cx, yaw = M["chassisStationX_m"], math.pi
    height = CONFIG["poses"]["receive"]["height"]/1000
    slide = CONFIG["poses"]["receive"]["stroke"]/1000
    for name,duration in PHASES:
        u = smooth(t/duration)
        if name=="retract":
            slide *= 1-u
        elif name=="lower":
            height += u*(CONFIG["poses"]["transport"]["height"]/1000-height)
        elif name=="drive-and-turn":
            cx += u*(M["chassisAtTargetX_m"]-cx)
            yaw *= 1-u
        elif name=="lower-at-target":
            height += u*(CONFIG["poses"]["place"]["height"]/1000-height)
        elif name=="extend-at-target":
            slide = u*CONFIG["poses"]["place"]["stroke"]/1000
        if t < duration:
            return name,cx,yaw,height-M["shelfTopBelowLift_m"],slide
        t -= duration
    return "place",cx,yaw,height-M["shelfTopBelowLift_m"],slide


def lateral(t, target_y):
    approach_start = sum(duration for _,duration in PHASES[:3])
    return target_y*smooth((t-approach_start)/PHASES[3][1])


def aim(t, target_y=0.0):
    name,cx,yaw,top,slide = mount(t)
    u = M["seatU_m"]+slide
    return cx+u*math.cos(yaw),lateral(t,target_y)+u*math.sin(yaw),yaw,top+R


def panel_support(x,y):
    slope = TARGET["floor_slope"]
    scale = math.hypot(1,slope)
    px,py = x*scale,y
    for panel in TARGET["panels"]:
        poly = [(a*scale,b) for a,b in panel["bounds_xy"]]
        edges = list(zip(poly,poly[1:]+poly[:1]))
        sign = math.copysign(1,sum(a*d-c*b for (a,b),(c,d) in edges))
        inside = all(sign*((c-a)*(py-b)-(d-b)*(px-a))/math.hypot(c-a,d-b)
                     >= TARGET["panel_inset_m"] for (a,b),(c,d) in edges)
        if inside and all(math.hypot((x-a)*scale,y-b)>r+TARGET["panel_inset_m"]
                          for a,b,r in panel["holes_xy_r"]):
            return True
    return False


def shelf_separation(x,z,yaw,front,top):
    nearest_x = x - abs(math.sin(yaw))*M["pipeLength_m"]/2
    return math.hypot(max(0,nearest_x-front),max(0,z-top))-R


def free_contacts(x,y,z,yaw,front,top):
    # Kinematics only after release. A nonzero yaw displaces the tube endpoints.
    # Check both end extents against the shelf front, and two support locations
    # against the sourced trimmed trough panels. Centered transport bounds yaw.
    half_len = M["pipeLength_m"]/2
    x_extent = abs(math.sin(yaw))*half_len
    local_x = x-M["targetLipX_m"]
    shelf_gap = shelf_separation(x,z,yaw,front,top)
    if shelf_gap < -1e-6:
        return "receiver-recontact", None
    l = TARGET["lip_depth_m"]
    gap_x = max(0,-(local_x+x_extent),local_x-x_extent-l)
    if math.hypot(gap_x,max(0,z-TARGET["lip_top_m"]))<R:
        return "trough-lip-contact", None
    for xmin,ymin,zmin,xmax,ymax,zmax in TARGET["riser_bounds_m"]:
        if abs(y-(ymin+ymax)/2) <= half_len+(ymax-ymin)/2:
            if math.hypot(max(0,xmin-(local_x+x_extent),local_x-x_extent-xmax),
                          max(0,zmin-z,z-zmax)) <= R:
                return "inner-riser-bound", None
    slope = TARGET["floor_slope"]
    den = math.hypot(1,slope)
    floor = TARGET["floor_intercept_m"]+slope*local_x
    foot = local_x+R*slope/den
    if (z-floor)/den <= R:
        valid = all(panel_support(foot,y+offset) for offset in (-.1,.1))
        return ("paired-trough-first-contact" if valid else "floor-support-unresolved"), foot
    return None,None


def run(case="reference", dt=.002, *, high_mass=True, piece_id="station-coral-001", target_y=0.0,
        sample_every_step=False, trajectory=None):
    if case not in {"reference","weak-grip","sensor-timeout","slow-release","occupied-target","light-pipe"}:
        raise ValueError("Unknown experiment")
    if not math.isfinite(dt) or not .00025 <= dt <= .005:
        raise ValueError("Use a finite step between 0.25 and 5 ms")
    if not piece_id or not math.isfinite(target_y):
        raise ValueError("Provide one identified pipe and an explicit finite target offset")
    mass = M["pipeMass_kg"] if high_mass and case!="light-pipe" else .498951607
    speed = .5
    stand_off_mm=0 if trajectory is None else trajectory.additional_stand_off_mm
    first = rx.first_impact(speed,stand_off_mm,240,6,json.loads(rx.DESIGN.read_text()))
    if first["surface"] != "top":
        raise ValueError("This hold model requires flat first contact")
    id_ = piece_id
    # Clock origin is lip departure. The exact first-contact time is inherited.
    start = first["time_s"]
    x,z = first["center_m"]
    vx = first["velocity_in_mps"][0]
    y = vy = omega = 0.0
    yaw = math.pi
    time = 0.0
    mode = "supported"
    trace = []
    events = []
    peak = {"belt_force_N":0.0,"centering_force_N":0.0,"yaw_error_deg":0.0,"lateral_error_mm":0.0}
    def robot(t):
        if trajectory is not None:
            return trajectory.pose(t)
        name,cx,heading,top,slide=mount(t)
        return name,cx,lateral(t,target_y),heading,top,slide
    def goal(t):
        if trajectory is None:
            return aim(t,target_y)
        _,cx,cy,heading,top,slide=robot(t)
        u=M["seatU_m"]+slide
        return cx+u*math.cos(heading),cy+u*math.sin(heading),heading,top+R
    def record(label, offset=0.0):
        events.append({"time_s":round(start+time+offset,8),"event":label,"piece_id":id_,
                       "xyz_m":[round(a,9) for a in (x,y,z)]})
        if trajectory is not None:
            events[-1]["target_xyz_m"]=trajectory.target_position(x,y,z)
    def row(phase):
        _,cx,cy,heading,height,stroke = robot(time)
        sample = {"time_s":round(start+time,8),"piece_id":id_,"phase":phase,"state":mode,
                "xyz_m":[round(a,9) for a in (x,y,z)],"vx_vy_mps":[round(vx,9),round(vy,9)],
                "yaw_rad":round(yaw,9),"yaw_rate_rad_s":round(omega,9),
                "chassis_xy_heading":[cx,cy,heading],
                "shelf_top_m":height,"slide_m":stroke}
        if trajectory is not None:
            sample["target_xyz_m"]=trajectory.target_position(x,y,z)
            sample["target_vxvy_mps"]=trajectory.target_velocity(vx,vy)
            sample["robot_relative_yaw_rad"]=angle_delta(yaw,heading)
        if sample_every_step:
            sample["shelf_front_robot_u_m"] = M["shelfRearU_m"]+M["shelfLength_m"]+stroke
            sample["pipe_pitch_rad"] = 0.0
            sample["pipe_pitch_basis"] = "fixed transverse hypothesis; axial roll unobserved"
        return sample
    def finish(reason):
        record(reason)
        trace.append(row(reason))
        return {"case":case,"dt_s":dt,"mass_kg":mass,"piece_id":id_,"target_offset_y_m":target_y,
                "outcome":reason,
                "physical_capture_tested":False,"score":None,"peak_supported":peak,
                "events":events,"trace":trace}
    record("first-top-contact")
    trace.append(row("station-capture"))
    # Initial impact is perfectly inelastic in normal direction, with zero tangential
    # impulse. The tube then obtains only the applied, limited forces below.
    last_phase = ""
    limit = 2*M["normalPerBelt_N"]
    if case=="weak-grip":
        limit=.04
    inertia = mass*(M["pipeLength_m"]**2/12+R**2/2)
    shelf_half = .210
    ticks = 0
    separated = False
    deadline = 17 if trajectory is None else trajectory.terminal_time+1
    while time < deadline:
        phase,cx,cy,heading,top,slide = robot(time)
        co,si = math.cos(heading), math.sin(heading)
        dy = y-cy
        u = (x-cx)*co+dy*si
        v = -(x-cx)*si+dy*co
        error = angle_delta(heading,yaw)
        low,high = M["shelfRearU_m"]+slide, M["shelfRearU_m"]+M["shelfLength_m"]+slide
        if phase != last_phase:
            record(phase)
            if phase=="retract" and (case=="sensor-timeout" or abs(u-(M["seatU_m"]+slide))>.008
                                     or abs(v)>.008 or abs(vx)+abs(vy)>.04):
                return finish("recover-at-station-unconfirmed-seat")
            if phase=="extend-at-target" and case=="occupied-target":
                return finish("hold-and-recover-target-occupied")
            last_phase = phase
        if sample_every_step or ticks % max(1,round(.02/dt)) == 0:
            trace.append(row(phase))
        if mode=="supported":
            if abs(error)>.15 or abs(v) + M["pipeLength_m"]/2 > shelf_half:
                return finish("recover-custody-envelope-lost")
            if u < low:
                return finish("recover-rear-edge-departure")
            if phase!="place" and u > high:
                return finish("recover-front-edge-departure")
            q = goal(time)
            qa = goal(time+dt)
            qb = goal(time-dt)
            vz = (qa[3]-qb[3])/(2*dt)
            normal = mass*(G+(qa[3]-2*q[3]+qb[3])/dt**2)
            if normal <= 0:
                return finish("recover-lost-normal-support")
            # Two provisional end-face belts apply bounded longitudinal rim forces.
            # Difference in their opposing normal loads centers the tube laterally.
            # Differential longitudinal force creates a yaw moment on the actual body.
            vel = [(a-b)/(2*dt) for a,b in zip(qa,qb)]
            acc = [(a-2*m+b)/dt**2 for a,m,b in zip(qa,q,qb)]
            kp,kd = M["servoGain_s2"],M["servoDamping_s"]
            fx = mass*(acc[0]+kp*(q[0]-x)+kd*(vel[0]-vx))
            fy = mass*(acc[1]+kp*(q[1]-y)+kd*(vel[1]-vy))
            longitudinal = fx*co+fy*si
            transverse = -fx*si+fy*co
            if phase=="place":
                release_speed=.35 if case=="slow-release" else M["releaseSpeed_mps"]
                longitudinal = mass*kd*(release_speed-(vx*co+vy*si))
            torque = inertia*(acc[2]+kp*error+kd*(vel[2]-omega))
            n_a = (limit+clip(transverse,limit))/2
            n_b = limit-n_a
            mu = M["tractionCoefficient"]
            arm = M["pipeLength_m"]/2
            f_a = clip(longitudinal/2+torque/(2*arm),mu*n_a)
            f_b = clip(longitudinal/2-torque/(2*arm),mu*n_b)
            peak["belt_force_N"] = max(peak["belt_force_N"],abs(f_a),abs(f_b))
            peak["centering_force_N"] = max(peak["centering_force_N"],abs(n_a-n_b))
            peak["yaw_error_deg"] = max(peak["yaw_error_deg"],abs(math.degrees(error)))
            peak["lateral_error_mm"] = max(peak["lateral_error_mm"],abs(v)*1000)
            ax = ((f_a+f_b)*co-(n_a-n_b)*si)/mass
            ay = ((f_a+f_b)*si+(n_a-n_b)*co)/mass
            alpha = (f_a-f_b)*arm/inertia
            x += vx*dt+ax*dt*dt/2
            y += vy*dt+ay*dt*dt/2
            yaw += omega*dt+alpha*dt*dt/2
            vx,vy,omega = vx+ax*dt,vy+ay*dt,omega+alpha*dt
            z = qa[3]
            if phase=="place" and (x-cx)*co+(y-cy)*si>=high:
                mode="free"
                record("left-powered-shelf-with-same-piece",dt)
                front = cx+high if trajectory is None else M["targetLipX_m"]-.49+high
                vz=0.0
        else:
            x,y,z = x+vx*dt,y+vy*dt,z+vz*dt-G*dt*dt/2
            vz -= G*dt
            yaw += omega*dt
            if trajectory is None:
                tx,ty,relative_yaw=x,y,yaw
            else:
                point=trajectory.target_position(x,y,z)
                tx,ty,relative_yaw=M["targetLipX_m"]+point[0],point[1],angle_delta(yaw,heading)
            gap = shelf_separation(tx,z,relative_yaw,front,top)
            if gap >= .001 and not separated:
                record("finite-pipe-envelope-clear-of-shelf-by-1mm",dt)
                events[-1]["conservative_separation_m"] = gap
                separated = True
            reason,foot = free_contacts(tx,ty,z,relative_yaw,front,top)
            if reason:
                time += dt
                return finish(reason)
        time += dt
        ticks += 1
    return finish("time-limit-unresolved")


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--case",default="reference")
    p.add_argument("--dt",type=float,default=.002)
    p.add_argument("--full",action="store_true")
    p.add_argument("--sample-every-step",action="store_true",help="Include every integration sample and registered shelf-front data")
    args = p.parse_args()
    result = run(args.case,args.dt,sample_every_step=args.sample_every_step)
    if not args.full and not args.sample_every_step:
        result.pop("trace")
    print(json.dumps(result,indent=2,allow_nan=False))
