"""Replay each registered station transfer with passive differential preload and finite drive ramp."""
import csv,hashlib,json,math,sys
from pathlib import Path
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
sys.path.insert(0,str(ROOT))
from scripts.station_l1_mission import run
from scripts.registered_transit import RegisteredMission
from scripts.station_l1_settle import continue_contact

UP5=68/13; UP4=76/21
R_GRIP=.060/(2*math.pi)
MASS=.816466266
SUPPLY=9.5


def motor(force,speed,radius,ratio,eff,shaft_accel=0,inertia=0,current_limit=20):
    motor_omega=speed*ratio/radius
    torque=force*radius/(ratio*eff)+inertia*shaft_accel*ratio/radius
    current=1.4+abs(torque)*(100-1.4)/.97
    needed=abs(motor_omega)/(11000*2*math.pi/60)*(12-.12*1.4)+.12*current
    return dict(rpm=round(abs(motor_omega)*60/(2*math.pi),2),phase_A=round(current,3),
                terminal_V=round(needed,3),input_W=round(needed*current/.90,2),
                bus_A_at_9p5V=round(needed*current/.90/SUPPLY,3),
                feasible=needed<=SUPPLY and current<=current_limit)


class Cartridge:
    def __init__(self,preload=10,mu=.4,ramp_s=.04,damping=37.52):
        self.preload=preload;self.mu=mu;self.ramp=ramp_s;self.start=None
        self.damping=damping;self.previous=None
        self.max_travel=0.;self.min_normal=1e6;self.max_normal=0.
    def __call__(self,v,yaw,phase,time):
        # Bounding contact on the end-rim silhouette; two 1 N/mm springs per carriage.
        yaw_gain=150.8125*(math.cos(yaw)-1)+57.15*abs(math.sin(yaw))
        offsets=[yaw_gain-1000*v,yaw_gain+1000*v]
        speeds=[0,0] if self.previous is None else [
            (x-old)/1000/(time-self.previous[0]) for x,old in zip(offsets,self.previous[1])]
        self.previous=(time,offsets)
        normals=[max(0,self.preload+2*x+self.damping*speed) for x,speed in zip(offsets,speeds)]
        self.max_travel=max(self.max_travel,*map(abs,offsets))
        self.min_normal=min(self.min_normal,*normals);self.max_normal=max(self.max_normal,*normals)
        speed=1.05
        if phase=="place":
            if self.start is None:self.start=time
            speed*=min(1,(time-self.start)/self.ramp)
        return *normals,self.mu,speed


def transfer(sector,id_,label,cartridge,dt=.001):
    path=RegisteredMission(sector)
    result=run(dt=dt,trajectory=path,sample_every_step=True,target_y=sector,piece_id=id_,grip_law=cartridge)
    valid=result["outcome"]=="paired-trough-first-contact"
    post=continue_contact(sector=sector,upstream=result,piece_id=id_,withdrawal="after-separation",
                          wait_after_true_separation_s=.10) if valid else None
    min_rim=1e6
    for row in result["trace"]:
        if row["state"]!="supported":continue
        cx,cy,heading=row["chassis_xy_heading"];wx,wy,_=row["xyz_m"]
        u=1000*((wx-cx)*math.cos(heading)+(wy-cy)*math.sin(heading))-1000*row["slide_m"]
        yaw=row["yaw_rad"]-heading
        for side in (-1,1):
            cap=u-side*150.8125*math.sin(yaw)
            intervals=[sorted((cap+sign*53.5*math.cos(yaw),cap+sign*57.15*math.cos(yaw)))
                       for sign in (-1,1)]
            min_rim=min(min_rim,sum(max(0,min(hi,154)-max(lo,-84.5)) for lo,hi in intervals))
    return dict(case=label,piece_id=id_,sector_m=sector,deadline_qualification=False,
                minimum_intersected_end_rim_width_mm=round(min_rim,4),
                first_outcome=result["outcome"],post=post["stop"] if post else None,
                peak_belt_force_N=result["peak_supported"]["belt_force_N"],max_cartridge_motion_mm=round(cartridge.max_travel,4),
                normal_range_N=[round(q,3) for q in (cartridge.min_normal,cartridge.max_normal)],
                within_sprung_travel=cartridge.max_travel<=1.5,terminal_events=result["events"][-3:])


def size():
    runs=[]
    for sector,piece in ((-.18,"station-coral-001"),(.18,"station-coral-002")):
        for name,args in (("dry nominal",{}),("no damping",dict(damping=0)),("low preload and wet",dict(preload=2,mu=.08)),
                          ("slow spin-up",dict(ramp_s=.22))):
            runs.append(transfer(sector,piece,name,Cartridge(**args)))
    dry=[r for r in runs if r["case"]=="dry nominal"]
    f=max(r["peak_belt_force_N"] for r in dry)
    # 0.40 N bearing/backing drag is added to transmitted force. Rotor-equivalent input
    # inertia brackets the motor rotor plus reflected cartridge inertia; not a vendor rating.
    grip={str(j):motor(f+.4,1.05,R_GRIP,UP5,.80,1.05/.04,j,8) for j in (1e-6,3e-6,7e-6)}
    # Physical moving mass assigned from the base BOM, hardware revisions and payload.
    # Separate the middle stage which sees half the outer travel/rate.
    baserows=list(csv.DictReader((ROOT/"design/station-l1/bom.csv").open()))
    removed={"slide-drive","belt-cartridge-L","belt-cartridge-R"}
    removed.update(f"belt-riser-{side}-{x}" for side in ("L","R") for x in (-90,136))
    removed.update(f"shelf-screw-{side}-{x}" for side in ("L","R") for x in (-80,140))
    newrows=list(csv.DictReader((HERE/"bom.csv").open(encoding="utf-8")))
    newmass=lambda frame:sum(float(q["highMassKg"]) for q in newrows if q["frame"]==frame)
    def mass(frame):
        return sum(float(row["maxMassKg"]) for row in baserows if row["kinematicFrame"]==frame and row["id"] not in removed)
    inner=mass("slide")+MASS+newmass("slide")+.50
    mid=mass("middle")+newmass("middle")+.50
    slide={}
    for label,travel,duration in (("receive-retract",.240,2),("place-extend",.370,3)):
        # quintic blend independent maxima conservatively combined
        speed=1.875*travel/duration;acc=10/math.sqrt(3)*travel/duration**2
        for drag in (8,25,80,140):
            rack_force=(2*inner+.5*mid)*acc+2*drag+4
            slide[f"{label} outer drag {drag} N"]=dict(outer_v_mps=speed,outer_acc_mps2=acc,
                middle_rack_force_N=round(rack_force,3),
                sync_slack_N=round(55-(inner*acc+drag)*.010/R_GRIP/2,3),**motor(rack_force,speed/2,.010,UP5*UP4,.70,acc/2,3e-6))
    convergence=[]
    for dt in (.002,.0005,.00025):
        for damping in (0.,37.52):
            trial=transfer(-.18,"step-control","timestep control",Cartridge(damping=damping),dt)
            convergence.append([dt,damping,trial["first_outcome"],trial["max_cartridge_motion_mm"]])
    basis=[HERE/"bom.csv",Path(__file__),ROOT/"scripts/station_l1_mission.py",ROOT/"scripts/registered_transit.py",
           ROOT/"scripts/station_l1_settle.py",ROOT/"mission/candidate.json",ROOT/"mission/field-registration.json"]
    return dict(classification="conditional desktop design, no measured hardware",transfers=runs,
                source_sha256={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in basis},
                convergence=convergence,
                grip_acceleration_and_peak_force_combined=grip,loaded_outer_mass_kg=round(inner,4),
                moving_middle_mass_kg=round(mid,4),slide=slide)


if __name__=="__main__":
    import argparse
    p=argparse.ArgumentParser();p.add_argument("--out",type=Path,default=HERE/"sizing.json")
    a=p.parse_args();a.out.write_text(json.dumps(size(),indent=2)+"\n",encoding="utf-8",newline="\n")
