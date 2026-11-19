"""Inverse dynamics per wheel and a simultaneous high-root battery/PDH/branch solution."""
import math
import sys
from pathlib import Path

import numpy as np
from route import DT,ROOT,HERE,read,sha

for directory in ("design/receiver-drives","fixed-low-power","design/drivetrain"):
    sys.path.insert(0,str(ROOT/directory))
import dual_camera as bus
import route_check as motor
from size_drives import UP5,UP4,R_GRIP

def shared(voltage,ohms,watts):
    # All time bins solved together. The high-voltage branch is approached from open circuit.
    names=list(watts)
    p=np.column_stack([watts[k] for k in names])
    resist=np.array([bus.RESISTANCE[k] for k in names])
    supply=np.full(len(p),voltage)
    valid=np.ones(len(p),dtype=bool)
    for _ in range(200):
        disc=supply[:,None]**2-4*p*resist
        valid&=(disc.min(axis=1)>0)&(supply>0)
        term=(supply[:,None]+np.sqrt(np.maximum(0,disc)))/2
        current=p/np.maximum(term,1e-9)
        updated=voltage-ohms*current.sum(axis=1)
        if not np.any(valid):break
        if np.max(abs(updated[valid]-supply[valid]))<1e-9:break
        supply=np.where(valid,updated,voltage)
    else:
        valid&=np.abs(updated-supply)<1e-8
    if not np.any(valid):raise ValueError("No energized operating points")
    supply[~valid]=np.nan;term[~valid]=np.nan;current[~valid]=np.nan
    return names,supply,term,current


def screen(t,arcs,labels,events,mass,mode,pack,raw,*,scrub=.38,side=.44,outer=.75,speed_scale=1,cap_grip_speed=False,calibrated=False,trace=None,rack_case=None,rack_paths=None):
    # Mid-interval speed, two adjacent acceleration estimates. Keep the more demanding
    # electrical/traction endpoint when the wheel torque changes sign.
    n=len(arcs);speed=arcs/DT*speed_scale
    both=np.mean(speed,axis=1);omega=(speed[:,1]-speed[:,0])/.580
    track=.582 if calibrated else .580
    radii=(.0762*.998,.0762*1.002) if calibrated else (.0762,.0762)
    speed=np.column_stack((both-omega*track/2,both+omega*track/2))
    loaded=np.zeros(n,dtype=bool)
    for ev in events.values():
        if "seat" in ev:loaded|=(t[:-1]>=ev["seat"])&(t[:-1]<ev.get("withdraw",156))
    kg=mass+.816466266*loaded
    phase={k:np.zeros(n) for k in bus.power.phase_keys}
    needed={k:np.zeros(n) for k in phase}
    mu=np.zeros(n)
    for dv in (np.diff(speed,axis=0,prepend=np.zeros((1,2))),np.diff(speed,axis=0,append=np.zeros((1,2)))):
        accel=dv/DT*speed_scale
        a=np.mean(accel,axis=1);alpha=(accel[:,1]-accel[:,0])/track
        turning=(abs(omega)>1e-6)|(abs(alpha)>1e-5)
        turning_sign=np.where(abs(omega)>1e-6,np.sign(omega),np.sign(alpha))
        for i,load in enumerate(("DriveLeft","DriveRight")):
            sgn=np.sign(np.where(abs(speed[:,i])>1e-7,speed[:,i],accel[:,i]))
            weight=kg*9.80665
            yaw_torque=kg*(2*.425**2)*alpha + turning_sign*turning*weight*outer*.24765*scrub
            force=kg*a/2+(2*i-1)*yaw_torque/track+sgn*weight*.02/2
            extra=kg*.050*(abs(alpha)+omega**2)/2
            lateral=turning*weight*scrub/2 + kg*abs(both*omega)/2+extra
            mu=np.maximum(mu,np.hypot(abs(force)+extra,lateral)/(weight*side))
            for longitudinal in (force-extra,force+extra):
                torque=longitudinal*radii[i]+.040*accel[:,i]/radii[i]
                # motor rotation determines whether loss acts on motoring or braking.
                braking=torque*sgn<0
                shaft=torque/20*np.where(braking,.82,1/.82)
                amps=shaft/motor.K_T+1.8*sgn
                v=motor.K_E*speed[:,i]/radii[i]*20+motor.R*amps
                phase[load]=np.maximum(phase[load],abs(amps))
                needed[load]=np.maximum(needed[load],abs(v)+abs(amps)*bus.power.electric.PHASE_R[load])
    # Cap-level joint grip bound during both actual feed/transfer windows. Overbounds
    # counterfeed, seated hold, short reverse pulse, forward probe and release.
    for piece,ev in events.items():
        if "feed" not in ev:continue
        mask=(t[:-1]>=ev["feed"]-DT)&(t[:-1]<=ev.get("stow",153))
        # The issued port schedule commands at most .65 m/s, then reaches zero
        # long before the drive clock. The release is the registered .90 m/s
        # ramp. Bound every sample by its actual phase's command range; retain
        # full phase current, including stalled belts in the carry interval.
        feed=read(ROOT/"relieved-match"/("first-feed.json" if piece=="coral-A" else "return-feed.json"))
        ticks=feed["cases"][0]["portTicks"]
        if max(abs(row[-1]) for row in ticks)>.65 or ticks[-1][-1]!=0:
            raise ValueError("Unregistered intake command")
        belt_speed=np.zeros(n)
        belt_speed[(t[:-1]>=ev["feed"]-.025)&(t[:-1]<ev.get("carry",153))]=.65
        carry_bound=ev.get("carryBeltSpeedBoundMps",0.)
        if not np.isfinite(carry_bound) or carry_bound<0:
            raise ValueError("Invalid carry belt speed bound")
        carry_window=(t[:-1]>=ev.get("carry",153))&(t[:-1]<ev.get("release",157)-.020)&mask
        belt_speed[carry_window]=carry_bound
        belt_speed[(t[:-1]>=ev.get("release",157)-.020)&mask]=.90
        if cap_grip_speed:belt_speed[mask]=1.05
        for load in ("LeftGrip","RightGrip"):
            phase[load][mask]=8.
            needed[load][mask]=belt_speed[mask]*UP5/R_GRIP/(11000*2*math.pi/60)*(12-.12*1.4)+8*(.12+bus.power.electric.PHASE_R[load])
        if "release" not in ev:continue
        slide="Slide"
        # Full extension starts 3 s before the issued release. Use a cap with
        # the highest quintic speed and 20 A while extending/holding.
        moving=(t[:-1]>=ev["release"]-3.020)&(t[:-1]<ev.get("withdraw",153))
        phase[slide][moving]=20
        needed[slide][moving]=.370/3*1.875/2*UP4*UP5/.010/(11000*2*math.pi/60)*(12-.12*1.4)+20*(.12+bus.power.electric.PHASE_R[slide])
        if "withdraw" in ev:
            case=rack_case or ("bridge26" if mode=="bridge26" else "nominal")
            path=raw/("path-"+case+"-"+piece+".csv")
            expected=(read(HERE/"rack.json")["cases"][case][piece]["rawSHA256"]
                      if rack_paths is None else rack_paths[path.name])
            if sha(path)!=expected:raise ValueError("Unregistered selected moving-mass rack trace")
            signal=np.genfromtxt(path,delimiter=",",names=True)
            for rec in signal:
                k=int((round(1000*ev["withdraw"])+int(rec["ms"]))//5)
                amps=abs(rec["phase_A"]);volts=abs(rec["motor_V"])+amps*bus.power.electric.PHASE_R[slide]
                phase[slide][k]=max(phase[slide][k],amps);needed[slide][k]=max(needed[slide][k],volts)
    watts={"Radio":np.full(n,12.),"RIO":np.full(n,15.),"PH":np.full(n,1.5)}
    watts.update({k:np.full(n,5.21*item[-1]/.75) for k,item in bus.SUPPLIES.items()})
    watts.update({k:(np.zeros(n) if k=="Lift" else .7+phase[k]*needed[k]/.90) for k in phase})
    names,supply,term,draw=shared(*pack,watts)
    idx,branch=np.unravel_index(np.nanargmin(term),term.shape)
    unsolved=np.isnan(supply)
    cap=np.maximum(phase["DriveLeft"],phase["DriveRight"])
    straight=(abs(both)>.01)&(abs(omega)<1e-6)
    corners={"translationPeakA":float(max(cap[straight],default=0))}
    turn_mask=abs(omega)>.02
    quiet=np.max(abs(np.diff(speed,axis=0,prepend=np.zeros((1,2))))/DT,axis=1)<.004
    constant=np.convolve(quiet.astype(int),np.ones(21,dtype=int),mode="same")==21
    corners["turnPeakA"]=float(max(cap[turn_mask],default=0))
    corners["steadyTurnA"]=float(max(cap[turn_mask&constant],default=0))
    corners["steadyStraightA"]=float(max(cap[(abs(both)>.05)&(~turn_mask)&constant],default=0))
    requirements=np.column_stack([np.maximum(9.5,needed.get(k,np.zeros(n))) for k in names])
    powers=np.column_stack([watts[k] for k in names])
    resist=np.array([bus.RESISTANCE[k] for k in names])
    required_bus=np.max(requirements+resist*powers/requirements,axis=1)
    devices=(required_bus[:,None]+np.sqrt(required_bus[:,None]**2-4*resist*powers))/2
    common_limit=(pack[0]-required_bus)/(powers/devices).sum(axis=1)
    # This is a design reference inhibit for insufficient available traction/current/voltage.
    # Subsequent samples are unattained demand once any gate is lost.
    below_motor=np.logical_or.reduce([term[:,names.index(k)]<v for k,v in needed.items()])
    blocked=np.flatnonzero(unsolved|(term.min(axis=1)<9.5)|(mu>.90)|(cap>38)|below_motor)
    voltage=float(min(np.nanmin(term[:,names.index(k)]-v) for k,v in needed.items()))
    if trace is not None:
        np.savetxt(trace,np.column_stack((t[:-1],term[:,names.index("DriveRight")],cap,mu)),delimiter=",",
                   fmt="%.7f",header="wall_s,right_terminal_v,drive_phase_a,required_mu",comments="")
    return {"bodyKg":mass,"onePipeKg":mass+.816466266,"peakFrictionRequired":float(np.max(mu)),
        "peakMotorPhaseA":float(np.max(cap)),"tractionAndCurrentByPhase":corners,"maxCommonA":float(np.nanmax(draw.sum(axis=1))),
        "minPDHV":float(np.nanmin(supply)),"floorBelowOrUnsolvedS":float(np.sum((term.min(axis=1)<9.5)|unsolved)*DT),"unsolvedS":float(np.sum(unsolved)*DT),
        "minTerminalV":float(term[idx,branch]),"minimumMotorHeadroomV":voltage,"limitingBranch":names[branch],
        "limitingWallS":float(t[idx]),"commonCurrentAtMinimumA":float(np.sum(draw[idx])),"firstReferenceInhibitWallS":float(t[blocked[0]]) if len(blocked) else None,
        "allowableCommonOhmAtVoc":float(np.min(common_limit)),"limitingLeg":str(labels[idx]),"maxYawRadS":float(max(abs(omega))),
        "maxWheelMps":float(np.max(abs(speed))),"maxDriveBranchA":float(np.nanmax(draw[:,[names.index('DriveLeft'),names.index('DriveRight')]])),
        "branchBounds":{k:{"minV":float(np.nanmin(term[:,j])),"maxA":float(np.nanmax(draw[:,j])),
                         "rmsA":float(np.sqrt(np.nanmean(draw[:,j]**2)))} for j,k in enumerate(names)}}
