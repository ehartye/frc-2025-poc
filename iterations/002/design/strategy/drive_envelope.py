"""First 002 drive and shared-bus screen on the two registered Red 2 station trips."""
import argparse
import csv
import hashlib
import json
import math
import sys
from pathlib import Path
import numpy as np


def file_id(path):
    with path.open('rb') as f:
        return hashlib.file_digest(f,'sha256').hexdigest()


def motor(shaft_torque, motor_rad_s, stall_torque, stall_current, free_rpm, free_current, wiring):
    phase = free_current + abs(shaft_torque)*stall_current/stall_torque
    resistance = 12/stall_current
    ke = (12-resistance*free_current)/(free_rpm*math.tau/60)
    terminal = phase*(resistance+wiring)+ke*abs(motor_rad_s)
    return dict(phaseA=phase,terminalV=terminal,supplyWatts=phase*terminal/.90 + .7)


def supply(pack, loads):
    current=0.
    for _ in range(500):
        bus = pack[0]-pack[1]*current
        if bus<=0: return dict(stable=False)
        branches=[]
        for count, watts, ohms in loads:
            d=bus*bus-4*ohms*watts
            if d<0: return dict(stable=False)
            amps=2*watts/(bus+math.sqrt(d))
            branches.append([count,amps,bus-ohms*amps])
        total=sum(count*i for count,i,v in branches)
        if abs(total-current)<1e-7:
            return dict(stable=True,totalSupplyA=total,busV=bus,branchTerminalV=[row[2] for row in branches])
        current=total
    return dict(stable=False)


def run(project, shared_artifacts, sources):
    sys.path.insert(0,str(project/'experiments/six-wheel/scripts'))
    import registered_transit as path
    events=project/'experiments/six-wheel/spatial-contact/events.csv'
    sector_file=project/'experiments/six-wheel/mission/field-registration.json'
    pan_file=project/'artifacts/red2-integrated/pan-duty-summary.json'
    cfg=sources/'configuration-20261121.json'
    demand=project/'experiments/swerve-drive/low-placement/red2-demand.json'
    source_paths=[events,sector_file,pan_file,cfg,demand,Path(path.__file__),
        project/'experiments/swerve-drive/motion/model.json',
        project/'experiments/swerve-drive/power/electrical.py']
    traces=[shared_artifacts/'cam-scoring/plus/route/path.npz',shared_artifacts/'low-placement/holdout/minus/route/path.npz']
    records={p.as_posix():file_id(p) for p in source_paths+traces}
    model=json.loads(source_paths[-2].read_bytes())['drive']
    pan=json.loads(pan_file.read_bytes())
    pan_peak=max(abs(r['throughReleasePeak']['value']) for r in pan['rows'].values())
    release={k:{} for k in ('coral-A','coral-B')}
    for r in csv.DictReader(events.open()):
        if r['approach']==r['scenario']=='nominal' and r['event'] in ('carry','release','stow','feed'):
            release[r['piece']][r['event']]=float(r['wall_ms'])/1000
    case_register={
        'nominal':dict(limits=[1.8,1.8,1.75,.90,6,28],mass=65,yawInertia=5,mu=.80,cRoll=.025,gearEfficiency=.83,pack=[12.4,.025],panEfficiency=.60,steerInertia=.017,scrubArm=.015),
        'adverse':dict(limits=[1.55,1.5,1.5,.85,5,22],mass=67,yawInertia=7,mu=.50,cRoll=.05,gearEfficiency=.75,pack=[11.8,.032],panEfficiency=.45,steerInertia=.03,scrubArm=.025)}
    results={}
    opts=json.loads(cfg.read_bytes())['selectableFamilies']
    for condition,c in case_register.items():
        speed,accel,yaw,angular_wheel_accel,steer_rate,steer_accel=c['limits']
        path.DRIVE.update(maximumWheelMps=speed,maximumWheelAccelMps2=accel,
            maximumTurnRadS=yaw,turnWheelAccelMps2=angular_wheel_accel)
        drive_r=.0508
        arm=math.hypot(.282,.277)
        steer_angle=math.atan2(.282,.277)
        steer_time=path.wheel_profile(steer_angle,steer_rate,steer_accel)[-1]
        m=c['mass']; g=9.80665; alpha=2*angular_wheel_accel/path.DRIVE['trackM']
        max_normal=m*g/4 + m*accel*.38/(4*.282)
        min_normal=m*g/4 - m*accel*.38/(4*.282)
        wheel_force=max(m*accel/4,c['yawInertia']*alpha/(4*arm))+c['cRoll']*max_normal
        radial_margin=c['mu']*min_normal-wheel_force
        # A bounded steering scrub work estimate includes tire friction and steer inertia.
        steer_eff=.70
        steer_torque=c['steerInertia']*steer_accel+c['mu']*max_normal*c['scrubArm']
        steer_motor=motor(steer_torque/((150/7)*steer_eff),steer_rate*150/7,model['stallTorqueNm'],model['stallCurrentA'],model['freeRpm'],model['freeCurrentA'],.013)
        # Pan contributes its entire actual instantaneous output peak while all
        # gears and all four azimuth motors are simultaneously at their peaks.
        pan_motor=motor(pan_peak/(pan['driveRatio']*c['panEfficiency']),1.4*pan['driveRatio'],.97,100,11000,1.4,.016)
        ratios={}
        for grade,option in opts.items():
            red=option['driveReduction'];drive_motor=motor(wheel_force*drive_r/(red*c['gearEfficiency']),max(speed,yaw*arm)/drive_r*red,model['stallTorqueNm'],model['stallCurrentA'],model['freeRpm'],model['freeCurrentA'],.012)
            bus=supply(c['pack'],[(4,drive_motor['supplyWatts'],.022),(4,steer_motor['supplyWatts'],.035),(1,pan_motor['supplyWatts'],.032),(1,68,.022)])
            if bus['stable']:
                bus['dutyVoltageMarginV']=[v-r['terminalV'] for v,r in zip(bus['branchTerminalV'],(drive_motor,steer_motor,pan_motor))]
                bus['speedAtRequiredTorqueMps']=max(0,(bus['branchTerminalV'][0]-drive_motor['phaseA']*(12/model['stallCurrentA']+.012)))/((12-12/model['stallCurrentA']*model['freeCurrentA'])/(model['freeRpm']*math.tau/60))*drive_r/red
            ratios[grade]=dict(ratio=red,drive=drive_motor,bus=bus,drivePhaseMarginTo40A=40-drive_motor['phaseA'])
        trips={}
        for (piece,wall),section in zip(release.items(),path.REG['redReef']['sectorOffsetsM']):
            _,leg,_=path.routes(section)
            kinds=['drive']+[s['kind'] for s in leg.segments]+['drive']
            changes=sum(a!=b for a,b in zip(kinds,kinds[1:]))
            travel=leg.duration+changes*(steer_time+.020)
            deadline=wall['release']-3.5
            trips[piece]=dict(stationFeedS=wall['feed'],carryStartS=wall['carry'],sectorM=section,
                routeCenterlineM=sum(s['wheel'] for s in leg.segments if s['kind']=='drive'),
                turnAnglesRad=[s['demand'] for s in leg.segments if s['kind']=='turn'],
                loadedTravelS=travel,azimuthChanges=changes,azimuthTimeEachS=steer_time+.020,
                sameClockTipStartS=deadline,releaseS=wall['release'],historicStowedS=wall['stow'],
                parkedLeadBeforeTipS=deadline-wall['carry']-travel-.25)
        results[condition]=dict(inputs=c,pieces=trips,minLoadedNormalN=min_normal,maxLoadedNormalN=max_normal,
            frictionMarginN=radial_margin,wheelDemandN=wheel_force,brakingDistanceAtSpeedM=speed*.02+speed**2/(2*accel),
            yawAccelerationRadS2=alpha,steer=steer_motor,pan=pan_motor,panDemandBeforeFreeCurrentNm=pan_peak,
            panCurrentMarginToRetired30ACap=30-pan_motor['phaseA'],ratioComparison=ratios)
    old={}
    for tr in traces:
        with np.load(tr) as d:
            old[tr.as_posix()]=dict(durationS=float(d['relativeS'][-1]-d['relativeS'][0]),peakMps=float(np.linalg.norm(d['worldVelocity'][:,:2],axis=1).max()),
                sampleCount=len(d['relativeS']))
    return dict(sources=records,requirement=json.loads(demand.read_bytes()),historicalPhysicalStationLegs=old,
        cases=results,scope='Early prebuild bound on new SDS 4-inch, NEO drive and NEO azimuth. Preserves registered centerline, six rest-to-rest azimuth changes, 20 ms transition latency each, 250 ms dock settle and a 3.5 s stationary pre-tip window. Differential profile class controls the center position and yaw only. Does not certify tracking, thermal capacity, DC braking energy acceptance, occupancy, 4-wheel traction distribution, sensed placement or source pan drive. Articulated inertia, pan speed limit, total mass/CG, scrub and wire resistance are provisional. Regeneration contributes zero available supply. All nine motors plus 68 W auxiliary concurrently budgeted as a sufficient envelope, not an observation. Weak pack, loaded tire radius, path/clock, achievable steering and current limits can reopen the ratios.')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('project','shared_artifacts','sources','output'):
        parser.add_argument(name,type=Path)
    a=parser.parse_args();d=run(a.project.resolve(),a.shared_artifacts,a.sources)
    a.output.write_text(json.dumps(d,indent=2)+'\n')
    for key,c in d['cases'].items():
        print(key,[round(x['parkedLeadBeforeTipS'],3) for x in c['pieces'].values()],c['ratioComparison']['L2']['bus'],c['panCurrentMarginToRetired30ACap'])
