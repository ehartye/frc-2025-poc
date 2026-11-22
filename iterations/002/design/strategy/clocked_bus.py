"""Replay signed pan reactions and a finite steering/drive schedule on the same two station clocks."""
import argparse
import csv
import json
import math
from pathlib import Path
import sys
from drive_envelope import motor,supply,file_id


def trapezoid(t, rise, coast, dt, acceleration):
    if t<rise:return acceleration*t,acceleration
    if t<rise+coast:return acceleration*rise,0.
    return max(0.,acceleration*(dt-t)),-acceleration


def loaded_route(field, meta):
    route=field.routes(meta['sectorM'])[1]
    retreat=meta.get('finalStandOffM',0)
    if not retreat:return route
    end=route.segments[-1]
    if end['kind']!='drive' or math.cos(end['start'][2])<.999999 or not 0<retreat<end['demand']:
        raise ValueError('Stand-off requires the commissioned final straight toward the reef')
    shortened=field.Route(end['start'])
    shortened.segment('drive',end['demand']-retreat)
    route.segments[-1]=shortened.segments[0]
    return route


def run(project, envelope, pan_trace, output):
    sys.path.insert(0,str(project/'experiments/six-wheel/scripts'))
    import registered_transit as field
    base=json.loads(envelope.read_bytes())
    inputs=json.loads(pan_trace.read_bytes())
    pan_duty=json.loads((project/'artifacts/red2-integrated/pan-duty-summary.json').read_bytes())
    half=base.get('moduleHalfSpacingM',[.282,.277])
    roll=.0508;turn_arm=math.hypot(*half);target_angle=math.atan2(*half)
    inactive=dict(phaseA=0.,terminalV=0.,supplyWatts=.7)
    cases={}
    with output.with_suffix('.csv').open('w',newline='') as raw:
        writer=csv.writer(raw)
        writer.writerow(('case','designSchedule','wallS','chassisPhase','panPhase','wheelMps','wheelN','drivePhaseA','steerPhaseA','panTorqueNm','panRateRadS','panPhaseA','busV','busA','minimumMotorHeadroomV'))
        for case,trace in inputs['runs'].items():
            condition,piece=case.split('/')[:2]
            basecase=base['cases'][condition];params=basecase['inputs'];meta=basecase['pieces'][piece]
            for schedule in (['baseline','extended-cruise'] if condition=='adverse' else ['baseline']):
                peak,accel,yaw,turn_accel,steer_rate,steer_accel=params['limits']
                if schedule=='extended-cruise':peak=1.70
                field.DRIVE.update(maximumWheelMps=peak,maximumWheelAccelMps2=accel,maximumTurnRadS=yaw,turnWheelAccelMps2=turn_accel)
                route=loaded_route(field,meta);events=[];cursor=meta['carryStartS'];orientation='drive'
                for segment in route.segments:
                    if orientation!=segment['kind']:
                        _,rise,coast,total=field.wheel_profile(target_angle,steer_rate,steer_accel)
                        events.append((cursor,cursor+total,'azimuth',rise,coast,steer_accel))
                        cursor+=total+.020
                    events.append((cursor,cursor+segment['dt'],segment['kind'],segment['rise'],segment['coast'],segment['a']))
                    cursor+=segment['dt'];orientation=segment['kind']
                margin=meta['releaseS']-3.5-(cursor+.25)
                worst={};close_clock=None
                def retain(k,v,time,phase,compare=min):
                    old=worst.get(k)
                    if old is None or compare(old['value'],v)!=old['value']:worst[k]=dict(value=v,wallS=time,phase=phase)
                archive_row=pan_duty['rows'][case+'/0']
                force_peak=0; sample_count=0
                for tick in range(math.floor(meta['stationFeedS']*50),math.ceil(meta['historicStowedS']*50)):
                    t=(tick+.5)/50; sample_count+=1
                    drive=steer=inactive;wheel_mps=wheel_force=0.;state='parked'
                    for start,end,kind,rise,coast,a in events:
                        if start<=t<end:
                            speed,acc=trapezoid(t-start,rise,coast,end-start,a)
                            state=kind
                            if kind=='azimuth':
                                tq=params['steerInertia']*abs(acc)+params['mu']*basecase['maxLoadedNormalN']*params['scrubArm']
                                steer=motor(tq/((150/7)*.70),speed*150/7,2.6,105,5676,1.8,.013)
                            else:
                                turn=kind=='turn'
                                wheel_mps=speed*(turn_arm*2/field.DRIVE['trackM'] if turn else 1)
                                wheel_force=(params['yawInertia']*abs(2*acc/field.DRIVE['trackM'])/(4*turn_arm) if turn else params['mass']*abs(acc)/4)+params['cRoll']*basecase['maxLoadedNormalN']
                                ratio=basecase['ratioComparison']['L2']['ratio']
                                drive=motor(wheel_force*roll/(ratio*params['gearEfficiency']),wheel_mps*ratio/roll,2.6,105,5676,1.8,.012)
                                force_peak=max(force_peak,wheel_force)
                            break
                    pan_torque=pan_rate=0.;pan_phase='waiting';pan=inactive
                    article=trace['bins'].get(str(tick))
                    if article:pan_torque,pan_rate,pan_phase=article
                    elif t>trace['releaseS']:
                        pan_torque=archive_row['afterReleaseUpperTorqueNm'];pan_rate=1.4;pan_phase='return torque bound'
                    if article or pan_phase.startswith('return'):
                        pan=motor(abs(pan_torque)/(pan_duty['driveRatio']*params['panEfficiency']),abs(pan_rate)*pan_duty['driveRatio'],.97,100,11000,1.4,.016)
                    bus=supply(params['pack'],[(4,drive['supplyWatts'],.022),(4,steer['supplyWatts'],.035),(1,pan['supplyWatts'],.032),(1,68,.022)])
                    if not bus['stable']:
                        raise ValueError('Bus solution collapsed on clock '+str((case,schedule,t)))
                    headroom=min(bus['branchTerminalV'][i]-m['terminalV'] for i,m in enumerate([drive,steer,pan]))
                    phases=state+' / '+pan_phase
                    for name,value in [('minimumBusV',bus['busV']),('minimumControllerVoltageHeadroomV',headroom)]:retain(name,value,t,phases)
                    for name,value in [('maximumDrivePhaseA',drive['phaseA']),('maximumSteerPhaseA',steer['phaseA']),('maximumPanPhaseA',pan['phaseA']),('maximumSupplyA',bus['totalSupplyA'])]:
                        retain(name,value,t,phases,max)
                    writer.writerow((case,schedule,round(t,5),state,pan_phase,round(wheel_mps,7),round(wheel_force,7),round(drive['phaseA'],7),round(steer['phaseA'],7),round(pan_torque,7),round(pan_rate,7),round(pan['phaseA'],7),round(bus['busV'],7),round(bus['totalSupplyA'],7),round(headroom,7)))
                cases[case+' / '+schedule]=dict(dockSettleMarginToTipS=margin,plannedParkAtS=cursor,records=sample_count,worst=worst)
    result=dict(sources={p.name:file_id(p) for p in (envelope,pan_trace,project/'artifacts/red2-integrated/pan-duty-summary.json')},
        wheelTraceSha256=file_id(output.with_suffix('.csv')),cases=cases,
        scope='20 ms branch-demand calculation. Pan peaks in each cell preserve measured phase, torque and joint velocity from the indicated original finite solver case; empty return uses its separately bounded torque. Stop/turn 002 traces use the same source station and two sectors. Pan reactions came from the prior design and short carry pulse. New accelerations and actual pan custody, bracket, sensed score and occupied full clock remain unverified. No regenerative supply credited. No phase-current cap is silently enforced. No high-rate motor-control or brake-energy test.')
    output.write_text(json.dumps(result,indent=2)+'\n')
    for i,v in cases.items():print(i,round(v['dockSettleMarginToTipS'],3),round(v['worst']['minimumBusV']['value'],3),round(v['worst']['maximumPanPhaseA']['value'],2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for key in ('project','envelope','pan_trace','output'):p.add_argument(key,type=Path)
    a=p.parse_args();run(a.project.resolve(),a.envelope,a.pan_trace,a.output)
