"""Run a field-receiver and physical return check on each actual current-limited exit."""
import argparse
import csv
import hashlib
import io
import json
import math
import shutil
import subprocess
import tempfile
from pathlib import Path

import numpy as np
from drive_envelope import file_id, supply
import clocked_bus

HERE=Path(__file__).parent
ROOT=HERE.parents[3]
HIST=ROOT/'artifacts/red2-integrated/single-pivot-rim-field'
OLD=ROOT/'cad/red2-split-drop/PanContactStudy.java'
FIELD=ROOT/'experiments/six-wheel/spatial-contact/SpatialContact.java'


def read_table(p):
    with p.open() as f:return list(csv.DictReader(f))


def run(jdk, stand_off_mm=0., executed=False, chassis_clock=None, simulated_pan=None):
    raw=simulated_pan or HERE/('limited-pan-12mm.json' if executed else 'limited-pan.json');runs=json.loads(raw.read_bytes())['runs']
    if len(runs)!=5 or runs['adverse/coral-A/6']['free'] is not None:raise ValueError('Incomplete trials or lost limited control')
    if file_id(OLD)!='8a730c66b863b8f92bd234639d9eea427b5167b3ad9c3e446ee33ac78f23ccba' or file_id(FIELD)!='3da5b7703edbfa0bbd1e1a45d6e5338f50bf1b73cad6a7533f818101ad1190c9':raise ValueError('Field solver changed')
    state=json.loads((HIST/'source-frames.json').read_bytes())
    inputs={r[0]:r for r in csv.reader((HIST/'cases.csv').open())}
    launch=json.loads((ROOT/'artifacts/split-drop-native/launch-inputs.json').read_bytes())['clocks']
    clock_source=chassis_clock or HERE.parent/'chassis/low-line-frame-clock.json'
    loaded=json.loads(clock_source.read_bytes())['clock']['cases']['heavy-offset-control']
    env=json.loads((HERE/'drive-envelope.json').read_bytes())
    if 'moduleHalfSpacingM' in loaded:env['moduleHalfSpacingM']=loaded['moduleHalfSpacingM']
    for c,row in env['cases'].items():
        row['inputs'].update(mass=loaded['fieldKg'],yawInertia=loaded['estimatedYawInertiaKgM2'])
        row['minLoadedNormalN']=loaded['loadedNormals'][c]['minimumN'];row['maxLoadedNormalN']=loaded['loadedNormals'][c]['maximumN']
        if executed:
            if stand_off_mm!=12.:raise ValueError('No such registered executed alternate')
            for meta in row['pieces'].values():meta['finalStandOffM']=.012
    actuator=json.loads((ROOT/'artifacts/red2-integrated/pan-mechanics.json').read_bytes())
    if actuator['movingStockInertiaKgM2']!=.11503479927879083 or actuator['movingShopStockKg']!=3.3452483934046366:raise ValueError('Changed rotating assembly')
    limits={};tags={};clocked={};post={};reported={}
    with tempfile.TemporaryDirectory() as temp:
        tmp=Path(temp)
        rows=[];clocks=[]
        shutil.copyfile(HIST/'panels.csv',tmp/'panels.csv')
        for name,r in runs.items():
            if not r['free']:continue
            case,piece,amp=name.split('/');q=r['free'];sched=r['clock']
            base=next(s.rsplit('/',1)[0] for s in state if s.startswith(f'{case}/{piece}/'+('1/1/-0.04' if case=='nominal' else '-1/1/0.04')+'/0.0001/17/'))
            clocked[base]=dict(bins={k:[v[6],v[5],'current-limited coupled pan'] for k,v in r['bins'].items()},releaseS=q['wallS'])
            for tag,frame in state.items():
                if not tag.startswith(base+'/'):continue
                mat=np.array(frame['rotation']);origin=np.array(frame['originM'])
                origin[0]-=stand_off_mm/1000
                position=mat@q['p']+origin
                remaining=inputs[tag][13:].copy()
                remaining[0]=origin[0]
                rows.append([tag,*position,*(mat@q['v']),*(mat@q['axis']),*(mat@q['omega']),*remaining])
                lc=launch[case][piece]
                if 'withdraw' in lc:permit=lc['withdraw']
                else:permit=lc['hold']
                clocks.append([tag,frame['pitch'],frame['originM'][2],q['wallS'],permit,sched['historicStowedS'],
                    q['thetaRad'],q['panRateRadS'],q['controller']['integral'],q['controller']['command'],q['controller']['next'],
                    r['actuatorModel']['efficiency']])
                tags[tag]=name
        for table,data in [('cases.csv',rows),('assessment-clock.csv',clocks)]:
            with (tmp/table).open('w',newline='') as handle:csv.writer(handle).writerows(data)
        source=OLD.read_text()
        def swap(a,b):
            nonlocal source
            if source.count(a)!=1:raise ValueError('Original field interface changed: '+a)
            source=source.replace(a,b)
        swap('    PanContactStudy(String[] row,String[] clock) {',(HERE/'limited_recovery.java.inc').read_text()+'\n    PanContactStudy(String[] row,String[] clock) {')
        swap('        super(row);','''        super(row);
        angle=Double.parseDouble(clock[6]);angularRate=Double.parseDouble(clock[7]);integral=Double.parseDouble(clock[8]);
        command=Double.parseDouble(clock[9]);nextSample=Double.parseDouble(clock[10]);gearEfficiency=Double.parseDouble(clock[11]);''')
        swap('double theta0=Math.toRadians(-2)-pitch,angle=theta0+(Math.toRadians(70)-theta0)*q*q*(3-2*q);','// Actual integrated angle at this sample before the next motor update.')
        swap('.40*(1.5/1.6)*(Math.toRadians(72)+Math.abs(pitch))','.40*(Math.abs(angularRate)+.05)')
        swap('if(i<(int)Math.ceil(end/dt))step(dt);','if(i<(int)Math.ceil(end/dt)){advancePan(freeClock+t,dt);step(dt);}')
        swap('double reset=(Double.isFinite(closeAt)?closeAt+1.6:-1);','''double reset=stowedAt;
        motorRecord.printf(Locale.ROOT,"%s,%.7f,%.7f,%.7f,%.7f,%.7f%n",label,closeAt,peakPhase,peakV,peakWatts,peakRate);''')
        swap('try(PrintWriter out=new PrintWriter(dir.resolve("post-impact-reset.csv").toFile())){','''try(PrintWriter out=new PrintWriter(dir.resolve("post-impact-reset.csv").toFile());
            PrintWriter motor=new PrintWriter(dir.resolve("return-motor.csv").toFile())){
            motorRecord=motor;motor.println("case,startS,phaseA,terminalV,watts,radS");''')
        java=tmp/'PanContactStudy.java';java.write_text(source)
        subprocess.run([jdk/'bin/javac.exe','-d',tmp,FIELD,java],check=True)
        subprocess.run([jdk/'bin/java.exe','-Xmx512m','-cp',tmp,'PanContactStudy',tmp],check=True)
        field_rows=read_table(tmp/'post-impact-reset.csv')
        if {r['case'] for r in field_rows}!=set(tags) or len(field_rows)!=len(tags):raise ValueError('Field coverage incomplete')
        motors={r['case']:r for r in read_table(tmp/'return-motor.csv')}
        for r in field_rows:
            tag=r['case'];name=tags[tag];motor=motors[tag];k=name.split('/')[0]
            after_bus=supply(env['cases'][k]['inputs']['pack'],[(4,.7,.022),(4,.7,.035),(1,float(motor['watts']),.032),(1,68,.022)])
            post[tag]=dict(field=r,motor=motor,parkedReturnBranchLowerV=after_bus['branchTerminalV'][2],
                 resetMarginS=float(r['reset_deadline_wall_s'])-float(r['reset_wall_s']) if float(r['reset_wall_s'])>=0 else None)
        # While the wheels are moving, resolve the original branch battery calculation with the new actual current and rate.
        trace=tmp/'limited_trace.json';trace.write_text(json.dumps({'runs':clocked}));efile=tmp/'envelope.json';efile.write_text(json.dumps(env))
        qfile=tmp/'bus.json';clocked_bus.run(ROOT,efile,trace,qfile)
        branch=read_table(qfile.with_suffix('.csv'))
        for k,record in clocked.items():
            name='/'.join(k.split('/')[:2])+'/30';q=runs[name];constraint=q['clock']['carryStartS']
            sampled=[r for r in branch if r['case']==k and r['panPhase']=='current-limited coupled pan' and
                    r['designSchedule']==('extended-cruise' if k.startswith('adverse') else 'baseline')]
            concurrent=[r for r in sampled if r['chassisPhase']!='parked']
            voltrow=min(sampled,key=lambda r:float(r['busV']))
            worst=max(concurrent,key=lambda r:float(r['panPhaseA']))
            maxwatts=max(v[0] for v in q['bins'].values())
            # Conservative lower branch voltage from lowest actual bus and highest actual pan power, independently bounded.
            v=float(voltrow['busV']);branchfloor=(v+math.sqrt(v*v-4*.032*maxwatts))/2
            reported[name]=dict(readyBeforeCarryS=constraint-q['readyS'],releaseMarginS=q['releaseMarginS'],
                heldSupportLossS=q['maxCarrySupportGapS'],actualMaxPhaseA=q['peakPhaseA'],simultaneousDrivePanPeak=worst,
                lowestBus=voltrow,conservativePanBranchV=branchfloor,positiveNineVoltAssumption=branchfloor>=9,
                modeledPeakTerminalV=q['peakMotorTerminalV'])
    return dict(activeLowerArchive=loaded['branchResults'] and json.loads((HERE.parent/'cad/fusion-authority.json').read_bytes())['source']['archive'],
        linked={p.name:file_id(p) for p in (raw,HIST/'panels.csv',HIST/'source-frames.json',HIST/'cases.csv',FIELD,OLD,HERE/'limited_recovery.java.inc',clock_source)},
        replayFieldMassKg=loaded['fieldKg'],
        additionalDockStandOffMm=stand_off_mm,changedChassisDynamicsExecuted=executed,runs=reported,fieldAndRecovery=post,lowCurrentAdverseControl='6 A lost coral-A before score',
        scope='Independent 17-station finite physical panel replays at the registered nominal docking frame and two enclosed frames per trip. Reset authorizes only after 250 ms finite support and another 50 ms, also waiting for the historical withdrawal or hold time. Return requires the actual modeled pan to stay within one degree and .03 rad/s of stow for 100 ms. Pan-vs-article clearance measurement starts at first field contact. Current traces include equally timed drive, yaw, real carried pipe, and simulated capped NEO550 motor-side torque. Controller tuning, gear efficiency/inertia, force law, contact sensing and final installed 002 complete robot remain unverified. No scores awarded to a physical 488-only chassis.')


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('jdk',type=Path)
    p.add_argument('--stand-off-mm',type=float,default=0.,choices=[0.,12.])
    p.add_argument('--executed',action='store_true');p.add_argument('--chassis-clock',type=Path)
    p.add_argument('--output',type=Path);a=p.parse_args()
    if a.chassis_clock and not a.output:p.error('Choose a separate output for a changed chassis case')
    r=run(a.jdk,a.stand_off_mm,a.executed,a.chassis_clock)
    file='limited-field-12mm-executed.json' if a.executed else 'limited-field-12mm.json' if a.stand_off_mm else 'limited-field.json'
    (a.output or HERE/file).write_text(json.dumps(r,indent=2)+'\n')
    for k,v in r['runs'].items():print(k,v['actualMaxPhaseA'],v['readyBeforeCarryS'],v['releaseMarginS'],v['conservativePanBranchV'])
    for k,v in r['fieldAndRecovery'].items():print(k,v['field']['supported_at_s'],v['field']['continuous_lower_gap_m'],v['resetMarginS'])

