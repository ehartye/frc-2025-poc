"""Compare attainable custody and independent field-only support on the same clock."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
OUT = ROOT/'artifacts/split-drop-native'
SPATIAL = ROOT/'experiments/six-wheel/spatial-contact'
MISSION = ROOT/'experiments/six-wheel/mission'
sys.path.insert(0, str(SPATIAL))
from physics import panel_file

def read(p): return json.loads(p.read_bytes())
def write(p, value): p.write_bytes((json.dumps(value, indent=2, allow_nan=False)+'\n').encode())
def ref(p): return dict(path=p.relative_to(ROOT).as_posix(),sha256=hashlib.sha256(p.read_bytes()).hexdigest())

LAYOUTS = {
    'narrow':'Two 12 mm entrance rails lead to an opposed pair of sliding cradle rails.',
    'wide-pan-guides':'High full-width receiving pan with fixed taper guides and two sliding cradle rails.',
    'lower-pan':'Lower passive receiving pan includes fixed taper guides and two sliding cradle rails.',
    'driven-pan':'Lower receiving conveyor has a 0.48 m/s driven surface and two sliding cradle rails.',
    'moving-cheeks':'Lower passive receiving pan guides the pipe into a sliding cradle with an end fence attached to each jaw.',
    'slow-carry':'The same guided cradle traverses the matched 0.945 m pulse distance with 1.05 m/s² peak and 0.30 s extra time.',
    'steeper-saddle':'A 17 mm higher aft end of the saddle and two moving cheek plates resist the original carry pulse, with a passive receiving pan.'
}

def field_test(runs,jdk,out=None):
    if out is None:out=OUT/'attained-field'
    out.mkdir(exist_ok=True)
    station=read(MISSION/'field-registration.json')
    shifts=station['redReef']['sectorOffsetsM']
    docks=read(ROOT/'experiments/swerve-drive/low-placement/results.json')['cases']
    panel_file(read(MISSION/'target.json'),out/'panels.csv')
    names={}
    with (out/'cases.csv').open('w',newline='') as stream:
        w=csv.writer(stream,lineterminator='\n')
        for layout,traces in runs.items():
            for key,r in traces.items():
                if not r['free']:continue
                scenario=key.split('-')[0]
                secB='coral-B' in key
                dock=docks['plus' if scenario=='nominal' else 'minus']['chassis']
                state=r['free']['state']
                for j,(dx,dy,yaw) in enumerate([dock['terminalMmMmRad'],*dock['enclosure']]):
                    for pitch,heave in [(-.04,-.005),(.04,.005)]:
                        def rotate(vec):
                            x,y,z=vec
                            xx=x*math.cos(pitch)-z*math.sin(pitch)
                            return [math.cos(yaw)*xx-math.sin(yaw)*y,math.sin(yaw)*xx+math.cos(yaw)*y,
                                    z*math.cos(pitch)+x*math.sin(pitch)]
                        x,y,z=rotate(state['p'])
                        xx,yy=dx/1000-.490,dy/1000+(shifts[1]-shifts[0] if secB else 0)
                        tag=f'{layout}/{key}/{j}/{pitch}'
                        names[tag]=(layout,scenario)
                        w.writerow([tag,x+xx,y+yy,z+heave,*rotate(state['v']),*rotate(state['axis']),
                                    *rotate(state['omega']),xx,yy,yaw,-.001,2.5,.0001,17,
                                    .35 if scenario=='nominal' else .10])
    (out/'retired-rack-control.csv').write_text('time_s,withdrawal_m\n0,.110\n.001,.110\n')
    subprocess.run([jdk/'bin/javac.exe','-d',out,SPATIAL/'SpatialContact.java'],check=True)
    subprocess.run([jdk/'bin/java.exe','-cp',out,'SpatialContact',out/'panels.csv',out/'retired-rack-control.csv',out/'cases.csv',
                    out/'result.csv',out/'trace.csv','.25','2.5','sweep-y'],check=True)
    with (out/'result.csv').open() as stream: return list(csv.DictReader(stream))

def main(jdk,plugin):
    fine={'slow-carry','steeper-saddle','moving-cheeks'}
    files={k:OUT/('r4-spatial-'+k+('-challenge' if k in fine else '')+'.json') for k in LAYOUTS}
    runs={k:read(v)['runs'] for k,v in files.items()}
    results=field_test(runs,jdk)
    common=read(HERE/'study.json')
    executions={}
    archive=read(ROOT/'cad/fusion-authority.json')['source']['archive']
    assert ref(ROOT/archive['file'])['sha256']==archive['sha256']
    for layout,entries in runs.items():
        executions[layout]={}
        for scenario in ['nominal','adverse']:
            trials={k:r for k,r in entries.items() if k.startswith(scenario+'-')}
            records={}
            for piece in ['coral-A','coral-B']:
                rows=[r for k,r in trials.items() if piece in k]
                ready=[r['stationReadyS'] for r in rows if r['stationReadyS'] is not None]
                exits=[r['free']['wallS'] for r in rows if r['free']]
                records[piece]=dict(clock=rows[0]['clock'],trials=len(rows),custody=len(ready),releases=len(exits),
                                    readyWallS=[min(ready),max(ready)] if ready else [],
                                    releaseWallS=[min(exits),max(exits)] if exits else [],
                                    retainedThroughoutDrivePulse=sum(r['carryHeld'] for r in rows),
                                    worstGuideForceN=max(r['guideForceN'] for r in rows),
                                    longestBilateralInterruptionMs=max((r.get('longestSupportInterruptionMs',0) for r in rows),default=0) if layout in fine else None,
                                    maximumUnilateralSupportMs=1000*max(r['unilateralSupportS'] for r in rows))
            panels=[r for r in results if r['case'].startswith(layout+'/'+scenario+'-')]
            field={outcome:sum(r['outcome']==outcome for r in panels) for outcome in set(r['outcome'] for r in panels)}
            e=dict(concept=layout,case=scenario,trace=ref(files[layout]),pieces=records,
                   metCarryOnTime=sum(all(r['readyVsCarryS'] is not None and r['readyVsCarryS']>=0 and r['carryHeld'] for k,r in trials.items() if p in k) for p in records),
                   measuredReleases=sum(v['trials']==v['custody']==v['releases'] for v in records.values()),
                   fieldOnlyOutcomes=field,fieldOnlyTrials=len(panels),acceptedPieces=0,verifiedResets=0,
                   unresolved='No complete native successor with guide mounts, paid-out cables, differential arrival sensing, clear-channel readback and loaded drive-to-reset. Field-only scoring cannot qualify the integrated robot.')
            e['runId']=hashlib.sha256(json.dumps(e,sort_keys=True).encode()).hexdigest()
            executions[layout][scenario]=e
    basis=[ROOT/'cad/fusion-authority.json',OUT/'red2-split-drop-fixture-r4.f3d',SPATIAL/'events.csv',MISSION/'field-registration.json',
           MISSION/'target.json',HERE/'blockouts.json',ROOT/'cad/red2-split-drop/rail_transfer.mjs',SPATIAL/'SpatialContact.java',
           OUT/'attained-field/result.csv',OUT/'attained-field/cases.csv',ROOT/'cad/red2-split-drop/station_clock.py']
    dest=HERE/'guided-executions.json'
    write(dest,dict(scope='Concept-only. Each measured free state continues into the alternate signed target dock. The field-only control excludes the robot above its bumper. The two broad sheet faces and proposed guides remain idealized.',sources=[ref(p) for p in basis],runs=executions))
    metrics={'metCarryOnTime':'Identified pipes held by their carry deadline and on both supports through the two-sided chassis acceleration',
             'measuredReleases':'Identified pipes attaining bilateral release in every step and offset challenge',
             'acceptedPieces':'Independently recognized integrated L1 deliveries','verifiedResets':'Confirmed loaded-cycle reset after scoring'}
    study=dict(schema=common['schema'],cases=common['cases'],objectives=[dict(id=k,unit=v,min=2) for k,v in metrics.items()],concepts=[])
    for layout,description in LAYOUTS.items():
        with_runs=[]
        for scenario in executions[layout]:
            prefix=f'/runs/{layout}/{scenario}'
            with_runs.append(dict(case=scenario,source=ref(dest),execution=prefix,values={k:prefix+'/'+k for k in metrics}))
        study['concepts'].append(dict(id=layout,architecture=description,interfaces=[
            dict({'from':'Station','to':'Receiver','exchange':'A unique transverse pipe from the source lip.'}),
            dict({'from':'Receiver','to':'Suspended field target','exchange':'Release from the physically propagated state.'})],runs=with_runs))
    source=HERE/'guided-study.json';write(source,study)
    result=subprocess.run(['node',plugin/'scripts/design-review.mjs',source,ROOT],stdout=subprocess.PIPE,check=True)
    write(HERE/'guided-comparison.json',json.loads(result.stdout))

if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('jdk',type=Path)
    parser.add_argument('installed_plugin',type=Path)
    args=parser.parse_args()
    main(args.jdk,args.installed_plugin)
