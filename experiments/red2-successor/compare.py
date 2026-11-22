"""Screen three original Red 2 concepts on common station stock, clock and field."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
from capture import capture
from gate_field import check as gate_check

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
EVID=ROOT/'artifacts/red2-architecture-evidence'
MISSION=ROOT/'experiments/six-wheel/mission'
SPATIAL=ROOT/'experiments/six-wheel/spatial-contact'
sys.path.insert(0,str(SPATIAL))
from physics import panel_file

def read(p):return json.loads(p.read_bytes())
def write(p,v):p.write_bytes((json.dumps(v,indent=2,allow_nan=False)+'\n').encode())
def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def ref(p):return dict(path=p.relative_to(ROOT).as_posix(),sha256=digest(p))
def csvrows(p,rows):
    with p.open('w',newline='') as f:csv.writer(f,lineterminator='\n').writerows(rows)

def render(name,g,common):
    import html
    def xy(x,z):return (225+x*.34,397-z*.32)
    def box(b,color,label):
        (x,y),(xr,yr)=xy(b[0],b[5]),xy(b[3],b[2])
        return f'<g><title>{html.escape(label)}</title><rect x="{x:.1f}" y="{y:.1f}" width="{xr-x:.1f}" height="{yr-y:.1f}" fill="{color}" fill-opacity=".2" stroke="{color}" /></g>'
    def line(p,q,color):
        x,y=xy(*p);u,v=xy(*q);return f'<line x1="{x}" y1="{y}" x2="{u}" y2="{v}" stroke="{color}" stroke-width="3"/>'
    def text(x,y,value):return f'<text x="{x}" y="{y}" font-family="system-ui" font-size="12">{html.escape(value)}</text>'
    items=[f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 650 460"><title>{name} candidate side elevation and plan, millimetres</title><rect width="650" height="460" fill="#faf8f4"/>',text(12,19,name+' | concept only, chassis origin X0 Z0'),text(12,40,'Blue = stowed housing. Orange = deployed support. Green = service. Purple = fixed camera sightlines.')]
    items.extend([box([-425,-425,42.5,425,425,181.5],'#374151','Registered bumper'),box(g['boundsStowed'],'#0369a1','Stowed concept'),box(g['access'],'#166534','Tool and removal corridor'),box(common['service']['batteryLiftBox'],'#166534','Battery upward service'),line(*g['supportDeployedXZ'],'#c45b14'),line(*g['supportXZ'],'#155e75'),line([490,0],[490,457.2],'#990f26'),text(386,385,'L1 lip')])
    for k in ['station','reef']:
        p=common['cameras'][k];q=common['cameras'][k+'RayTo'];items.append(line([p[0],p[2]],[q[0],q[2]],'#7141a3'))
    for a,b in zip(g['receiveProfileMm'],g['receiveProfileMm'][1:]):items.append(line(a,b,'#257b73'))
    items.extend([text(455,80,'Plan view'),'<g transform="translate(540,165) scale(.16,-.16)">','<polygon points="-350,-285 -285,-350 350,-350 350,350 -285,350 -350,285" fill="none" stroke="#374151" stroke-width="8"/>'])
    b=g['boundsStowed'];items.append(f'<rect x="{b[0]}" y="{b[1]}" width="{b[3]-b[0]}" height="{b[4]-b[1]}" fill="#0369a1" opacity=".35"/>')
    for x in [-282,282]:
        for y in [-277,277]:items.append(f'<circle cx="{x}" cy="{y}" r="38" fill="#222"/>')
    items+=['</g>',text(18,428,'Robot front →   •   Diagram is parametrically generated from blockouts.json.'),'</svg>']
    (HERE/(name+'.svg')).write_text(''.join(items),encoding='utf-8')

def assumed_state(name,g,pitch,heave,case):
    raw=g['releaseLocalMm'];x,z=raw[0]/1000,raw[2]/1000
    if name=='fixed-rollers':
        speed=g['exitSpeedMps']*(.77 if case=='adverse' else 1)
        v=(speed*math.cos(pitch),speed*math.sin(pitch))
        spin=1.5 if case=='nominal' else 12.
    else:
        v=(.035 if case=='adverse' else 0.,-.08)
        spin=.6 if case=='adverse' else 0.
    return [x*math.cos(pitch)-z*math.sin(pitch),z*math.cos(pitch)+x*math.sin(pitch)+heave,*v,spin]

def main(jdk):
    EVID.mkdir(parents=True,exist_ok=True)
    block=read(HERE/'blockouts.json');layouts=block['layouts'];shared=block['shared']
    frozen=read(MISSION/'field-registration.json')
    snapshots=read(ROOT/'experiments/swerve-drive/low-placement/results.json')['cases']
    target=MISSION/'target.json';panel_file(read(target),EVID/'panels.csv')
    events=list(csv.DictReader((SPATIAL/'events.csv').open()))
    clock={c:[e for e in events if e['approach']=='nominal' and e['scenario']==scenario] for c,scenario in [('nominal','nominal'),('adverse','lateB')]}
    tip=read(HERE/'inputs/tray-release.json')
    panel_rows=[];trials={}
    for name,g in layouts.items():
        render(name,g,shared)
        for case,k,mu in [('nominal','plus',.35),('adverse','minus',.10)]:
            c=snapshots[k]['chassis'];poses=[c['terminalMmMmRad'],*c['enclosure']]
            # The same three dock poses, two pitch/heave extremes, solver and
            # allowance are applied to every candidate. Runs cover both pipes.
            pieces={p['id']:{'readyS':p['readyS'],'sector':sector} for p,sector in zip(frozen['supply'],('A','B'))}
            for e in clock[case]:
                pieces[e['piece']][e['event']+'AtWallS']=float(e['wall_ms'])/1000
            for pid in pieces:
                for j,(dx,dy,yaw) in enumerate(poses):
                    for pitch,heave in [(-.04,-.005),(.04,.005)]:
                        if name=='tip-tray':
                            row=next(q for q in tip['cases'] if q['mu']==mu and q['pitch']==pitch and q['heaveM']==heave and q['dtS']==.0001 and q['gateSpeedMps']==.125)
                            a=row['result'];x,_,z=a['localPositionM'];vx,_,vz=a['localVelocityMps'];omega=a['omegaRadS']
                        else:x,z,vx,vz,omega=assumed_state(name,g,pitch,heave,case)
                        sector_shift=0. if pid=='coral-A' else frozen['redReef']['sectorOffsetsM'][1]-frozen['redReef']['sectorOffsetsM'][0]
                        co,si=math.cos(yaw),math.sin(yaw);xx,yy=dx/1000-.490,dy/1000+sector_shift
                        tag=f'{name}/{case}/{pid}/{j}/{pitch}';panel_rows.append([tag,xx+co*x,yy+si*x,z,co*vx,si*vx,vz,-si,co,0,-si*omega,co*omega,0,xx,yy,yaw,-.001,3.,.0001,17,mu])
            half=block['datum']['frameHalfXY'];cg=g['mechanismCGmm'];base=shared['limits']
            cg_cases=[]
            for base_kg in base['bareBaseAllowanceKg']:
                for mech_kg in [g['massLowerKg'],g['massUpperKg']]:
                    m=[(base_kg,base['baseCGmm']),(mech_kg,cg),(shared['power']['batteryKg'],shared['power']['batteryCenter']),(base['withBumpersKg'],[0,0,112]),(base['pipeKg'],g['releaseLocalMm'])]
                    total=sum(q for q,_ in m);cg_cases.append([sum(q*v[i] for q,v in m)/total for i in range(3)])
            resources=[]
            for pid,v in pieces.items():
                t=v['releaseAtWallS'];start=t-1.2
                def use(r,a,b,n,label):resources.append(dict(resource=r,startS=a,endS=b,amount=n,activity=pid+' '+label))
                use('battery',start,t+.7,shared['power']['otherA'],'electronics and control')
                use('battery',start,t,shared['power']['driveBranchPeakA'],'drive allocation on final approach')
                use('battery',t,t+.7,20,'stopped drive hold allocation')
                use('battery',start if name=='fixed-rollers' else t,t+.7,g['handlingA'],'handling current allowance')
                use('human',v['feedAtWallS']-3,v['feedAtWallS'],1,'one station keeper')
                use('receiver',v['feedAtWallS'],t,1,'occupied receiver allocation')
            arrival=capture(name,case,g)
            trial=dict(concept=name,case=case,pieces=pieces,resourceUses=resources,capture=arrival,
                       retainedOnMatchedPulse=int(arrival['heldAfterAcceleration']),
                       staticStartMarginMm=half-g['boundsStowed'][3],inspectionAllocationMarginKg=base['inspectionMassKg']-base['bareBaseAllowanceKg'][1]-g['massUpperKg']-base['reserveEmptyMassKg'],
                       loadedCGMinMm=[min(p[i] for p in cg_cases) for i in range(3)],loadedCGMaxMm=[max(p[i] for p in cg_cases) for i in range(3)],
                       hypothesis='Allocated current, mass, geometry and window. The new robot has no integrated source-to-score episode.')
            trial['runId']=hashlib.sha256(json.dumps(trial,sort_keys=True).encode()).hexdigest();trials[name,case]=trial
    csvrows(EVID/'cases.csv',panel_rows)
    csvrows(EVID/'removed-hardware-control.csv',[['t','withdraw_m'],[0,.110],[.001,.110]])
    subprocess.run([jdk/'bin/javac.exe','-d',EVID,SPATIAL/'SpatialContact.java'],check=True)
    subprocess.run([jdk/'bin/java.exe','-cp',EVID,'SpatialContact',EVID/'panels.csv',EVID/'removed-hardware-control.csv',EVID/'cases.csv',EVID/'field.csv',EVID/'trace.csv','1.0','0','sweep-y'],check=True)
    results=list(csv.DictReader((EVID/'field.csv').open()))
    gate=gate_check(ROOT)
    write(HERE/'straight-gate-field-control.json',gate)
    out={n:{} for n in layouts}
    for (name,case),t in trials.items():
        subset=[r for r in results if r['case'].startswith(name+'/'+case+'/')]
        t['fieldOnlySupportedFraction']=sum(r['outcome']=='supported' for r in subset)/len(subset)
        t['fieldOnlyOutcomes']={v:sum(r['outcome']==v for r in subset) for v in sorted(set(r['outcome'] for r in subset))}
        if name=='tip-tray':t['straightGateFieldClearanceMm']=-gate['worst']['verticalBreachMm']
        for pid,p in t['pieces'].items():
            p['chain']=[dict(stage='station feed',wallS=p['feedAtWallS'],scope='Same scheduled identified pipe and clock'),dict(stage='contact',wallS=p['feedAtWallS']+t['capture']['feedToLipS']+t['capture']['firstContactS'],scope='Candidate modeled mouth contact'),dict(stage='capture-and-hold',success=bool(t['capture']['heldAfterAcceleration']),scope='Seating and applied acceleration, before transit'),dict(stage='release window',wallS=p['releaseAtWallS'],scope='Conditioned exit state, independently of upstream capture'),dict(stage='field assessment',outcomes=[r['outcome'] for r in subset if '/'+pid+'/' in r['case']],scope='Hardware-removed control only'),dict(stage='reset',scheduledAtWallS=p.get('stowAtWallS'),verified=False,scope='A missing end-state inhibits next feed; fallback to stationary loaded hold or empty park.')]
        t['unresolved']=['Complete installed pipe custody and two sector-specific scores','Deployed-hardware separation','Station acquisition with pipe attitude and camera occlusion','Repeatable reset and safe stow','Measured electrical load and CG','Endgame parking with five peers']
        out[name][case]=t
    write(HERE/'executions.json',dict(scope='Concept screening only. The field solver intentionally removes deployed hardware beyond X425. A field-only supported pipe cannot establish a score. Separate sectors use their signed local target offset. Both concepts with defined exit velocities are conditional bounds.',sources=[ref(p) for p in (ROOT/'cad/fusion-authority.json',MISSION/'field-registration.json',MISSION/'moving-traffic.json',target,SPATIAL/'events.csv',SPATIAL/'SpatialContact.java',ROOT/'experiments/swerve-drive/low-placement/red2-demand.json',ROOT/'experiments/swerve-drive/low-placement/results.json',HERE/'blockouts.json',HERE/'inputs/tray-release.json',EVID/'field.csv')],runs=out))
    route=ref(HERE/'executions.json');concepts=[]
    for n,g in layouts.items():
        runs=[]
        for c in ['nominal','adverse']:
            path='/runs/'+n+'/'+c
            runs.append(dict(case=c,source=route,execution=path,values={k:path+'/'+k for k in ('retainedOnMatchedPulse','fieldOnlySupportedFraction','staticStartMarginMm','inspectionAllocationMarginKg')}))
        concepts.append(dict(id=n,architecture=g['path'],interfaces=[{'from':'station operator','to':'receiver','exchange':g['capture']},{'from':'receiver','to':'reef','exchange':g['release']},{'from':'custody sensing','to':'driver','exchange':g['reset']},{'from':'pit crew','to':'mechanism','exchange':g['service']}],runs=runs))
    study=dict(schema='agent-frc/design-review@1',objectives=[dict(id='acceptedPieces',unit='distinct independently scored pipes per common 135 s window',min=2),dict(id='fieldOnlySupportedFraction',unit='conditional fraction',min=1),dict(id='staticStartMarginMm',unit='mm fore-aft stow allowance',min=0),dict(id='inspectionAllocationMarginKg',unit='kg modeled inspection allocation',min=0)],cases=[dict(id='nominal',condition='Recorded Red 2 plus-wheel dock and nominal station clock.'),dict(id='adverse',condition='Opposite wheel calibration, biased release, low friction and late-B clock.')],resources=[dict(id='battery',unit='A allocated',capacity=150,basis='Provisional simultaneous-bus allocation pending native load model.'),dict(id='receiver',unit='slots',capacity=1,basis='Single-pipe custody.'),dict(id='human',unit='operators',capacity=1,basis='One station operator, 3 seconds per feed.')],concepts=concepts)
    study['objectives'].append(dict(id='retainedOnMatchedPulse',unit='registered pipe remaining inside the receiving supports after the acceleration pulse',min=1))
    study['objectives'].append(dict(id='straightGateFieldClearanceMm',unit='mm mechanism to registered L1 floor',min=0))
    for c in concepts:
        if c['id']=='tip-tray':
            for run in c['runs']:run['values']['straightGateFieldClearanceMm']=run['execution']+'/straightGateFieldClearanceMm'
    write(HERE/'study.json',study)

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('jdk',type=Path)
    main(parser.parse_args().jdk)
