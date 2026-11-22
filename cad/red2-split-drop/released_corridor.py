"""Follow attained R5 pipe departure to the first independent field contact."""
import argparse
import csv
import json
import math
import sys
from pathlib import Path

import numpy as np
from output_mount import OUT,ROOT,read,digest,place

EYE=np.eye(3)
J=.816466266*(3*(.05715**2+.0508**2)+4*.1508125**2)/12
RADIUS=57.15
HALF_LENGTH=150.8125

def rotate(axis,angle):
    x,y,z=axis
    K=np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])
    return EYE+math.sin(angle)*K+(1-math.cos(angle))*(K@K)

def jaw(time):
    ramp=.6/8;cruise=.09/.6-ramp;total=2*ramp+cruise
    if time<=0:return 0.
    if time<ramp:return 4000*time*time
    if time<ramp+cruise:return 1000*(.5*8*ramp*ramp+.6*(time-ramp))
    if time<total:return 1000*(.09-4*(total-time)**2)
    return 90.

def main(plugin,selected,physical,edge):
    sys.path.insert(0,str(plugin/'scripts'))
    from mechanical_motion import common,distance,bounds
    from showcase_contacts import load_solids
    authority=read(ROOT/'cad/fusion-authority.json')['source']['archive']
    cold=read(OUT/'r5-load-01.json');witness=read(OUT/'r5-seat-01.json')
    step=OUT/'r5-export.step';arc=OUT/'red2-split-drop-fixture-r5.f3d'
    assert digest(ROOT/authority['file'])==cold['authority']['archiveSha256']==authority['sha256']
    assert cold['sha256']==digest(arc)==witness['sha256'] and digest(step)==cold['step']['sha256']
    article=witness['loadedWitness']['pieceStep'];pt=OUT/article['file']
    assert digest(pt)==article['sha256'] and cold['restored'] and witness['restored']
    cargo,extra=load_solids(pt);assert len(cargo)==1 and not extra
    pipe=next(iter(cargo.values()));center=np.array(article['centerMm'])
    solids,extra=load_solids(step);assert len(solids)==93 and not extra
    inventory=read(OUT/'r5-home-native.json')
    assert inventory['archiveSha256']==cold['sha256'] and inventory['contacts']['complete']
    home={b['path']+'/'+b['name']:b['boundsMm'] for b in inventory['bodies']}
    reg={}
    for key,shape in solids.items():
        box=np.array(bounds(shape))
        names=[k for k,v in home.items() if max(abs(np.array(v)-box))<.04]
        if len(names)!=1:raise ValueError('The cold STEP failed to resolve '+key+' '+repr(names))
        n=names[0];group=n.split(':',1)[0]
        if group not in {'Fixture bench rails','Longitudinal carriage','Left splitting support','Right splitting support','Synchronized jaw drive'}:
            raise ValueError('Unregistered moving body '+n)
        reg[n]=[group,box,shape]
    assert len(reg)==len(home)
    if edge and not physical:raise ValueError('Edge source requires physical release')
    label='edge-' if edge else 'physical-' if physical else ''
    sim=OUT/('r5-spatial-r5-as-drawn-challenge-strike-physical-edge.json' if edge else
             'r5-spatial-r5-as-drawn-challenge-strike-physical.json' if physical else 'r4-spatial-r5-as-drawn-challenge-strike.json')
    trace_source=read(sim)
    if physical and trace_source['fixture']!=cold['sha256']:raise ValueError('Release trace is not from the active fixture')
    runs=trace_source['runs'];field=OUT/('r5-'+label+'field/result.csv')
    with field.open() as stream:panel=list(csv.DictReader(stream))
    results={}
    path=OUT/('r5-corridor-'+label+('subset' if selected else 'all')+'.json')
    evidence=dict(authority=authority,archive=cold['sha256'],step=cold['step'],article=article,
        releaseStateSha256=digest(sim),fieldObservationSha256=digest(field),results=results,
        expectedCases=len(selected or runs)*2,
        scope='Measured nominal and adverse launch through pre-impact flight. Bounds include rigid pipe rotation and time-dependent linked rack/pinion opening. The receiver-held aftermath, chassis candidate, dual action sequences, visibility and reset each require separate review.')
    for tag,trace in runs.items():
        if selected and tag not in selected:continue
        for pitch in (-.04,.04):
            relevant=[row for row in panel if row['case'].split('/')[1]==tag and float(row['case'].split('/')[-1])==pitch]
            assert len(relevant)==3
            # Bound until the latest first steel contact among these registered
            # dock samples. Actual dynamics after a particular contact have
            # to follow that receiver's response separately.
            end=max(min(float(r[k]) for k in ('first_floor_s','first_lip_s') if float(r[k])>=0) for r in relevant)
            free=trace['free'];state=free['state'];start=free['wallS']
            p0=np.array(state['p'])*1000;v0=np.array(state['v'])*1000
            a0=np.array(state['axis']);L=np.array(state['angularMomentum']);speed=np.linalg.norm(L)/J
            axis=L/np.linalg.norm(L) if speed else a0
            grav=np.array([-9806.65*math.sin(pitch),0.,-9806.65*math.cos(pitch)])
            d0=math.acos(np.clip(a0[1],-1,1));ax=np.cross([0.,1.,0.],a0)
            R0=rotate(ax/np.linalg.norm(ax),d0) if d0>1e-10 else EYE
            unresolved=[];positive=[];separated=[];narrow=0;initial=[]
            for name,(group,bb,shape) in reg.items():
                sign=1 if group=='Left splitting support' else -1 if group=='Right splitting support' else 0
                rotor=group=='Synchronized jaw drive'
                presented=0 if group=='Fixture bench rails' else 404
                bb0=bb+np.tile([presented,0,0],2)
                rotor_radius=math.hypot(max(abs(bb[0]+147),abs(bb[3]+147)),max(abs(bb[1]),abs(bb[4])))
                if rotor:
                    bb0=np.array([257-rotor_radius,-rotor_radius,bb[2],257+rotor_radius,rotor_radius,bb[5]])
                pending=[(0.,end,0),(0.,0.,0)]
                while pending:
                    lo,hi,depth=pending.pop();mid=(lo+hi)/2;half=(hi-lo)/2
                    c=p0+v0*mid+.5*grav*mid*mid;r=rotate(axis,speed*mid)@R0
                    a=r[:,1];span=HALF_LENGTH*abs(a)+RADIUS*np.sqrt(np.maximum(0,1-a*a))
                    extent=np.concatenate((c-span,c+span))
                    j=jaw(start+mid-trace['openAt'])
                    target=bb0+np.tile([0,sign*j,0],2)
                    speed_piece=np.linalg.norm(v0+grav*mid)*half+.5*np.linalg.norm(grav)*half*half
                    piece_motion=speed_piece+speed*math.hypot(HALF_LENGTH,RADIUS)*half
                    dj=max(abs(j-jaw(start+t-trace['openAt'])) for t in (lo,hi))
                    body_motion=dj*(rotor_radius/10.16 if rotor else abs(sign))
                    budget=piece_motion+body_motion+.0002
                    low=max(*(extent[:3]-target[3:]),*(target[:3]-extent[3:]))
                    if low>budget and (hi>lo or low>2):continue
                    cargo_shape=place(pipe,np.column_stack((r,c-r@center)))
                    joint=rotate([0,0,1],j/10.16) if rotor else EYE
                    shift=np.array([presented,sign*j,0.])
                    if rotor:
                        pivot=np.array([-147.,0.,0.]);shift+=pivot-joint@pivot
                    member=place(shape,np.column_stack((joint,shift)))
                    gap=distance(cargo_shape,member);narrow+=1
                    if hi==lo:
                        initial.append(dict(body=name,gapMm=gap,overlapMm3=common(cargo_shape,member) if gap<1e-6 else 0.))
                        continue
                    if gap>budget:continue
                    if gap<1e-6:
                        volume=common(cargo_shape,member)
                        if volume>.001:
                            positive.append(dict(body=name,timeAfterReleaseS=mid,overlapMm3=volume))
                            break
                    if depth>=17:
                        unresolved.append(dict(body=name,intervalS=[lo,hi],gapMm=gap,motionBoundMm=budget))
                        break
                    # Walk earlier intervals first to preserve first contact.
                    pending.extend([(mid,hi,depth+1),(lo,mid,depth+1)])
                else:separated.append(name)
            result=dict(sampledFieldCases=len(relevant),sourceWallS=start,secondsToLatestFirstFieldContact=end,
                memberCount=len(reg),provenSeparated=len(separated),narrowSamples=narrow,
                positive=positive,unresolved=unresolved,initialNearby=initial)
            results[tag+'/'+str(pitch)]=result
            print(tag,pitch,len(separated),len(positive),len(unresolved),flush=True)
            evidence['allCasesProcessed']=len(results)==evidence['expectedCases']
            tmp=path.with_suffix('.tmp')
            tmp.write_text(json.dumps(evidence,indent=2)+'\n')
            tmp.replace(path)

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('plugin',type=Path);ap.add_argument('traces',nargs='*');ap.add_argument('--physical',action='store_true');ap.add_argument('--edge',action='store_true')
    a=ap.parse_args();main(a.plugin,set(a.traces),a.physical,a.edge)
