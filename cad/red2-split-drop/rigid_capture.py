"""Advance the incoming pipe by impulses against finite receiver faces from the source lip."""
import hashlib
import json
import math
import sys
from pathlib import Path

ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'artifacts/split-drop-native'
sys.path.insert(0,str(ROOT/'experiments/six-wheel/scripts'))
from receiver_contact import departure,flight_center

def simulate(slow,dt):
    feed=json.loads((ROOT/'experiments/six-wheel/sourced-match/source-545.json').read_bytes())['feed']
    adm=json.loads((ROOT/'experiments/red2-successor/blockouts.json').read_bytes())
    layout=next(v['split-drop'] for v in adm.values() if isinstance(v,dict) and 'split-drop' in v)
    poly=[(x/1000,z/1000) for x,z in layout['receiveProfileMm']]
    mass,r=.816466266,.05715
    inertia=mass*(r*r+.0508**2)/2
    speed=feed['lip_mps']*(.85 if slow else 1.)
    launch=departure(speed);x,z=flight_center(launch,0)
    x-=.444525+(.020 if slow else 0.)
    vx,vz,w=launch['velocity_x_mps'],launch['velocity_z_mps'],speed/r
    mu=.15 if slow else .35
    last=[];seat=None;dwell=0;first=None;maxpen=0.;trace=[]
    for i in range(round(5/dt)):
        t=i*dt
        accel=(1.5 if slow else .9)*(-1 if 2.8<t<3.5 else 1 if 3.7<t<4.4 else 0)
        vx+=accel*dt;vz-=9.80665*dt
        contacts=[]
        for j,((x0,z0),(x1,z1)) in enumerate(zip(poly,poly[1:])):
            dx,dz=x1-x0,z1-z0
            a=max(0,min(1,((x-x0)*dx+(z-z0)*dz)/(dx*dx+dz*dz)))
            rx,rz=x-x0-a*dx,z-z0-a*dz
            length=math.hypot(rx,rz);gap=length-r
            nx,nz=rx/length,rz/length
            vn=vx*nx+vz*nz
            if gap>max(.000005,-vn*dt):continue
            maxpen=max(maxpen,-gap);target=-gap/dt
            if vn<-.18:target=max(target,-.10*vn)
            contacts.append([j,nx,nz,target,0.,0.])
        for _ in range(10):
            for contact in contacts:
                j,nx,nz,target,normal,friction=contact;tx,tz=nz,-nx
                augmented=max(0,normal+mass*(target-vx*nx-vz*nz))
                dn=augmented-normal
                vx+=dn*nx/mass;vz+=dn*nz/mass
                speed_t=vx*tx+vz*tz-w*r
                corrected=min(mu*augmented,max(-mu*augmented,friction-speed_t/(1/mass+r*r/inertia)))
                df=corrected-friction;vx+=df*tx/mass;vz+=df*tz/mass;w-=df*r/inertia
                contact[4:]=[augmented,corrected]
        x+=dt*vx;z+=dt*vz
        last=[c[0] for c in contacts if c[4]>1e-10]
        if last and first is None:first=t
        dwell=dwell+dt if len(last)>=2 and math.hypot(vx,vz)<.005 and abs(w)<.05 else 0
        if dwell>.12 and seat is None:seat=t
        if i%max(1,round(.025/dt))==0:trace.append([round(t,7),x*1000,z*1000,vx,vz,w,last])
        if z<.15 or abs(x)>1.2:break
    return dict(dtS=dt,firstContactS=first,settledS=seat,centerMm=[x*1000,z*1000],velocityMps=[vx,vz],
                spinRadS=w,finalSupportFaces=last,largestStartingPenetrationMm=maxpen*1000,tailTimeS=t,trace=trace)

def main():
    register=json.loads((ROOT/'cad/fusion-authority.json').read_bytes())['source']['archive']
    if hashlib.sha256((ROOT/register['file']).read_bytes()).hexdigest()!=register['sha256']:raise ValueError('Changed canonical source')
    native=json.loads((OUT/'r3-seat-01.json').read_bytes())
    if hashlib.sha256((OUT/'red2-split-drop-fixture-r3.f3d').read_bytes()).hexdigest()!=native['sha256']:raise ValueError('Changed fixture')
    ref=native['loadedWitness']['worldCenterMm']
    trials={}
    for slow in (False,True):
        case='adverse' if slow else 'nominal'
        trials[case]=[]
        for dt in [.0004,.0002,.0001]:
            run=simulate(slow,dt);run['distanceFromNativeSeatMm']=math.dist(run['centerMm'],[ref[0]-404,ref[2]])
            trials[case].append(run)
    evidence=dict(authority=register['sha256'],fixture=native['sha256'],nativeRigidSeatMm=ref,
       method='Finite 2D rigid transverse annulus from registered station exit, gravity, unilateral normal impulses, Coulomb friction, restitution 0.1 and matched chassis acceleration. Captured coordinates are integrated. Opposing support span and asymmetric lateral release remain separate.',
       trials=trials)
    (OUT/'r3-rigid-station.json').write_text(json.dumps(evidence,indent=2))
    for case,runs in trials.items():print(case,[(r['settledS'],r['distanceFromNativeSeatMm'],r['finalSupportFaces']) for r in runs])
if __name__=='__main__':main()
