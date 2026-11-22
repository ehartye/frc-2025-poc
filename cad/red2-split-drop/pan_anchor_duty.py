"""Join every matched contact result to its bolt reactions and every programmed reset clock."""
import csv
import json
import math
from pathlib import Path

import numpy as np

from output_mount import ROOT, digest, read


def run():
    d=ROOT/'artifacts/red2-integrated'
    folder=d/'pan-thrust'
    full=d/'single-pivot-spatial-rim-installed-reactions-frame-anchors.json'
    ref=d/'single-pivot-spatial-rim-installed-reactions.json'
    source=read(full);r=source['runs'];original=read(ref)['runs']
    if set(r)!=set(original) or len(r)!=40:raise ValueError('All 40 source clocks required')
    base=read(folder/'r3-native-plan.json')
    if digest(ROOT/base['authority']['file'])!=source['sourceSha256']:raise ValueError('Changed current authority')
    peaks={};samples=0
    def keep(group,metric,row,case):
        store=peaks.setdefault(group,{})
        if metric not in store or row['value']>store[metric]['value']:
            store[metric]={**row,'case':case}
    for case,row in r.items():
        if {k:v for k,v in row.items() if k not in ['anchorRows','anchorSamples']} != original[case]:
            raise ValueError('The physical source clock or its established reaction evidence changed: '+case)
        samples+=row['anchorSamples']
        for group,metrics in row['anchorRows'].items():
            for metric,q in metrics.items():
                if 'boltPointMm' in q:
                    side=1 if group.endswith('/port') else -1
                    fx,fy,fz=q['forceToTowerN']
                    m=np.cross([0,side*(.195775-.2235),.1],[fx,fy,fz])
                    if max(abs(m-np.array(q['momentAtFourBoltCenterNm'])))>1e-8:raise ValueError('Wrong moment datum')
                    x=(q['boltPointMm'][0]-205)/1000.;y=(q['boltPointMm'][1]-side*223.5)/1000.
                    ix=4*.019**2;iy=4*.0185**2
                    expect=[fx/4-m[2]*y/(ix+iy),fy/4+m[2]*x/(ix+iy),fz/4+m[0]*y/iy-m[1]*x/ix]
                    if max(abs(np.array(expect)-q['forceN']))>1e-8:raise ValueError('Per-station wrench mismatch')
                keep(group,metric,q,case)
    field=d/'single-pivot-rim-field'
    clocks={p[0]:p for p in csv.reader((field/'assessment-clock.csv').read_text().splitlines())}
    resets=list(csv.DictReader((field/'post-impact-reset.csv').read_text().splitlines()))
    if len(resets)!=120 or {k['case'] for k in resets}!=set(clocks):raise ValueError('All post-release field clocks required')
    shop=read(d/'pan-mechanics.json')
    m=shop['movingShopStockKg'];cg=np.array(shop['movingShopCentreMm'])/1000.
    reset_samples=0;return_types={'return':0,'hold':0}
    for reset in resets:
        key=reset['case'];info=clocks[key];pitch=float(info[1]);free=float(info[3])
        if abs(r[key.rsplit('/',1)[0]]['free']['wallS']-free)>1e-7:raise ValueError('Different physical release')
        end=float(reset['reset_wall_s']);duration=float(reset['elapsed_s'])
        t=np.linspace(0,duration,max(2,math.ceil(duration/.001))+1);reset_samples+=len(t)
        start=end-1.6-free if end>=0 else None
        mode='return' if start is not None else 'hold';return_types[mode]+=1
        u=np.clip((t-start)/1.6,0,1) if start is not None else np.zeros_like(t)
        lo=math.radians(-2)-pitch;sweep=math.radians(70)-lo
        a=lo+sweep*u*u*(3-2*u);vel=sweep*6*u*(1-u)/1.6
        acc=sweep*(6-12*u)/(1.6**2)*((t>=start)&(t<start+1.6)) if start is not None else np.zeros_like(t)
        rx=np.cos(a)*(cg[0]-.205)-np.sin(a)*(cg[2]-.580)
        rz=np.sin(a)*(cg[0]-.205)+np.cos(a)*(cg[2]-.580)
        sx=m*(-9.80665*np.sin(pitch)+acc*rz+vel*vel*rx)
        sz=m*(-9.80665*np.cos(pitch)-acc*rx+vel*vel*rz)
        for side,tower in [(1,'port'),(-1,'starboard')]:
            fx=sx*(1+side*cg[1]/.195775)/2;fz=sz*(1+side*cg[1]/.195775)/2
            lever=side*(.195775-.2235);mx=lever*fz;my=.1*fx;mz=-lever*fx
            group='pipeAndShopMass/'+tower
            def pack(metric,values,extra=None):
                idx=int(np.argmax(values))
                row=dict(value=float(values[idx]),timeS=float(free+t[idx]),phase='post-release-'+mode,forceToTowerN=[float(fx[idx]),0.,float(fz[idx])],
                    momentAtFourBoltCenterNm=[float(mx[idx]),float(my[idx]),float(mz[idx])],shopPhaseDeg=math.degrees(a[idx]))
                if extra is not None:row.update(extra(idx))
                keep(group,metric,row,key)
            pack('netFootUpliftN',np.maximum(fz,0))
            pack('netFootCompressionN',np.maximum(-fz,0))
            pack('radialBearingN',np.hypot(fx,fz))
            for i,x in enumerate([-.019,.019]):
                for j,y in enumerate([-.0185*side,.0185*side]):
                    prefix=f'x{224 if i else 186}/'+('sill/' if j else 'cross/')
                    ix=4*.019**2;iy=4*.0185**2
                    vx=fx/4-mz*y/(ix+iy);vy=mz*x/(ix+iy)
                    vz=fz/4+mx*y/iy-my*x/ix;shear=np.hypot(vx,vy);tens=np.maximum(vz,0);compression=np.maximum(-vz,0)
                    def extra(k):
                        return dict(boltPointMm=[224 if i else 186,side*(242 if j else 205),480],forceN=[float(vx[k]),float(vy[k]),float(vz[k])],
                          shearN=float(shear[k]),tensileN=float(tens[k]),compressionN=float(compression[k]),normalOnSupportN=float(vz[k]),
                          footBearingMPa=float(shear[k])/(4.826*7),singleHostWallSleeveBearingMPa=float(shear[k])/(11*(2 if j else 2.5)))
                    for name,values in [('upliftN',tens),('compressionN',compression),('torsionInclusiveShearN',shear),('combinedEquivalentN',np.hypot(tens,np.sqrt(3)*shear))]:
                        pack(prefix+name,values,extra)
    result=dict(authority=base['authority'],sourceSha256=digest(full),matchedPhysicalReferenceSha256=digest(ref),fieldClockSha256=digest(field/'assessment-clock.csv'),
        fieldResetSha256=digest(field/'post-impact-reset.csv'),shopMassBasisSha256=digest(d/'pan-mechanics.json'),
        matchedPhysicalCases=len(r),actualContactTimesteps=samples,postReleaseCases=len(resets),postReleaseSamples=reset_samples,
        returns=return_types,perTowerAndStation=peaks,model='Four symmetric, equally stiff axes per tower. Equilibrate complete applied force and moment. In-plane torsion apportioned proportional to radius. Compression is unilateral in material; bolts carry uplift. The known R2 authored moving stock mass is included in the named sensitivity; rotating purchases, connector inertia, clamped-joint stiffness, pretension, weld/host capacities, actual clearance or slip, impact life and stamped material certificates remain independent obligations.')
    dest=folder/'r3-all-clock-station-reactions.json'
    dest.write_text(json.dumps(result,indent=2)+'\n')
    for group,rows in peaks.items():
        print(group,[(metric,round(r['value'],3),r['timeS']) for end in ['upliftN','compressionN','torsionInclusiveShearN','combinedEquivalentN'] for metric,r in [max(((k,v) for k,v in rows.items() if k.endswith('/'+end)),key=lambda p:p[1]['value'])]])
    print(len(r) if isinstance(r,dict) else 0,samples,reset_samples,return_types)

if __name__=='__main__':run()
