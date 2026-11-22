"""Size the crossframe-to-lower-rail anchors in three dimensions under the recorded and bounded impact cases."""
from pathlib import Path
import json
import hashlib
import math
import numpy as np

HERE=Path(__file__).parent


def run():
    src=HERE/'low-line-frame-clock.json';moments=json.loads(src.read_bytes())
    spec=json.loads((HERE/'socket-holes.json').read_bytes())['rails']
    # Lower anchor axis = midline of actual overlap of lower socket width and inner upper terminal bar.
    limits=[344.6,339.6]; xyz=[]; names=[]
    for name,rail in spec.items():
        ax=rail['lengthAxis'];rad=1-ax
        lo,hi=rail['minimumMm'],rail['maximumMm'];sign=np.sign(lo[rad])
        rng=sorted([sign*(limits[rad]-25.4),sign*limits[rad]])
        radial=(max(lo[rad],rng[0])+min(hi[rad],rng[1]))/2
        if min(hi[rad],rng[1])-max(lo[rad],rng[0])<20:raise ValueError('Missing broad double-wall base')
        for station in (-65,-35,35,65):
            a=np.array([0.,0.,hi[2]])
            a[ax]=station;a[rad]=radial;xyz.append(a);names.append(name)
    xyz=np.array(xyz)
    # Bound all lower installation force as if it crossed the complete group. An unrestrained field robot
    # normally accelerates; this submodel does not assert that all impact force is transmitted to the wheels.
    face_pts=np.array(json.loads((HERE/'full-height-perimeter.json').read_bytes())['outline'])
    settings={}
    for key in ('low-line-budget','heavy-offset-control'):
        role=moments['clock']['cases'][key];m=role['fieldKg'];weak_v=.5477641362858607
        # Recompute the quasi-static foam energy on this proposed mass.
        def foam_force(d):
            h=69.5;A=.100*.138
            return A*((65000/(.25*h))*d if d<=.25*h else 65000+(125000-65000)*(d-.25*h)/(.25*h))
        E=m*weak_v**2/2
        def work(d):
            h=.25*69.5
            return (.5*foam_force(d)*d if d<=h else .5*foam_force(h)*h+.5*(foam_force(h)+foam_force(d))*(d-h))/1000
        left,right=0.,34.75
        if work(right)<E:raise ValueError('Interpolation outside supplier range')
        for _ in range(50):
            middle=(left+right)/2
            if work(middle)<E:left=middle
            else:right=middle
        settings[key]={'routineN':foam_force(right),'routineJ':E,'routineStrokeMm':right,
            'upperN':role['conditional18mmCruiseForceN'],'upperJ':role['conditionalCruiseEnergyJ']}
    statuses={}
    for which,vals in settings.items():
        for phase in ('routine','upper'):
            load=vals[phase+'N']
            for missing in [None,*range(16),'entire port rail']:
                active=[j for j in range(16) if j!=missing and not(missing=='entire port rail' and names[j]=='Port longitudinal socket rail')]
                coords=xyz[active];ctr=coords.mean(axis=0);uv=coords[:,:2]-ctr[:2];group=len(active)
                norm=np.vstack([np.ones(group),uv[:,1],-uv[:,0]])
                for k,(p,q) in enumerate(zip(face_pts,np.roll(face_pts,-1,axis=0))):
                    t=q-p;unit=np.array([-t[1],t[0],0.])/np.linalg.norm(t)
                    for fraction in (0,.5,1):
                        xy=p+t*fraction
                        for h in (114.3,150.):
                            for sign in (1,-1):
                                F=unit*sign*load;M=np.cross(np.r_[xy,h]-ctr,F)
                                shear=F[:2]/group+M[2]*np.column_stack((-uv[:,1],uv[:,0]))/(uv*uv).sum()
                                vertical=np.linalg.lstsq(norm,[0,M[0],M[1]],rcond=None)[0]
                                if np.linalg.norm(shear.sum(axis=0)-F[:2])>1.e-6 or np.linalg.norm(norm@vertical-np.array([0,M[0],M[1]]))>1.e-6:
                                    raise ValueError('Anchor wrench did not close')
                                tag=which+'/'+phase+'/'+('intact' if missing is None else 'all-port-lost' if isinstance(missing,str) else 'one-lost')
                                bucket=statuses.setdefault(tag,{})
                                for i,j in enumerate(active):
                                    axial=vertical[i];sv=float(np.linalg.norm(shear[i]));long=spec[names[j]]['lengthAxis'];side=1-long
                                    projected=min(12.65*2.0*120,12.65*3.0*120)
                                    metrics={'maxShearN':sv,'maxTensionN':max(axial,0),'maxCompressionN':max(-axial,0),
                                          'maxPilotWallBearingFraction':sv/projected,
                                          'maxFlangeThreadProofScreen':(9000+max(0,axial)*2)/(.5*math.pi*4.97*11*(105000*.006894757293168/math.sqrt(3)/2))}
                                    for kind,n in metrics.items():
                                        if n>bucket.get(kind,{}).get('value',-1):
                                            bucket[kind]=dict(value=n,station=names[j],anchorMm=coords[i].tolist(),lost=missing,contactMm=[*xy.tolist(),h],signedF=F.tolist(),
                                            simultaneousLocalN=dict(axial=axial,along=shear[i,long],radial=shear[i,side]))
    return dict(massAndDutySha256=hashlib.sha256(src.read_bytes()).hexdigest(),anchors=[dict(rail=n,xyzMm=q.tolist()) for n,q in zip(names,xyz)],
        settings=settings,results=statuses,scope='16 foundation anchors in the actual 488 coordinates. Static rigid/equal in-plane stiffness and elastic equal axial springs bound trial force and moment sharing. Compression acting in the negative fastener direction actually goes into the 6.97 mm contact saddle plate. One entire lost rail is a severe configuration control. Positive engagement, local web/sidewall strain, differing bracket stiffness, net section and under-head preload require installed checks; this is not ground friction, a dynamic collision reconstruction, or full-frame qualification.')

if __name__=='__main__':
    r=run();(HERE/'lower-terminal-anchor-loads.json').write_text(json.dumps(r,indent=2)+'\n')
    for k,v in r['results'].items():print(k,*(round(v[n]['value'],3) for n in ('maxShearN','maxTensionN','maxPilotWallBearingFraction')))
