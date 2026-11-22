"""Bound the sidewall fasteners from matched bearing loads and retain simultaneous real events."""
import json
import itertools
from pathlib import Path
import numpy as np
from output_mount import ROOT,digest,read


def solve():
    d=ROOT/'artifacts/red2-integrated';p=d/'pan-thrust'
    g=read(p/'sidewall-foot-geometry.json');full=read(p/'r3-all-clock-station-reactions.json')
    assert g['authority']==full['authority'] and digest(ROOT/g['authority']['file'])==g['authority']['sha256']
    assert digest(d/'single-pivot-spatial-rim-installed-reactions-frame-anchors.json')==full['sourceSha256']
    events=read(d/'single-pivot-spatial-rim-installed-reactions-frame-anchors.json')['runs']
    comparison=read(p/'underframe-brace-strength.json')
    fact=2.; rows={}
    for side,s in [('port',1),('starboard',-1)]:
        group='pipeAndShopMass/'+side;measures=full['perTowerAndStation'][group]
        radius=measures['radialBearingN']['value'];axial=measures['axialBearingN']['value']
        for sx,sz in itertools.product([-1,1],repeat=2):
            id=f'{side}/{205+28*sx:g}/{465+7.5*sz:g}'
            x=.028*sx;z=.0075*sz;iy=4*x*x;iz=4*z*z;J=iy+iz
            arm=np.array([0.,s*(.195775-.251),.115])
            def reaction(force):
                fx,fy,fz=force;mx,my,mz=np.cross(arm,force)
                return np.array([fx/4+my*z/J,fy/4-mx*z/iz+mz*x/iy,fz/4-my*x/J])
            mat=np.column_stack([reaction(e) for e in np.eye(3)])
            # Extrema of a linear form on |Fy|<=A and Fx²+Fz²<=R².
            # This contains every synchronized case and every intermediate
            # contact/reset timestep, and need not occur as one physical sample.
            tension=fact*(abs(mat[1,1])*axial+np.linalg.norm(mat[1,[0,2]])*radius)
            shear=fact*(np.linalg.norm(mat[np.ix_([0,2],[0,2])],ord=2)*radius+
                        np.linalg.norm(mat[[0,2],1])*axial)
            real={}
            for name,run in events.items():
                for label,m in run['anchorRows'][group].items():
                    f=np.array(m['forceToTowerN']);q=fact*reaction(f)
                    for k,v in [('tensionN',s*q[1]),('compressionN',-s*q[1]),('shearN',np.linalg.norm(q[[0,2]]))]:
                        if v>real.get(k,{'value':-1})['value']:
                            real[k]=dict(value=float(v),case=name,timeS=m['timeS'],sourceChannel=label,actualF=f.tolist(),reaction=q.tolist())
            row=g['sites'][id]
            min_thread=row['screwIntoThreadMm']-.65-2*25.4/32
            r={k:np.asarray(v).tolist() for k,v in dict(absoluteCompleteClockTensionN=tension,absoluteCompleteClockShearN=shear).items()}
            r.update(sampledSynchronizedPeaks=real,expandedClockEnvelope={'radialN':radius,'axialN':axial},boltXZmm=[205+x*1000,465+z*1000],
                minThreadAfterToleranceAndTwoPitchesMm=min_thread,guaranteedClampAfterWorstEnvelopeN=3600*.65*.8-tension,
                oneWallRadialBearingMPa=shear/(9.86*4),screwAtLowAchievedClampWithSampledTensionN=3600*.65*.8-real['tensionN']['value'],
                shellGrossTopAndBottomLigamentMm=30-2*14.1,maxTwoHolesNetWallVerticalAreaMm2=2*(30.-2*14.1))
            rows[id]=r
    res={'authority':g['authority'],'sourceCompleteClock':digest(p/'r3-all-clock-station-reactions.json'),
        'geometry':digest(p/'sidewall-foot-geometry.json'),'matchedStoredPhaseCases':len(events),'safetyFactor':2,
        'cases':rows,'baseline':{k:v['tests']['0.35']['clampAfterFactoredUpliftN'] for k,v in comparison['cases'].items()},
        'scope':'Independent coupled equilibrium per coincident applied force. Bounds cover both extrema and full existing contact/reset clock. New weld, aluminum HAZ, frame longitudinal bending, tube shear and local bearing need engineering. Peak matched rows are a lower bound. Hole-plus-weld dimensions require host restoration. No mount acceptance.'}
    (p/'sidewall-foot-strength.json').write_text(json.dumps(res,indent=2)+'\n')
    print('max synchronized tension',max((r['sampledSynchronizedPeaks']['tensionN']['value'],i) for i,r in rows.items()))
    print('min synchronized clamp',min((r['screwAtLowAchievedClampWithSampledTensionN'],i) for i,r in rows.items()))
    print('max enclosed tension',max((r['absoluteCompleteClockTensionN'],i) for i,r in rows.items()))
    print('max enclosed shear',max((r['absoluteCompleteClockShearN'],i) for i,r in rows.items()))


if __name__=='__main__':solve()
