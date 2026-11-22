"""Bind R5 native materials, seats, bolt contacts and field tests to the actual issue."""
import collections
import csv
import json
from pathlib import Path
import numpy as np
from output_mount import ROOT,OUT,digest,read

def ref(path):return dict(file=path.relative_to(ROOT).as_posix(),sha256=digest(path))

def mate_keys(scan):
    rows={}
    for pair in scan['contacts']['pairs']:
        volumes=[pair['interferenceMm3'],pair['temporaryCommonMm3']]
        if max(volumes)<=.01:continue
        bodies=[scan['bodies'][i] for i in pair['pair']]
        if 'REV-29-1016' in bodies[0]['path']:bodies.reverse()
        if 'REV-29-1016' not in bodies[1]['path'] or not any(s in bodies[0]['path'] for s in ('REV-29-2947','REV-29-2951','REV-29-2959')):
            raise ValueError('New unqualified hardware conflict '+str(pair['identities']))
        key=tuple('/'.join([*b['path'].split('+')[:2],b['name']]) for b in bodies)
        m,n=(np.array(b['worldTransform']).reshape(4,4) for b in bodies)
        if key in rows:raise ValueError('Duplicate relative placement')
        rows[key]=(volumes,np.linalg.inv(n)@m)
    assert len(rows)==48 and len({k[0] for k in rows})==24
    return rows

def main():
    authority=read(ROOT/'cad/fusion-authority.json')['source']['archive']
    assert digest(ROOT/authority['file'])==authority['sha256']
    artifacts={}
    def data(name):
        p=OUT/name;artifacts[name]=ref(p);return read(p)
    cold=data('r5-load-01.json');seat=data('r5-seat-01.json')
    archive=OUT/'red2-split-drop-fixture-r5.f3d'
    assert cold['authority']['archiveSha256']==authority['sha256'] and cold['sha256']==seat['sha256']==digest(archive)
    assert cold['restored'] and seat['restored'] and cold['step']['sha256']==digest(OUT/'r5-export.step')
    raw=data('r5-material-preflight-01.json');assert len(raw['actual'])==76 and raw['restored']
    mats=dict(collections.Counter(cold['native']['materialReadback'].values()));assert sum(mats.values())==93
    scans=[];compare=data('r4-qualified-catalog-mates.json')
    for pose in ('home','present','clear','before-release'):
        new=data(f'r5-{pose}-native.json');old=data(f'r4-{pose}-native.json')
        assert new['archiveSha256']==cold['sha256'] and old['archiveSha256']==compare['archive']['sha256']
        a,b=mate_keys(old),mate_keys(new)
        assert a.keys()==b.keys() and new['contacts']['complete'] and not new['contacts']['unresolvedPairs']
        d=max(np.max(np.abs(a[k][1]-b[k][1])) for k in a)
        v=max(np.max(np.abs(np.array(a[k][0])-np.array(b[k][0]))) for k in a)
        assert d<.00001 and v<.02
        scans.append(dict(pose=pose,measured=len(new['contacts']['pairs']),positive=len(b),maxSourceRelativeTransformDifference=float(d),
                          maxDifferenceFromIndependentlySourceClassifiedR4Mm3=float(v)))
    floor=next(iter(data('r5-exact-floor-plane.json')['result'].values()))
    narrow=floor['allAmbiguousFinite'];assert len(narrow)==48 and floor['measurements']==320
    assert not any(q['surfaceCrossingsMm'] for q in narrow)
    riser=data('r5-jaw-riser-clearance.json')['result'];assert len(riser)==2
    challenge=data('r4-spatial-r5-as-drawn-challenge-strike.json')['runs']
    max_strip=max(x['guideMoment']['momentNmm'] for x in challenge.values() if x['guideMoment'])
    upper=[x['guideAboveRibs']['cantileverMomentNmm'] for x in challenge.values() if x['guideAboveRibs']]
    assert None not in upper
    outside=max(upper);design=1.5*max(outside,max_strip)
    # Rectangular strip at the real point along the rib span. This is a
    # conditional structural bound; the effective contact width is unmeasured.
    strips={w:6*design/(w*5**2) for w in (1,3,5,10)}
    field=OUT/'r5-field/result.csv';artifacts['r5-field/result.csv']=ref(field)
    with field.open() as f:actions=dict(collections.Counter(r['outcome'] for r in csv.DictReader(f)))
    for n in ('r5-build-02.json','r5-finalize-01.json','r5-census-01.json','output-mount-fit.json','output-thread-witness.json','seam-options.json'):
        data(n)
    output=dict(authority=authority,qualifyingWholeRobot=False,
        checkpoint=ref(OUT/'red2-split-drop-fixture-r5-geometry.f3d'),archive=ref(archive),exchange=cold['step'],
        bodyCount=sum(mats.values()),materials=mats,poses=scans,
        floor=dict(signedNormalSamples=320,matchedFiniteSamples=48,minFiniteMm=min(q['minFiniteFieldSurfaceGapMm'] for q in narrow),
              minimumWithOneMmLocalFabricationError=floor['worst']['oneMmPerAxisNetFloorNormalGapMm']),
        risers={k:dict(samples=len(v['cases']),minConservativeMm=v['conservativeMinMm']) for k,v in riser.items()},
        pipeWitnesses={v['pose']:dict(bodies=len(v['pairs']),maximumOverlapMm3=max(max(p['volumeMm3'],p['temporaryCommonMm3']) for p in v['pairs'])) for v in [cold['loadedWitness'],seat['loadedWitness']]},
        asDrawnGuide=dict(cases=len(challenge),maxForceN=max(r['guideForceN'] for r in challenge.values()),
            peakTwoRibMomentNmm=max_strip,peakSingleReachableRibCantileverMomentNmm=outside,designFactor=1.5,
            peakStressMPaByEffectiveStrikeWidthMm=strips,effectiveWidthRequiredFor105MPaMm=6*design/(25*105),
            limits='Effective impact width, rib and HAZ stiffness, fracture, repeated weld loads and physical validation remain open.'),
        fieldOnly=actions,recognizedCompleteRobotScores=0,verifiedCompleteRobotResets=0,inputs=artifacts)
    (ROOT/'cad/red2-split-drop/r5-evidence.json').write_text(json.dumps(output,indent=2)+'\n')
    print(scans);print(output['asDrawnGuide'])

if __name__=='__main__':main()
