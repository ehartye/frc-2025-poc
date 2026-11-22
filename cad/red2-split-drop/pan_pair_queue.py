"""Estimate native census batches from the actual cold issued solid bounds at four poses."""
import itertools
import json
import sys
from pathlib import Path

from output_mount import ROOT, digest, place, read
from pan_mechanics import rotation


def run(plugin):
    sys.path.insert(0,str(plugin/'scripts'))
    from mechanical_motion import bounds
    from showcase_contacts import load_solids
    station=ROOT/'artifacts/red2-integrated/pan-native'
    issue=read(station/'pan-r2-foundation-install-02.json')
    authority=read(ROOT/'cad/fusion-authority.json')['source']['archive']
    archive=station/'integrated-340mm-pan-r2.f3d'
    step=archive.with_suffix('.step')
    if digest(ROOT/authority['file'])!=issue['authority']['archiveSha256'] or digest(step)!=issue['derivativeSha256'] or digest(archive)!=issue[archive.stem]:
        raise ValueError('Changed issue')
    meta={}
    solids,other=load_solids(step,meta)
    if len(solids)!=issue['totalInstalledSolidBodies'] or other:
        raise ValueError('Unexpected STEP scope')
    moving={k for k in solids if '340 mm pan rotating installation' in meta[k]['occurrencePath']}
    initial={k:bounds(s) for k,s in solids.items()}
    seen=set();cases={}
    for row in issue['cold']['poses']:
        angle=row['angleDeg'];boxes=dict(initial)
        if angle!=70:
            for k in moving:
                boxes[k]=bounds(place(solids[k],rotation(angle-70)))
        candidates=[]
        for a,b in itertools.combinations(boxes,2):
            x,y=boxes[a],boxes[b]
            if min(min(x[j+3],y[j+3])-max(x[j],y[j]) for j in range(3))>=-1e-6:
                candidates.append([a,b])
        seen.update(map(tuple,candidates))
        cases[row['pose']]=dict(degrees=angle,estimatedCandidates=len(candidates),pages128=(len(candidates)+127)//128)
    record=dict(authority=authority,nativeArchiveSha256=digest(archive),coldStepSha256=digest(step),installed=len(solids),
        moving=len(moving),allPairs=len(solids)*(len(solids)-1)//2,poses=cases,distinctCandidatePairsAcrossPoses=len(seen),
        scope='OCCT STEP broad-phase scheduling only. Actual native contact collector determines eligible pairs including tolerance and retains measurement or unresolved disposition.')
    (station/'native-page-estimate.json').write_text(json.dumps(record,indent=2))
    print(record)


if __name__=='__main__':run(Path(sys.argv[1]))
