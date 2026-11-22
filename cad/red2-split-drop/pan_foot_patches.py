"""Measure the real pan-foot and sill bearing faces in the cold installed assembly."""
import json
import sys
from pathlib import Path

import numpy as np

from output_mount import ROOT, digest, read


def study(plugin,revision):
    sys.path.insert(0,str(plugin/'scripts'))
    from showcase_contacts import load_solids
    from mechanical_motion import bounds, common, distance
    from support_equilibrium import bearing_cells
    out=ROOT/'artifacts/red2-integrated'
    station=out/'pan-native'
    step=station/f'integrated-340mm-pan-{revision}.step'
    issue=read(station/('pan-installation-01.json' if revision=='r1' else 'pan-r2-foundation-install-02.json'))
    proof=read(out/'pan-shop-plan.json')['authority']
    if digest(ROOT/proof['file'])!=proof['sha256'] or digest(step)!=issue.get('derivativeSha256',issue.get('stepSha256')):
        raise ValueError('Unknown issued source or derivative')
    paths={}
    shapes,_=load_solids(step,paths)
    def get(n):
        all_hits=[key for key,v in paths.items() if n in v['occurrencePath'] or n+':1' in v['occurrencePath'] or v['definition']==n or key==n]
        if len(all_hits)!=1:raise ValueError('Ambiguous physical mount '+str((n,all_hits)))
        return all_hits[0],shapes[all_hits[0]]
    sills=[get(n) for n in ['left upper carriage sill','right upper carriage sill']]
    results={}
    for side in ['Port','Starboard']:
        for tail in ['bearing tower foot','fixed inlet mounting foot 1','fixed inlet mounting foot 2']:
            key,foot=get(side+' '+tail)
            sill_name,sill=min(sills,key=lambda pair:distance(pair[1],foot))
            # From the foot side the outward face normal points down. These
            # cells can transmit upward compression to the foot, unilaterally.
            cells=bearing_cells(foot,sill,[0.,0.,-1.],1.)
            area=sum(c['areaMm2'] for c in cells)
            if area<=0:raise ValueError('No measurable finite bearing contact '+key)
            centre=np.average([c['pointMm'] for c in cells],axis=0,weights=[c['areaMm2'] for c in cells])
            results[key]=dict(sill=sill_name,areaMm2=area,centroidMm=centre.tolist(),footBoundsMm=bounds(foot),
                gapMm=distance(foot,sill),commonVolumeMm3=common(foot,sill),pressureCells=cells)
    record=dict(authority=proof,stepSha256=digest(step),pluginProgramSha256=digest(plugin/'scripts/support_equilibrium.py'),
        patches=results,limitations='Measured geometric seating only. No positive axial hold, tie-down, welded throat, shear allowable or host-frame qualification follows from these pressure cells.')
    (station/f'foot-patches-{revision}.json').write_text(json.dumps(record,indent=2))
    print([(k,round(v['areaMm2'],2),v['sill']) for k,v in results.items()])


if __name__=='__main__':study(Path(sys.argv[1]),sys.argv[2])
