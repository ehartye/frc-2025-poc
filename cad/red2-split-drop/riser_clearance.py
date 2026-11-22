"""Compare both installed R5 jaws with the two complete enclosing field riser boxes."""
import argparse
import itertools
import json
import sys
from pathlib import Path

import numpy as np
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
from OCP.gp import gp_Pnt
from field_plane import rotation
from output_mount import ROOT,OUT,digest,read,place

def main(plugin):
    sys.path.insert(0,str(plugin/'scripts'))
    from mechanical_motion import bounds,distance,common
    from showcase_contacts import load_solids,box_gap
    authority=read(ROOT/'cad/fusion-authority.json')['source']['archive']
    native=read(OUT/'r5-load-01.json')
    assert digest(ROOT/authority['file'])==authority['sha256']==native['authority']['archiveSha256']
    assert digest(OUT/'red2-split-drop-fixture-r5.f3d')==native['sha256']
    assert digest(OUT/'r5-export.step')==native['step']['sha256']
    cases=read(ROOT/'experiments/swerve-drive/low-placement/clearance.json')['cases']
    target=read(ROOT/'experiments/six-wheel/mission/target.json')
    risers=1000*np.array(target['riser_bounds_m'])
    assert len(risers)==2
    identities={}
    solids,surfaces=load_solids(OUT/'r5-export.step',identities)
    assert not surfaces and len(solids)==93
    states={s['pose']:s for s in native['native']['states']}
    assert set(states)=={'home','present','open','before-release','reset'}
    result={};tests={}
    for side in ['Left','Right']:
        name=side+' splitting support:1/'+side+' pipe rail'
        home=np.array(states['home']['bodies'][name])
        ids=[key for key,s in solids.items() if name.split('/')[0] in identities[key]['occurrencePath']
             and np.max(np.abs(np.array(bounds(s))-home))<.05]
        if len(ids)!=1:raise ValueError('Unresolved installed jaw '+name)
        source=solids[ids[0]];rows=[];rotated={}
        for pose,state in states.items():
            bb=np.array(state['bodies'][name]);offset=bb[:3]-home[:3]
            assert np.linalg.norm(bb[3:]-home[3:]-offset)<.002
            for sign,case in cases.items():
                for dx,dy,yaw in itertools.product(*zip(*case['enclosure'])):
                    for pitch,dz,sector in itertools.product((-.04,.04),(-5.,5.),(0.,360.)):
                        r=rotation(pitch,yaw)
                        center=r@offset+np.array([dx-490,dy+sector,dz])
                        key=(pitch,yaw)
                        if key not in rotated:
                            rotated[key]=np.array(bounds(place(source,np.column_stack((r,np.zeros(3))))))
                        sb=rotated[key]+np.tile(center,2)
                        # Conservatively contain every combination of 1 mm
                        # fabrication error along each signed local robot axis.
                        grow=np.abs(r).sum(axis=1)
                        for idx,riser in enumerate(risers):
                            box=riser+np.concatenate((-grow,grow))
                            separation=max(*(box[:3]-sb[3:]),*(sb[:3]-box[3:]))
                            witness=dict(pose=pose,dock=[sign,dx,dy,yaw,pitch,dz,sector],riser=idx,
                                         axisBoundMm=float(separation))
                            if separation<6:
                                casekey=(side,tuple(np.column_stack((r,center)).round(7).flatten()),idx)
                                if casekey not in tests:
                                    actual=place(source,np.column_stack((r,center)))
                                    obstacle=BRepPrimAPI_MakeBox(gp_Pnt(*box[:3]),gp_Pnt(*box[3:])).Shape()
                                    gap=distance(actual,obstacle)
                                    tests[casekey]=dict(inflatedRiserGapMm=gap,
                                        inflatedRiserCommonMm3=common(actual,obstacle) if gap<1e-6 else 0.)
                                witness.update(tests[casekey])
                            rows.append(witness)
        assert len(rows)==5*128*2
        result[name]=dict(sourceSolid=ids[0],occurrence=identities[ids[0]]['occurrencePath'],
            conservativeMinMm=min(x.get('inflatedRiserGapMm',x['axisBoundMm']) for x in rows),cases=rows)
    (OUT/'r5-jaw-riser-clearance.json').write_text(json.dumps(dict(
        authority=authority,archive=native['sha256'],step=native['step'],riserBoundsM=target['riser_bounds_m'],
        uniqueNarrowSweeps=len(tests),result=result,limitations='Sampled installed poses and signed dock corners. Riser bounds enclose actual steel. A positive gap establishes separation. An intersection with a box must be resolved against exact field stock.'),indent=2)+'\n')
    for key,data in result.items():
        print(key,len(data['cases']),data['conservativeMinMm'],sum(q.get('inflatedRiserCommonMm3',0)>.001 for q in data['cases']))

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('plugin',type=Path);main(parser.parse_args().plugin)
