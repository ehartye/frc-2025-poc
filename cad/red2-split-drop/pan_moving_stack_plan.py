"""Commission the retained axial stack from the cold stowed successor and its exact article sources."""
import json
import sys
from pathlib import Path
import numpy as np
from output_mount import ROOT, digest, read, place
from pan_mechanics import rotation
from pan_thrust_layout import translate


def solve(plugin):
    sys.path.insert(0,str(plugin/'scripts'))
    from mechanical_motion import distance,common,bounds
    from showcase_contacts import load_solids,box_gap
    p=ROOT/'artifacts/red2-integrated/pan-thrust'
    receipt=read(p/'fixed-races/pan-r3-fixed-races-01.json');d=read(p/'r3-native-plan.json')
    register=read(ROOT/'cad/fusion-authority.json')['source']['archive']
    native=p/'fixed-races/checked-297.step'
    if digest(ROOT/register['file'])!=receipt['authority']['archiveSha256'] or digest(native)!=receipt['stepSha256']:
        raise ValueError('Need verified issued private successor and unchanged engineering authority')
    world,extra=load_solids(native)
    assert len(world)==297 and not extra
    neighbors={k:bounds(s) for k,s in world.items()}
    spin=np.vstack((rotation(70.),[0,0,0,1]))
    inserts=[]
    for side in ['inboard','outboard']:
        key,row=next((k,r) for k,r in d['bearingPlacements'].items() if k.startswith(side) and r['group']=='moving')
        inserts.append((side+' true shaft race',key,row['originalTransfer'],(spin@np.vstack((row['matrixMm'],[0,0,0,1])))[:3],
                        'Idler headed stub hex shank' if side=='inboard' else 'Turned 6061-T6 removable cap'))
    extracted=read(p/'guarded-support/pan-r3-checked-support-01.json')['sources']
    for key,name,mate in [('Match-ground axial shim','Match-ground 302 stainless thrust shim','Idler headed stub hex shank'),
                          ('Hardened removable end cap','Turned 6061-T6 removable cap','Match-ground 302 stainless thrust shim')]:
        inserts.append((name,key,d['stock'][key],spin[:3],mate))
    prepared={};targets={}
    for name,key,ref,matrix,mate in inserts:
        shape_file=ROOT/ref['file'];assert digest(shape_file)==ref['sha256']
        shape,=load_solids(shape_file)[0].values()
        s=place(shape,matrix);prepared[name]=s
        targets[name]=dict(label=key,mate=mate,source=ref,actualTargetWorldMatrixMm=matrix.tolist(),
                          nativeEditable=extracted.get(name),worldBoundsMm=bounds(s))
    for name,shape in prepared.items():
        target=targets[name];mate=target['mate']
        partner=(prepared|world)[mate]
        pot=[(k,s) for k,s in (world|prepared).items() if k!=name and
             box_gap(target['worldBoundsMm'],neighbors.get(k) or targets[k]['worldBoundsMm'])<.1]
        target['close']={k:{'gapMm':distance(shape,s),'commonMm3':common(shape,s)} for k,s in pot}
        target['seated']=dict(gapMm=distance(shape,partner),commonMm3=common(shape,partner))
        if max(target['seated'].values())>.001:raise ValueError((name,mate,target['seated']))
    # A face-located internal washer cannot be assembled through the closed diaphragm.
    # Its documented physical insertion must precede attachment of the diaphragm.
    washer=prepared['inboard true shaft race'];support=world['Piloted bilateral bearing support']
    blocked={dy:common(translate(washer,[0,-dy,0]),support) for dy in [3,5,8]}
    out=dict(authority=register,parentF3DSha256=receipt['archiveSha256'],parentStepSha256=digest(native),
             order=['inboard true shaft race','Match-ground 302 stainless thrust shim','Turned 6061-T6 removable cap','outboard true shaft race'],
             placed=targets,inboardRaceAcrossClosedDiaphragmCommonMm3=blocked,
             physicalSequence='Bench: slide inboard shaft washer and its real cage/balls onto shaft before fitting the two-face diaphragm cartridges, then fit the outboard rolling set and shaft washer, shim, cap and retainer. Actual installation sweeps, temporary restraint of the loose components, axial play under load, cage motion, screw retention and calibration remain open.')
    (p/'moving-stack-install-plan.json').write_text(json.dumps(out,indent=2)+'\n')
    print([(n,a['seated'],[k for k,v in a['close'].items() if v['commonMm3']>.001]) for n,a in targets.items()])
    print('closed diaphragm adverse',blocked,flush=True)


if __name__=='__main__':solve(Path(sys.argv[1]))
