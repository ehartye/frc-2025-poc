"""Compare complete period SDS module against the 2025 cold chassis at its physical wheel datum."""
import json
import math
import sys
from pathlib import Path
import numpy as np
from output_mount import ROOT,read,digest,place


def run(plugin,source):
    sys.path.insert(0,str(plugin/'scripts'))
    from showcase_contacts import load_solids,box_gap
    from mechanical_motion import common,bounds
    p=ROOT/'artifacts/red2-integrated';dest=p/'module-option'
    issue=p/'pan-thrust/moving-stack-issued';ref=read(issue/'r3-moving-proxy-short-01.json')
    step=issue/'checked-301.step';authority=read(ROOT/'cad/fusion-authority.json')['source']['archive']
    assert digest(ROOT/authority['file'])==ref['authority']['archiveSha256'] and digest(step)==ref['stepSha256']
    assert digest(source)=='ee5e74af75e1b89fde94e37dbfbafc0b1e131a018c08cb906877d75cc1a34720'
    lab={};parts,other=load_solids(step,lab);assert len(parts)==301 and not other
    stock={k:bounds(v) for k,v in parts.items()}
    host=read(p/'foundation-r2-cold-inventory.json')['bodies']
    omitted={k for k,v in lab.items() if any(i.startswith('FL ') and (i+':1') in v['occurrencePath'] for i in host)}
    if len(omitted)!=22:raise ValueError(('Expected exactly 22 superseded FL REV fitment parts',len(omitted)))
    named={};donor,other=load_solids(source,named);assert len(donor)==125 and not other
    source_boxes={k:bounds(v) for k,v in donor.items()}
    tire=next(k for k in donor if k.startswith('Tread, Blue Nitrile'))
    axles=[(a,b) for a,b in source_boxes.items() if a.startswith('Bearing, 6802') and abs((b[0]+b[3])/2+66.675)<.01 and abs((b[2]+b[5])/2+66.675)<.01]
    if len(axles)!=3:raise ValueError(('Lost original steering shaft bearing centers',axles))
    axis=np.mean([[(b[0]+b[3])/2,(b[2]+b[5])/2] for _,b in axles],axis=0)
    origin=np.array([axis[0],source_boxes[tire][1],axis[1]])
    target=np.array([282.,277.,0.])
    results={};report=dict(authority=authority,currentNative=ref['archiveSha256'],currentStep=digest(step),foundationBasis=digest(p/'foundation-r2-cold-inventory.json'),
        vendorFile=source.name,vendorStepSha256=digest(source),sourceSolids=len(donor),actualSteeringDatumBearings=[s for s,_ in axles],
        steeringAxisAndContactDatumMm=origin.tolist(),targetKingpinAndFloorMm=target.tolist(),removedPriorHardware=sorted(omitted),cases=results)
    out=dest/'sds-2023-fl-fit.json'
    def save():out.write_text(json.dumps(report,indent=2)+'\n')
    for clock in [0,90,180,270]:
        a=math.radians(clock);c,s=round(math.cos(a)),round(math.sin(a))
        M=np.array([[c,0,s],[s,0,-c],[0,1,0]],dtype=float)
        mat=np.c_[M,target-M@origin]
        case={'physicalPlacementMm':mat.tolist(),'hits':{},'staticBodiesChecked':0}
        results[str(clock)]=case
        # Transform each original source AABB first; only act on candidates
        # that can physically touch the registered 2025 chassis envelope.
        for k,body in donor.items():
            b=source_boxes[k]
            lo=M@np.array(b[:3])+mat[:,3];hi=M@np.array(b[3:])+mat[:,3]
            bb=np.r_[np.minimum(lo,hi),np.maximum(lo,hi)]
            possibles=[n for n in parts if n not in omitted and box_gap(bb,stock[n])<.0002]
            if possibles:
                placed=place(body,mat)
                for n in possibles:
                    if box_gap(bb,stock[n])>.0002:continue
                    amount=common(placed,parts[n])
                    if amount>.01:case['hits'][k+' vs '+n]=amount
            case['staticBodiesChecked']+=1
            if case['staticBodiesChecked']%25==0:save()
        save()
        print(clock,len(case['hits']),sorted(case['hits'].items(),key=lambda p:-p[1])[:3],flush=True)
    report['scope']='125 genuine 2023 SDS MK4i NEO exchange solids on their own 2025 shaft/floor datum. FL REV saddles, fasteners and module plates deliberately removed. Original alloy, blue-tread procurement, installation screws/secondary plate, module mounting registration, internal rolling pairs, steering, travel, loaded deflection and wiring remain separate.'
    save()


if __name__=='__main__':run(Path(sys.argv[1]),Path(sys.argv[2]))
