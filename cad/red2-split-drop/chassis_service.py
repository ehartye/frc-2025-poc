"""Test frame fastener access and the rigid cartridge lowering path without a native station."""
import argparse
import json
import math
import sys
from pathlib import Path
import numpy as np
from chassis_interfaces import blank
from chassis_layout import ROOT,OUT,read,digest
from chassis_mounts import roster,cylinder
from output_mount import place


def run(plugin,cache):
    base,rails,hardware,reg=roster(plugin)
    sys.path.insert(0,str(ROOT/'experiments/swerve-drive/packaging'))
    from envelopes import load_raw
    from mechanical_motion import bounds,distance,common
    from showcase_contacts import box_gap
    drive=read(ROOT/'experiments/swerve-drive/packaging/module.json')
    assert digest(cache/'REV-21-3005.STEP')==drive['sourceStepSha256']
    original=read(cache/'original.json')
    assert len(original)==72
    raw=load_raw(cache)
    # Full supplier occurrences include the older molded wheel. Register that
    # donor only as a dimensional control. The new V2 wheel is still open.
    fixed={r['name']:blank(r) for g in ('upperSupport','bumper') for r in base[g]}|rails
    fixed.update({k:s for k,s in hardware.items() if 'rail boss' in k})
    by_corner={};tool_result={};pull_result={}
    for corner,info in drive['modules'].items():
        m=np.column_stack((info['sourceXyzToRobot'],[info['xM']*1000,info['yM']*1000,38.1]))
        components={corner+'/'+k:place(s,m) for k,s,_ in raw}
        components.update({k:s for k,s in hardware.items() if k.startswith(corner) and 'rail boss' not in k})
        by_corner[corner]=components
    def neighbors(shape,allshapes,boxes):
        bb=bounds(shape);bad=[]
        for k,s in allshapes.items():
            if box_gap(bb,boxes[k])>.001:continue
            gap=distance(shape,s)
            if gap>.001:continue
            v=common(shape,s) if gap<1e-6 else 0.
            if v>.001:bad.append(dict(body=k,volumeMm3=v))
        return bad
    for corner,module in by_corner.items():
        info=drive['modules'][corner];moving={}
        fixed_full=fixed.copy()
        for other,components in by_corner.items():
            if other!=corner:fixed_full.update(components)
        local=fixed_full|module;boxes={k:bounds(s) for k,s in local.items()}
        work=[(k,r) for k,r in reg['installed'].items() if k.startswith(corner) and r.get('sku')=='REV-29-2951']
        assert len(work)==4
        for k,r in work:
            mat=np.array(r['matrixMm']);z=mat[:3,2]
            # Two conservative cylindrical paths contain the real head/shaft
            # through 35 mm extraction and a 1/8 inch key of 68 mm reach.
            # Its socket end is deliberately excluded from the stock check.
            body=hardware[k];spec=ROOT/r['article']
            from showcase_contacts import load_solids
            supplier=next(iter(load_solids(spec)[0].values()));bbox=bounds(supplier)
            bearing=bbox[2]+25.4
            at=mat@np.array([0.,0.,bearing,1.])
            guide=cylinder(at+2.5654*z,z,3.0,68.)
            head=cylinder(at,z,4.5847,35.+2.5654)
            # Shaft follows the head within the predrilled clearance and its
            # authentic tapped mate until the end. Exclude only that target.
            shaft=cylinder(at-25.4*z,z,2.413,60.4)
            targets={n:s for n,s in local.items() if n!=k}
            boss=k.split('frame ')[0]+'rail boss '+k.split('frame ')[1].split(' ')[0]
            tool_result[k]=dict(keyReachMm=68,keyEnvelopeOdMm=6,extractionMm=35,
                keyObstructions=neighbors(guide,targets,boxes),headObstructions=neighbors(head,targets,boxes),
                shankObstructions=neighbors(shaft,{n:s for n,s in targets.items() if n!=boss},boxes))
        moving={k:s for k,s in module.items() if k not in dict(work)}
        # The screws and their heads are outside. Both captured tab saddles
        # and shims remain on the cartridge. Open the two orthogonal mounts
        # together before the long downward stroke.
        dy=np.array([-math.copysign(1.,info['xM']),-math.copysign(1.,info['yM']),0.])
        periods=[('diagonally unseat',np.zeros(3),dy),('lower cartridge',dy,dy+[0.,0.,-195.])]
        stages={}
        from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
        from OCP.gp import gp_Trsf,gp_Vec
        def shift(shape,xyz):
            t=gp_Trsf();t.SetTranslation(gp_Vec(*xyz));return BRepBuilderAPI_Transform(shape,t,False).Shape()
        for name,start,end in periods:
            unsettled=[];contact=[];measured=0;proved=0
            for k,s in moving.items():
                b0=np.array(boxes[k]);sweep=np.r_[b0[:3]+np.minimum(start,end),b0[3:]+np.maximum(start,end)]
                for n,stock in fixed_full.items():
                    if box_gap(sweep,boxes[n])>.002:continue
                    # Each external mounting plane is unilateral with its
                    # matching saddle. Prove a separating halfspace at t=0.
                    if name=='diagonally unseat' and 'saddle' in k and (n in rails or 'rail boss' in n):
                        lo=np.array(boxes[n]);axis=0 if abs(dy[0]) and (b0[0]>=lo[3]-.00001 and dy[0]>0 or b0[3]<=lo[0]+.00001 and dy[0]<0) else 1
                        if dy[axis]>0 and b0[axis]>=lo[axis+3]-.00001 or dy[axis]<0 and b0[axis+3]<=lo[axis]+.00001:
                            contact.append([k,n]);continue
                    pending=[(0.,1.,0)]
                    while pending:
                        t0,t1,depth=pending.pop();t=(t0+t1)/2
                        part=shift(s,start+(end-start)*t)
                        gap=distance(part,stock);measured+=1
                        if gap>np.linalg.norm(end-start)*(t1-t0)/2+.0001:continue
                        if gap<1e-7:
                            v=common(part,stock)
                            if v>.001:
                                unsettled.append(dict(moving=k,fixed=n,interval=[t0,t1],positiveMm3=v));break
                        if depth==16:
                            unsettled.append(dict(moving=k,fixed=n,interval=[t0,t1],gapMm=gap));break
                        pending.extend([(t,t1,depth+1),(t0,t,depth+1)])
                    else:proved+=1
            stages[name]=dict(registeredMoving=len(moving),registeredFixed=len(fixed_full),nearPairsCleared=proved,
                              separatingPlaneContacts=contact,distanceQueries=measured,unresolvedOrPositive=unsettled)
        pull_result[corner]=stages
        print(corner,'tools',sum(len(r[g]) for k,r in tool_result.items() if k.startswith(corner) for g in ('keyObstructions','headObstructions','shankObstructions')),
              'pull exceptions',[len(x['unresolvedOrPositive']) for x in stages.values()],flush=True)
        (OUT/'cartridge-service.json').write_text(json.dumps(dict(authority=base['authority'],sourceModule=drive['sourceStepSha256'],
           scope='2022 control module geometry; V1.1 changes, harness, operator grip and V2 wheel are unqualified.',
           cornersComplete=len(pull_result),tools=tool_result,withdrawal=pull_result),indent=2)+'\n')

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('plugin',type=Path);p.add_argument('cache',type=Path)
    a=p.parse_args();run(a.plugin,a.cache)
