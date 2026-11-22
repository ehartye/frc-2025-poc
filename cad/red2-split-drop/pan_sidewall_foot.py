"""Compare a sidewall-supported cartridge mount on the true cold frame with the bottom-driven controls."""
import json
import math
import sys
from pathlib import Path

import numpy as np
from output_mount import ROOT,digest,read,place
from pan_counterbrace import box
from pan_thrust_layout import cylinder,transform,translate,subtract,fuse


def mounted(plugin):
    sys.path.insert(0,str(plugin/'scripts'))
    from mechanical_motion import bounds,common
    from showcase_contacts import box_gap,load_solids
    src=ROOT/'artifacts/red2-integrated/pan-thrust'
    authority=read(ROOT/'cad/fusion-authority.json')['source']['archive']
    issue=read(src/'fixed-races/pan-r3-fixed-races-01.json')
    step=src/'fixed-races/checked-297.step'
    assert digest(ROOT/authority['file'])==issue['authority']['archiveSha256'] and digest(step)==issue['stepSha256']
    labels={};whole,extras=load_solids(step,labels);assert len(whole)==297 and not extras
    def named(name):
        k,=[k for k,v in labels.items() if name in v['occurrencePath']]
        return k
    screws=read(src/'fastener-option/sds-threaded-screws.json')
    sd=src/'fastener-option'/screws['parts']['91251A351']['leaf']
    assert digest(sd)==screws['parts']['91251A351']['sha256']
    bolt,=load_solids(sd)[0].values()
    alternatives={};sites={};new_shapes={}
    for side,sgn in [('port',1),('starboard',-1)]:
        foot=named(side.title()+' bearing tower foot')
        sill=named(('left' if sgn==1 else 'right')+' upper carriage sill')
        def cube(x0,x1,y0,y1,z0,z1):
            u,v=sorted([sgn*y0,sgn*y1]);return box((x0,u,z0),(x1,v,z1))
        def along(x,z,y,dia,depth):
            return cylinder((x,sgn*y,z),(0,-sgn,0),dia/2,depth)
        # Replace the foot with one machined L-shaped foot, retaining its top
        # interface to tower and knees. Outside leg reacts against the rail face.
        # The top has the same 7 mm cross section at the old tower.
        new_foot=fuse(whole[foot],cube(160,250,251,258,450,487))
        host=whole[sill];pieces={};drives={}
        # Each detachable inner H1150 strip ties two screw axes, so it cannot
        # rotate when the first of those screws is serviced from outside.
        for x in [177.,233.]:
            nut=cube(x-7,x+7,218.,231.5,450.,480.)
            for z in [457.5,472.5]:
                id=f'{side}/{x:g}/{z:g}'
                # Two structural walls see a fitted 14 mm weld-in aluminum
                # spool. Its turned pilot seats into the stepped outer toe.
                host=subtract(host,along(x,z,251.2,14.1,18.4))
                new_foot=subtract(new_foot,along(x,z,258.2,5.18,7.5))
                new_foot=subtract(new_foot,along(x,z,255.2,10.0,4.25))
                spool=fuse(along(x,z,251.,14.0,19.5),along(x,z,255.,9.86,4.2))
                spool=fuse(spool,along(x,z,233.,14.8,1.5))
                spool=subtract(spool,along(x,z,255.1,5.18,24))
                pieces[id+'/transverse spool']=spool
                nut=subtract(nut,along(x,z,231.6,4.826,14))
                washer=subtract(along(x,z,260.,14.,2.),along(x,z,260.1,5.1,2.2))
                pieces[id+'/head spreader']=washer
                matrix=transform([[1,0,0],[0,0,-sgn],[0,sgn,0]],[x,sgn*260,z])
                pieces[id+'/screw']=place(bolt,matrix)
                sites[id]=dict(x=x,z=z,sign=sgn,boltWorldMatrixMm=matrix.tolist(),foot=foot,sill=sill,
                    locatorDiameterMm=9.86,diametralPilotClearanceMm=.14,fullPilotSeatDepthMm=4.,
                    screwIntoThreadMm=38.1-(260.-231.5),trueBoltLengthMm=38.1)
            pieces[f'{side}/{x:g}/two-hole nut strip']=nut
        new_shapes[foot]=new_foot;new_shapes[sill]=host;new_shapes.update(pieces)
    whole.update(new_shapes)
    bodies={k:bounds(v) for k,v in whole.items()}
    for id,s in sites.items():
        x,z,sgn=s['x'],s['z'],s['sign']
        def outward(y,r,length):
            return cylinder((x,sgn*y,z),(0,sgn,0),r,length)
        tasks={'outer straight 5/32 key':outward(264.8,3.0,85.),
               'socket drive body envelope':outward(300.,15.,110.),
               'withdraw whole fastener':outward(260.1,4.1,43.)}
        candidates={k:v for k,v in new_shapes.items() if k.startswith(id+'/')}
        if z==457.5:candidates.update({k:v for k,v in new_shapes.items() if k==id.rsplit('/',1)[0]+'/two-hole nut strip'})
        checks={}
        for key,item in list(candidates.items())+list(tasks.items()):
            measured={};own=(id+'/screw' if key in tasks else key)
            bb=bounds(item)
            for k,v in whole.items():
                if k==own or box_gap(bb,bodies[k])>.0002:continue
                a=common(item,v)
                if a>.001:measured[k]=a
            checks[key]=measured
        alternatives[id]=checks
    # Test newly extended stock against every other actual stock; the original
    # tower joint is an unmodified face contact from the baseline.
    foot_checks={}
    for foot in {s['foot'] for s in sites.values()}:
        hits={}
        for k,v in whole.items():
            if k!=foot and box_gap(bodies[k],bodies[foot])<.0002:
                amount=common(whole[foot],v)
                if amount>.001:hits[k]=amount
        foot_checks[foot]=hits
    result=dict(authority=authority,issued297=issue['archiveSha256'],derivative=digest(step),boltSource=screws['parts']['91251A351'],sites=sites,
                siteChecks=alternatives,footChecks=foot_checks,
                trial='Two shop L-feet with side access, 18 mm rail, captive tapped nut strips and stepped weld-in spools. Keep all original tower shaft/gearbox datums. Applied duty, weld detail, root strength and fitted-pilot migration require distinct checks.')
    (src/'sidewall-foot-geometry.json').write_text(json.dumps(result,indent=2)+'\n')
    for k,v in alternatives.items():print(k,{n:h for n,h in v.items() if h},flush=True)
    print('feet',foot_checks,flush=True)


if __name__=='__main__':mounted(Path(sys.argv[1]))
