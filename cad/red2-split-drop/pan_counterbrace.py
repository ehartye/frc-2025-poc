"""Size and fit the removable pan towers on the two actual sills and cross-members."""
import itertools
import json
import math
import sys
from pathlib import Path

import numpy as np
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
from OCP.gp import gp_Pnt
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps

from output_mount import ROOT, read, digest, place
from pan_thrust_layout import cylinder, subtract, fuse, transform, translate
from pan_mechanics import rotation
from pan_tower_reactions import study as record_tower_loads


def box(a,b):
    return BRepPrimAPI_MakeBox(gp_Pnt(*a),gp_Pnt(*b)).Shape()


def test(plugin, flanged):
    sys.path.insert(0,str(plugin/'scripts'))
    from mechanical_motion import bounds, common
    from showcase_contacts import box_gap, load_solids
    d=ROOT/'artifacts/red2-integrated'
    folder=d/'pan-thrust'
    plan=read(folder/'r3-native-plan.json')
    native=d/'pan-native/integrated-340mm-pan-r2.step'
    if digest(native)!=plan['parentStep'] or digest(ROOT/plan['authority']['file'])!=plan['authority']['sha256']:
        raise ValueError('Current source or R2 derivative changed')
    paths={}
    parent,other=load_solids(native,paths)
    if len(parent)!=294 or other:raise ValueError('Incomplete parent issue')
    def owner(name):
        matches=[k for k in parent if name in paths[k]['occurrencePath']]
        if len(matches)!=1:raise ValueError('No unique parent '+repr((name,matches)))
        return matches[0]
    original_change={owner(x) for x in ['Idler headed stub hex shank','Starboard bearing tower','Starboard driven hex socket']}
    fixed={}
    mobile={}
    for k,s in parent.items():
        if k in original_change:continue
        if '340 mm pan rotating installation' in paths[k]['occurrencePath']:
            mobile[k]=place(s,rotation(-70))
        else:fixed[k]=s
    for k,row in plan['stock'].items():
        f=ROOT/row['file'];assert digest(f)==row['sha256']
        shape,=load_solids(f)[0].values()
        target=fixed if k in ['Drilled starboard bearing tower','Stationary diaphragm with four bosses'] else mobile
        target[k]=shape
    cage={}
    def add(row,k,source,signature):
        f=ROOT/source;assert digest(f)==signature
        shape,=load_solids(f)[0].values()
        {'moving':mobile,'fixed':fixed,'cage':cage}[row['group']][k]=place(shape,np.array(row['matrixMm']))
    for k,q in plan['bearingPlacements'].items():
        x=q['originalTransfer'];add(q,k,x['file'],x['sha256'])
    for k,q in plan['screwPlacements'].items():add(q,k,q['source'],q['sourceSha256'])
    if len(fixed)+len(mobile)+len(cage)!=325:raise ValueError('Incorrect candidate base roster')
    # These two members are corner-welded four-plate 25 x 25 x 2.5 mm shop boxes,
    # not a sharp-corner substitute for a purchased hollow profile.
    beam_x=[186.,224.]
    members={}
    for i,x in enumerate(beam_x,1):
        name='Cross member '+str(i)
        members[name+' upper sheet']=box((x-12.5,-233,477.5),(x+12.5,233,480))
        members[name+' lower sheet']=box((x-12.5,-233,455),(x+12.5,233,457.5))
        for n,lo in enumerate([x-12.5,x+10.],1):
            members[name+' web '+str(n)]=box((lo,-233,457.5),(lo+2.5,233,477.5))
    hardware={}
    punched={}
    sleeves={}
    articles=ROOT/'artifacts/split-drop-native/supplier'
    boltFile=articles/'REV-29-2959.STEP';nutFile=articles/'REV-29-1016.STEP'
    bolt,=load_solids(boltFile)[0].values();nuts=load_solids(nutFile)[0]
    tools={};pairs={};through_tests={}
    for side in ['Port','Starboard']:
        sign=1 if side=='Port' else -1
        foot=owner(side+' bearing tower foot')
        rail=owner(('left' if sign==1 else 'right')+' upper carriage sill')
        for n,x in enumerate(beam_x,1):
            for where,y0 in [('cross',205.),('sill',242.)]:
                y=y0*sign
                id=f'{side} {n} {where}'
                path=cylinder((x,y,445),(0,0,1),2.65,45.)
                pocket=cylinder((x,y,445),(0,0,1),5.55,45.)
                fixed[foot]=subtract(fixed[foot],path);punched[foot]=True
                if where=='cross':
                    top='Cross member '+str(n)+' upper sheet'
                    bottom='Cross member '+str(n)+' lower sheet'
                    members[top]=subtract(members[top],pocket)
                    members[bottom]=subtract(members[bottom],pocket)
                    seat=455.
                else:
                    fixed[rail]=subtract(fixed[rail],pocket);punched[rail]=True
                    seat=450.
                # A real drop-in through sleeve spans both walls, avoiding an
                # impossible loose spacer sealed in the existing hollow sill.
                barrel=cylinder((x,y,seat),(0,0,1),5.5,480.-seat)
                thickness=3. if flanged else 0.
                if flanged:
                    barrel=fuse(barrel,cylinder((x,y,seat-thickness),(0,0,1),8.,thickness))
                bore=cylinder((x,y,seat-thickness-.5),(0,0,1),2.65,481-seat+thickness)
                sleeves[id]=subtract(barrel,bore)
                host=fixed[rail] if where=='sill' else members[bottom]
                bolt_id=id+' screw'
                hardware[bolt_id]=translate(bolt,[x,y,487.-24.1173])
                # Supplier nut entry is y=0; its nylon and screw tip face downward.
                m=transform([[1,0,0],[0,0,1],[0,-1,0]],[x,y,seat-thickness])
                nut_keys=[]
                for j,(name,s) in enumerate(nuts.items(),1):
                    key=id+' nut '+str(j);hardware[key]=place(s,m);nut_keys.append(key)
                pairs[bolt_id]=nut_keys
                # Remove no load-carrying geometry. Apply a positive upward
                # perturbation to the shaft/sleeve/real nut load path. A nut
                # seated only on an undersized insert cannot retain the tube.
                at=[sleeves[id],hardware[bolt_id]]+[hardware[k] for k in nut_keys]
                weights={}
                for travel in (.05,.25):
                    weights[str(travel)] = sum(common(translate(v,[0,0,travel]),host) for v in at)
                area=weights['0.05']/.05
                if abs(4.0*weights['0.05']-(weights['0.25']-weights['0.05']))>.02:raise ValueError('Variable plane-bearing region')
                through_tests[id]=dict(upwardPerturbationCommonMm3=weights,activeProjectedBearingAreaMm2=area,
                    hostWallMm=2. if where=='sill' else 2.5,boltBeyondNutMm=50.8-(487-seat)-thickness-4.365752)

                tools[id]={'driver':cylinder((x,y,489.5654),(0,0,1),3.,68.),
                           'bolt head withdrawal':cylinder((x,y,487),(0,0,1),4.585,56.0),
                           'bolt shank withdrawal':cylinder((x,y,436.2),(0,0,1),2.4131,105),
                           'nut socket':cylinder((x,y,seat-thickness),(0,0,-1),7.5,62.),
                           'nut withdrawal':cylinder((x,y,seat-thickness),(0,0,-1),5.51,58.)}
    candidate={**fixed,**members,**{k+' through sleeve':v for k,v in sleeves.items()},**hardware}
    added=set(members)|set(hardware)|{k+' through sleeve' for k in sleeves}|set(punched)
    c=read(folder/'r3-tower-foot-reactions-radial-center.json')
    hand=[]
    for sense in ['extremePositiveAxial','extremeNegativeAxial','maximumTowerUplift']:
        clock,P=c[sense]
        # Axial (Y) shear and roll moment are allocated to the two real signed
        # Y bolt rows; pitch acts between the two X rows. Each tool point is a
        # bolt axis, not an abstract point placed on the edge of an old foot.
        f=np.array(P['forceToTowerN'])
        m=np.array(P['simultaneousFootMomentNm'])
        old_c=np.array(c['footCentroidMm'])
        mid=np.array([205.,-223.5,480.])
        m=m+np.cross((old_c-mid)/1000.,f)
        xy=[(x,y) for x in beam_x for y in [-205.,-242.]]
        qx,qy=zip(*[((x-mid[0])/1000,(y-mid[1])/1000) for x,y in xy])
        forces=[]
        for x,y in zip(qx,qy):
            forces.append(float(f[2]/4+m[0]*y/sum(v*v for v in qy)-m[1]*x/sum(v*v for v in qx)))
        mz_check=sum(y*z for y,z in zip(qy,forces));my_check=-sum(x*z for x,z in zip(qx,forces))
        if max(abs(sum(forces)-f[2]),abs(mz_check-m[0]),abs(my_check-m[1]))>1e-7:raise ValueError('Load or moment residual')
        hand.append(dict(case=sense,clockCase=clock,timeS=P['timeS'],forceN=f.tolist(),momentAtBoltCentroidNm=m.tolist(),
            requiredNormalN=forces,boltAxesMm=xy,positiveMeansTension=True,shearResultantPerBoltBeforeTorsionN=float(np.linalg.norm(f[:2])/4),
            torsionAboutZ_Nm=float(m[2])))
    results=dict(authority=plan['authority'],parentStep=plan['parentStep'],retainerPlanSha256=digest(folder/'r3-native-plan.json'),
                 throughTests=through_tests,flangeODmm=16. if flanged else None,flangeThicknessMm=3. if flanged else 0.,boltSha256=digest(boltFile),nutSha256=digest(nutFile),reactions=hand,cases={},screwsAndNuts=pairs,
                 dimensions=dict(memberOuterSectionMm=[25,25],webMm=2.5,clearSpanMm=466,railWallMm=2,boltLengthMm=50.8,throughSleeveOutsideMm=11.0),
                 limits='Provisional four-plate weldment, signed normal loads per balanced four-support model. Elastic bolt/seat distribution, weld throat, heat affected zone, host sill/tower, preload, in-plane slip, torsion and proof load remain open. The six upstream inlet/gearbox feet have distinct duties.')
    static_cache={}
    for angle in [70,34,-2]:
        case=candidate|{k:place(v,rotation(angle)) for k,v in mobile.items()}|{k:place(v,rotation(angle/2)) for k,v in cage.items()}
        boxes={k:bounds(v) for k,v in case.items()}
        clashes={};intentional={}
        for a,b in itertools.combinations(case,2):
            if (a not in added and b not in added) or box_gap(boxes[a],boxes[b])>.002:continue
            key=(a,b,(angle if a in mobile or b in mobile or a in cage or b in cage else 0))
            if key not in static_cache:static_cache[key]=common(case[a],case[b])
            v=static_cache[key]
            if v>.001:
                if a in pairs and b in pairs[a] or b in pairs and a in pairs[b]:intentional[a+' / '+b]=v
                else:clashes[a+' / '+b]=v
        failures={}
        for id,gauges in tools.items():
            for operation,gauge in gauges.items():
                bb=bounds(gauge);bad={}
                for k,v in case.items():
                    if k==id+' screw' or k.startswith(id+' nut ') or box_gap(bb,boxes[k])>.001:continue
                    cache_key=(id,operation,k,(angle if k in mobile or k in cage else 0))
                    if cache_key not in static_cache:static_cache[cache_key]=common(gauge,v)
                    overlap=static_cache[cache_key]
                    if overlap>.001:bad[k]=overlap
                if bad:failures[id+' / '+operation]=bad
        results['cases'][str(angle)]=dict(totalBodies=len(case),installedClashesMm3=clashes,modeledThreadContacts=len(intentional),
            intendedThreadVolumesMm3=intentional,attemptedPaths=sum(map(len,tools.values())),pathBlockersMm3=failures)
        print('angle',angle,'installed',clashes,'tools',failures,flush=True)
        (folder/('r3-counterbrace-flanged.json' if flanged else 'r3-counterbrace-straight-withdrawal.json')).write_text(json.dumps(results,indent=2)+'\n')

if __name__=='__main__':
    test(Path(sys.argv[1]),'--flanged' in sys.argv)
