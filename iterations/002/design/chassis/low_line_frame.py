"""Compare the high sheet to a folded member whose axial load line is near both bumper contacts."""
import argparse
import json
import math
from pathlib import Path
import sys
from crossmember_envelope import box,cylinder

from four_corner_packaging import polygon
from OCP.BRepPrimAPI import BRepPrimAPI_MakePrism
from OCP.gp import gp_Vec

HERE=Path(__file__).parent
ROOT=HERE.parents[3]


def centered():
    part={}
    X,Y=344.6,339.6
    W=115; th=3.175; lo=101.6;hi=155.575
    # End bars consume 25.4 mm of each arm. Open C-shaped sections meet the real central beam
    # by a central upper web and two separated lower plates; no overlapping arm stocks in the center.
    x,y=X-25.4,Y-25.4
    shape=[(W,y),(-W,y),(-W,W),(-x,W),(-x,-W),(-W,-W),(-W,-y),(W,-y),(W,-W),(x,-W),(x,W),(W,W)]
    part['upper surface']=BRepPrimAPI_MakePrism(polygon([(u,v,hi-th) for u,v in shape]),gp_Vec(0,0,th)).Shape()
    part['center throughmember']=box((-38.1,-y,lo),(38.1,y,152.4))
    for sign in (-1,1):
        ab=sorted((sign*38.1,sign*W))
        part[f'{sign} lower center plate']=box((ab[0],-W,lo),(ab[1],W,lo+th))
        ab=sorted((sign*38.1,sign*(38.1+th)))
        part[f'{sign} center web at primary beam']=box((ab[0],-W,lo+th),(ab[1],W,hi-th))
    for axis,extent in ((0,X),(1,Y)):
        for sign in (-1,1):
            def xyz(radial,tangent,z):
                return (sign*radial,tangent,z) if axis==0 else (tangent,sign*radial,z)
            def rect(n,ab,bc):
                a=xyz(*ab);b=xyz(*bc)
                part[f'{axis}/{sign}/{n}']=box(tuple(min(v,w) for v,w in zip(a,b)),tuple(max(v,w) for v,w in zip(a,b)))
            rect('closed terminal torque box',(extent-25.4,-W,lo),(extent,W,177.8))
            for edge in (-1,1):
                rect(f'{edge} upright',(W,edge*W,lo+th),(extent-25.4,edge*(W-th),hi-th))
                rect(f'{edge} lower return',(W,edge*W,lo),(extent-25.4,edge*(W-60),lo+th))
            # These straight entries are commissioned after removing the externally detachable bumpers.
            # They cover a 14 mm service tool sweep at the actual fastener heights on both sides.
            for station in (-100,0,100):
                for z in (114.3,165.1):
                    direc=(sign,0,0) if axis==0 else (0,sign,0)
                    part[f'{axis}/{sign}/{station}/{z} external tool']=cylinder(xyz(extent+3.175,station,z),direc,7,110)
    return part


def section(top,height,lower_width,fold_up=False):
    t=3.;web=230.;leg=height-2*t
    if fold_up:
        rects=[(web,t,top-t/2),(2*t,24,top+12)]
    else:
        rects=[(web,t,top-t/2),(2*t,leg,top-t-leg/2),(2*lower_width,t,top-height+t/2)]
    A=sum(w*h for w,h,zc in rects); z=sum(w*h*zc for w,h,zc in rects)/A
    I=sum(w*h**3/12+w*h*(zc-z)**2 for w,h,zc in rects)
    far=max(abs(z-(q+d)) for w,h,q in rects for d in (-h/2,h/2))
    data={}
    for label,P in [('clocked-stop',1009.8997101394822),('conditional-cruise',11700)]:
        for h in (114.3,150.):
            for sign in (1,-1):
                M=sign*P*(h-z)
                data[f'{label}/{h}/{sign}']=dict(signedN=sign*P,offsetMm=h-z,momentNm=M/1000,
                    axialPlusExtremeFiberStressMPa=abs(P)/A+abs(M)*far/I)
    return dict(sectionAreaMm2=A,centroidHeightMm=z,sectionInertiaMm4=I,cases=data)


def run(plugin):
    sys.path.insert(0,str(plugin/'scripts'))
    from fusion_authority import register_path,verify,load,resolve,digest
    from mechanical_motion import bounds,distance,common
    from showcase_contacts import load_solids,box_gap
    reg=register_path(ROOT);lineage=reg.parent/'issue-003/lineage.json'
    auth=verify(ROOT,reg,lineage);actual,sheets=load_solids(resolve(ROOT,load(lineage)['step']))
    if sheets or len(actual)!=488:raise ValueError('Uncommissioned issue')
    physical={k:bounds(v) for k,v in actual.items()}
    prepared=centered();result={}
    for name,shape in prepared.items():
        broad=bounds(shape);allbounds=sorted((box_gap(broad,b),k) for k,b in physical.items())
        rows=[]
        for low,k in allbounds:
            if low>3:break
            gap=distance(shape,actual[k]);rows.append(dict(id=k,gapMm=gap,commonMm3=common(shape,actual[k]) if gap<.0001 else 0))
        result[name]=dict(near=rows,untestedLowerBoundMm=next((low for low,k in allbounds if low>3),None))
    ext=sum((2*(344.6-25.4-115),2*(339.6-25.4-115)))
    rho=2680e-9;t=3.175
    # Replace eight 25 mm edge returns of the earlier plate and the single sheet end flaps.
    old=load(HERE/'folded-crossframe-screen.json')
    take_out=old['newDevelopedStockKg']['eightEdgeReturnsKg']+old['newDevelopedStockKg']['fourEndFacesKg']
    upper_short=4*25.4*230*t*rho
    weights=dict(eightVerticalWebAndBottomReturnsKg=2*ext*(53.975-2*t+60)*t*rho,
        twoLowerCollectorsKg=2*(115-38.1)*230*t*rho,twoCenterWebsKg=2*230*(53.975-2*t)*t*rho,
        fourEndTubesKg=4*.230*1.102*.45359237/.3048,extraPocketedCenterConnectionsAllowanceKg=.35)
    new_field=old['fieldMassTargetKg']-take_out-upper_short+sum(weights.values())
    return dict(authority=auth,sourceBudgetSha256=digest(HERE/'folded-crossframe-screen.json'),shapes=len(prepared),totalScenePairs=len(prepared)*len(actual),
        above=section(180.975,0,0,True),centered=section(155.575,53.975,60),
        placement=result,addedMassKg=weights,subtractedMassKg=take_out+upper_short,currentFieldAllocationKg=new_field,
        inspectionBareAllocKg=new_field-6.17-7.5-.816466,
        scope='The pressure centroid is bounded at two adopted heights, 114.3 and 150 mm. The 1,009.90 N case follows the recorded weak-pack approach with assumed contact patch. The 11.7 kN case is a separate upper triangular force/stroke assumption. Compare adverse X/Y directions and both signs on the same section. Gross 3 mm minimum 5052-H32 section; folded lower return takes residual moment. Elastic section modes, heat affected material, skin shear, connection prying/fatigue and actual discrete contact must be verified. Member boxes and full 110 mm external socket approaches are finite keepouts against 488 true fixed parts. Adjacent design stock/tool clearances, installed procurement, motion, center-web joint stiffness, 3D torsion and chassis-side connections remain unqualified.')

if __name__=='__main__':
    r=run(Path(sys.argv[1]).resolve());(HERE/'low-line-frame.json').write_text(json.dumps(r,indent=2)+'\n')
    hits=[(k,p['id'],p['commonMm3']) for k,v in r['placement'].items() for p in v['near'] if p['commonMm3']>.01]
    print(r['totalScenePairs'],r['currentFieldAllocationKg'],hits)
