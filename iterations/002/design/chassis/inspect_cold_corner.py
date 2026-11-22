"""Compare the active cold STEP to all selected supplier leaves. Inspect each existing mounting article."""
import argparse
from itertools import product
import json
import math
from pathlib import Path
import sys
import numpy as np
from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder
from OCP.gp import gp_Ax2,gp_Dir,gp_Pnt


def cylinder(x,y,z,r,h):
    return BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(x,y,z),gp_Dir(0,0,1)),r,h).Shape()


def run(root,plugin,source,selector):
    sys.path.insert(0,str(plugin/'scripts'))
    from fusion_authority import register_path,verify_active,resolve,load,digest
    from showcase_contacts import load_solids,box_gap
    from mechanical_motion import common,distance,bounds,bearing_area
    from component_shape import volume
    sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'supplier'))
    from verify_l2 import points
    active=register_path(root);spec=load(active);verify_active(root,active,spec)
    native=resolve(root,spec['source']['archive']);issue=load(active.parent/'issue-123.json')
    step=resolve(root,issue['issuedStep']);cold=load(resolve(root,issue['nativeReceipt']))
    assert cold['nativeSha256']==digest(native) and cold['stepSha256']==digest(step)
    plan=load(active.parent.parent/'chassis/front-corner.json')
    assert digest(source)==plan['sourceSha256'] and digest(selector)==plan['optionSha256']
    configured=load(selector);material=set(configured['commonSolids']+configured['selectableFamilies']['L2']['selected'])
    raw_labels={};placed_labels={};raw,extra=load_solids(source,raw_labels);placed,sheets=load_solids(step,placed_labels)
    assert len(placed)==123 and len(material)==121 and len(raw)==125 and not extra and not sheets
    matrix=np.array(plan['worldMatrixMm']);keys={};boxes={k:bounds(v) for k,v in placed.items()};used=set()
    witness={}
    for k in sorted(material):
        s=raw[k];src=raw_labels[k]['definition']
        b=bounds(s)
        corners=np.array([matrix[:3,:3]@np.array(x)+matrix[:3,3] for x in product(*zip(b[:3],b[3:]))])
        wanted=np.concatenate((corners.min(axis=0),corners.max(axis=0)))
        candidates=sorted((float(np.max(np.abs(boxes[n]-wanted))),n) for n,v in placed_labels.items() if v['definition']==src)
        if not candidates or candidates[0][0]>.15 or len(candidates)>1 and candidates[1][0]-candidates[0][0]<.1:
            raise ValueError('Ambiguous supplier registration '+str((k,candidates[:3])))
        err,n=candidates[0]
        if n in used:raise ValueError('Repeated source binding '+n)
        used.add(n);keys[k]=n
        p=points(s);p=p@matrix[:3,:3].T+matrix[:3,3];q=points(placed[n])
        # Three far-apart non-collinear genuine vertices are independently
        # registered. Preserve seams introduced or merged by Fusion separately.
        a=p[np.argmin(p[:,0])];b=p[np.argmax(np.linalg.norm(p-a,axis=1))]
        c=p[np.argmax(np.linalg.norm(np.cross(p-a,b-a),axis=1))]
        deltas=[float(np.min(np.linalg.norm(q-x,axis=1))) for x in (a,b,c)]
        witness[k]=dict(placed=n,maximumBoundsDifferenceMm=err,sourceVolumeMm3=volume(s),placedVolumeMm3=volume(placed[n]),
            sourceVertices=len(p),exportedVertices=len(q),sourceWorldVerticesMm=[x.tolist() for x in (a,b,c)],nearestExportedVertexErrorsMm=deltas)
    if len(used)!=121:raise ValueError('Incomplete mapping')
    known=set(placed)-used
    if known!={rail['name'] for rail in plan['rails']}:raise ValueError('Unknown frame member')
    donor_plate={side:keys[f'MK4i Master Part V2_NEO Config [1.43.{i}]'] for side,i in [('upper',17),('lower',16)]}
    interfaces={}
    for rail in known:
        b=placed[rail]
        interfaces[rail]={}
        for side,n in donor_plate.items():
            inter=common(b,placed[n])
            area=bearing_area(b,placed[n],[0,0,1 if side=="upper" else -1])
            interfaces[rail][side]=dict(name=n,gapMm=distance(b,placed[n]),overlapMm3=inter,commonAreaMm2=area)
    indexed={}
    for item in configured['mount']['supplierScrewCenterAndPlateSamples']:
        s=item['id'];n=keys[s];v=placed[n];b=boxes[n];cx=(b[0]+b[3])/2;cy=(b[1]+b[4])/2
        fixed={k:q for k,q in placed.items() if k!=n}
        # Determine head and shank dimensions from the actual STEP cylindrical
        # faces. A larger coaxial envelope bounds all thread and head rotations.
        from OCP.BRepAdaptor import BRepAdaptor_Surface
        from OCP.GeomAbs import GeomAbs_Cylinder
        from OCP.TopExp import TopExp_Explorer
        from OCP.TopAbs import TopAbs_FACE
        from OCP.TopoDS import TopoDS
        cast=getattr(TopoDS,'Face_s',None) or TopoDS.Face
        it=TopExp_Explorer(v,TopAbs_FACE);cyl=[]
        while it.More():
            face=cast(it.Current());surf=BRepAdaptor_Surface(face)
            if surf.GetType()==GeomAbs_Cylinder:
                x=surf.Cylinder();origin=x.Location().Coord();ax=x.Axis().Direction().Coord()
                if abs(ax[2])>.99999 and math.dist(origin[:2],[cx,cy])<.03:
                    cyl.append([x.Radius(),bounds(face)[2],bounds(face)[5]])
            it.Next()
        if not cyl:raise ValueError('Socket head outside cylinder absent')
        head=max(cyl);thin=max((j for j in cyl if j[0]<head[0]-.1 and j[2]-j[1]>20),default=None)
        if not thin:raise ValueError('Cannot distinguish screw shank')
        travel=70.;h0=float(b[2]-travel);headtop=float(head[2]);shanktop=float(b[5])
        # The head and shank surfaces sweep a complete centered cylinder. Head
        # is never lifted through the smaller upper bore during downward travel.
        heads=cylinder(cx,cy,h0,head[0],headtop-h0);shafts=cylinder(cx,cy,h0,thin[0],shanktop-h0)
        obstacles={}
        for k,q in fixed.items():
            if box_gap(bounds(shafts),boxes[k])>.03 and box_gap(bounds(heads),boxes[k])>.03:continue
            overlaps=[common(g,q) for g in (heads,shafts)]
            gaps=[distance(g,q) for g in (heads,shafts)]
            obstacles[k]=dict(maxAxiallySweptGaugeCommonMm3=max(overlaps),minimumSweptGaugeGapMm=min(gaps))
        indexed[n]=dict(worldAxisMm=[cx,cy],actualZmm=[float(b[2]),float(b[5])],outsideCylindersMm=cyl,fullDownTravelMm=travel,
                        minimumShopLiftAboveNominalFloorMm=max(0.,-h0),closedSweptGaugeNeighbors=obstacles)
    return dict(nativeSha256=digest(native),stepSha256=digest(step),supplierSha256=digest(source),selected=121,placedTotal=123,
        sourceChecks=witness,railSeats=interfaces,withdrawal=indexed,
        scope='70 mm service path studied by conservative full-rotation head and shank cylinders. Gauge interference is a potential restriction, including intended original thread fits; actual phase/thread lead needs separate evaluation. Ground clearance is a lower bound excluding the tool. Full robot loaded mounts, bumper and cables pending.')


if __name__=='__main__':
    p=argparse.ArgumentParser()
    for n in ('project','plugin','source','selector','out'):p.add_argument(n,type=Path)
    a=p.parse_args();result=run(a.project.resolve(),a.plugin,a.source,a.selector)
    a.out.write_text(json.dumps(result,indent=2)+'\n')
    print(result['selected'],result['railSeats'])

