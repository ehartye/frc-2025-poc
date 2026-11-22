"""Measure physical release at the finite left rail with the attained pipe twist."""
import argparse
import math
import sys

import numpy as np
from OCP.BRep import BRep_Tool
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common, BRepAlgoAPI_Cut
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
from OCP.gp import gp_Pnt
from OCP.TopAbs import TopAbs_VERTEX
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS

from released_corridor import EYE, J, OUT, ROOT, digest, jaw, place, read, rotate
import json


def solve(plugin):
    sys.path.insert(0,str(plugin/'scripts'))
    from mechanical_motion import bounds, common
    from showcase_contacts import load_solids

    root=read(ROOT/'cad/fusion-authority.json')['source']['archive']
    issued=read(OUT/'r5-load-01.json')
    native=OUT/'red2-split-drop-fixture-r5.f3d'
    step=OUT/'r5-export.step'
    assert root['sha256']==digest(ROOT/root['file'])==issued['authority']['archiveSha256']
    assert issued['sha256']==digest(native) and issued['step']['sha256']==digest(step)
    article=read(OUT/'r5-seat-01.json')['loadedWitness']['pieceStep']
    assert digest(OUT/article['file'])==article['sha256']
    center=np.array(article['centerMm'])
    pipe=next(iter(load_solids(OUT/article['file'])[0].values()))
    inv=read(OUT/'r5-home-native.json')
    assert inv['archiveSha256']==issued['sha256']
    ident='Left splitting support:1/Left pipe rail'
    item,=[b for b in inv['bodies'] if b['path']+'/'+b['name']==ident]
    rail,=[s for s in load_solids(step)[0].values()
           if max(abs(np.array(bounds(s))-item['boundsMm']))<.04]
    def evaluate(state,stroke):
        p=np.array(state['p'])*1000
        a=np.array(state['axis'])
        axis=np.cross([0,1,0],a)
        orient=rotate(axis/np.linalg.norm(axis),math.atan2(np.linalg.norm(axis),a[1]))
        stock=place(pipe,np.column_stack((orient,p-orient@center)))
        shift=np.array([404,stroke,0])
        mate=place(rail,np.column_stack((EYE,shift)))
        search=BRepExtrema_DistShapeShape(stock,mate)
        gap=search.Value()
        output=dict(jawMm=stroke,gapMm=gap,overlapMm3=0.)
        if gap<1e-7:
            output['overlapMm3']=common(stock,mate)
            if output['overlapMm3']>1e-10:
                region=BRepAlgoAPI_Common(stock,mate).Shape()
                output['railLocalIntersectionBoundsMm']=(np.array(bounds(region))-np.tile(shift,2)).tolist()
                verts=set()
                ex=TopExp_Explorer(region,TopAbs_VERTEX)
                while ex.More():
                    q=np.array(BRep_Tool.Pnt_s(TopoDS.Vertex(ex.Current())).Coord())
                    verts.add(tuple((q-shift).round(8)))
                    ex.Next()
                output['railLocalIntersectionVerticesMm']=sorted(verts)
        else:
            x=np.array(search.PointOnShape1(1).Coord())
            y=np.array(search.PointOnShape2(1).Coord())
            n=(x-y)/np.linalg.norm(x-y)
            arm=x-p
            pipe_rate=np.array(state['v'])*1000+np.cross(state['omega'],arm)
            output.update(pipeWitnessWorldMm=x.tolist(),railWitnessLocalMm=(y-shift).tolist(),
                          pipeAxialCoordinateMm=float((x-p)@a),
                          pipeRadialCoordinateMm=float(np.linalg.norm((x-p)-((x-p)@a)*a)),
                          separatingNormalTowardPipe=n.tolist(),
                          featureTypes=[str(search.SupportTypeShape1(1)),str(search.SupportTypeShape2(1))])
            for label,rate in [('opening',600.),('held',0.),('drivenInward',-600.)]:
                rel=pipe_rate-[0,rate,0]
                output[label]=dict(normalMmPerS=float(n@rel),tangentMmPerS=float(np.linalg.norm(rel-(n@rel)*n)))
        return output

    base=OUT/'r5-spatial-r5-as-drawn-challenge-strike-physical.json'
    corrected=OUT/'r5-spatial-r5-as-drawn-challenge-strike-physical-edge.json'
    t0=read(base);t1=read(corrected)
    assert t0['fixture']==t1['fixture']==digest(native)
    tag='nominal-coral-A-0.00005-1-1'
    old=t0['runs'][tag];new=t1['runs'][tag];free=old['free']
    src=free['state'];p=np.array(src['p']);v=np.array(src['v'])
    a=np.array(src['axis']);L=np.array(src['angularMomentum'])
    axis=L/np.linalg.norm(L);rate=np.linalg.norm(L)/J
    p_axis_inertia=.816466266*(.05715**2+.0508**2)/2
    def continuation(us,mode='opening'):
        t=us/1e6
        direction=rotate(axis,rate*t)@a
        omega=L/J+direction*(L@direction)*(1/p_axis_inertia-1/J)
        g=np.array([0.,0.,-9.80665])
        state=dict(p=p+v*t+.5*g*t*t,v=v+g*t,axis=direction,omega=omega)
        start=free['wallS']-old['openAt']
        stroke=jaw(start+t) if mode=='opening' else jaw(start)-(600*t if mode=='drivenInward' else 0)
        return evaluate(state,stroke)
    frames=[dict(wallS=f['wallS'],sideLoadsN=f['sideLoadsN'],occupied=f['occupied'],
                 **evaluate(f['state'],f['jawMm'])) for f in new['free']['precedingFrames'][-4:]]
    samples={str(t):continuation(t) for t in (0,5,10,15,20,25,30,35,40,45,50,75,100,250)}
    controls={mode:{str(t):continuation(t,mode) for t in (0,25,50,100,250,500,1000)} for mode in ('held','drivenInward')}
    # The closest separated witnesses give a consistent oriented normal. A
    # solid Boolean may report zero below CAD resolution before measurable
    # positive separation appears. Keep the two bounds independently visible.
    lo=max(float(t) for t,r in samples.items() if r['overlapMm3']>1e-9)
    hi=min(float(t) for t,r in samples.items() if r['gapMm']>1e-5)
    assert lo<hi
    bracket={}
    for t in np.linspace(lo,hi,17):
        bracket[str(float(t))]=continuation(t)
    # Only a short piece of the rail's planar top/inner corner participates
    # in this boundary. Check that the remainder stays separated through the
    # complete transition, including all other faces of the native rail.
    window=BRepPrimAPI_MakeBox(gp_Pnt(135,109,524),gp_Pnt(203,133,579)).Shape()
    local=BRepAlgoAPI_Common(rail,window).Shape()
    remainder=BRepAlgoAPI_Cut(rail,window).Shape()
    near=next(r for r in bracket.values() if r['gapMm']>1e-5)
    n=np.array(near['separatingNormalTowardPipe'])
    # Project onto the exact normal plane of the actual sloping finite edge.
    tangent=np.array([270.,0.,-52.]);tangent/=np.linalg.norm(tangent)
    n-=tangent*(n@tangent);n/=np.linalg.norm(n)
    top=np.array([205.,110.,550.])
    # Local BRep faces are stock planes; every real vertex must be behind
    # the chosen plane. The furthest support of a planar polytope is a vertex.
    from OCP.BRepAdaptor import BRepAdaptor_Surface
    from OCP.TopAbs import TopAbs_FACE
    from OCP.GeomAbs import GeomAbs_Plane
    fs=TopExp_Explorer(local,TopAbs_FACE)
    while fs.More():
        assert BRepAdaptor_Surface(TopoDS.Face(fs.Current())).GetType()==GeomAbs_Plane
        fs.Next()
    vs=TopExp_Explorer(local,TopAbs_VERTEX);projections=[]
    while vs.More():
        q=np.array(BRep_Tool.Pnt_s(TopoDS.Vertex(vs.Current())).Coord())
        projections.append(float((q-top)@n));vs.Next()
    assert projections and max(projections)<1e-7 and max(projections)>-1e-7
    def plane_distance(us):
        dt=us/1e6
        direction=rotate(axis,rate*dt)@a
        origin=(p+v*dt)*1000+[0,0,-.5*9806.65*dt*dt]
        section=top+[404,jaw(free['wallS']-old['openAt']+dt),0]
        axial=n@direction
        return float(n@(origin-section)-150.8125*abs(axial)-57.15*math.sqrt(1-axial**2))
    left,right=15.,20.
    assert plane_distance(left)<0<plane_distance(right)
    for _ in range(34):
        mid=(left+right)/2
        if plane_distance(mid)>0:right=mid
        else:left=mid
    # A bound on all point speeds, including angular speed, independent of
    # which point achieves the support function during this interval.
    duration=50e-6
    wbound=rate+abs(L@a)*abs(1/p_axis_inertia-1/J)
    rbound=math.hypot(150.8125,57.15)
    normal_rate=float((v*1000-[0,600,0])@n-9806.65*duration*abs(n[2])-wbound*rbound)
    r0=rotate(np.cross([0,1,0],a)/np.linalg.norm(np.cross([0,1,0],a)),math.acos(a[1]))
    stock0=place(pipe,np.column_stack((r0,p*1000-r0@center)))
    rest0=place(remainder,np.column_stack((EYE,[404.,jaw(free['wallS']-old['openAt']),0.])))
    rest_gap=BRepExtrema_DistShapeShape(stock0,rest0).Value()
    sweep_budget=(np.linalg.norm(v)*1000+600+wbound*rbound)*duration+.5*9806.65*duration**2
    assert normal_rate>0 and rest_gap>sweep_budget
    monotonic=dict(localWorldNormal=n.tolist(),minSignedOpeningSpeedMmPerS=normal_rate,
                   localRailSupportMaxMm=max(projections),restOfRailGapAtOldClaimMm=rest_gap,
                   allPointTravelBoundMm=sweep_budget,coveredTimeUs=[0.,50.],
                   localAnalyticalSeparationAfterUs=right,
                   lowerBoundAtFirstEdgeFreeSourceMm=plane_distance(50.),
                   localRailSectionMm=[[135,109,524],[203,133,579]])
    return dict(authority=root,fixtureSha256=digest(native),derivativeSha256=digest(step),
                article=article,body=ident,originalTraceSha256=digest(base),edgeTraceSha256=digest(corrected),
                case=tag,sourceStepS=new['dt'],oldClaimedReleaseWallS=free['wallS'],
                firstMeasuredSeparatedSourceWallS=new['free']['wallS'],
                sourceFrames=frames,elapsedMicrosecondsFromOldClaim=samples,
                edgeCrossingMicrosecondsFromOldClaim=bracket,controls=controls,continuousSeparation=monotonic,
                basis='The source has zero reaction at the former declaration. Propagate its unchanged attained linear and angular momenta under gravity to the physical separation. Material compliance before the separation is retained in the source. Controls reverse or hold the actual jaw at the same original pose and twist.')

if __name__=='__main__':
    from pathlib import Path
    ap=argparse.ArgumentParser();ap.add_argument('plugin',type=Path)
    result=solve(ap.parse_args().plugin)
    (OUT/'r5-release-edge-control.json').write_text(json.dumps(result,indent=2)+'\n')
    for t,r in result['elapsedMicrosecondsFromOldClaim'].items():
        print(t,r['gapMm'],r['overlapMm3'],r.get('opening'),r.get('railWitnessLocalMm'))
