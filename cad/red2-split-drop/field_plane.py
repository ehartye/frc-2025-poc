"""Measure the exact floor-normal extent and section a solid against finite panels."""
import argparse
import hashlib
import itertools
import json
import math
import sys
from pathlib import Path
import numpy as np
from OCP.gp import gp_Trsf
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeEdge,BRepBuilderAPI_MakeFace,BRepBuilderAPI_MakePolygon,BRepBuilderAPI_MakeWire
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut,BRepAlgoAPI_Section
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
from OCP.Geom import Geom_Circle
from OCP.gp import gp_Ax2,gp_Dir,gp_Pnt
from OCP.TopAbs import TopAbs_EDGE
from OCP.TopExp import TopExp_Explorer

ROOT=Path(__file__).resolve().parents[2];OUT=ROOT/'artifacts/split-drop-native'
def read(p):return json.loads(p.read_bytes())
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def rotation(p,yaw):
    co,si,cp,sp=math.cos(yaw),math.sin(yaw),math.cos(p),math.sin(p)
    return np.array([[co*cp,-si,-co*sp],[si*cp,co,-si*sp],[sp,0,cp]])
def main(rev,plugin,guide_end=None):
    sys.path.insert(0,str(plugin/'scripts'))
    from mechanical_motion import bounds,distance
    from showcase_contacts import load_solids
    reg=read(ROOT/'cad/fusion-authority.json')['source']['archive']
    native=read(OUT/(rev+'-load-01.json'));broad=read(OUT/(rev+'-all-geometry-field.json'))
    step=OUT/(rev+'-export.step')
    assert sha(ROOT/reg['file'])==reg['sha256']==native['authority']['archiveSha256']
    assert sha(step)==native['step']['sha256'] and broad['native']==native['sha256']
    target=read(ROOT/'experiments/six-wheel/mission/target.json')
    cases=read(ROOT/'experiments/swerve-drive/low-placement/clearance.json')['cases']
    slope=target['floor_slope'];norm=math.hypot(1,slope)
    align=np.array([[1/norm,0,slope/norm],[0,1,0],[-slope/norm,0,1/norm]])
    carriage='Longitudinal carriage:1/Welded carriage with two captive guide tracks'
    needs={carriage}|{x['body'] for state in broad['states'] for x in state['ambiguous'] if x['hazard']=='floor'}
    identities={};bodies,surfaces=load_solids(step,identities)
    assert not surfaces
    poses={s['pose']:s for s in native['native']['states']}
    expected_poses={'home','present','open','before-release','reset'}
    if set(poses)!=expected_poses or not all(carriage in s['bodies'] for s in poses.values()):
        raise ValueError('The native issue must resolve the carriage across all five poses')
    reference=set()
    if rev!='r4':
        previous=read(OUT/'r4-exact-floor-plane.json')['result'][carriage]['allAmbiguousFinite']
        if len(previous)!=48 or sum(bool(row['surfaceCrossingsMm']) for row in previous)!=12:
            raise ValueError('The exact R4 known-strike control is incomplete')
        reference={(row['pose'],tuple(row['dock'])) for row in previous}
    panels=[]
    intercept=1000*target['floor_intercept_m']
    for panel in target['panels']:
        wire=BRepBuilderAPI_MakePolygon()
        points=panel['bounds_xy'];winding=sum(x*v-y*u for (x,y),(u,v) in zip(points,points[1:]+points[:1]))
        for x,y in points:wire.Add(gp_Pnt(1000*x,1000*y,intercept+1000*x*slope))
        wire.Close();maker=BRepBuilderAPI_MakeFace(wire.Wire())
        for x,y,r in panel['holes_xy_r']:
            axis=gp_Ax2(gp_Pnt(1000*x,1000*y,intercept+1000*x*slope),gp_Dir(slope,0,-1) if winding>0 else gp_Dir(-slope,0,1))
            edge=BRepBuilderAPI_MakeEdge(Geom_Circle(axis,1000*r)).Edge()
            maker.Add(BRepBuilderAPI_MakeWire(edge).Wire())
        panels.append(maker.Face())
    result={}
    for name in sorted(needs):
        home=poses['home']['bodies'][name];branch=name.split('/')[0]
        hit=[k for k,s in bodies.items() if branch in identities[k]['occurrencePath'] and max(abs(a-b) for a,b in zip(bounds(s),home))<.05]
        if len(hit)!=1:raise ValueError('No unique derived solid for '+name)
        source=bodies[hit[0]];measured=[];samples={};narrow=[];field_cached={}
        if guide_end is not None:
            if rev!='r4' or 'Welded carriage' not in name or not 311<=guide_end<320:
                raise ValueError('Trim study requires the R4 carriage and intact front transverse support')
            for y in (-240,240):
                blank=BRepPrimAPI_MakeBox(gp_Pnt(guide_end,y-8,503),gp_Pnt(322,y+8,510.001)).Shape()
                cut=BRepAlgoAPI_Cut(source,blank)
                if not cut.IsDone():raise ValueError('Guide-tip cut did not finish')
                source=cut.Shape()
        for pose,state in poses.items():
            # Candidate bodies checked here must translate rigidly without rotation.
            offset=np.array(state['bodies'][name][:3])-np.array(home[:3])
            if np.linalg.norm(np.array(state['bodies'][name][3:])-np.array(home[3:])-offset)>.02:
                raise ValueError('Rotation requires measured full native transform')
            for sign,case in cases.items():
                for dx,dy,yaw in itertools.product(*zip(*case['enclosure'])):
                    for p,dz in itertools.product((-.04,.04),(-5.,5.)):
                        key=(yaw,p);rot=align@rotation(p,yaw)
                        if key not in samples:
                            tr=gp_Trsf();M=np.column_stack((rot,np.zeros(3)));tr.SetValues(*M.flatten())
                            samples[key]=bounds(BRepBuilderAPI_Transform(source,tr,False).Shape())[2]
                        loc=np.array([dx-490,dy,dz-1000*target['floor_intercept_m']])
                        actual=samples[key]+(rot@offset+align@loc)[2]
                        # The broad-phase expanded each local axis by 1 mm.
                        inflation=np.abs(rot[2]).sum()
                        measured.append(dict(pose=pose,dock=[sign,dx,dy,yaw,p,dz],
                            continuousSolidFloorNormalGapMm=actual,oneMmPerAxisNetFloorNormalGapMm=actual-inflation))
                        dock=(sign,dx,dy,yaw,p,dz)
                        if actual-inflation<0 or any((pose,(*dock,sector)) in reference for sector in (0.,360.)):
                            r=rotation(p,yaw)
                            for sector in (0.,360.):
                                translation=r@offset+np.array([dx-490,dy+sector,dz])
                                cache=tuple(np.round(np.column_stack((r,translation)).flatten(),7))
                                if cache not in field_cached:
                                    tr=gp_Trsf();tr.SetValues(*np.column_stack((r,translation)).flatten())
                                    placed=BRepBuilderAPI_Transform(source,tr,False).Shape();distances=[];edges=[]
                                    for panel in panels:
                                        gap=distance(placed,panel);distances.append(gap)
                                        if gap<1e-5:
                                            section=BRepAlgoAPI_Section(placed,panel,False);section.Build()
                                            if not section.IsDone():raise ValueError('Incomplete finite field section')
                                            it=TopExp_Explorer(section.Shape(),TopAbs_EDGE)
                                            while it.More():
                                                edges.append(bounds(it.Current()));it.Next()
                                    field_cached[cache]=dict(minFiniteFieldSurfaceGapMm=min(distances),surfaceCrossingsMm=edges)
                                narrow.append(dict(pose=pose,dock=[sign,dx,dy,yaw,p,dz,sector],**field_cached[cache]))
        if len(measured)!=len(expected_poses)*64:
            raise ValueError('Incomplete signed floor-normal enclosure')
        done={(r['pose'],tuple(r['dock'])) for r in narrow}
        if reference and name==carriage and not reference<=done:
            raise ValueError('The R4 finite negative controls were omitted from this recheck')
        result[name]=dict(derivativeSolid=hit[0],fullPath=identities[hit[0]]['occurrencePath'],
            worst=min(measured,key=lambda q:q['oneMmPerAxisNetFloorNormalGapMm']),measurements=len(measured),allAmbiguousFinite=narrow)
    basename=rev if guide_end is None else f'{rev}-concept-guide-end-{guide_end:g}'
    (OUT/(basename+'-exact-floor-plane.json')).write_text(json.dumps(dict(authority=reg,archive=native['sha256'],stepSha256=sha(step),conceptGuideEndMm=guide_end,result=result,
            scope='Exact BRep support along the infinite floor normal. Positive signed gap separates any finite real panel, seam or holes for the sampled dock and mechanism poses. An ambiguous or negative gap requires a real finite-panel section inspection.'),indent=2)+'\n')
    print([(k,v['worst']) for k,v in result.items()])
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('revision');p.add_argument('plugin',type=Path);p.add_argument('--guide-end',type=float);a=p.parse_args();main(a.revision,a.plugin,a.guide_end)
