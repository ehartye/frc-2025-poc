"""Challenge the fresh corner layout against a six-face bumper support and the original four kingpins."""
import argparse
import json
import math
import sys
from pathlib import Path
import numpy as np
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeEdge,BRepBuilderAPI_MakeWire,BRepBuilderAPI_MakeFace
from OCP.BRepPrimAPI import BRepPrimAPI_MakePrism
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
from OCP.gp import gp_Trsf,gp_Ax1,gp_Pnt,gp_Dir,gp_Vec
from OCP.TopLoc import TopLoc_Location


def polygon(xyz):
    w=BRepBuilderAPI_MakeWire()
    for a,b in zip(xyz,xyz[1:]+xyz[:1]):w.Add(BRepBuilderAPI_MakeEdge(gp_Pnt(*a),gp_Pnt(*b)).Edge())
    return BRepBuilderAPI_MakeFace(w.Wire()).Face()


def rotated(solid,delta,translation=(0,0,0)):
    m=gp_Trsf();m.SetRotation(gp_Ax1(gp_Pnt(0,0,0),gp_Dir(0,0,1)),delta)
    m.SetTranslationPart(gp_Vec(*translation))
    return solid.Moved(TopLoc_Location(m))


def outline(hx,hy,cut,inset):
    # Parallel offsets preserve the miter and its true planar wall thickness.
    a=hx-inset;b=hy-inset;c=cut-inset*(2-math.sqrt(2))
    return [[a,b],[-a+c,b],[-a,b-c],[-a,-b+c],[-a+c,-b],[a,-b]]


def band(hx,hy,cut,width,bottom,height):
    outer=outline(hx,hy,cut,0);inner=outline(hx,hy,cut,width)
    members=[]
    for i in range(6):
        j=(i+1)%6
        pts=[outer[i],outer[j],inner[j],inner[i]]
        solid=BRepPrimAPI_MakePrism(polygon([[*v,bottom] for v in pts]),gp_Vec(0,0,height)).Shape()
        members.append(solid)
    return outer,members


def run(project,plugin):
    sys.path.insert(0,str(plugin/'scripts'))
    from fusion_authority import register_path,verify,load,resolve,digest
    from showcase_contacts import load_solids,box_gap
    from mechanical_motion import common,bounds,distance
    cad=register_path(project).parent;lineage=cad/'issue-001/lineage.json';basis=verify(project,cad/'fusion-authority.json',lineage)
    step=resolve(project,load(lineage)['step']);lab={};items,sheets=load_solids(step,lab)
    rails={'Front transverse socket rail','Port longitudinal socket rail'}
    src={k:v for k,v in items.items() if k not in rails}
    if len(src)!=121 or sheets or len(items)!=123:raise ValueError('Wrong saved master')
    centers={'FL':(282,277,0),'FR':(282,-277,-90),'RL':(-282,277,90),'RR':(-282,-277,180)}
    scenes={};limits={};output={}
    for corner,(x,y,deg) in centers.items():
        a=math.radians(deg);c,s=math.cos(a),math.sin(a)
        t=(x-282*c+277*s,y-282*s-277*c,0)
        scenes[corner]={k:(v if corner=='FL' else rotated(v,a,t)) for k,v in src.items()}
        limits[corner]={k:bounds(v) for k,v in scenes[corner].items()}
    for name,hx,hy,cut in [('square-datum-control',348.675,343.675,68.735),('outside-bumper-support',370.,365.,73.),('shorter-rear-miters',370.,365.,42.)]:
        outer,members=band(hx,hy,cut,25.4,101.6,25.4)
        row={'outlineXYmm':outer,'outerPerimeterMm':sum(math.dist(p,q) for p,q in zip(outer,outer[1:]+outer[:1])),
             'bottomMm':101.6,'supportOuterSectionMm':[25.4,25.4],'possible':0,'overlaps':{},'minNearGapMm':None}
        bb=[bounds(m) for m in members]
        for corner,solids in scenes.items():
            for k,part in solids.items():
                for i,m in enumerate(members):
                    gap=box_gap(limits[corner][k],bb[i])
                    if gap>2:continue
                    row['possible']+=1
                    v=common(part,m)
                    key=f'{corner} segment {i} / '+k
                    if v>.01:row['overlaps'][key]=v
                    elif row['minNearGapMm'] is None or gap<row['minNearGapMm']:
                        g=distance(part,m)
                        row['minNearGapMm']=min(row['minNearGapMm'],g) if row['minNearGapMm'] is not None else g
        output[name]=row
    return dict(nativeSha256=basis['archiveSha256'],canonicalStepSha256=basis['stepSha256'],kingpinsXYmmAndRelativeYawDeg=centers,
                selectedModuleBodiesPerCorner=121,scenarios=output,scope='Four signed instances of the issued module. Six proposed solid enclosing support sections and their real miter planes. Open stock-wall and joining details, wheel azimuth, loaded deflection, bumper drop and fasteners. The published robot remains historical.')


if __name__=='__main__':
    p=argparse.ArgumentParser()
    for n in ('root','plugin','output'):p.add_argument(n,type=Path)
    a=p.parse_args();v=run(a.root.resolve(),a.plugin);a.output.write_text(json.dumps(v,indent=2)+'\n')
    for k,r in v['scenarios'].items():print(k,r['outerPerimeterMm'],r['possible'],len(r['overlaps']),r['minNearGapMm'])

