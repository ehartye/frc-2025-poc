"""Retain the exact named socket screw occurrence and its true under-head bearing datum."""
import json
import sys
from pathlib import Path
import numpy as np
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.GeomAbs import GeomAbs_Cylinder,GeomAbs_Plane
from OCP.STEPControl import STEPControl_Writer, STEPControl_AsIs
from OCP.IFSelect import IFSelect_RetDone
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE, TopAbs_VERTEX
from OCP.TopoDS import TopoDS
from OCP.BRep import BRep_Tool
from output_mount import ROOT,digest,read,place


def extract(plugin,donor):
    sys.path.insert(0,str(plugin/'scripts'))
    from mechanical_motion import bounds
    from showcase_contacts import load_solids
    folder=ROOT/'artifacts/red2-integrated/pan-thrust/fastener-option'
    info=read(donor/'source.json');step=donor/'SDS-MK4i-10-32-NEO.step'
    assert digest(step)==info['sha256']
    labels={};items,_=load_solids(step,labels)
    result={'source':info['source'],'sha256':digest(step),'license':info['rights'],'parts':{}}
    for part,length in [('91251A351',38.1),('91251A354',57.15)]:
        matches=[k for k in items if part in k]
        if not matches:raise ValueError(part)
        key=matches[0];source=items[key];faces=TopExp_Explorer(source,TopAbs_FACE)
        planes=[];circles=[]
        while faces.More():
            f=TopoDS.Face(faces.Current());faces.Next();sf=BRepAdaptor_Surface(f)
            metric=GProp_GProps();BRepGProp.SurfaceProperties_s(f,metric)
            if sf.GetType()==GeomAbs_Plane:
                q=sf.Plane();planes.append((metric.Mass(),np.array(q.Location().Coord()),np.array(q.Axis().Direction().Coord())))
            if sf.GetType()==GeomAbs_Cylinder:
                q=sf.Cylinder();circles.append((q.Radius(),np.array(q.Location().Coord()),np.array(q.Axis().Direction().Coord())))
        _,center,axis=max(circles,key=lambda x:x[0])
        _,point,normal=max((row for row in planes if abs(row[2]@axis)>.9999),key=lambda x:x[0])
        center=center+axis*(normal@(point-center))/(normal@axis)
        corners=[];vertices=TopExp_Explorer(source,TopAbs_VERTEX)
        while vertices.More():
            v=TopoDS.Vertex(vertices.Current());vertices.Next()
            corners.append(np.array(BRep_Tool.Pnt_s(v).Coord()))
        spans=(np.array(corners)-center)@axis
        if -min(spans)>max(spans):axis=-axis;spans=-spans
        if abs(max(spans)-length)>.01 or abs(min(spans)+4.826)>.01:
            raise ValueError((part,center,axis,min(spans),max(spans)))
        ref=np.eye(3)[np.argmin(abs(axis))]
        u=ref-np.dot(ref,axis)*axis;u/=np.linalg.norm(u)
        v=np.cross(axis,u);R=np.vstack((u,v,axis));matrix=np.column_stack((R,-R@center))
        path=folder/(part+'-datum.step')
        wr=STEPControl_Writer();wr.Transfer(place(source,matrix),STEPControl_AsIs)
        if wr.Write(str(path))!=IFSelect_RetDone:raise ValueError('Neutral issue failed')
        result['parts'][part]=dict(sourceId=key,occurrence=labels[key]['occurrencePath'],sourceCount=len(matches),
                         commissionedSourceMatrixMm=matrix.tolist(),bearingPlaneOriginMm=center.tolist(),installedAxis=axis.tolist(),
                         screwLengthMm=max(spans),headHeightMm=-min(spans),leaf=path.name,sha256=digest(path))
    (folder/'sds-threaded-screws.json').write_text(json.dumps(result,indent=2)+'\n')
    print(result['parts'],flush=True)


if __name__=='__main__':extract(Path(sys.argv[1]),Path(sys.argv[2]))
