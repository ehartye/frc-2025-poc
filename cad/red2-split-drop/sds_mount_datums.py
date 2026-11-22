"""Measure the two material faces and predrilled mounting lines at all six SDS frame screws."""
import json,sys
from pathlib import Path
import numpy as np
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Cylinder
from OCP.IntCurvesFace import IntCurvesFace_ShapeIntersector
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE
from OCP.TopoDS import TopoDS
from OCP.gp import gp_Ax1,gp_Dir,gp_Lin,gp_Pnt
from output_mount import ROOT,read,digest,place


def run(plugin,source):
    sys.path.insert(0,str(plugin/'scripts'))
    from mechanical_motion import bounds
    from showcase_contacts import load_solids
    folder=ROOT/'artifacts/red2-integrated/module-option'
    fit=read(folder/'sds-2023-fl-fit.json')
    assert digest(source)==fit['vendorStepSha256'] and fit['cases']['90']['staticBodiesChecked']==125
    all,_=load_solids(source);M=np.array(fit['cases']['90']['physicalPlacementMm'])
    refs={n:place(s,M) for n,s in all.items() if '91251A355' in n or n.endswith(('[1.43.16]','[1.43.17]'))}
    lower,upper=[next(s for n,s in refs.items() if n.endswith(k)) for k in ['[1.43.16]','[1.43.17]']]
    def ray(body,x,y):
        query=IntCurvesFace_ShapeIntersector();query.Load(body,1e-7);query.Perform(gp_Lin(gp_Pnt(x,y,-5.),gp_Dir(0,0,1)),0,165)
        if not query.IsDone():raise ValueError('Incomplete through-plate ray')
        return sorted({round(query.Pnt(i).Z(),7) for i in range(1,query.NbPnt()+1)})
    def cylinders(body,x,y):
        out=[];it=TopExp_Explorer(body,TopAbs_FACE)
        while it.More():
            f=TopoDS.Face(it.Current());it.Next();s=BRepAdaptor_Surface(f)
            if s.GetType()!=GeomAbs_Cylinder:continue
            cylinder=s.Cylinder();axis=cylinder.Axis();n=axis.Direction();p=axis.Location()
            if abs(n.Z())>.99999 and abs(p.X()-x)<.02 and abs(p.Y()-y)<.02:
                out.append(dict(axisXYmm=[p.X(),p.Y()],diameterMm=2*cylinder.Radius(),boundsMm=bounds(f)))
        return out
    rows={}
    for n,screw in refs.items():
        if '91251A355' not in n:continue
        b=bounds(screw);x,y=(b[0]+b[3])/2,(b[1]+b[4])/2
        cs=[cylinders(sh,x,y) for sh in (screw,lower,upper)]
        if any(not q for q in cs):raise ValueError(('Missing real coaxial axes',n,[len(a) for a in cs]))
        probes={}
        for dx,dy in [(5.,0.),(-5.,0.),(0.,5.),(0.,-5.),(9.,0.),(-9.,0.),(0.,9.),(0.,-9.)]:
            L=ray(lower,x+dx,y+dy);U=ray(upper,x+dx,y+dy)
            probes[str((dx,dy))]=dict(bottomPlateZmm=L,mainPlateZmm=U,seatSeparationMm=(U[0]-L[-1] if U and L else None))
        rows[n]=dict(XYmm=[x,y],screwAndPlateCylinders=cs,stockProbes=probes)
    reg=dict(authority=fit['authority'],actualAssemblySha256=fit['currentNative'],sourceSha256=fit['vendorStepSha256'],transformMm=M.tolist(),
        upperNamedBody='MK4i Master Part V2_NEO Config [1.43.17]',lowerNamedBody='MK4i Master Part V2_NEO Config [1.43.16]',rows=rows)
    (folder/'sds-six-installed-datums.json').write_text(json.dumps(reg,indent=2)+'\n')
    for n,row in rows.items():print(n,row['XYmm'],sorted({q['seatSeparationMm'] for q in row['stockProbes'].values() if q['seatSeparationMm'] is not None}),flush=True)


if __name__=='__main__':run(Path(sys.argv[1]),Path(sys.argv[2]))
