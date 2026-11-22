from pathlib import Path
import json,sys
from inset_chassis_shop import HERE


def define(pkg):
    sys.path.insert(0,str(pkg/"scripts"))
    from fusion_authority import register_path,verify
    from showcase_contacts import load_solids
    from mechanical_motion import bounds
    from OCP.BRepAdaptor import BRepAdaptor_Surface
    from OCP.TopAbs import TopAbs_FACE
    from OCP.TopoDS import TopoDS
    from OCP.TopExp import TopExp_Explorer
    from OCP.GeomAbs import GeomAbs_Plane,GeomAbs_Cylinder
    from OCP.GProp import GProp_GProps
    from OCP.BRepGProp import BRepGProp
    import numpy as np
    root=HERE.parents[3];reg=register_path(root);f=reg.parent/"issue-005/assembly.step";auth=verify(root,reg,f.with_name("lineage.json"))
    s,_=load_solids(f);rows={}
    for sku in ["91251A351","91251A354","91251A542","91251A539"]:
        name=next(k for k in s if sku in k and "[1.1.1." in k);shape=s[name];b=bounds(shape)
        it=TopExp_Explorer(shape,TopAbs_FACE);faces=[];axes=[]
        while it.More():
            face=TopoDS.Face(it.Current());surf=BRepAdaptor_Surface(face)
            if surf.GetType()==GeomAbs_Cylinder:
                c=surf.Cylinder()
                axes.append((c.Radius(),np.asarray(c.Location().Coord()),np.asarray(c.Axis().Direction().Coord())))
            if surf.GetType()==GeomAbs_Plane:
                props=GProp_GProps();BRepGProp.SurfaceProperties_s(face,props)
                plane=surf.Plane()
                faces.append((props.Mass(),np.asarray(plane.Location().Coord()),np.asarray(plane.Axis().Direction().Coord())))
            it.Next()
        radius,o,axis=max(axes,key=lambda r:r[0])
        cap=[(area,o+axis*((p-o)@n/(axis@n))) for area,p,n in faces if abs(axis@n)>.99999 and area>12.]
        middle=(np.array(b[:3])+b[3:])/2
        cap.sort(key=lambda v:abs((v[1]-middle)@axis))
        head=cap[0][1];direction=axis*np.sign((middle-head)@axis)
        rows[sku]=dict(source=name,boundsMm=b,underHeadCenterMm=head.tolist(),intoThreadUnit=direction.tolist(),
                       headRadiusMm=radius,headFaces=[dict(areaMm2=a,center=p.tolist()) for a,p in cap])
    return dict(authority=auth,originals=rows)
if __name__=="__main__":
    r=define(Path(sys.argv[1]));(HERE/"inset-mount-purchases.json").write_text(json.dumps(r,indent=2)+"\n");print(r["originals"])
