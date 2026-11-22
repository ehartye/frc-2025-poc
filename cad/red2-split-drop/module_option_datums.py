"""Inspect exact supplier faces for the selected keyed NEO and V1.1 top-stack datums."""
import json
import sys
from pathlib import Path

from OCP.BRep import BRep_Builder
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.BRepGProp import BRepGProp
from OCP.BRepTools import BRepTools
from OCP.GeomAbs import GeomAbs_Plane, GeomAbs_Cylinder
from OCP.GProp import GProp_GProps
from OCP.TopAbs import TopAbs_FACE
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS, TopoDS_Shape

from output_mount import ROOT, digest, read


def datums(plugin,shared,donor):
    sys.path.insert(0,str(plugin/'scripts'))
    from mechanical_motion import bounds
    from showcase_contacts import load_solids
    reg=read(ROOT/'cad/fusion-authority.json')['source']['archive']
    assert digest(ROOT/reg['file'])==reg['sha256']
    manifest=shared/'manifest.json'
    assert digest(manifest)=='ec7fb1790d5ee4e35e5b7eceaf68b2f35c7c32fb7fe7095d7c27379201819adf'
    originals={row['file']:row for row in read(manifest)['files']}
    assert digest(donor/'REV-21-3005.STEP')==originals['original-2022/REV-21-3005.STEP']['sha256']
    output=ROOT/'artifacts/red2-integrated/module-option';output.mkdir(exist_ok=True)
    parts={}
    for name in ['0-REV-21-1650','21-REV-21-3005-P09','22-REV-21-3005-P10','24-REV-11-1271','28-REV-21-3005-P13','12-REV-21-3005-P06']:
        source=donor/'vendor-native'/(name+'.brep')
        shape=TopoDS_Shape()
        assert BRepTools.Read_s(shape,str(source),BRep_Builder())
        parts[name]=(shape,source)
    for name in ['P24','P25','P26']:
        article='v1-1-detail/'+name+'.step'
        source=shared/article
        assert digest(source)==originals[article]['sha256']
        shapes=load_solids(source)[0]
        assert len(shapes)==1
        parts[name]=(next(iter(shapes.values())),source)
    report={}
    for name,(shape,source) in parts.items():
        it=TopExp_Explorer(shape,TopAbs_FACE)
        faces=[]
        while it.More():
            face=(getattr(TopoDS,'Face_s',None) or TopoDS.Face)(it.Current())
            it.Next()
            surf=BRepAdaptor_Surface(face)
            props=GProp_GProps()
            BRepGProp.SurfaceProperties_s(face,props)
            row=dict(areaMm2=props.Mass(),centerMm=props.CentreOfMass().Coord(),boundsMm=bounds(face))
            if surf.GetType()==GeomAbs_Cylinder:
                primitive=surf.Cylinder()
                row.update(kind='cylinder',radiusMm=primitive.Radius())
            elif surf.GetType()==GeomAbs_Plane:
                primitive=surf.Plane()
                row.update(kind='plane')
            else:
                continue
            row.update(direction=primitive.Axis().Direction().Coord(),originMm=primitive.Axis().Location().Coord())
            faces.append(row)
        report[name]=dict(sourceSha256=digest(source),boundsMm=bounds(shape),faces=sorted(faces,key=lambda a:-a['areaMm2']))
        print(name,[round(v,3) for v in bounds(shape)])
    (output/'source-datums.json').write_text(json.dumps(dict(authority=reg,manifestSha256=digest(manifest),parts=report),indent=2))


if __name__=='__main__':datums(*[Path(arg) for arg in sys.argv[1:]])
