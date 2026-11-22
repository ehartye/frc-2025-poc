from pathlib import Path
import sys,json
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE
from OCP.TopoDS import TopoDS
from OCP.GeomAbs import GeomAbs_Plane
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from four_corner_packaging import rotated
HERE=Path(__file__).resolve().parent
sys.path.insert(0,sys.argv[1]+"/scripts")
from fusion_authority import verify,register_path
from mechanical_motion import bounds
from showcase_contacts import load_solids
reg=register_path(HERE.parents[3]);line=reg.parent/"issue-005/lineage.json"; auth=verify(HERE.parents[3],reg,line)
parts,_=load_solids(line.with_name("assembly.step"));out={}
for name,s in parts.items():
    if not any(x in name for x in ("[1.1.1.21.16]","[1.1.1.21.17]")):continue
    s=rotated(s,0,(-12.7,-12.7,0));it=TopExp_Explorer(s,TopAbs_FACE);data=[]
    while it.More():
        face=(getattr(TopoDS,"Face_s",None) or TopoDS.Face)(it.Current());it.Next();surf=BRepAdaptor_Surface(face)
        if surf.GetType()!=GeomAbs_Plane:continue
        n=surf.Plane().Axis().Direction().Coord()
        bb=bounds(face);props=GProp_GProps();(getattr(BRepGProp,"SurfaceProperties_s",None) or BRepGProp.SurfaceProperties)(face,props)
        if bb[1]>334 or bb[4]<295 or bb[0]>242 or bb[3]<150 or abs(n[2])>.05 or props.Mass()<4:continue
        data.append(dict(n=n,bb=bb,area=props.Mass()))
    out[name]=data
(HERE/"inset-factory-bearing-faces.json").write_text(json.dumps(dict(authority=auth,candidates=out),indent=2)+"\n")
for k,ls in out.items():
    print(k)
    for r in ls:print(round(r['area'],1),[round(z,2) for z in r['n']],[round(z,2) for z in r['bb']])


