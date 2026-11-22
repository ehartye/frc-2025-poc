from pathlib import Path
import sys,json
from OCP.BRepAlgoAPI import BRepAlgoAPI_Section
from OCP.BRepAdaptor import BRepAdaptor_Curve
from OCP.GeomAbs import GeomAbs_Circle,GeomAbs_Line
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_EDGE
from OCP.TopoDS import TopoDS
from four_corner_packaging import rotated,polygon
HERE=Path(__file__).resolve().parent
sys.path.insert(0,sys.argv[1]+"/scripts")
from fusion_authority import register_path,verify
from showcase_contacts import load_solids
reg=register_path(HERE.parents[3]);line=reg.parent/"issue-005/lineage.json";authority=verify(HERE.parents[3],reg,line)
parts,_=load_solids(line.with_name("assembly.step"));out={}
for key,dz in (("[1.1.1.21.17]",97.),("[1.1.1.21.16]",40.)):
    name,s=next((n,s) for n,s in parts.items() if key in n)
    win=polygon([[157,285,dz],[228,285,dz],[228,334,dz],[157,334,dz]])
    op=BRepAlgoAPI_Section(rotated(s,0,(-12.7,-12.7,0)),win)
    rows=[];it=TopExp_Explorer(op.Shape(),TopAbs_EDGE)
    while it.More():
        e=(getattr(TopoDS,'Edge_s',None) or TopoDS.Edge)(it.Current());it.Next()
        a=BRepAdaptor_Curve(e);p0=a.FirstParameter();p1=a.LastParameter()
        pts=[list(a.Value(p0+v*(p1-p0)).Coord()) for v in (0,.5,1)]
        row=dict(endpointsAndMidMm=pts,kind=str(a.GetType()))
        if a.GetType()==GeomAbs_Circle:row.update(center=list(a.Circle().Location().Coord()),r=a.Circle().Radius())
        rows.append(row)
    out[name]=rows
(HERE/"inset-factory-edge-section.json").write_text(json.dumps(dict(authority=authority,sections=out),indent=2)+"\n")
for name,rows in out.items():
    print(name)
    for s in rows:
        pts=s['endpointsAndMidMm']
        if min(a[1] for a in pts)>315.5:continue
        print(s['kind'],[[round(x,2) for x in v[:2]] for v in pts],round(s.get('r',0),3))

