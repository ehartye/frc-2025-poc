"""Check two explicit power/signal dressing directions from actual purchased cut-end faces."""
from pathlib import Path
import hashlib,json,sys
import numpy as np
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeEdge
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.GeomAbs import GeomAbs_Plane
from OCP.GC import GC_MakeArcOfCircle
from OCP.TopAbs import TopAbs_FACE
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS
from OCP.gp import gp_Pnt
from inset_stock_check import HERE,form
from inset_v11_screw_tip import read,transform

def run(scripts,src,clocks=(0,180)):
    sys.path.insert(0,str(scripts))
    from mechanical_motion import bounds,distance
    from showcase_contacts import load_solids,box_gap
    from fusion_authority import register_path,verify
    reg=register_path(HERE.parents[3])
    authority=verify(HERE.parents[3],reg,reg.parent/"issue-005/lineage.json")
    key=read("inset-oem-steering-key.json")
    d=HERE.parent/"private-inputs/matched-sds/corner.step"
    if authority!=key["authority"] or hashlib.sha256(src.read_bytes()).hexdigest()!=key["motorSHA256"] or hashlib.sha256(d.read_bytes()).hexdigest()!=key["donorStep"]:
        raise ValueError("Unpinned input")
    motor=next(iter(load_solids(src)[0].values()))
    source=load_solids(d)[0];donor={}
    for n,s in source.items():
        if ".36.1]" in n and "Motor, NEO" in n: continue
        donor[n]=(s,bounds(s))
    data=read("inset-chassis-shop.json");references=read("inset-stock-check.json")
    if data["authority"]!=authority or hashlib.sha256((HERE/"inset-chassis-shop.json").read_bytes()).hexdigest()!=references["shopPlanSHA256"]:raise ValueError("Changed stock")
    stock={r["name"]:r for r in data["articles"]}
    stock_shapes={}
    def part(n):
        if n not in stock_shapes:
            r=stock[n];s=form(r);stock_shapes[n]=transform(s,np.array(r["matrixMm"])) if "matrixMm" in r else s
        return stock_shapes[n]
    caps=[]
    it=TopExp_Explorer(motor,TopAbs_FACE)
    while it.More():
        face=TopoDS.Face(it.Current());it.Next();surf=BRepAdaptor_Surface(face)
        if surf.GetType()!=GeomAbs_Plane:continue
        prop=GProp_GProps();BRepGProp.SurfaceProperties_s(face,prop)
        point=np.array(prop.CentreOfMass().Coord());area=prop.Mass()
        if np.linalg.norm(point[:2])>42 and 12<area<16.5:
            tang=np.array(surf.Plane().Axis().Direction().Coord())
            if tang[:2]@point[:2]<0:tang=-tang
            caps.append((point,tang,np.sqrt(area/np.pi)))
    if len(caps)!=4:raise ValueError("Lost actual cut-end power or sensor lead")
    caps.sort(key=lambda c:c[0][0])
    shaft0=np.array(key["referenceShaftPoseMm"])
    placements=read("inset-matched-corners.json")["newWorldMm"]
    report=[]
    for face,(p,t,r) in enumerate(caps,1):
        for sign in [-1,1]:
            perp=np.array([0.,0.,float(sign)]);perp-=t*(t@perp);perp/=np.linalg.norm(perp)
            radius=r*2*6
            start=p+12*t
            curve=lambda a:start+radius*(np.sin(a)*t+(1-np.cos(a))*perp)
            end=curve(np.pi/2)
            edges=[BRepBuilderAPI_MakeEdge(gp_Pnt(*p),gp_Pnt(*start)).Edge(),
                   BRepBuilderAPI_MakeEdge(GC_MakeArcOfCircle(*[gp_Pnt(*curve(a)) for a in [0,np.pi/4,np.pi/2]]).Value()).Edge(),
                   BRepBuilderAPI_MakeEdge(gp_Pnt(*end),gp_Pnt(*(end+20*perp))).Edge()]
            for clock in clocks:
                a=np.deg2rad(clock);c=np.cos(a);s=np.sin(a)
                shaft=shaft0 @ np.array([[c,-s,0,0],[s,c,0,0],[0,0,1,0],[0,0,0,1]])
                for label,W in placements.items():
                    W=np.array(W);segments=[]
                    for shape in edges:
                        local=transform(shape,shaft);world=transform(local,W)
                        bs=bounds(local);bw=bounds(world)
                        candidates=[(box_gap(bs,bb),n,False) for n,(s,bb) in donor.items()]
                        candidates.extend((box_gap(bw,row["bounds"]),n,True) for n,row in references["parts"].items())
                        best=float("inf");near=None
                        for lower,n,is_stock in sorted(candidates):
                            if lower>=best:break
                            gap=distance(world,part(n)) if is_stock else distance(local,donor[n][0])
                            if gap<best:best=gap;near=n
                        segments.append(dict(nearest=near,surfaceGapMm=best-r))
                    report.append(dict(lead=face,rootLocalMm=p.tolist(),diameterMm=2*r,localOutward=t.tolist(),
                                  casingClockDeg=clock,turnToLocalZ=sign,designBendRadiusMm=radius,corner=label,segments=segments))
    return dict(authority=authority,motorSHA256=key["motorSHA256"],moduleSHA256=key["donorStep"],shopSHA256=references["shopPlanSHA256"],
                rows=report,scope="12 mm straight tangent, six lead diameters bend radius, 90 degree turn, and 20 mm terminal straight in the prescribed direction. Signed surface gap = minimum edge-to-stock distance minus actual cut-end radius. Full source module except the withdrawn old steering motor plus all 146 neighboring chassis stock elements. Source leads outside the CAD ends are prospective. No vibration, boot strain relief, bend fatigue, remaining harness, service sequence, or moving geometry credited.")
if __name__=="__main__":
    side=len(sys.argv)>3 and sys.argv[3]=="side"
    clocks=(-90,90) if side else (0,180)
    report=run(Path(sys.argv[1]),Path(sys.argv[2]),clocks)
    destination="inset-v11-quarter-turn-leads.json" if side else "inset-v11-dressed-leads.json"
    (HERE/destination).write_text(json.dumps(report,indent=1)+"\n")
