"""Locate the casing/sill intersection and compare the registered and half-turn motor/pinion clock."""
import hashlib,json,sys
from pathlib import Path
import numpy as np
from inset_stock_check import HERE,form
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.TopAbs import TopAbs_FACE,TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS
from OCP.GeomAbs import GeomAbs_Cylinder
from OCP.gp import gp_Trsf
from OCP.TopLoc import TopLoc_Location

def run(scripts,alt,clocks=(0,180)):
    sys.path.insert(0,str(scripts))
    from fusion_authority import register_path,verify
    from showcase_contacts import load_solids,box_gap
    from mechanical_motion import bounds,distance
    reg=register_path(HERE.parents[3]);auth=verify(HERE.parents[3],reg,reg.parent/"issue-005/lineage.json")
    basis=json.loads((HERE/"inset-v11-mounted-steering.json").read_bytes());cap=json.loads((HERE/"inset-oem-steering-key.json").read_bytes())
    if auth!=basis["authority"] or hashlib.sha256(alt.read_bytes()).hexdigest()!=basis["alternateSHA256"]:raise ValueError("Changed source")
    plan=json.loads((HERE/"inset-matched-corners.json").read_bytes());refs=json.loads((HERE/"inset-stock-check.json").read_bytes())["parts"]
    stock={v["name"]:v for v in json.loads((HERE/"inset-chassis-shop.json").read_bytes())["articles"]}
    source=next(iter(load_solids(alt)[0].values()));shaft=np.array(cap["referenceShaftPoseMm"])
    def move(body,M):
        t=gp_Trsf();t.SetValues(*M[:3].ravel());return body.Moved(TopLoc_Location(t))
    def witness(a,b):
        op=BRepAlgoAPI_Common(a,b)
        if not op.IsDone():raise ValueError("No common produced")
        it=TopExp_Explorer(op.Shape(),TopAbs_SOLID);out=[]
        while it.More():
            q=it.Current();p=GProp_GProps();BRepGProp.VolumeProperties_s(q,p)
            out.append(dict(commonMm3=p.Mass(),centerMm=list(p.CentreOfMass().Coord()),boundsMm=bounds(q)))
            it.Next()
        return out
    bodies={}
    def host(name):
        if name not in bodies:
            item=stock[name];v=form(item);bodies[name]=move(v,np.array(item["matrixMm"])) if "matrixMm" in item else v
        return bodies[name]
    cylinders=[];it=TopExp_Explorer(source,TopAbs_FACE)
    while it.More():
        q=TopoDS.Face(it.Current());surface=BRepAdaptor_Surface(q)
        if surface.GetType()==GeomAbs_Cylinder:
            cy=surface.Cylinder();axis=np.array(cy.Axis().Direction().Coord());pt=cy.Location().Coord()
            if abs(axis[2])>.999 and abs(np.hypot(pt[0],pt[1])-25.4)<.10:
                cylinders.append(dict(centerXY=pt[:2],radius=cy.Radius(),zRange=[bounds(q)[2],bounds(q)[5]]))
        it.Next()
    outcomes=[]
    fit=json.loads((HERE/"inset-corner-local-fits.json").read_bytes())
    for corner,wm in plan["newWorldMm"].items():
        W=np.array(wm)
        for angle in clocks:
            a=np.deg2rad(angle);cs=np.cos(a);sn=np.sin(a)
            spin=np.array([[cs,-sn,0,0],[sn,cs,0,0],[0,0,1.,0],[0,0,0,1.]])
            M=W@shaft@spin
            motor=move(source,M);mb=bounds(motor);hits=[]
            for lb,k in sorted((box_gap(mb,v["bounds"]),n) for n,v in refs.items()):
                if lb>3:break
                solid=host(k);ww=witness(motor,solid) if lb<.001 else []
                for item in ww:
                    item["centerInMotorMm"]=(np.linalg.inv(M)@[*item["centerMm"],1])[:3].tolist()
                hits.append(dict(name=k,boundsGapMm=lb,clearanceMm=distance(motor,solid),overlap=ww))
            axes=[]
            for r in fit["motorCases"]:
                if r["corner"]!=corner or ".36.1]" not in r["case"]["host"]:continue
                local=(np.linalg.inv(M)@[*r["case"]["underHeadCenterMm"],1])[:2]
                near=[q for q in cylinders if np.linalg.norm(np.array(q["centerXY"])-local)<.005]
                axes.append(dict(shaftLocalXYmm=local.tolist(),coaxialCylinders=near))
            outcomes.append(dict(corner=corner,casingClockDeg=angle,mountAxes=axes,geometry=hits))
    return dict(authority=auth,sourceMotor=cap["motorSHA256"],assembly508=cap["cold508"],rows=outcomes,
                scope="Half turn keeps a complete 14-tooth input pinion on an integral seven-tooth pitch and moves the key and motor together. Confirms actual casing radial hole locations and static frame packaging only; reinstating the selected clock requires purchased key, pinion, leads, retention and internal assembly checks.")
if __name__=="__main__":
    side=len(sys.argv)>3 and sys.argv[3]=="side"
    d=run(Path(sys.argv[1]),Path(sys.argv[2]),(-90,90) if side else (0,180))
    if side:d["scope"]="Two alternate three-hole motor casing mounts, stock intersections and external packaging only. Source rotor position must be decoupled and registered to the pinion mesh; operating torque and lead removal are separate."
    target="inset-v11-quarter-turn-casing.json" if side else "inset-v11-casing-clock.json"
    (HERE/target).write_text(json.dumps(d,indent=1)+"\n")
