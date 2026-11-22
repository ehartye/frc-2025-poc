"""Measure actual OEM screw core coverage over both finite fitted-shoe bearing lands."""
from pathlib import Path
import hashlib,json,sys
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeEdge
from OCP.BRepClass3d import BRepClass3d_SolidClassifier
from OCP.TopAbs import TopAbs_IN,TopAbs_SHELL
from OCP.TopExp import TopExp_Explorer
from OCP.gp import gp_Pnt
from crossmember_envelope import cylinder

HERE=Path(__file__).resolve().parent


def run(plugin):
    sys.path.insert(0,str(plugin/"scripts"))
    from fusion_authority import register_path,verify
    from showcase_contacts import load_solids
    from component_shape import volume
    from mechanical_motion import bounds,distance
    reg=register_path(HERE.parents[3]);line=reg.parent/"issue-005/lineage.json"
    authority=verify(HERE.parents[3],reg,line);parts,_=load_solids(line.with_name("assembly.step"))
    f=HERE/"inset-socket-shoe.json";shoe=json.loads(f.read_bytes())
    if authority!=shoe["authority"]:raise ValueError("Registration mismatch")
    rows=[]
    for j,(rank,t) in enumerate((("inner",9.),("middle",4.25),("outer",6.))):
        x=shoe["axesMm"][j]+12.7;y=330.975
        keys=[(k,s) for k,s in parts.items() if "91251A355" in k and "[1.1.1." in k
              and abs((bounds(s)[0]+bounds(s)[3])/2-x)<.15 and abs((bounds(s)[1]+bounds(s)[4])/2-y)<.15]
        if len(keys)!=1:raise ValueError("Duplicate or missing actual article")
        k,s=keys[0]
        req=shoe["holeScreens"][rank]["requiredEffectiveDiameterAt120Mm"]
        for end,start in (("bottom",43.83336287672979+3.2),("top",94.63336287672979-3.2-t)):
            segment=BRepBuilderAPI_MakeEdge(gp_Pnt(x,y,start),gp_Pnt(x,y,start+t)).Edge()
            if any(BRepClass3d_SolidClassifier(s,gp_Pnt(x,y,start+q*t),1e-7).State()!=TopAbs_IN for q in (0.,.5,1.)):
                raise ValueError("Gauge centerline not in the selected screw")
            shell=TopExp_Explorer(s,TopAbs_SHELL); clearance=[]
            while shell.More():
                clearance.append(distance(segment,shell.Current()))
                shell.Next()
            if not clearance:raise ValueError("Missing true article boundary")
            # A continuous inside segment with positive distance to the actual closed
            # shell contains the full finite test cylinder up to this radius.
            guaranteed=2*min(clearance)
            if guaranteed>=5.15:raise ValueError("Oversized negative did not discriminate")
            rows.append(dict(rank=rank,article=k,startZmm=start,land=end,lengthMm=t,
                requiredCoreMm=req,guaranteedInscribedDiameterMm=guaranteed,passed=guaranteed>=req,
                oversizedDiameterMm=5.15,oversizedCertified=False))
    return dict(authority=authority,registeredShoeSha256=hashlib.sha256(f.read_bytes()).hexdigest(),rows=rows,
        scope="A positive contained steel core over the complete modeled bearing length is only a geometric lower bound on cross section. Actual screw contact stress and unequal sharing, bending over the tube height, preload, grade, surface, hole alignment and torque remain outside this test.")


if __name__=="__main__":
    d=run(Path(sys.argv[1]));(HERE/"inset-shoe-screw-cores.json").write_text(json.dumps(d,indent=2)+"\n")
    for row in d["rows"]:print(row["rank"],row["land"],row["guaranteedInscribedDiameterMm"],row["requiredCoreMm"])

