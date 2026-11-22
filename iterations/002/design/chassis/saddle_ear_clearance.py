"""Locate the four exact stock/motor overlaps and evaluate the remaining receiver behind each tapped hole."""
import json, math, sys
from pathlib import Path
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common

from four_corner_packaging import rotated
from port_anchor_fit import cut
from crossmember_envelope import box,cylinder
from direct_saddle_packaging import exterior_shapes

HERE=Path(__file__).resolve().parent


def run(package):
    sys.path.insert(0,str(package/"scripts"))
    from fusion_authority import verify,register_path
    from component_shape import volume
    from mechanical_motion import bounds,common,distance
    from showcase_contacts import load_solids
    from structural_perimeter import relocate_lower
    root=HERE.parents[3];reg=register_path(root);lin=reg.parent/"issue-005/lineage.json"
    auth=verify(root,reg,lin);parts,sheets=load_solids(lin.with_name("assembly.step"))
    lower=next(p for k,p in parts.items() if "91251A351" in k and "[1.1.1.2.1]" in k)
    upper=next(p for k,p in parts.items() if "91251A354" in k and "[1.1.1.3.1]" in k)
    stock,_=exterior_shapes(lower,12.7,upper)
    saddle=stock["saddle"]
    if sheets or len(parts)!=552:raise ValueError("Wrong source issue")
    regions=[]
    for side,x,angle,group,stem,shift,s in (
        ("front",370.,-90.,1,13,(-12.7,-12.7),-127.3),("front",370.,-90.,4,36,(-12.7,12.7),127.3),
        ("rear",-370.,90.,6,13,(12.7,12.7),-127.3),("rear",-370.,90.,8,36,(12.7,-12.7),127.3)):
        key=next(k for k in parts if k.startswith("Motor, NEO [") and f"[1.{group}.1.{stem}.1]" in k)
        native=parts[key];motor=rotated(rotated(native,0,(shift[0]-x,shift[1],0)),-math.radians(angle),(-s,0,0))
        op=BRepAlgoAPI_Common(motor,saddle)
        if not op.IsDone():raise ValueError("Failed motor/ear intersection")
        common_shape=op.Shape();bb=bounds(common_shape);v=volume(common_shape)
        print(side,s,key,bb,v,flush=True)
        regions.append(dict(side=side,station=s,motor=key,localOverlapBoundsMm=bb,commonMm3=v))
    return dict(authority=auth,regions=regions)


if __name__=="__main__":
    result=run(Path(sys.argv[1]))
    (HERE/"inset-ear-witness.json").write_text(json.dumps(result,indent=2)+"\n")
