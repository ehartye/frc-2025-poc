"""Fit two mitered cassette members against the complete parked FL commercial corner."""
from pathlib import Path
import hashlib,json,sys
import numpy as np
from OCP.BRepPrimAPI import BRepPrimAPI_MakePrism
from OCP.gp import gp_Vec
from four_corner_packaging import rotated,polygon
from port_anchor_fit import cut

HERE=Path(__file__).resolve().parent


def shell(points,z,t):
    return BRepPrimAPI_MakePrism(polygon([[*p,z] for p in points]),gp_Vec(0,0,t)).Shape()


def run(pkg):
    sys.path.insert(0,str(pkg/"scripts"))
    from fusion_authority import register_path,verify
    from mechanical_motion import common,bounds
    from showcase_contacts import load_solids,box_gap
    from component_shape import volume
    root=HERE.parents[3];reg=register_path(root);step=reg.parent/"issue-005/assembly.step";auth=verify(root,reg,step.with_name("lineage.json"))
    study=HERE/"inset-corner-bypass.json";e=json.loads(study.read_bytes())
    if e["authority"]!=auth:raise ValueError("Changed mounted source")
    pts=np.array(e["outcomes"]["single-face cassette at 10000"]["kneePointsMm"][0])
    # Each open end extends ten mm beyond the outermost attachment center.
    (xc,yc)=pts[2];y0=pts[0,1]-10;x0=pts[4,0]-10
    wall=3.175;w=12.7;zb=43.83336287672979
    def member(role):
        def per(width,extension):
            if role=="front":return [(xc-width,y0-extension),(xc+width,y0-extension),(xc+width,yc+width),(xc-width,yc-width)]
            return [(x0-extension,yc-width),(xc-width,yc-width),(xc+width,yc+width),(x0-extension,yc+width)]
        return cut(shell(per(w,0),zb,50.8),shell(per(w-wall,1),zb+wall,50.8-2*wall))
    parts,sheets=load_solids(step);module={k:rotated(s,0,(-12.7,-12.7,0)) for k,s in parts.items() if "[1.1.1." in k}
    if len(module)!=121 or sheets:raise ValueError("Wrong module")
    bb={k:bounds(s) for k,s in module.items()}
    out={};actual={}
    for role in ("front","port"):
        item=member(role);actual[role]=item;b=bounds(item)
        candidates={k:common(item,s) for k,s in module.items() if box_gap(b,bb[k])<.005}
        out[role]=dict(boundsMm=b,actualStockKg=volume(item)*2.7e-6,bodyCheckCount=len(module),candidateCommonMm3=candidates)
        print(role,out[role]["actualStockKg"],[(n,round(v,3)) for n,v in candidates.items() if v>.005],flush=True)
    return dict(authority=auth,studySha256=hashlib.sha256(study.read_bytes()).hexdigest(),rows=out,kneeMiterCommonMm3=common(*actual.values()),
                scope="Before final connection holes, weld or internal corner gusset. The member ends have a true registered 45 degree miter and 10 mm end stock beyond each mounting center. Withdrawing the complete cassette and full azimuth motion remain separate.")


if __name__=="__main__":
    r=run(Path(sys.argv[1]));(HERE/"inset-cassette-fit.json").write_text(json.dumps(r,indent=2)+"\n")
