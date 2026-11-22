"""Fit finite diagonal rails at the commercial corner and expose the actual conflicting components."""
import json,sys,re
from pathlib import Path
import numpy as np
from OCP.BRepPrimAPI import BRepPrimAPI_MakePrism
from OCP.gp import gp_Vec
from four_corner_packaging import polygon,rotated
from port_anchor_fit import cut
from crossmember_envelope import box

HERE=Path(__file__).resolve().parent


def tube(p,q,z):
    p,q=np.array(p),np.array(q)
    along=(q-p)/np.linalg.norm(q-p); normal=np.array([-along[1],along[0]])
    def billet(w,h,over):
        points=[[*(p-over*along+normal*w),z-h],[*(p-over*along-normal*w),z-h],
                [*(p-over*along-normal*w),z+h],[*(p-over*along+normal*w),z+h]]
        return BRepPrimAPI_MakePrism(polygon(points),gp_Vec(*((q-p)+2*over*along),0)).Shape()
    return cut(billet(12.7,25.4,0),billet(12.7-3.175,25.4-3.175,1.))


def run(pkg):
    sys.path.insert(0,str(pkg/"scripts"))
    from fusion_authority import register_path,verify
    from mechanical_motion import common,bounds
    from showcase_contacts import load_solids,box_gap
    root=HERE.parents[3];register=register_path(root);line=register.parent/"issue-005/lineage.json"
    authority=verify(root,register,line)
    layout=json.loads((HERE/"inset-electrical-package.json").read_bytes())
    study=json.loads((HERE/"inset-corner-bypass.json").read_bytes())
    if not authority==layout["authority"]==study["authority"]:raise ValueError("Inconsistent issue")
    solids,sheets=load_solids(line.with_name("assembly.step"))
    module={k:rotated(v,0,(-12.7,-12.7,0)) for k,v in solids.items() if "[1.1.1." in k}
    if sheets or len(solids)!=552 or len(module)!=121:raise ValueError("Full corner missing")
    known={k:(s,bounds(s)) for k,s in module.items()}
    for name,part in layout["candidatePlacements"].items():
        b=part["boundsMm"];known["planning envelope "+name]=(box(b[:3],b[3:]),b)
    ties=study["outcomes"]["two tubes between the lower sockets"]["ties"][:2]
    rows=[]
    for z in (69.2333628767,161.925,229.):
        for tie in ties:
            beam=tube(tie["startMm"],tie["endMm"],z);bound=bounds(beam)
            positives={};open_pairs={}
            for name,(s,b) in known.items():
                if box_gap(bound,b)>.00001:continue
                try:
                    v=common(beam,s)
                    if v>.005:positives[name]=v
                except (ValueError,RuntimeError) as err:open_pairs[name]=str(err)
            rows.append(dict(zMm=z,rank=tie["rank"],materialCommonMm3=positives,unresolved=open_pairs))
            print(tie["rank"],z,len(positives),sorted(positives.items(),key=lambda e:-e[1])[:3],flush=True)
    return dict(authority=authority,sourceCount=len(module),rows=rows,scope="Actual complete FL supplier article bounds/solid intersections at parked azimuth following the commissioned 12.7 mm inset. Electronics are conservative placement envelopes. The socket attachment overlaps are intentionally not qualified by this fit probe. An occupied tie needs a different physical route.")


if __name__=="__main__":
    result=run(Path(sys.argv[1]));(HERE/"inset-corner-tie-fit.json").write_text(json.dumps(result,indent=2)+"\n")
