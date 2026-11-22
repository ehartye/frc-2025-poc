"""Place the proposed wider structural perimeter over the four original wheel seats and their service tools."""
import argparse
import json
import math
from pathlib import Path
import re
import sys

from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
from OCP.BRepPrimAPI import BRepPrimAPI_MakePrism
from OCP.gp import gp_Vec
from four_corner_packaging import polygon, outline
from crossmember_envelope import cylinder

HERE=Path(__file__).resolve().parent


def tube_segment(outer,inner,index,width,wall,z,height):
    j=(index+1)%6
    def prism(out,ins,bottom,h):
        verts=[out[index],out[j],ins[j],ins[index]]
        return BRepPrimAPI_MakePrism(polygon([[*v,bottom] for v in verts]),gp_Vec(0,0,h)).Shape()
    outer_solid=prism(outer,inner,z,height)
    # Extend the internal void beyond both miter intersections to keep the stock ends open.
    import numpy as np
    face_inner=outline(370.,365.,42.,wall)
    face_outer=outline(370.,365.,42.,width-wall)
    u=np.array(outer[j])-outer[index]
    u=u/np.linalg.norm(u)
    for pts in (face_inner,face_outer):
        pts[index]=(np.array(pts[index])-u*20).tolist()
        pts[j]=(np.array(pts[j])+u*20).tolist()
    void=prism(face_inner,face_outer,z+wall,height-2*wall)
    op=BRepAlgoAPI_Cut(outer_solid,void)
    if not op.IsDone():raise ValueError("Open miter operation failed")
    return op.Shape()


def relocate_lower(installed,rail_definitions,inset):
    from mechanical_motion import bounds
    if inset:
        import numpy as np
        from crossmember_envelope import box
        from port_anchor_fit import cut
        from four_corner_packaging import rotated
        corners={1:(1,1),4:(1,-1),6:(-1,-1),8:(-1,1)}
        moved={}
        for name,part in installed.items():
            g=int(re.search(r"\[1\.(\d+)\.",name)[1])
            if g in corners:
                sx,sy=corners[g]
                moved[name]=rotated(part,0,(-sx*inset,-sy*inset,0))
            else:
                bb=bounds(part)
                rail=next(r for r in rail_definitions.values() if all(abs(a-b)<.001 for a,b in zip(bb,r["minimumMm"]+r["maximumMm"])))
                ax=rail["lengthAxis"]
                lo=np.array(rail["minimumMm"]);hi=np.array(rail["maximumMm"])
                lo[ax]+=inset;hi[ax]-=inset;sgn=np.sign(lo[1-ax]);lo[1-ax]-=inset*sgn;hi[1-ax]-=inset*sgn
                inside_lo=lo+3.175;inside_hi=hi-3.175;inside_lo[ax]=lo[ax]-1;inside_hi[ax]=hi[ax]+1
                body=cut(box(lo.tolist(),hi.tolist()),box(inside_lo.tolist(),inside_hi.tolist()))
                for x,y in rail["boltCentersMm"]:
                    body=cut(body,cylinder((x-np.sign(x)*inset,y-np.sign(y)*inset,lo[2]-1),(0,0,1),2.65,hi[2]-lo[2]+2))
                moved[name]=body
        installed=moved
    return installed


def run(package,z,width,height,inset=0.):
    sys.path.insert(0,str(package/"scripts"))
    from fusion_authority import verify,register_path
    from mechanical_motion import bounds,common,distance
    from showcase_contacts import load_solids,box_gap
    from component_shape import volume
    root=HERE.parents[3]
    reg=register_path(root)
    lin=reg.parent/"issue-005/lineage.json"
    auth=verify(root,reg,lin)
    all_bodies,sheets=load_solids(lin.with_name("assembly.step"))
    if sheets or len(all_bodies)!=552:raise ValueError("Wrong source lower")
    installed={k:p for k,p in all_bodies.items() if int(re.search(r"\[1\.(\d+)\.",k)[1])<=8}
    if len(installed)!=488:raise ValueError("Missing module or source socket")
    rail_definitions=json.loads((HERE/"socket-holes.json").read_bytes())["rails"]
    installed=relocate_lower(installed,rail_definitions,inset)
    objects=dict(installed)
    for definition in json.loads((HERE/"socket-holes.json").read_bytes())["rails"].values():
        for x,y in definition["boltCentersMm"]:
            # Vertical extraction of each 2.5 inch cap screw, including the 5/32 in hex drive.
            x-=inset*math.copysign(1,x);y-=inset*math.copysign(1,y)
            objects[f"Module top driver and screw axis {x},{y}"]=cylinder((x,y,94.64),(0,0,1),4.15,82.)
    bb={k:bounds(p) for k,p in objects.items()}
    outer=outline(370,365,42,0)
    inner=outline(370,365,42,width)
    members=[tube_segment(outer,inner,i,width,3.175,z,height) for i in range(6)]
    outcome=[]
    for i,body in enumerate(members):
        b=bounds(body)
        row=dict(segment=i,kg=volume(body)*2700e-9,interferences=[],tools=[],near=[])
        for name,other in objects.items():
            if box_gap(b,bb[name])>1:
                continue
            d=distance(body,other)
            if d>1:continue
            v=common(body,other) if d<.0001 else 0.
            record=dict(id=name,gapMm=d,overlapMm3=v)
            if v>.005:
                row["tools" if name.startswith("Module top driver") else "interferences"].append(record)
            else:row["near"].append(record)
        outcome.append(row)
    return dict(authority=auth,cornerInsetXYmm=inset,newKingpinAbsXYmm=[282-inset,277-inset],includedRetained=488,retiredCandidateTerminalBodies=64,bottomMm=z,
        radialWidthMm=width,heightMm=height,outerXY=outer,stockWallMm=3.175,segments=outcome,
        grossMassKg=sum(s["kg"] for s in outcome),
        scope="Candidate envelope on true issue-005 source, all 484 parked module articles and four actual lower rails. Two rear chamfers remain 42 mm. Optional removal of the trial structural perimeter before unscrewing modules must be physically demonstrated. Four prior terminal cartridges are separately retained in the unchanged canonical source, omitted only from the candidate. Original factory internal fit and full steering/lead service remain open. No holes or attachment mass credited.")


if __name__=="__main__":
    p=argparse.ArgumentParser();p.add_argument("package",type=Path);p.add_argument("z",type=float);p.add_argument("width",type=float);p.add_argument("height",type=float);p.add_argument("--inset",type=float,default=0.);a=p.parse_args()
    r=run(a.package,a.z,a.width,a.height,a.inset)
    (HERE/("structural-perimeter-"+f"{a.z:g}-{a.width:g}-{a.height:g}"+(f"-inset-{a.inset:g}" if a.inset else "")+".json")).write_text(json.dumps(r,indent=2)+"\n")
    for seg in r["segments"]:print(seg["segment"],len(seg["interferences"]),len(seg["tools"]),[(k["id"],round(k["overlapMm3"],1)) for k in seg["interferences"]][:3])
    print(r["grossMassKg"])
