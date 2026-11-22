"""Measure removable contact lands on existing OEM upper and lower plate edges from the registered cut."""
import hashlib,json,sys
from pathlib import Path
import numpy as np
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from crossmember_envelope import box,cylinder
from four_corner_packaging import rotated
from port_anchor_fit import cut

HERE=Path(__file__).resolve().parent


def run(pkg):
    sys.path.insert(0,str(pkg/"scripts"))
    from fusion_authority import register_path,verify
    from mechanical_motion import common,bounds
    from showcase_contacts import load_solids,box_gap
    root=HERE.parents[3];reg=register_path(root);line=reg.parent/"issue-005/lineage.json"
    auth=verify(root,reg,line);parts,sheets=load_solids(line.with_name("assembly.step"))
    refs=HERE/"inset-factory-edge-section.json";sec=json.loads(refs.read_bytes())
    if len(parts)!=552 or sheets or sec["authority"]!=auth:raise ValueError("Wrong canonical assembly")
    scene={n:rotated(s,0,(-12.7,-12.7,0)) for n,s in parts.items() if "[1.1.1." in n}
    ext={n:bounds(s) for n,s in scene.items()};out=[]
    top,= [k for k in sec["sections"] if ".17]" in k]
    lo,=[k for k in sec["sections"] if ".16]" in k]
    scoop,=[r for r in sec["sections"][top] if 30<r.get("r",0)<32]
    cx,cy=scoop["center"][:2];radius=scoop["r"]
    def inspect(name,land,article,force_dir,center=None):
        b=bounds(land);ne={n:common(land,scene[n]) for n in scene if box_gap(b,ext[n])<.001}
        pairs={}
        for disp in (-.25,-.1,.025,.05):
            v=rotated(land,0,tuple(np.array(force_dir)*disp))
            pairs[str(disp)]=dict(workingFaceMm3=common(v,scene[article]))
        row=dict(name=name,sourcePlate=article,machinedBoundsMm=b,pressureDirection=force_dir,
                 neighborsMm3=ne,displacementMm=pairs,sourceArc=center,projectedBearingAreaMm2=(pairs["0.025"]["workingFaceMm3"]-ne.get(article,0))/.025)
        out.append(row);print(name,round(row["projectedBearingAreaMm2"],2),[(n,round(v,2)) for n,v in ne.items() if v>.005],flush=True)
    # A machineable cylindrical OD follows the purchased plate's concave edge.
    # End faces split the independent contact intervals and expose any occupied zone.
    z0,z1=95.04,98.97
    ann=cut(cylinder((cx,cy,z0),(0,0,1),radius,z1-z0),
            cylinder((cx,cy,z0-.1),(0,0,1),radius-3.175,z1-z0+.2))
    for left,right in [(177.3,185.3),(185.3,193.3),(193.3,201.3),(201.3,208.0)]:
        op=BRepAlgoAPI_Common(ann,box((left,280,z0-.1),(right,310,z1+.1)))
        if not op.IsDone():raise ValueError("Arc clip failed")
        inspect(f"upper scallop {left}",op.Shape(),top,[0,1,0],[cx,cy,radius])
    for tag,plate,z0,z1 in (("upper",top,95.04,98.97),("lower",lo,36.31,43.40)):
        inspect(tag+" tangential end",box((162.9375,315.15,z0),(166.1125,321.4,z1)),plate,[1,0,0])
        # The two genuine rounded nose corners can be cut with a common end-mill path.
        for cy0,sign in ((315.1,-1),(321.45,1)):
            cx0=175.6375;r=9.525
            ring=cut(cylinder((cx0,cy0,z0),(0,0,1),r+3.175,z1-z0),cylinder((cx0,cy0,z0-.1),(0,0,1),r,z1-z0+.2))
            y0,y1=sorted((cy0,cy0+13*sign))
            op=BRepAlgoAPI_Common(ring,box((cx0-13,y0,z0-.1),(cx0,y1,z1+.1)))
            if not op.IsDone():raise ValueError("Nose clip failed")
            inspect(tag+(" inner" if sign<0 else " outer")+" nose",op.Shape(),plate,[0,-sign,0],[cx0,cy0,r])
    return dict(authority=auth,registeredSectionSha256=hashlib.sha256(refs.read_bytes()).hexdigest(),moduleBodies=len(scene),rows=out,
                scope="Unconnected 3.175 mm machined lands on source-identified OEM edges at one port socket. Pressure direction defines the signed relative movement. Finite displaced common divided by displacement estimates projected bearing area. Mounting body, opposing load sharing, axial installation/removal, calibration clearance and tolerances require their own checks.")


if __name__=="__main__":
    result=run(Path(sys.argv[1]));(HERE/"inset-plate-nose.json").write_text(json.dumps(result,indent=2)+"\n")
