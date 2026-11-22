"""Recheck the frame/encoder candidates at matched finite translations and retain an end relief control."""
from pathlib import Path
import hashlib,json,sys
import numpy as np
from four_corner_packaging import rotated,outline
from structural_perimeter import tube_segment
from direct_saddle_packaging import hollow
from crossmember_envelope import box
from port_anchor_fit import cut

HERE=Path(__file__).resolve().parent


def run(pkg):
    sys.path.insert(0,str(pkg/"scripts"))
    from mechanical_motion import common,bounds
    from showcase_contacts import load_solids
    from fusion_authority import register_path,verify
    root=HERE.parents[3];reg=register_path(root);step=reg.parent/"issue-005/assembly.step";auth=verify(root,reg,step.with_name("lineage.json"))
    src=HERE/"inset-cassette-travel.json";r=json.loads(src.read_bytes());travel=r["physicalXYEnvelopeMm"]
    if r["authority"]!=auth:raise ValueError("Different issue")
    solids,sheets=load_solids(step);centers={1:(1,1),4:(1,-1),6:(-1,-1),8:(-1,1)}
    ao=outline(370,365,42,0);ai=outline(370,365,42,38.1)
    rings={j:tube_segment(ao,ai,j,38.1,3.175,101.6,101.6) for j in range(6)}
    tube=hollow((224.6,-242,161.925),(275.4,242,187.325),1)
    relief=tube
    for sy in (-1,1):
        la,lb=sorted((sy*231.5,sy*243.))
        relief=cut(relief,box((224.5,la,161.8),(275.5,lb,174.)))
    out={}
    def source(k,group):
        x,y=centers[group];return rotated(solids[k],0,(-12.7*x,-12.7*y,0))
    for row in r["unresolvedOtherBounds"]:
        group=row["corner"];part=source(row["row"],group)
        inward=-centers[group][1]
        out[row["row"]]=dict(corner=group,host="transverse x250",sourceBoundsMm=bounds(part),offsetMm=[0,inward*travel,0],
            nominalCommonMm3=common(tube,part),shiftedCommonMm3=common(tube,rotated(part,0,(0,inward*travel,0))),
            shiftedRelievedCommonMm3=common(relief,rotated(part,0,(0,inward*travel,0))))
    for key,rec in r["corners"].items():
        group=int(key)
        for row in rec["unresolvedFrameBounds"]:
            j=row["face"];p,q=np.array(ai[j]),np.array(ai[(j+1)%6]);t=q-p
            n=np.array([t[1],-t[0]])/np.linalg.norm(t)
            if n@p<0:n=-n
            part=source(row["id"],group);v=(*list(n*travel),0.)
            out[row["id"]+"@"+str(j)]=dict(corner=group,host="ring face "+str(j),sourceBoundsMm=bounds(part),offsetMm=v,
                nominalCommonMm3=common(part,rings[j]),shiftedCommonMm3=common(rotated(part,0,v),rings[j]))
    return dict(authority=auth,inputSha256=hashlib.sha256(src.read_bytes()).hexdigest(),rows=out,
                scope="A full 8.3 mm relative motion directed toward each local fixed member. These are admissible envelope controls, separate from synchronized displacements. The diagnostic crossmember end relief removes the lower 12.075 mm over the last 10.5 mm at each end with full top face preserved. Strength and manufacturing need independent review.")


if __name__=="__main__":
    r=run(Path(sys.argv[1]));(HERE/"inset-cassette-narrow.json").write_text(json.dumps(r,indent=2)+"\n")
    for host in sorted({x["host"] for x in r["rows"].values()}):
        rows=[x for x in r["rows"].values() if x["host"]==host];print(host,len(rows),max(x["shiftedCommonMm3"] for x in rows))
