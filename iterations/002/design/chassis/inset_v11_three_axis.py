"""Independently check the alternate motor casing, three tapped axes and nearest external stock."""
import hashlib,json,sys
from pathlib import Path
import numpy as np
from inset_stock_check import HERE,form
from OCP.gp import gp_Trsf
from OCP.TopLoc import TopLoc_Location

def run(plugin,alt):
    sys.path.insert(0,str(plugin))
    from fastener_stack import measure
    from showcase_contacts import load_solids,box_gap
    from mechanical_motion import bounds,distance
    from fusion_authority import register_path,verify
    reg=register_path(HERE.parents[3]);auth=verify(HERE.parents[3],reg,reg.parent/"issue-005/lineage.json")
    ref=json.loads((HERE/"inset-oem-steering-key.json").read_bytes())
    if ref["authority"]!=auth or hashlib.sha256(alt.read_bytes()).hexdigest()!=ref["motorSHA256"]:raise ValueError("Unbound alternate")
    src=HERE.parent/"private-inputs/matched-sds/corner.step"
    if hashlib.sha256(src.read_bytes()).hexdigest()!=ref["donorStep"]:raise ValueError("Wrong corrected package")
    parts,_=load_solids(src);motor=next(iter(load_solids(alt)[0].values()));axes=json.loads((HERE/"inset-corner-local-fits.json").read_bytes());locations=json.loads((HERE/"inset-matched-corners.json").read_bytes())["newWorldMm"]
    old=json.loads((HERE/"inset-stock-check.json").read_bytes())["parts"];stock={a["name"]:a for a in json.loads((HERE/"inset-chassis-shop.json").read_bytes())["articles"]}
    def move(body,M):
        t=gp_Trsf();t.SetValues(*np.array(M)[:3].ravel());return body.Moved(TopLoc_Location(t))
    rows=[]
    for corner,W in locations.items():
        W=np.array(W);m=move(motor,W@np.array(ref["referenceShaftPoseMm"]));mb=bounds(m);nearby=[]
        near=sorted((box_gap(mb,row["bounds"]),n) for n,row in old.items())
        for g,name in near:
            if g>10:break
            q=form(stock[name])
            if "matrixMm" in stock[name]:q=move(q,stock[name]["matrixMm"])
            nearby.append(dict(name=name,boxGapMm=g,exactGapMm=distance(q,m)))
        motors=[]
        for r in axes["motorCases"]:
            if r["corner"]!=corner or ".36.1]" not in r["case"]["host"]:continue
            c={**r["case"],"host":"Exact V1.1"};s={a:move(parts[a],W) for a in (c["seat"],c["screw"])};s[c["host"]]=m
            try:result=measure(s,c,dict(underHeadLengthMm=[37.95,38.25],gripMm=[33.169,33.429],boreRadiusMm=[2.018,2.021]))
            except (RuntimeError,ValueError) as exc:result=dict(error=str(exc))
            motors.append(dict(case=c,result=result))
        rows.append(dict(corner=corner,threeMounts=motors,nearby=nearby,smallestBroadBoundMm=near[0][0]))
    return dict(authority=auth,reference=ref["cold508"],sourceAlternate=ref["motorSHA256"],rows=rows,scope="Three source screw axes, shaft end and keyseat register; exact external clearances from nearby edited frame stock, nearest bound for the rest. Distinguish matching motor mount pilot geometry, represented threads and torque capacity. No claim about leads or first-article real fit.")
if __name__=="__main__":
    r=run(*map(Path,sys.argv[1:]));(HERE/"inset-v11-three-axis.json").write_text(json.dumps(r,indent=1)+"\n")
