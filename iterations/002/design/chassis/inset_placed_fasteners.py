"""Place the actual three purchased screw variants and distinguish the thread-stock band from other hard parts."""
from pathlib import Path
import json,sys
import numpy as np
from inset_stock_check import HERE,run as stock
from port_anchor_fit import oriented
from inset_electrical_package import gap


def run(pkg):
    parts,base=stock(pkg,True)
    from showcase_contacts import load_solids
    from mechanical_motion import bounds,common,distance
    from component_shape import volume
    from fusion_authority import register_path
    from OCP.TopAbs import TopAbs_FACE
    from OCP.TopExp import TopExp_Explorer
    from OCP.BRepAdaptor import BRepAdaptor_Surface
    from OCP.TopoDS import TopoDS
    reg=register_path(HERE.parents[3]);step=reg.parent/"issue-005/assembly.step";source,sheets=load_solids(step)
    registered=json.loads((HERE/"inset-mount-purchases.json").read_bytes());requests=json.loads((HERE/"inset-chassis-shop.json").read_bytes())["purchases"]
    if base["authority"]!=registered["authority"] or len(source)!=552 or sheets:raise ValueError("Unbound purchased reference")
    measured={};rows=[];file=HERE/"inset-placed-fasteners.json"
    for sku,o in registered["originals"].items():
        s=source[o["source"]];types={};it=TopExp_Explorer(s,TopAbs_FACE)
        while it.More():
            kind=str(BRepAdaptor_Surface(TopoDS.Face(it.Current())).GetType());types[kind]=types.get(kind,0)+1;it.Next()
        measured[sku]=dict(volumeMm3=volume(s),nominalSteelMassKg=volume(s)*7.86e-6,surfaces=types)
    for idx,r in enumerate(requests):
        sku=r["sku"];donor=registered["originals"][sku];axis=np.array(r["direction"]);seat=np.array(r["underHead"])
        s=oriented(source[donor["source"]],donor["underHeadCenterMm"],donor["intoThreadUnit"],seat,axis)
        b=bounds(s);z=int(np.argmax(abs(axis)));projected=sorted((np.sign(axis[z])*(b[z]-seat[z]),np.sign(axis[z])*(b[z+3]-seat[z])))
        length={"91251A539":15.875,"91251A354":57.15,"91251A351":38.1}[sku]
        if abs(projected[1]-length)>.005:raise ValueError(("Unseated original part",r["name"],projected,length))
        print(idx + 1, r["name"], flush=True)
        pairs=[]
        for name,a in parts.items():
            if gap(b,base["parts"][name]["bounds"])>.01:continue
            try:
                vol=common(s,a)
            except (ValueError,RuntimeError) as exc:
                pairs.append(dict(stock=name,commonMm3=None,commissionedTap=name==r["host"],geometryError=str(exc)))
                continue
            if vol>.002 or name==r["host"]:
                pairs.append(dict(stock=name,commonMm3=vol,commissionedTap=name==r["host"]))
        rows.append(dict(name=r["name"],sku=sku,seatMm=seat.tolist(),axis=axis.tolist(),actualUnderheadToTipMm=projected[1],bounds=b,host=r["host"],pairs=pairs))
        if rows:
            by={k:sum(measured[v["sku"]]["nominalSteelMassKg"] for v in rows if v["sku"]==k) for k in sorted({v["sku"] for v in rows})}
            file.write_text(json.dumps(dict(authority=base["authority"],shopPlan=base["shopPlanSHA256"],targetCount=len(requests),nativeSource=registered["originals"],sourceGeometry=measured,
                              completed=len(rows),steelMassKg=by,rows=rows,scope="Actual 552-origin stock screw shapes at the commissioned head/axis on the current machined parts. Raw common with a shop tap-drill is not a thread or torque pass. Positive material with any other part is a hold."),indent=1)+"\n")
    print(by);print([(r["name"],p["stock"],p["commonMm3"]) for r in rows for p in r["pairs"] if not p["commissionedTap"]])
if __name__=="__main__":run(Path(sys.argv[1]))
