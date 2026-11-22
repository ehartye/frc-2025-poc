"""Inspect the proposed V1.1 motor at the actual three mounting axes and all four chassis poses."""
from pathlib import Path
import hashlib, json, sys
import numpy as np
from inset_stock_check import HERE, run as stock
from OCP.gp import gp_Trsf
from OCP.TopLoc import TopLoc_Location


def run(plugin, inspector, alt):
    parts,base=stock(plugin,True)
    sys.path.insert(0,str(inspector))
    from fastener_stack import measure
    from mechanical_motion import bounds,common,distance,bearing_area
    from showcase_contacts import load_solids,box_gap
    d=HERE.parent/"private-inputs/matched-sds";key=json.loads((HERE/"inset-oem-steering-key.json").read_bytes())
    placements=json.loads((HERE/"inset-matched-corners.json").read_bytes());axes=json.loads((HERE/"inset-corner-local-fits.json").read_bytes())
    sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    if not key["authority"]==base["authority"]==axes["authority"] or key["motorSHA256"]!=sha(alt) or placements["sourceFiles"]["corner.step"]!=sha(d/"corner.step"):raise ValueError("Reference mismatch")
    source=next(iter(load_solids(alt)[0].values()));donor=load_solids(d/"corner.step")[0]
    def move(shape,M):
        t=gp_Trsf();t.SetValues(*M[:3].ravel());return shape.Moved(TopLoc_Location(t))
    motor=move(source,np.array(key["referenceShaftPoseMm"]));mb=bounds(motor);internal=[]
    for k,s in donor.items():
        if "Motor, NEO" in k:continue
        if box_gap(bounds(s),mb)>.3:continue
        try:
            v=[common(s,motor),common(motor,s)];dg=distance(s,motor)
            # Only articles on the broad external motor/concentrated screw interface carry this motor's mounting loads.
            area=bearing_area(s,motor,[0,-1,0]) if dg<.001 and (k.startswith("Spacer") or "Motor Mount" in k or "Plate" in k) else None
            internal.append(dict(name=k,gapMm=dg,sharesMm3=v,axialPlaneAreaMm2=area))
        except (ValueError,RuntimeError) as e:internal.append(dict(name=k,error=str(e)))
    rows=[]
    for corner,M in placements["newWorldMm"].items():
        M=np.array(M);installed=move(motor,M);bb=bounds(installed);world={k:move(v,M) for k,v in donor.items()};world["V1.1 motor"]=installed
        station=[]
        for a in axes["motorCases"]:
            if a["corner"]!=corner or ".36.1]" not in a["case"]["host"]:continue
            case={**a["case"],"host":"V1.1 motor"}
            try:v=measure(world,case,dict(underHeadLengthMm=[37.95,38.25],gripMm=[33.169,33.429],boreRadiusMm=[2.018,2.021]))
            except (ValueError,RuntimeError) as e:v=dict(error=str(e))
            station.append(dict(case=case,actual=v))
        if len(station)!=3:raise ValueError((corner,station))
        quick=sorted((box_gap(bb,v["bounds"]),k) for k,v in base["parts"].items());nearest=[]
        for lower,k in quick:
            if lower>15:break
            v=parts[k]
            nearest.append(dict(name=k,lowerMm=lower,gapMm=distance(v,installed),commonMm3=common(v,installed)))
        rows.append(dict(corner=corner,bodyBoundsMm=bb,threeMounts=station,nearbyStock=nearest))
    return dict(authority=base["authority"],stockPlanSHA256=base["shopPlanSHA256"],keyStudy=sha(HERE/"inset-oem-steering-key.json"),
                receipt=key["cold508"],alternateSHA256=sha(alt),internal=internal,installed=rows,
        scope="The complete alternate motor body follows the actual shaft end, keyseat clock and shaft center; the steering screw pilot depths act as independent casing datums. All held supplier skeletons and nearby new frame stock included. Hard internal overlaps or open hole datums prevent promotion. Loose production cables, commercial clamp tolerance and running alignment remain open.")
if __name__=="__main__":
    r=run(*map(Path,sys.argv[1:]));(HERE/"inset-v11-mounted-steering.json").write_text(json.dumps(r,indent=1)+"\n")
