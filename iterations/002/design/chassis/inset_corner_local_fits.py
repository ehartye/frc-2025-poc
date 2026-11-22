"""Replay the corrected blind-hole and axle geometry on each actual 2025 placement."""
from pathlib import Path
import hashlib,json,sys
import numpy as np
from inset_chassis_shop import HERE


def run(scripts,stations):
    sys.path.insert(0,str(scripts))
    from fusion_authority import register_path,verify
    from fastener_stack import measure
    from mechanical_motion import common,distance,bounds
    from showcase_contacts import load_solids
    from OCP.gp import gp_Trsf
    from OCP.TopLoc import TopLoc_Location
    root=HERE.parents[3];reg=register_path(root);authority=verify(root,reg,reg.parent/"issue-005/lineage.json")
    d=HERE.parent/"private-inputs/matched-sds";source=json.loads((d/"cold.json").read_bytes());step=d/"corner.step"
    p=HERE/"inset-matched-corners.json";xy=json.loads(p.read_bytes());parent=json.loads(stations.read_bytes())
    if xy["authority"]!={k:v for k,v in authority.items() if k in xy["authority"]} or hashlib.sha256(step.read_bytes()).hexdigest()!=source["stepSha256"]:raise ValueError("Source mismatch")
    allparts,sheets=load_solids(step);local={n:bounds(s) for n,s in allparts.items()}
    if sheets or len(allparts)!=127:raise ValueError("Wrong corrected donor")
    cases=[];wheels=[]
    for name,M in xy["newWorldMm"].items():
        M=np.array(M);t=gp_Trsf();t.SetValues(*M[:3].ravel())
        live={k:v.Moved(TopLoc_Location(t)) for k,v in allparts.items()}
        for station in parent["stations"]:
            x,z=station["axisXZmm"];y=station["bearingFaceYmm"]+3.2
            near=[n for n,b in local.items() if n.startswith("Screw, 10-32 X 1.5,") and abs((b[0]+b[3])/2-x)<.005 and abs((b[2]+b[5])/2-z)<.005]
            motor=[n for n,b in local.items() if n.startswith("Motor, NEO") and b[0]<x<b[3] and b[2]<z<b[5]]
            if len(near)!=1 or len(motor)!=1:raise ValueError("No unique installed motor or screw")
            q=dict(kind="blind",screw=near[0],host=motor[0],seat=station["label"],underHeadCenterMm=(M[:3,:3]@[x,y,z]+M[:3,3]).tolist(),axisIntoHost=(M[:3,:3]@[0,-1,0]).tolist())
            outcome=measure(live,q,dict(underHeadLengthMm=[37.95,38.25],gripMm=[33.169,33.429],boreRadiusMm=[2.018,2.021]))
            if outcome.get("tipToDrillPointMm",0)<1.90:raise ValueError(("Bottoming",name,q,outcome))
            cases.append(dict(corner=name,case=q,result=outcome))
        take=lambda s:next(n for n in live if n.startswith(s))
        bearing=take("Bearing, R6ZZ");wheel=take("Wheel, SDS, Nitrile");block=take("Project WMA-01");sleeve=take("Project WS-01")
        for peer in (wheel,block,sleeve):
            v=common(live[bearing],live[peer]);w=common(live[peer],live[bearing]);gap=distance(live[bearing],live[peer])
            if v>.002 or w>.002 or gap>.001:raise ValueError(("Axle receiving seat",name,peer,v,w,gap))
            wheels.append(dict(corner=name,bearing=bearing,other=peer,gapMm=gap,commonMm3=[v,w]))
    return dict(authority=authority,correctedDonorSHA256=source["stepSha256"],placementSHA256=hashlib.sha256(p.read_bytes()).hexdigest(),
                fastenerInspectorSHA256=hashlib.sha256((scripts/"fastener_stack.py").read_bytes()).hexdigest(),stationPlanSHA256=hashlib.sha256(stations.read_bytes()).hexdigest(),
                motorCases=cases,wheelSeats=wheels,scope="Independent four-station placement of real corrected BReps on 2025 datum before target native cold issuance. Geometrically seated contacts and blind depth. Thread/runout/clamp specification, shield/endplay, concentration and supplier mechanical allowances remain open.")
if __name__=="__main__":
    q=run(Path(sys.argv[1]),Path(sys.argv[2]));(HERE/"inset-corner-local-fits.json").write_text(json.dumps(q,indent=1)+"\n");print(len(q["motorCases"]),len(q["wheelSeats"]))
