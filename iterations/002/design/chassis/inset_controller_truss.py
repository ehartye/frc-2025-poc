"""Use exact controller and carrier geometry for a removable electronics bank below the pan truss."""
from pathlib import Path
import json,sys
import numpy as np
from inset_electrical_package import HERE,CAD,Y_UP,box_corners,bb,gap,sha
from inset_pan_truss import path_gap

def run(pkg):
    sys.path.insert(0,str(pkg/"scripts"))
    from fusion_authority import register_path,verify
    from showcase_contacts import load_solids
    from mechanical_motion import bounds
    root=HERE.parents[3];reg=register_path(root);auth=verify(root,reg,reg.parent/"issue-005/lineage.json")
    truss=HERE/"inset-pan-truss.json";t=json.loads(truss.read_bytes())
    previous=HERE/"inset-electrical-package.json";p=json.loads(previous.read_bytes())
    if not auth==t["authority"]==p["authority"]:raise ValueError("Unbound source")
    files={role:CAD/n for role,n in (("controller","REV-11-2158.STEP"),("bracket","REV-21-2151.STEP"))}
    parts={}
    for role,file in files.items():
        shapes,sheets=load_solids(file)
        if sheets:raise ValueError("Unexpected open sheet")
        parts[role]={k:bounds(v) for k,v in shapes.items()}
    if len(parts["controller"])!=18 or len(parts["bracket"])!=1:raise ValueError("Changed purchase")
    bars=[(np.array(r["start"]),np.array(r["end"])) for r in t["stock"]]
    records={}
    for xpos in (194.,178.,170.):
        rows=[];lim=[]
        for name,old in p["candidatePlacements"].items():
            if old["source"]!="SPARK MAX":continue
            center=(np.array(old["boundsMm"][:2])+old["boundsMm"][3:5])/2
            if center[0]>100:center[0]=xpos
            origin=np.r_[center,203.2+2.032+.5]
            transforms={};
            for role,rotation,offset in (("bracket",Y_UP,origin),("controller",Y_UP@np.diag([-1,-1,1]),origin+Y_UP@[0,22.45,0])):
                transforms[role]=np.column_stack((rotation,offset)).tolist()
                for key,source in parts[role].items():
                    box=np.array(bb([rotation@v+offset for v in box_corners(source)]))
                    travel_y=(np.sign(center[1])*(305.-abs(center[1]) if abs(center[1])>150 else 54.-abs(center[1]))) if center[0]>100 else 0.
                    horizontal=np.array([0.,travel_y,4.])
                    segments=[("park",np.zeros(3),np.zeros(3)),("straight35",np.zeros(3),np.array([0.,0.,35.])),
                              ("raise4",np.zeros(3),np.array([0.,0.,4.])),("translate",np.array([0.,0.,4.]),horizontal),
                              ("withdraw",horizontal,horizontal+[0.,0.,323.])]
                    for phase,start,end in segments:
                        swept=box+np.r_[np.minimum(start,end),np.maximum(start,end)]
                        # Same bolted chassis posture. Lifting the unpowered controller+carrier as a unit
                        # avoids crediting a path through resilient clips. The screw is removed first.
                        distance,i=min((path_gap(a,b,swept,19.05/np.sqrt(2)),j) for j,(a,b) in enumerate(bars))
                        rows.append(dict(name=name,role=role,body=key,phase=phase,bar=i,gap=distance))
            lim.append(dict(name=name,worldMatricesMm=transforms))
        records[str(xpos)]=dict(minimum={phase:min((v for v in rows if v["phase"]==phase),key=lambda r:r["gap"]) for phase in ("park","straight35","raise4","translate","withdraw")},instances=lim)
    return dict(authority=auth,inputHashes={f.name:sha(f) for f in [previous,truss,*files.values()]},records=records,
                scope="All nine vendor controller/carrier assemblies; positive gaps cover the complete purchased body via its true world bounds and the square strut via an enclosing cylinder. Disconnected and unbolted front controllers rise 4 mm, translate in Y into a clear lane at absolute Y54/305 and rise 323 mm. Rear controllers lift in place. The 35 mm direct lift is retained. Other neighbors, screw removal, wire bend, upper assembly and loaded motion require separate checks.")
if __name__=="__main__":
    r=run(Path(sys.argv[1]));(HERE/"inset-controller-truss.json").write_text(json.dumps(r,indent=2)+"\n")
    for n,v in r["records"].items():print(n,v["minimum"])
