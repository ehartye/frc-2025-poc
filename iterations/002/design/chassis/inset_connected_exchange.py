"""Compare the issued whole neutral exchange to all commercial descendants and all cut lists."""
import hashlib,json,re,sys
from pathlib import Path
import numpy as np
from inset_stock_check import HERE
from inset_v11_screw_tip import transform

def run(scripts):
    sys.path.insert(0,str(scripts))
    from showcase_contacts import load_solids
    from mechanical_motion import bounds
    from component_shape import volume
    from fusion_authority import register_path,verify
    reg=register_path(HERE.parents[3]);auth=verify(HERE.parents[3],reg,reg.parent/"issue-005/lineage.json")
    cold=json.loads((HERE/"connected-native-03-cold.json").read_bytes())
    donor=HERE.parent/"private-inputs/matched-sds/corner.step";full=donor.parent/"connected-654.step"
    plan=json.loads((HERE/"inset-matched-corners.json").read_bytes());sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    if cold["authority"]!=auth or sha(full)!=cold["stepSha256"] or sha(donor)!=plan["sourceFiles"]["corner.step"]:raise ValueError("Changed exchange or authority")
    items,other=load_solids(full);source,other_source=load_solids(donor)
    if len(items)!=654 or len(source)!=127 or other or other_source:raise ValueError("Wrong exchanged roster")
    stock=json.loads((HERE/"inset-stock-check.json").read_bytes())["parts"]
    common_name=lambda s:re.sub(r" \[[0-9.]+\]$","",s)
    reading={n:(np.array(bounds(q)),volume(q)) for n,q in items.items()}
    assigned=set();rows=[];held=[]
    def match(target,name,bb,vol):
        pp=[(np.max(np.abs(b-bb)),abs(v-vol),n) for n,(b,v) in reading.items() if common_name(n)==name]
        found=[(g,d,n) for g,d,n in pp if g<=.006 and d<=max(.01,abs(vol)*1e-6)]
        if len(found)!=1 or found[0][2] in assigned:
            held.append(dict(target=target,nearest=sorted(pp)[:3],matches=len(found)));return
        g,d,n=found[0];assigned.add(n);rows.append(dict(target=target,exchange=n,worldBoundsDeltaMm=g,deltaMm3=d))
    for n,data in stock.items():match(n,n,np.array(data["bounds"]),data["volumeMm3"])
    for corner,M in plan["newWorldMm"].items():
        for n,q in source.items():
            placed=transform(q,np.array(M));match(corner+"/"+n,common_name(n),np.array(bounds(placed)),volume(q))
    return dict(authority=auth,native=cold["nativeSha256"],export=sha(full),localSource=sha(donor),pairs=rows,held=held,
                unassigned=sorted(set(reading)-assigned),scope="One exact source name plus independently measured world AABB and material volume per matched exchanged solid. Source-bound geometry registration scope. Internal material assignments, occupied shape equivalence beyond these dimensions, loads, hardware service and readiness remain separate.")
if __name__=="__main__":
    r=run(Path(sys.argv[1]));(HERE/"connected-654-exchange.json").write_text(json.dumps(r,indent=1)+"\n")
    print(len(r["pairs"]),len(r["held"]),len(r["unassigned"]))
