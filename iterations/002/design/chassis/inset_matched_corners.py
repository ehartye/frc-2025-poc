"""Register the matched commercial module directly on the four 2025 axes and test the actual stock."""
import hashlib,json,re,sys
from pathlib import Path
import numpy as np
from inset_stock_check import HERE,run as stock
from inset_electrical_package import box_corners,bb

def run(pkg):
    parts,report=stock(pkg,True)
    from showcase_contacts import load_solids,box_gap
    from mechanical_motion import bounds,common
    from component_shape import volume
    from fusion_authority import register_path
    from OCP.gp import gp_Trsf
    from OCP.TopLoc import TopLoc_Location
    root=HERE.parents[3];reg=register_path(root)
    d=HERE.parent/"private-inputs/matched-sds";sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    cold=json.loads((d/"cold.json").read_bytes())
    if not cold["archiveSha256"]==sha(d/"corner.f3d")=="3ca51360361f7d40586e0a53ee938ba47c4a09d22edb88974c0049ab7456f0e1" or not cold["stepSha256"]==sha(d/"corner.step")=="5deecd2778ac0da657198f7cd91a934f3c428acbfd2c2e635046cad8aef87ed8":raise ValueError("Unbound donor")
    from2025,sheets=load_solids(reg.parent/"issue-005/assembly.step");donor,dsheets=load_solids(d/"corner.step")
    if (len(from2025),len(donor),len(sheets),len(dsheets))!=(552,127,0,0):raise ValueError("Incomplete issue")
    old_corners={"FL":(1,1,1),"FR":(4,1,-1),"RR":(6,-1,-1),"RL":(8,-1,1)}
    prior_front=json.loads((HERE/"front-corner.json").read_bytes())["worldMatrixMm"]
    froot=Path.home()/"Documents/FRC Native/2025"
    first_right=json.loads((froot/"i002-p1/report.json").read_bytes())["placement"]["worldTransformCm"]
    later=json.loads((froot/"i002-l4/report.json").read_bytes())["stages"]
    def from_cm(flat):
        M=np.array(flat).reshape(4,4);M[:3,3]*=10;return M
    registered={"FL":np.array(prior_front),"FR":from_cm(first_right),"RR":from_cm(later[1]["outcome"]["worldTransformCm"]),"RL":from_cm(later[3]["outcome"]["worldTransformCm"])}
    ref=json.loads((HERE.parent/"private-inputs/first-native-result.json").read_bytes())["supplierLocalBodies"]
    poses={};outcomes={};roster={}
    for corner,(group,sx,sy) in old_corners.items():
        original_world=registered[corner];pose=original_world.copy();pose[:2,3]-=np.array([sx,sy])*12.7
        poses[corner]=pose.tolist();frame=gp_Trsf();frame.SetValues(*pose[:3].ravel())
        group_old={n:s for n,s in from2025.items() if re.search(r"\[1\."+str(group)+r"\.1\.",n)}
        old_bounds={n:bounds(s) for n,s in group_old.items()}
        if len(old_bounds)!=121:raise ValueError("Expected 121 installed leaves per prior station")
        # Bind the exact old station transform to all 121 original installed
        # STEP leaves; use a one-to-one spatial match to independently named native material.
        unused=set(group_old);witness=[]
        for local in ref:
            expected=bb([original_world[:3,:3]@p+original_world[:3,3] for p in box_corners(local["boundsMm"])])
            err,match=min((float(np.max(np.abs(np.array(old_bounds[k])-expected))),k) for k in unused)
            if err>.05:raise ValueError(("Wrong world frame",corner,local["path"],err))
            unused.remove(match);witness.append([local["path"],local["name"],match,err])
        projected={n:s.Moved(TopLoc_Location(frame)) for n,s in donor.items()}
        selected={n:bounds(s) for n,s in projected.items()}
        peer=[]
        for label,b in selected.items():
            row_min=sorted((box_gap(p["bounds"],b),name) for name,p in report["parts"].items())
            for g,name in row_min:
                if g>.005:break
                peer.append(dict(donor=label,stock=name,commonMm3=common(projected[label],parts[name])))
        # Native bound for every corrected leaf, tested against actual independently transferred STEP.
        unused=set(selected);matches=[]
        for local in cold["roster"]:
            expected=bb([pose[:3,:3]@p+pose[:3,3] for p in box_corners(local["boundsMm"])])
            err,match=min((float(np.max(np.abs(np.array(selected[k])-expected))),k) for k in unused)
            if err>.05:raise ValueError(("Wrong corrected shape or frame",corner,local["path"],err))
            unused.remove(match);matches.append([local["path"],local["body"],match,err])
        outcomes[corner]=dict(stockBodies=len(parts),parkedChecks=len(parts)*127,near=peer,
                      materialPassingFromPrior552=witness,correctedInstalledDerivation=matches)
        roster[corner]=[dict(path=r["path"],body=r["body"],volumeMm3=r["volumeMm3"],material=r["material"],
                             expectedBoundsMm=bb([pose[:3,:3]@p+pose[:3,3] for p in box_corners(r["boundsMm"])])) for r in cold["roster"]]
    result=dict(authority=report["authority"],shopSha256=report["shopPlanSHA256"],sourceFiles={p.name:sha(p) for p in d.iterdir() if p.is_file()},
                registeredSourceWorldMm={k:v.tolist() for k,v in registered.items()},newWorldMm=poses,
                corners=outcomes,scope="2025-only registered four-body transforms bound to all 484 original installed leaves of 552. Then 508 corrected source leaves compared on those independent inset transforms against the complete named shop roster. Parked geometry only. Measured loads, vendor grade, tread edition, screw thread engagement, fit and held-travel remain separate.")
    (HERE/"inset-matched-corners.json").write_text(json.dumps(result,indent=1)+"\n")
    (HERE/"inset-matched-native-plan.json").write_text(json.dumps(dict(authority=report["authority"],donorSHA256=cold["archiveSha256"],poses=poses,expected=roster),indent=1)+"\n")
    print([(c,v["parkedChecks"],[(p["donor"],p["stock"],p["commonMm3"]) for p in v["near"] if p["commonMm3"]>.01]) for c,v in outcomes.items()])
if __name__=="__main__":run(Path(sys.argv[1]))
