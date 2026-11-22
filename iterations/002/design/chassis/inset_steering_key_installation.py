"""Measure both actual SDS motor keys at four native-bound poses, including edition-specific shaft controls."""
import hashlib, json, sys
from pathlib import Path
import numpy as np
from inset_chassis_shop import HERE

def run(pkg, station, reference, alternate):
    sys.path.insert(0,str(pkg/"scripts"))
    from fusion_authority import register_path, verify
    from showcase_contacts import load_solids
    from mechanical_motion import bounds, common
    from OCP.gp import gp_Trsf
    from OCP.TopLoc import TopLoc_Location
    sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    root=HERE.parents[3]; reg=register_path(root);authority=verify(root,reg,reg.parent/"issue-005/lineage.json")
    report=json.loads((station/"native-receipt.json").read_bytes());plan=json.loads((HERE/"inset-matched-native-plan.json").read_bytes())
    p=HERE.parent/"private-inputs/matched-sds";warm=json.loads((station/"source-127-readback.json").read_bytes())
    if report["authority"]!=authority or sha(station/"RL-matched.f3d")!=report["checks"][-1]["successorSha256"] or sha(p/"corner.step")!="5deecd2778ac0da657198f7cd91a934f3c428acbfd2c2e635046cad8aef87ed8":raise ValueError("Source issue mismatch")
    donor,sheets=load_solids(p/"corner.step");native={r["path"].split("+")[-1]:r for r in warm if any(k in r["path"].split("+")[-1] for k in ("Motor, NEO:","Machine Key,","Gear, 20DP, 14T"))}
    def unique(prefix,occ):
        record=next(v for k,v in native.items() if k.startswith(prefix) and k.endswith(":"+str(occ)))
        ids=[k for k,s in donor.items() if k.startswith(prefix) and max(abs(np.array(bounds(s))-record["boundsMm"]))<.003]
        if len(ids)!=1:raise ValueError((record["path"],ids))
        return ids[0]
    def move(s,M):
        t=gp_Trsf();t.SetValues(*np.array(M)[:3].ravel());return s.Moved(TopLoc_Location(t))
    def pair(a,b):return [common(a,b),common(b,a)]
    def ax(d):M=np.eye(4);M[1,3]=d;return M
    ref=json.loads(reference.read_bytes());neo_assets,_=load_solids(alternate)
    if len(neo_assets)!=1 or sha(alternate)!=ref["reference"]["REV-21-1650-V1.1.step"]["sha256"]:raise ValueError("Unexpected standalone motor")
    c=-ref["keyFloorDirection"][2];s=ref["keyFloorDirection"][0];T=np.eye(4);T[:3,:3]=[[c,s,0],[0,0,1],[s,-c,0]]
    T[:3,3]=ref["translationMm"]
    rows=[]
    for corner,M in plan["poses"].items():
        M=np.array(M)
        if report["508PlacedMaterials"][corner]["count"]!=127 or report["508PlacedMaterials"][corner]["maxWorldBoundsErrorMm"]>.003:raise ValueError("Missing native corner")
        for role,kidx,midx,gidx in (("steering",2,2,1),("drive",1,1,2)):
            key,motor,gear=unique("Machine Key,",kidx),unique("Motor, NEO",midx),unique("Gear, 20DP, 14T",gidx)
            mm=move(donor[motor],M);gg=move(donor[gear],M)
            alt=move(next(iter(neo_assets.values())),M@T) if role=="steering" else None
            samples=[]
            for delta in ((0.,2.15,5.65) if alt is not None else (0.,)):
                kk=move(donor[key],M@ax(delta));yb=bounds(donor[key]);gb=bounds(donor[gear])
                hit=pair(kk,mm);alt_hit=pair(kk,alt) if alt is not None else None
                samples.append(dict(liftAlongLocalPositiveYmm=delta,donorMotorCommonMm3=hit,donorPinionCommonMm3=pair(kk,gg),
                    exactV11ShaftDiagnosticCommonMm3=alt_hit,envelopeEngagementUpperMm=max(0,min(yb[4]+delta,gb[4])-max(yb[1]+delta,gb[1]))))
            rows.append(dict(corner=corner,role=role,key=key,motor=motor,gear=gear,shaftWorldPositiveUnit=M[:3,1].tolist(),samples=samples))
    return dict(authority=authority,cold508=sha(station/"RL-matched.f3d"),cornerStep=sha(p/"corner.step"),
        sourceNativeReceipt=sha(station/"native-receipt.json"),standaloneV11=sha(alternate),alternateDatumStudy=sha(reference),
        rows=rows,scope="World-pose reconstruction from the actual source native and 508 cold full-leaf placement checks. Standalone alternate motor registers the shaft end, keyseat radial plane and axis at the same purchased datum, without presuming housing installation. Pinion axial overlap is a geometric ceiling. Retention, bilateral loaded flank area, minimum key shear, housing clearance and repair access require separate checks. A stock or alternate drawing does not guarantee the actual article geometry.")
if __name__=="__main__":
    r=run(*map(Path,sys.argv[1:]));(HERE/"inset-steering-key-installation.json").write_text(json.dumps(r,indent=1)+"\n")
