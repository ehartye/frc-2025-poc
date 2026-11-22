"""Bind the supplier material evidence to each cold-checked constituent on all four 2025 corners."""
import collections,hashlib,json,sys
from pathlib import Path
from inset_chassis_shop import HERE

def run(plugin,catalog):
    sys.path.insert(0,str(plugin/"scripts"))
    from fusion_authority import register_path,verify
    sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    reg=register_path(HERE.parents[3]);authority=verify(HERE.parents[3],reg,reg.parent/"issue-005/lineage.json")
    donor=HERE.parent/"private-inputs/matched-sds"; cold=json.loads((donor/"cold.json").read_bytes())
    issued=json.loads((HERE/"connected-native-02-held.json").read_bytes());plan=json.loads((HERE/"inset-matched-native-plan.json").read_bytes());m=json.loads(catalog.read_bytes())
    if sha(catalog)!="5e3162885c6fa397454f736ac1b6e85de8e1692a8ed9bf8da98e572cfa4720e4" or sha(donor/"corner.f3d")!=m["nativeModule"] or issued["authority"]!=authority:raise ValueError("Unbound sources")
    baseline={(r["path"],r["body"]):r for r in cold["roster"]}
    roster={(r["sourcePath"],r["sourceBody"]):r for r in m["nativeRows"]}
    if set(baseline)!=set(roster) or len(roster)!=127 or sha(donor/"four-corners-508.f3d")!=issued["checks"][-1]["successorSha256"]:raise ValueError("Incomplete roster")
    out={}
    for side,expected in plan["expected"].items():
        measured=issued["508PlacedMaterials"][side]
        if measured["count"]!=127 or measured["maxWorldBoundsErrorMm"]>.003:raise ValueError("Incomplete cold placement")
        records=[]
        for row in expected:
            k=row["path"],row["body"];src=roster[k];orig=baseline[k]
            if orig["material"]!=row["material"] or row["material"]!=src["rawFusionMaterial"] or abs(row["volumeMm3"]-src["sourceVeryHighVolumeMm3"])>.0001:raise ValueError("Changed matching article")
            records.append(dict(path=k[0],body=k[1],nativeMaterial=row["material"],procurementScope=src["scope"],basis=src["reason"],references=src["citations"],boundsMm=row["expectedBoundsMm"]))
        out[side]=records
    return dict(authority=authority,privateCheckpoint=issued["checks"][-1]["successorSha256"],supplierSourceSHA256=m["nativeModule"],sourceCatalogSHA256=sha(catalog),sourceLinks=m["sources"],
            stockLabelsPerModule=dict(collections.Counter(v["rawFusionMaterial"] for v in roster.values())),classesPerModule=dict(collections.Counter(v["scope"] for v in roster.values())),bodyCount=sum(map(len,out.values())),
            catalogApproxKgPerModule=m["purchaseMassKgPerNEOModuleApprox"],rows=out,scope="Source and installed archive contains inherited physical material labels. The catalog scope and four mapped world bounds establish which purchased constituent has which evidence; this does not yet change the native material. The complete NEO module catalog mass includes motors, encoder, plate and wiring. Component weight deltas, exact tire mass, internal CG, spinning rotor and azimuth inertia, plus broadcast rights, remain separate.")
if __name__=="__main__":
    d=run(*map(Path,sys.argv[1:]));(HERE/"inset-matched-material-ledger.json").write_text(json.dumps(d,indent=1)+"\n")
