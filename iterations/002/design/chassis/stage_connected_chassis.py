"""Freeze the exact 2025 native authoring inputs for the shared workstation."""
import importlib.util,json,hashlib,sys
from pathlib import Path
HERE=Path(__file__).resolve().parent
root=HERE.parents[3]
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def stage(station,revision):
    base=station/"i002-jobs/red2-002-terminals-01"
    scripts=base/"plugin/scripts"
    spec=importlib.util.spec_from_file_location("native_stage",scripts/"native_job.py");native=importlib.util.module_from_spec(spec);spec.loader.exec_module(native)
    source=json.loads((station/"i002-jobs/red2-002-inset-shoe-01/job.json").read_bytes())
    prefix="iterations/002/design/"
    paths=[r["path"] for r in source["inputs"] if "/chassis/" not in r["path"]]
    paths += [prefix+"chassis/"+n for n in ("native_corner.py","native_connected_chassis.py","inset-matched-native-plan.json","inset-chassis-shop.json","inset-stock-check.json")]
    paths += [prefix+"private-inputs/matched-sds/"+n for n in ("corner.f3d","cold.json")]
    if revision>=3:
        paths.extend((prefix+"private-inputs/matched-sds/four-corners-508.f3d",prefix+"chassis/connected-native-02-held.json"))
    plan=dict(schema=source["schema"],jobId=f"red2-002-connected-chassis-{revision:02d}",pluginCommit=source["pluginCommit"],
              pluginInventorySha256=source["pluginInventorySha256"],recipe="inputs/"+prefix+"chassis/native_connected_chassis.py",
              inputs=[dict(path=n,sha256=sha(root/n)) for n in paths])
    registration=HERE/f"connected-native-job-{revision:02d}.json";registration.write_text(json.dumps(plan,indent=2)+"\n")
    digest=sha(registration);bundle=station/"i002-jobs"/plan["jobId"]
    print(native.stage(base/"plugin",root,registration,digest,station/"i002-jobs"))
    client=station/f"i002-cc{revision}-client";out=station/f"i002-cc{revision}";client.mkdir(exist_ok=True);out.mkdir(exist_ok=True);journal=client/"journal";journal.mkdir(exist_ok=True)
    (client/"client.py").write_bytes((station/"i002-sh1-client/client.py").read_bytes())
    for mode in ("access","build"):
        cfg=dict(stage=mode,jobId=plan["jobId"],requestId=mode+"-01",resultDirectory=str(out),nativeJob=dict(bundle=str(bundle),registrationSha256=digest,writeDirectories=[str(out),str(journal)]))
        script=native.fusion_access_script(bundle,digest,plan["jobId"],[str(out),str(journal)]) if mode=="access" else native.fusion_script(bundle,digest,cfg)
        # Each operation has its own stationary driver with an independent named recipe.
        directory=client/mode;directory.mkdir(exist_ok=True)
        (directory/"inspect_case.py").write_text(script,encoding="utf-8")
        (directory/"client.py").write_bytes((client/"client.py").read_bytes())
        (directory/"request.json").write_text(json.dumps(cfg,indent=2))
    access=native.preflight(bundle,digest,[str(out),str(journal)]);(client/"preflight.json").write_text(json.dumps(access,indent=2))
    print("REGISTRATION",digest)
if __name__=="__main__":stage(Path(sys.argv[1]),int(sys.argv[2]))
