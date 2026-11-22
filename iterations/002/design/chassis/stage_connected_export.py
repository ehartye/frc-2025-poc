"""Stage the exact native source issuance with its private immutable dependencies."""
from pathlib import Path
from hashlib import sha256
import json, sys, shutil
from importlib.util import spec_from_file_location,module_from_spec
ROOT=Path(__file__).resolve().parents[4]
CHASSIS=Path(__file__).resolve().parent
def stage(station,mode="export"):
    jobs=station/"i002-jobs";base=jobs/"red2-002-connected-chassis-03"
    previous=json.loads((base/"job.json").read_bytes())
    scripts=base/"plugin/scripts";sp=spec_from_file_location("job",scripts/"native_job.py");native=module_from_spec(sp);sp.loader.exec_module(native)
    paths=[r["path"] for r in previous["inputs"] if "/chassis/" not in r["path"] and "/private-inputs/" not in r["path"]]
    prefix="iterations/002/design/"
    paths.extend(prefix+n for n in ["chassis/native_job_telemetry.py","chassis/native_connected_export.py","chassis/connected-native-03-cold.json","cad/private/connected-654-precursor.f3d"])
    stages={
        "export":("red2-002-connected-export-01","i002-654x","native_connected_export.py","connected-export-01.json",[]),
        "seven":("red2-002-connected-seven-01","i002-7c","native_connected_seven.py","connected-seven-01.json",
                 ["chassis/inset-corrected-brace-foot-clearance.json","chassis/connected-seven-input.json","chassis/native_corner.py","chassis/native_connected_chassis.py"]),
        "taps":("red2-002-formed-nutbar-threads-01","i002-4t","native_connected_taps.py","connected-thread-01.json",
                 ["chassis/connected-thread-input.json","chassis/connected-seven-stock-receipt.json","cad/private/connected-seven.f3d"])
    }
    name,short,entry,regname,extra=stages[mode]
    if mode!="export":
        paths=[n for n in paths if "issue-005" not in n and not n.endswith(("terminals-552.f3d","native_connected_export.py","connected-native-03-cold.json"))]
        paths.extend(prefix+n for n in ["cad/issue-006/lineage.json","cad/issue-006/assembly.step","chassis/"+entry,*extra])
    recipe="inputs/"+prefix+"chassis/"+entry
    plan=dict(schema=previous["schema"],jobId=name,pluginCommit=previous["pluginCommit"],pluginInventorySha256=previous["pluginInventorySha256"],
              recipe=recipe,inputs=[dict(path=n,sha256=sha256((ROOT/n).read_bytes()).hexdigest()) for n in paths])
    registration=CHASSIS/regname;registration.write_bytes((json.dumps(plan,indent=2)+"\n").encode())
    digest=sha256(registration.read_bytes()).hexdigest()
    native.stage(base/"plugin",ROOT,registration,digest,jobs)
    bundle=jobs/name
    out=station/short;client=station/(short+"-client");journal=client/"journal"
    for f in (out,journal):f.mkdir(parents=True,exist_ok=True)
    # The live Fusion process runs unelevated, with this same user SID. Windows'
    # default frozen bundle limits read access to an elevated administrator.
    import subprocess
    sid=json.loads((station/"i002-cc3-client/journal/red2-002-connected-chassis-03--access-01.result.json").read_bytes())["principal"]["windowsSid"]
    for path,rights in [(bundle,"RX"),(out,"M"),(journal,"M")]:
        command=["icacls",str(path),"/grant",f"*{sid}:(OI)(CI)({rights})"]
        subprocess.run(command,check=True,capture_output=True)
    for mode in ("access","build"):
        directory=client/mode;directory.mkdir(parents=True,exist_ok=True)
        cfg=dict(stage=mode,jobId=name,requestId=mode+"-01",resultDirectory=str(out),nativeJob=dict(bundle=str(bundle),registrationSha256=digest,writeDirectories=[str(out),str(journal)]))
        script=native.fusion_access_script(bundle,digest,name,[str(out),str(journal)]) if mode=="access" else native.fusion_script(bundle,digest,cfg)
        (directory/"inspect_case.py").write_text(script,encoding="utf-8")
        shutil.copyfile(station/"i002-cc3-client"/mode/"client.py",directory/"client.py")
        (directory/"request.json").write_text(json.dumps(cfg,indent=2))
    check=native.preflight(bundle,digest,[str(out),str(journal)])
    (client/"preflight.json").write_text(json.dumps(check,indent=2));print(digest,check.get("status"))
if __name__=="__main__":stage(Path(sys.argv[1]),sys.argv[2] if len(sys.argv)>2 else "export")
