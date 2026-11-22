"""Issue the saved 654-body machining archive through the canonical native exporter."""
from pathlib import Path
import json
import shutil
import traceback
import types
from adsk import core

def module(path):
    m=types.ModuleType(path.stem);m.__file__=str(path)
    exec(compile(path.read_bytes(),str(path),"exec"),m.__dict__)
    return m

def run(context):
    app=core.Application.get()
    baseline=[d.creationId for d in app.documents]
    destination=Path(CONFIG["resultDirectory"])
    incoming=Path(CONFIG["jobInputs"])
    chassis=incoming/"iterations/002/design/chassis"
    authority=module(Path(CONFIG["plugin"])/"fusion_authority.py")
    result={"before":baseline}
    try:
        if baseline or app.activeDocument:raise ValueError("Require empty station")
        NATIVE_JOB["verifyDependencies"]()
        registered=authority.register_path(incoming)
        original=authority.verify(incoming,registered,registered.parent/"issue-005/lineage.json")
        receipt=json.loads((chassis/"connected-native-03-cold.json").read_bytes())
        name="iterations/002/design/cad/private/connected-654-precursor.f3d"
        archive=incoming/name
        if (receipt["authority"]!=original or receipt["status"]!="machining checkpoint" or receipt["coldBodies"]!=654 or
                receipt["restored"] is not True or receipt["nativeSha256"]!=authority.digest(archive)):
            raise ValueError("Saved issue differs from its completed stock check")
        progress=module(chassis/"native_job_telemetry.py").NativeProgress(destination,archive)
        mirror=destination/"mirror"
        with progress.operation("bind engineering register", archive.name,1,2):
            if mirror.exists():raise ValueError("Fresh issue destination required")
            for rel in ["iterations/active.json","iterations/001/iteration.json","iterations/001/retrospective.json",
                        "iterations/seals/001.json","iterations/002/iteration.json",name]:
                path=mirror/rel;path.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(incoming/rel,path)
            fresh=authority.register_path(mirror)
            current=authority.load(registered)
            current.update(configuration="Four parked matched SDS L2 modules and editable inset perimeter, electronics decks and pan support trusses",
                        revision="002-connected-machining-654-r6",source={"kind":"offline","archive":{"file":name,"sha256":receipt["nativeSha256"]}},
                        basis="654 cold-matched solids. Attached electronics, battery, moving pan, bumper materials and remaining fasteners are outside this private machining issue. The stock steering-key clash and two inaccessible original truss screw paths are retained controls.")
            fresh.write_bytes((json.dumps(current,indent=2)+"\n").encode())
        with progress.operation("cold export registered native source",archive.name,2,2):
            issue=fresh.parent/"issue-006"
            result["lineage"]=authority.export_issue(app,mirror,fresh,issue)
            result["verified"]=authority.verify(mirror,fresh,issue/"lineage.json")
            result["lineageSHA256"]=authority.digest(issue/"lineage.json")
        result["status"]="exported"
    except Exception:
        result.update(status="held",exception=traceback.format_exc())
    finally:
        for d in reversed(list(app.documents)):
            if d.creationId not in baseline:d.close(False)
        result["after"]=[d.creationId for d in app.documents]
        result["restored"]=result["after"]==baseline and app.activeDocument is None
        (destination/"native-receipt.json").write_text(json.dumps(result,indent=1)+"\n")
    print(json.dumps({k:result.get(k) for k in ("status","restored","lineageSHA256")}))
