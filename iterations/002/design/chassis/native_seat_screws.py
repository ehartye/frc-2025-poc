"""Install four genuine 91251A542 screws into the modeled nutbar taps, one checked placement per screw.

Each pass copies the unmodified SDS source screw component into the root at
    worldDeltaFromSourceCm * measuredSourceWorld,
where worldDeltaFromSourceCm = T(u) * R(-z, +57 deg) * T(-u) * studyDeltaWorldCm and u is the under-head seat.
The successor F3D of each pass becomes the next accepted archive. Any veto or exception holds the chain.
"""
from hashlib import sha256
from pathlib import Path
from tempfile import TemporaryDirectory
import json,math,shutil,traceback,types

INPUT="iterations/002/design/chassis/seat-screws-input.json"
HERE="iterations/002/design/chassis"
SCREW="@screw"
TOL_CM=1e-6


# ---------- pure helpers (offline-testable; no adsk) ----------
def require(ok,message):
    if not ok:raise ValueError(message)

def product(a,b):
    return [sum(a[4*i+k]*b[4*k+j] for k in range(4)) for i in range(4) for j in range(4)]

def inverse(m):
    r=[m[j*4+i] if i<3 and j<3 else float(i==j) for i in range(4) for j in range(4)]
    for i in range(3):r[i*4+3]=-sum(r[i*4+k]*m[k*4+3] for k in range(3))
    return r

def apply(m,p):
    return [sum(m[4*i+k]*p[k] for k in range(3))+m[4*i+3] for i in range(3)]

def translation(v):
    return [1.,0.,0.,v[0], 0.,1.,0.,v[1], 0.,0.,1.,v[2], 0.,0.,0.,1.]

def rotation_about(axis,degrees,through):
    """Right-handed rotation by degrees about the line through `through` along unit `axis` (Rodrigues)."""
    n=math.sqrt(sum(a*a for a in axis));x,y,z=(a/n for a in axis)
    t=math.radians(degrees);c,s=math.cos(t),math.sin(t);k=1-c
    r=[c+x*x*k, x*y*k-z*s, x*z*k+y*s, 0.,
       y*x*k+z*s, c+y*y*k, y*z*k-x*s, 0.,
       z*x*k-y*s, z*y*k+x*s, c+z*z*k, 0.,
       0.,0.,0.,1.]
    return product(translation(through),product(r,translation([-v for v in through])))

def seat_delta(under,seat):
    """Study selection transform T(under) * Rx(180) * T(-seat): seat -> under, +z -> -z."""
    flip=[1.,0.,0.,0., 0.,-1.,0.,0., 0.,0.,-1.,0., 0.,0.,0.,1.]
    return product(translation(under),product(flip,translation([-v for v in seat])))

def compose_delta(study_delta,under,axis,degrees):
    return product(rotation_about(axis,degrees,under),study_delta)

def phase_about(rotation,axis):
    """Signed right-handed angle (deg) of a rotation's linear part about unit axis."""
    n=math.sqrt(sum(a*a for a in axis));a=[v/n for v in axis]
    ref=[1.,0.,0.] if abs(a[0])<0.9 else [0.,1.,0.]
    d=sum(p*q for p,q in zip(ref,a));u=[p-d*q for p,q in zip(ref,a)]
    m=math.sqrt(sum(v*v for v in u));u=[v/m for v in u]
    w=[sum(rotation[4*i+k]*u[k] for k in range(3)) for i in range(3)]
    cross=[u[1]*w[2]-u[2]*w[1],u[2]*w[0]-u[0]*w[2],u[0]*w[1]-u[1]*w[0]]
    return math.degrees(math.atan2(sum(p*q for p,q in zip(cross,a)),sum(p*q for p,q in zip(u,w))))

def close(a,b,tol=1e-9):
    return len(a)==len(b) and all(abs(x-y)<=tol for x,y in zip(a,b))

def distance(a,b):
    return math.sqrt(sum((x-y)**2 for x,y in zip(a,b)))

def check_row(row,seat_cm):
    """Recompose the row delta from its stated parts and prove seat landing and install phase."""
    t=row["transform"];u=row["underHeadCm"]
    require(close(seat_delta(u,seat_cm),t["studyDeltaWorldCm"],1e-12),"Study delta differs from its seat composition: "+row["label"])
    delta=compose_delta(t["studyDeltaWorldCm"],t["phaseThroughCm"],t["phaseAxis"],t["phaseDeg"])
    require(close(delta,t["worldDeltaFromSourceCm"],1e-12),"Stored delta differs from its composition: "+row["label"])
    require(t["phaseThroughCm"]==u and t["phaseAxis"]==[0.0,0.0,-1.0],"Phase must turn about the into-host axis through underHead")
    landing=distance(apply(delta,seat_cm),u)
    phase=product(delta,inverse(t["studyDeltaWorldCm"]))
    turned=phase_about(phase,t["phaseAxis"])
    fixed=distance(apply(phase,u),u)
    require(landing<=1e-12 and fixed<=1e-12 and abs(turned-t["phaseDeg"])<=1e-9,"Phase composition fails its seat/axis proof: "+row["label"])
    return dict(seatLandingErrorCm=landing,phaseDeg=turned,axisPointErrorCm=fixed)

def plan_rows(spec,allowed):
    allowed=set(allowed or [])
    pending=[r["label"] for r in spec["rows"] if r["status"]!="settled"]
    require(all(r["status"] in ("settled","pending-study") for r in spec["rows"]),"Unknown row status")
    require(set(pending)<=allowed,"Refusing pending-study rows not named in CONFIG.allowPendingRows: "+", ".join(sorted(set(pending)-allowed)))
    require(allowed<=set(pending),"allowPendingRows names a row that is not pending: "+", ".join(sorted(allowed-set(pending))))
    return spec["rows"]

def bind_interfaces(row,occurrence):
    """Guard-shaped interfaces with the new screw's exact rooted path substituted."""
    out=[]
    for item in row["interfaces"]:
        q=json.loads(json.dumps(item["request"]))
        q["bodies"]=[[occurrence if p==SCREW else p,n] for p,n in q["bodies"]]
        require(sum(p==occurrence for p,_ in q["bodies"])==1,"Each interface needs exactly one new-screw body")
        out.append(q)
    return out

def placement_request(row,occurrence):
    return {"occurrence":occurrence,"sourceOccurrence":row["sourceOccurrence"],
            "worldDeltaFromSourceCm":list(row["transform"]["worldDeltaFromSourceCm"]),
            "interfaces":bind_interfaces(row,occurrence)}

def stationary_identities(rows):
    return sorted({tuple(b) for r in rows for i in r["interfaces"] for b in i["request"]["bodies"] if b[0]!=SCREW})

def file_sha(p):
    h=sha256()
    with Path(p).open("rb") as s:
        for block in iter(lambda:s.read(1<<20),b""):h.update(block)
    return h.hexdigest()

def pinned(root,ref):
    p=(root/ref["file"]).resolve()
    require(not Path(ref["file"]).is_absolute() and ".." not in Path(ref["file"]).parts and p.is_relative_to(root.resolve()) and p.is_file(),
            "Unstaged or external input: "+ref["file"])
    require(file_sha(p)==ref["sha256"],"Changed input: "+ref["file"])
    return p

def module(p):
    m=types.ModuleType(p.stem);m.__file__=str(p)
    exec(compile(p.read_bytes(),str(p),"exec"),m.__dict__)
    return m


# ---------- native recipe ----------
def run(context):
    from adsk import core, fusion
    app=core.Application.get();start=[d.creationId for d in app.documents];docs=[]
    root=Path(CONFIG["jobInputs"]);out=Path(CONFIG["resultDirectory"]);h=root/HERE
    receipt_path=out/"native-receipt.json"
    if receipt_path.exists() or any(out.glob("screws-step-*")):raise FileExistsError("Result directory belongs to an earlier run")
    r={"schema":"red2/seat-screws-receipt@1","before":start,"status":"working","steps":[]}
    try:
        if start or app.activeDocument:raise ValueError("Home/0 required")
        NATIVE_JOB["verifyDependencies"]()
        plugin=Path(CONFIG["plugin"])
        authority=module(plugin/"fusion_authority.py")
        spec=json.loads((root/INPUT).read_bytes());r["inputSha256"]=file_sha(root/INPUT)
        rows=plan_rows(spec,CONFIG.get("allowPendingRows"))
        r["allowedPendingRows"]=sorted(CONFIG.get("allowPendingRows") or [])
        # Authority: the canonical starting assembly is unchanged and is the one the taps job cut into.
        reg=authority.register_path(root)
        base=authority.verify(root,reg,reg.parent/spec["authority"]["lineage"])
        thread=json.loads(pinned(root,{"file":spec["authority"]["threadInput"],"sha256":spec["authority"]["threadInputSha256"]}).read_bytes())
        if base!=thread["authority"]:raise ValueError("Canonical starting assembly changed")
        r["authority"]=base
        # Accepted taps-02 archive and its issuing receipt.
        acc=spec["accepted"];accepted=pinned(root,acc["archive"]);accepted_sha=acc["archive"]["sha256"]
        prior=json.loads(pinned(root,acc["receipt"]).read_bytes())
        if (prior.get("status")!=acc["status"] or prior.get("nativeSha256")!=accepted_sha
                or prior.get("stepSha256")!=acc["stepSha256"] or prior.get("restored") is not True):
            raise ValueError("taps-02 receipt does not issue this accepted archive")
        study=json.loads(pinned(root,spec["study"]).read_bytes())
        if study.get("schema")!=spec["study"]["schema"] or study["inputsSha256"].get("taps-02/four-tapped-holes.f3d")!=accepted_sha:
            raise ValueError("Screw study was not run on the accepted archive")
        src=spec["source"]
        if "+".join(study["source"]["identity"]["occurrencePath"][1::2])!=src["occurrence"]:raise ValueError("Study source occurrence differs")
        settled={s["label"]:s for s in study["screws"]}
        proofs={}
        for row in rows:
            proofs[row["label"]]=check_row(row,src["seatCm"])
            if row["status"]=="settled" and not close(settled[row["label"]]["transform"]["deltaWorldCm"],row["transform"]["studyDeltaWorldCm"],1e-12):
                raise ValueError("Row delta differs from the issued study: "+row["label"])
        r["compositionProofs"]=proofs
        guard=module(plugin/"fusion_checked_placement.py")
        read=module(h/"native_job_telemetry.py").NativeProgress(out,accepted)

        def opened(p):
            d=app.importManager.importToNewDocument(app.importManager.createFusionArchiveImportOptions(str(p)));docs.append(d);return d
        def closed(d):
            d.close(False);docs.remove(d)
        def des_of(d):
            return fusion.Design.cast(d.products.itemByProductType("DesignProductType"))
        def census(des):
            seen={};count=des.rootComponent.bRepBodies.count
            for o in des.rootComponent.allOccurrences:
                for b in o.component.bRepBodies:
                    k=(o.fullPathName,b.name);seen[k]=seen.get(k,0)+1;count+=1
            return seen,count
        def find(des,path):
            hits=[o for o in des.rootComponent.allOccurrences if o.fullPathName==path]
            if len(hits)!=1:raise ValueError("Expected one occurrence "+path+", found "+str(len(hits)))
            return hits[0]
        def target_matrix(world,row):
            m=core.Matrix3D.create()
            if not m.setWithArray(product(row["transform"]["worldDeltaFromSourceCm"],world)):raise ValueError("Invalid target matrix")
            return m
        local_seat=None

        # Probe: confirm identities and learn Fusion's names for the four new occurrences in a discarded copy.
        names=[]
        with read.operation("probe accepted archive: source, interface bodies, new occurrence names",accepted.name,1,1):
            with TemporaryDirectory(dir=out) as scratch:
                copy=Path(scratch)/"probe.f3d";shutil.copyfile(accepted,copy)
                if file_sha(copy)!=accepted_sha:raise ValueError("Probe copy differs")
                doc=opened(copy)
                try:
                    des=des_of(doc)
                    seen,count=census(des)
                    if count!=acc["bodyCount"]:raise ValueError(f"Accepted roster has {count} bodies, expected {acc['bodyCount']}")
                    for k in stationary_identities(rows):
                        if seen.get(k)!=1:raise ValueError("Interface body is missing or ambiguous: "+repr(k))
                    source=find(des,src["occurrence"])
                    if (source.childOccurrences.count or [b.name for b in source.component.bRepBodies]!=[src["body"]]
                            or source.component.name!=src["component"]):
                        raise ValueError("Source screw is not the single-body purchased part")
                    world=list(source.transform2.asArray())
                    if not close(world,src["worldCm"],1e-5):raise ValueError("Native source pose differs from the study's STEP source pose")
                    local_seat=apply(inverse(world),src["seatCm"])
                    r["sourceWorldCm"]=world
                    probe=[]
                    for row in rows:
                        occ=des.rootComponent.occurrences.addExistingComponent(source.component,target_matrix(world,row))
                        if not occ:raise ValueError("Probe insertion failed")
                        names.append(occ.fullPathName)
                        got=list(occ.transform2.asArray())
                        if not close(got,product(row["transform"]["worldDeltaFromSourceCm"],world),1e-7):raise ValueError("Probe pose differs: "+row["label"])
                        probe.append(dict(label=row["label"],occurrence=occ.fullPathName,worldCm=got,
                                          seatLandingErrorMm=10*distance(apply(got,local_seat),row["underHeadCm"])))
                    if len(set(names))!=len(names) or any("+" in n for n in names):raise ValueError("Probe names are not distinct root occurrences")
                    r["probe"]=probe
                finally:
                    closed(doc)

        # Chain: one checked placement per screw.
        current,current_sha=accepted,accepted_sha
        r["lastAccepted"]=dict(path=str(current),sha256=current_sha)
        for i,(row,name) in enumerate(zip(rows,names),1):
            output=out/spec["expected"]["outputs"][i-1]
            request=placement_request(row,name)
            observed={}
            def edit(design,row=row,name=name,observed=observed):
                source=find(design,row["sourceOccurrence"])
                world=list(source.transform2.asArray())
                occ=design.rootComponent.occurrences.addExistingComponent(source.component,target_matrix(world,row))
                if not occ:raise ValueError("Insertion failed")
                observed.update(occurrence=occ.fullPathName,sourceWorldCm=world,attainedWorldCm=list(occ.transform2.asArray()))
                if occ.fullPathName!=name:raise ValueError(f"New occurrence named {occ.fullPathName!r}, request names {name!r}")
            step=dict(ordinal=i,label=row["label"],status=row["status"],acceptedArchive=str(current),acceptedSha256=current_sha,
                      output=str(output),request=request,observed=observed)
            r["steps"].append(step)
            try:
                with read.operation("checked placement of seated 91251A542",row["label"],i,len(rows)):
                    report=guard.checked_placement(app,str(current),current_sha,str(output),request,edit,evidence_root=str(root))
            except guard.PlacementPoseError as e:
                report=dict(status="held",reason="pose-veto",poseVeto=e.receipt)
            except Exception:
                report=dict(status="held",reason="exception",exception=traceback.format_exc())
            step["report"]=report
            (out/f"screws-step-{i}-receipt.json").write_text(json.dumps(step,indent=1,default=str)+"\n")
            if report.get("status")!="pass":
                r.update(status=f"held at {row['label']} ({report.get('status')}); last accepted archive retained")
                break
            successor=file_sha(output)
            if successor!=report["successorSha256"]:
                r.update(status=f"held at {row['label']}: successor bytes differ from the guard report");break
            read.checkpoint(output)
            current,current_sha=output,successor
            r["lastAccepted"]=dict(path=str(current),sha256=current_sha,afterLabel=row["label"])
        else:
            with read.operation("cold reopen final archive, export and re-read STEP","screws",1,1):
                doc=opened(current);des=des_of(doc)
                seen,count=census(des)
                source=find(des,src["occurrence"]);world=list(source.transform2.asArray())
                cold=[]
                for row,name,step in zip(rows,names,r["steps"]):
                    occ=des.rootComponent.occurrences.itemByName(name)
                    if not occ or occ.fullPathName!=name or occ.component.name!=src["component"]:raise ValueError("Cold screw missing: "+name)
                    got=list(occ.transform2.asArray())
                    want=product(row["transform"]["worldDeltaFromSourceCm"],world)
                    landing=10*distance(apply(got,local_seat),row["underHeadCm"])
                    if not close(got,want,1e-7) or not close(got,step["report"]["worldTransformCm"],1e-7) or landing>1e-4:
                        raise ValueError("Cold screw pose differs: "+name)
                    cold.append(dict(label=row["label"],occurrence=name,worldCm=got,seatLandingErrorMm=landing))
                if count!=spec["expected"]["finalBodyCount"]:raise ValueError(f"Cold roster has {count} bodies")
                stepfile=out/spec["expected"]["step"]
                if stepfile.exists() or not des.exportManager.execute(des.exportManager.createSTEPExportOptions(str(stepfile),des.rootComponent)):
                    raise ValueError("Cold STEP export failed")
                closed(doc)
                NATIVE_JOB["verifyDependencies"]()
                back=app.importManager.importToNewDocument(app.importManager.createSTEPImportOptions(str(stepfile)));docs.append(back)
                _,solids=census(des_of(back))
                closed(back)
                if solids!=spec["expected"]["finalBodyCount"]:raise ValueError(f"STEP re-read has {solids} solids")
                r.update(status="four seated purchased screws placed; mechanical review pending",
                         nativeSha256=current_sha,stepSha256=file_sha(stepfile),bodyCount=count,stepSolids=solids,cold=cold)
    except Exception:r.update(status="held",exception=traceback.format_exc())
    finally:
        for d in list(docs):
            try:d.close(False)
            except Exception:pass
        r["after"]=[d.creationId for d in app.documents];r["restored"]=r["after"]==start and app.activeDocument is None
        receipt_path.write_text(json.dumps(r,indent=1,default=str)+"\n")
    print(json.dumps({k:r.get(k) for k in ("status","restored")}))
