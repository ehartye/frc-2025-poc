"""Install the guarded pan, its upper support and the fixed inlet on the accepted taps-02 chassis.

Proposal recipe for review (geometry revision r3: riveted cheek ladder, plate-stack posts and lugs, fail-closed checks). Runs inside Fusion through the staged native job
(CONFIG / NATIVE_JOB supplied by scripts/native_job.py). It reads only the files
listed in job-inputs.txt. Purchased articles (bearings, thrust stacks, gearbox,
motor, hex shaft, stop pads, closing spring, fasteners) are NOT authored. The
recipe leaves named seat sketches so each one can be placed later with
fusion_checked_placement. Chassis bodies are resolved by their own rooted
occurrence paths (139 under the structural machining root, 7 under the brace
revision root), never by an assumed single parent.

House pattern: Home/0 required, verifyDependencies, authority verify, telemetry
operations, save F3D, cold reopen, STEP export, restore the document roster.
Fusion's internal length unit is cm. geometry.json is in mm.
"""
from pathlib import Path
import json, math, re, traceback, types
import adsk
from adsk import core, fusion

PAN = "iterations/002/design/pan"
GROUPS = ("Pan upper support", "Guarded pan", "Fixed inlet")
ATTR = "red2-pan-002"
ACCURACY = fusion.CalculationAccuracy.VeryHighCalculationAccuracy
VOLUME_REL_TOL, VOLUME_ABS_TOL_MM3 = 2e-4, 0.5      # see volume_tolerance()
COMMON_TOL_MM3 = 0.05                               # contact (no shared volume) vs overlap, on native interference volume


def volume_tolerance(part):
    """Native-vs-OCP volume tolerance. Both are exact B-rep integrations; at very high accuracy their difference is far below
    2e-4 relative. A real geometry error the recipe must catch is a 0.05 mm face offset (the bounds tolerance), which changes the
    volume by at least 0.05 mm x the smallest face; for these plate and tube parts that exceeds 2e-4 of the volume."""
    return max(VOLUME_ABS_TOL_MM3, VOLUME_REL_TOL * part["volumeMm3"])


def module(p):
    m = types.ModuleType(p.stem); m.__file__ = str(p)
    exec(compile(p.read_bytes(), str(p), "exec"), m.__dict__)
    return m


def cm(v):
    return v / 10.0


def pt(p):
    return core.Point3D.create(*[cm(v) for v in p])


def slug(text):
    return "t_" + re.sub(r"[^0-9A-Za-z]+", "_", text).strip("_").lower()


def plane(comp, axis, coordinate):
    selected = [comp.yZConstructionPlane, comp.xZConstructionPlane, comp.xYConstructionPlane][axis]
    sign = selected.geometry.normal.asArray()[axis]
    if abs(abs(sign) - 1) > 1e-6:
        raise ValueError("Unexpected construction plane")
    req = comp.constructionPlanes.createInput()
    req.setByOffset(selected, core.ValueInput.createByReal(cm(coordinate) / sign))
    made = comp.constructionPlanes.add(req); made.isLightBulbOn = False
    return made, sign


def polygon(sk, points):
    q = [sk.modelToSketchSpace(pt(p)) for p in points]
    lines = sk.sketchCurves.sketchLines
    first = prev = None
    for i in range(len(q)):
        a = prev if prev else q[0]
        b = first if i + 1 == len(q) else q[i + 1]
        ln = lines.addByTwoPoints(a, b)
        if first is None:
            first = ln.startSketchPoint
        prev = ln.endSketchPoint


def profile(sk, loops):
    hits = [p for p in sk.profiles if p.profileLoops.count == loops]
    if len(hits) != 1:
        raise ValueError(f"{sk.name}: expected one profile with {loops} loop(s); found {len(hits)} of {sk.profiles.count}")
    return hits[0]


def prism_feature(des, comp, spec, operation, params, name, body=None):
    """One extrude (pure translation along the section normal) or loft between two axis-normal sections, with optional void."""
    axis = spec["axis"]; a, b = spec["sections"]
    shift = [b[0][i] - a[0][i] for i in range(3)]
    pure = all(abs(b[k][i] - a[k][i] - shift[i]) < 1e-6 for k in range(len(a)) for i in range(3)) and \
        all(abs(shift[i]) < 1e-6 for i in range(3) if i != axis)
    feats = comp.features
    p0, sign = plane(comp, axis, a[0][axis])
    sk = comp.sketches.add(p0); sk.name = name + " section"
    polygon(sk, a)
    void = spec.get("void")
    if pure:
        if void:
            polygon(sk, void[0])
        travel = shift[axis]
        pname = slug(name)
        if not des.userParameters.itemByName(pname):
            des.userParameters.add(pname, core.ValueInput.createByString(f"{abs(travel):.4f} mm"), "mm", name + " extrusion")
        params.append(pname)
        req = feats.extrudeFeatures.createInput(profile(sk, 2 if void else 1), operation)
        direction = fusion.ExtentDirections.PositiveExtentDirection if travel / sign > 0 else fusion.ExtentDirections.NegativeExtentDirection
        req.setOneSideExtent(fusion.DistanceExtentDefinition.create(core.ValueInput.createByString(pname)), direction)
        if body is not None:
            req.participantBodies = [body]
        feats.extrudeFeatures.add(req).name = name
    else:
        p1, _ = plane(comp, axis, b[0][axis])
        sk1 = comp.sketches.add(p1); sk1.name = name + " end section"
        polygon(sk1, b)
        req = feats.loftFeatures.createInput(operation)
        req.loftSections.add(profile(sk, 1)); req.loftSections.add(profile(sk1, 1)); req.isSolid = True
        if body is not None:
            req.participantBodies = [body]
        feats.loftFeatures.add(req).name = name
        if void:
            sv0 = comp.sketches.add(p0); sv0.name = name + " void section"; polygon(sv0, void[0])
            sv1 = comp.sketches.add(p1); sv1.name = name + " void end section"; polygon(sv1, void[1])
            cut = feats.loftFeatures.createInput(fusion.FeatureOperations.CutFeatureOperation)
            cut.loftSections.add(profile(sv0, 1)); cut.loftSections.add(profile(sv1, 1))
            cut.participantBodies = [body if body is not None else comp.bRepBodies.item(comp.bRepBodies.count - 1)]
            feats.loftFeatures.add(cut).name = name + " void"
            sv0.isLightBulbOn = False; sv1.isLightBulbOn = False
        sk1.isLightBulbOn = False
    sk.isLightBulbOn = False


def build_part(des, comp, part, material, params):
    """Primary prism, then joins and cuts (each its own editable sketch and feature), then bores and hexes: one named body."""
    feats = comp.features
    prism_feature(des, comp, part, fusion.FeatureOperations.NewBodyFeatureOperation, params, part["id"] + " stock")
    if comp.bRepBodies.count != 1:
        raise ValueError("Split or missing stock " + part["id"])
    body = comp.bRepBodies.item(0)
    for i, j in enumerate(part.get("joins") or [], 1):
        prism_feature(des, comp, j, fusion.FeatureOperations.JoinFeatureOperation, params, f"{part['id']} join {i}", body)
        body = comp.bRepBodies.item(0)
    for i, c in enumerate(part.get("cuts") or [], 1):
        prism_feature(des, comp, c, fusion.FeatureOperations.CutFeatureOperation, params, f"{part['id']} cut {i}", body)
        body = comp.bRepBodies.item(0)
    if comp.bRepBodies.count != 1:
        raise ValueError("Join/cut split " + part["id"])
    for bore in part.get("bores") or []:
        ax = [i for i in range(3) if abs(bore["direction"][i]) > .5][0]
        pl, sg = plane(comp, ax, bore["start"][ax])
        s = comp.sketches.add(pl); s.name = part["id"] + " bore"
        s.sketchCurves.sketchCircles.addByCenterRadius(s.modelToSketchSpace(pt(bore["start"])), cm(bore["radius"]))
        req = feats.extrudeFeatures.createInput(profile(s, 1), fusion.FeatureOperations.CutFeatureOperation)
        req.setDistanceExtent(False, core.ValueInput.createByReal(cm(bore["depth"]) * bore["direction"][ax] / sg))
        req.participantBodies = [body]
        feats.extrudeFeatures.add(req).name = part["id"] + f" bore r{bore['radius']:g}"
        s.isLightBulbOn = False
    for h in part.get("hexes") or []:
        pl, sg = plane(comp, 1, h["y0"])
        s = comp.sketches.add(pl); s.name = part["id"] + " hex"
        r = h["flats"] / math.sqrt(3)
        polygon(s, [[h["center"][0] + r * math.cos(math.radians(30 + 60 * i)), h["y0"],
                     h["center"][1] + r * math.sin(math.radians(30 + 60 * i))] for i in range(6)])
        req = feats.extrudeFeatures.createInput(profile(s, 1), fusion.FeatureOperations.CutFeatureOperation)
        req.setDistanceExtent(False, core.ValueInput.createByReal(cm(h["y1"] - h["y0"]) / sg))
        req.participantBodies = [body]
        feats.extrudeFeatures.add(req).name = part["id"] + " 1/2 in hex"
        s.isLightBulbOn = False
    if comp.bRepBodies.count != 1:
        raise ValueError("Machining split " + part["id"])
    body = comp.bRepBodies.item(0)
    body.name = part["id"]; body.material = material
    body.attributes.add(ATTR, "id", part["id"])
    for key in ("status", "source", "role", "material", "stock"):
        body.attributes.add(ATTR, key, str(part.get(key, "")))
    return body


def seat(comp, row):
    """Hidden named sketch with the seat point and a 20 mm construction axis, for later checked placement."""
    sk = comp.sketches.add(comp.xYConstructionPlane); sk.name = row["name"]
    p, d = row["pointMm"], row["axis"]
    a = sk.modelToSketchSpace(pt(p)); b = sk.modelToSketchSpace(pt([p[i] + 20 * d[i] for i in range(3)]))
    sk.sketchPoints.add(a); sk.sketchCurves.sketchLines.addByTwoPoints(a, b).isConstruction = True
    sk.attributes.add(ATTR, "seat", json.dumps(row)); sk.isLightBulbOn = False


def run(context):
    app = core.Application.get(); start = [d.creationId for d in app.documents]; doc = None
    root = Path(CONFIG["jobInputs"]); out = Path(CONFIG["resultDirectory"]); here = root / PAN
    r = {"schema": "red2/pan-build-receipt@1", "before": start, "status": "working", "built": [], "seats": [],
         "joints": {}, "checks": {}, "findings": []}
    try:
        if start or app.activeDocument:
            raise ValueError("Home/0 required")
        NATIVE_JOB["verifyDependencies"]()
        authority = module(Path(CONFIG["plugin"]) / "fusion_authority.py")
        geo = json.loads((here / "geometry.json").read_bytes()); bi = geo["buildInput"]
        reg = authority.register_path(root)
        base = authority.verify(root, reg, reg.parent / bi["authority"]["lineage"])
        if base != bi["authority"]["record"]:
            raise ValueError("Canonical authority record changed: " + json.dumps({k: (base.get(k), v) for k, v in bi["authority"]["record"].items() if base.get(k) != v}))
        r["authority"] = base
        parent = root / bi["parent"]["stagedPath"]
        if not parent.is_file() and CONFIG.get("parentArchive"):
            parent = Path(CONFIG["parentArchive"])
        if authority.digest(parent) != bi["parent"]["sha256"]:
            raise ValueError("Accepted taps-02 parent changed")
        receipt = root / bi["parent"]["receipt"]
        if not receipt.is_file():
            raise ValueError("taps-02 native receipt is not staged: " + bi["parent"]["receipt"])
        prior = json.loads(receipt.read_bytes())
        if authority.digest(receipt) != bi["parent"]["receiptSha256"] or not prior.get("restored") or prior.get("nativeSha256") != bi["parent"]["sha256"]:
            raise ValueError("Unclosed or changed taps-02 checkpoint")
        # independent pin: the STEP the clearances used must be the STEP the parent's own receipt issued
        if geo["inputs"].get("external/taps-02/four-tapped-holes.step") != prior.get("stepSha256"):
            raise ValueError("geometry.json clearances were not computed on the STEP issued by the taps-02 receipt")
        read = module(root / "iterations/002/design/chassis/native_job_telemetry.py").NativeProgress(out, parent)
        f3d = out / bi["outputs"]["archive"]; step = out / bi["outputs"]["step"]
        if f3d.exists() or step.exists():
            raise ValueError("Output already exists")

        def opened(p):
            return app.importManager.importToNewDocument(app.importManager.createFusionArchiveImportOptions(str(p)))

        def des_of(d):
            return fusion.Design.cast(d.products.itemByProductType("DesignProductType"))

        def bodies(des):
            return sum(o.component.bRepBodies.count for o in des.rootComponent.allOccurrences) + des.rootComponent.bRepBodies.count

        def errors(des):
            return sorted(t.name for t in des.timeline if t.healthState == fusion.FeatureHealthStates.ErrorFeatureHealthState)

        parts = [p for p in geo["parts"] if p["build"]]
        with read.operation("cold open accepted taps-02 parent", parent.name, 1, 1):
            doc = opened(parent); des = des_of(doc); top = des.rootComponent
            if des.designType != fusion.DesignTypes.ParametricDesignType:
                raise ValueError("Parametric design required")
            if bodies(des) != bi["parent"]["bodyCount"]:
                raise ValueError("Predecessor roster differs")
            # M7: resolve every fixed chassis body by its own rooted occurrence path, never by an assumed single parent
            expected_paths = geo["chassisOccurrences"]["bodyPaths"]
            owner, counts = {}, {}
            for occ in top.allOccurrences:
                if occ.bRepBodies.count != 1 or occ.fullPathName.split("+")[0].startswith("Matched SDS"):
                    continue
                name = occ.bRepBodies.item(0).name
                if name in owner:
                    raise ValueError("Ambiguous chassis body name " + name)
                owner[name] = occ
                head = occ.fullPathName.split("+")[0]
                counts[head] = counts.get(head, 0) + 1
            if counts != bi["parent"]["expectedRootBodyCounts"]:
                raise ValueError(f"Chassis root split {counts} != {bi['parent']['expectedRootBodyCounts']}")
            moved = {n: (owner[n].fullPathName if n in owner else None) for n in expected_paths if n not in owner or owner[n].fullPathName != expected_paths[n]}
            if moved:
                raise ValueError("Chassis bodies not at their census occurrence paths: " + json.dumps(moved)[:400])
            roots = {}
            for head in counts:
                occ = top.occurrences.itemByName(head)
                if not occ or not (occ.isGrounded or occ.isGroundToParent):
                    raise ValueError("Ungrounded chassis root occurrence " + head)
                roots[head] = occ
            anchor_body = geo["joints"][1]["twoBody"]
            anchor = roots[owner[anchor_body].fullPathName.split("+")[0]]
            if any(top.occurrences.itemByName(g + ":1") for g in GROUPS):
                raise ValueError("Pan already installed")
            inherited = errors(des)
            r["checks"]["parent"] = dict(rootBodyCounts=counts, anchor=anchor.name, anchorBody=anchor_body,
                                         grounded={h: o.isGrounded for h, o in roots.items()}, inheritedTimelineErrors=inherited)

        with read.operation("materials", "design", 1, 1):
            def library(match):
                return next((m for lib in app.materialLibraries for m in lib.materials if match(m.name.lower().replace("-", ""))), None)
            al6061 = des.materials.itemByName(bi["materials"]["6061-T6"])
            if not al6061:
                src = library(lambda n: "6061" in n and "t6" in n)
                if not src:
                    raise ValueError("Required native 6061-T6 absent")
                al6061 = des.materials.addByCopy(src, bi["materials"]["6061-T6"])
            m5052 = des.materials.itemByName(bi["materials"]["5052-H32"])
            density_5052 = 2.68e-6
            if not m5052:
                src = library(lambda n: "5052" in n)
                if src:
                    m5052 = des.materials.addByCopy(src, bi["materials"]["5052-H32"])
                    r["checks"]["5052Source"] = src.name
                else:
                    # never relabel 6061 as 5052: use an explicitly named substitute and correct the expected mass
                    name = "002 6061-T6 standing in for 5052-H32 (no library 5052)"
                    m5052 = des.materials.itemByName(name) or des.materials.addByCopy(al6061, name)
                    density_5052 = 2.70e-6
                    r["findings"].append("5052-H32 absent from the material libraries: 5052 parts use '" + name + "' (+0.75 % mass on those parts)")
                    r["checks"]["5052Source"] = name
            mats = {"6061-T6": al6061, "5052-H32": m5052}
            density = {"6061-T6": 2.70e-6, "5052-H32": density_5052}

        groups, native, proxy, params = {}, {}, {}, []
        for gi, gname in enumerate(GROUPS, 1):
            with read.operation("create group component", gname, gi, len(GROUPS)):
                occ = top.occurrences.addNewComponent(core.Matrix3D.create()); occ.component.name = gname
                groups[gname] = occ
        for i, part in enumerate(parts, 1):
            with read.operation("author fabricated article", part["id"], i, len(parts)):
                g = groups[part["group"]]
                child = g.component.occurrences.addNewComponent(core.Matrix3D.create())
                child.component.name = part["id"]
                body = build_part(des, child.component, part, mats[part["material"]], params)
                child.isGroundToParent = True
                native[part["id"]] = child; proxy[part["id"]] = child.createForAssemblyContext(g)
                vol = body.getPhysicalProperties(ACCURACY).volume * 1000.0
                if abs(vol - part["volumeMm3"]) > volume_tolerance(part):
                    raise ValueError(f"Volume mismatch {part['id']}: {vol} vs {part['volumeMm3']}")
                pts = [[c * 10 for c in v.geometry.asArray()] for v in body.vertices]   # vertex extents are tight for these planar-bounded parts
                seen = [min(q[i] for q in pts) for i in range(3)] + [max(q[i] for q in pts) for i in range(3)]
                worst = max(abs(a - b) for a, b in zip(seen, part["boundsMm"]))
                if worst > 0.05:
                    raise ValueError(f"Bounds mismatch {part['id']}: {worst:.4f} mm (native {seen} vs {part['boundsMm']})")
                r["built"].append(dict(id=part["id"], path=proxy[part["id"]].fullPathName, volumeMm3=vol,
                                       geometryVolumeMm3=part["volumeMm3"], material=part["material"]))

        with read.operation("viewing appearances by palette role", "design", 1, 1):
            r["checks"]["appearance"] = {}
            for role, hexcolor in bi["palette"].items():
                try:
                    members = [p["id"] for p in parts if p["role"] == role]
                    name = "Red 2 role " + role
                    ap = des.appearances.itemByName(name) or des.appearances.addByCopy(native[members[0]].component.bRepBodies.item(0).appearance, name)
                    rgb = [int(hexcolor[k:k + 2], 16) for k in (1, 3, 5)]
                    colors = [c for c in (core.ColorProperty.cast(p) for p in ap.appearanceProperties) if c]
                    for c in colors:
                        c.value = core.Color.create(*rgb, 255)
                    for m in members:
                        native[m].component.bRepBodies.item(0).appearance = ap
                    r["checks"]["appearance"][role] = dict(hex=hexcolor, colorProperties=len(colors), bodies=len(members))
                except Exception as error:
                    r["checks"]["appearance"][role] = "not applied: " + repr(error)
                    r["findings"].append(f"Viewing appearance for role {role} not applied: {error!r}")

        with read.operation("seat datums for deferred purchased articles", "seats", 1, 1):
            for row in geo["deferredSeats"]:
                host = native.get(row["host"])
                seat(host.component if host else groups[row["host"]].component, row); r["seats"].append(row["name"])

        with read.operation("joints: rigid upper support, rigid inlet, revolute pan pivot", "joints", 1, 1):
            rj = top.asBuiltJoints.createInput(groups["Pan upper support"], anchor, None)
            rj.setAsRigidJointMotion()
            rigid = top.asBuiltJoints.add(rj); rigid.name = geo["joints"][1]["name"]
            ij = top.asBuiltJoints.createInput(groups["Fixed inlet"], groups["Pan upper support"], None)
            ij.setAsRigidJointMotion()
            inlet_joint = top.asBuiltJoints.add(ij); inlet_joint.name = geo["joints"][2]["name"]
            spec = geo["joints"][0]
            sk = top.sketches.add(top.xYConstructionPlane); sk.name = "Pan pivot axis datum"
            pivot = sk.sketchPoints.add(sk.modelToSketchSpace(pt(spec["pointMm"])))
            ji = top.asBuiltJoints.createInput(groups["Guarded pan"], groups["Pan upper support"], fusion.JointGeometry.createByPoint(pivot))
            ji.setAsRevoluteJointMotion(fusion.JointDirections.YAxisJointDirection)
            joint = top.asBuiltJoints.add(ji); joint.name = spec["name"]; sk.isLightBulbOn = False
            motion = fusion.RevoluteJointMotion.cast(joint.jointMotion)
            floor = proxy[bi["landmark"]["body"]].bRepBodies.item(0)

            def landmark_error(expected):
                adsk.doEvents()
                seen = [[c * 10 for c in v.geometry.asArray()] for v in floor.vertices]
                return max(min(math.dist(s, e) for s in seen) for e in expected)

            flat_err = landmark_error(bi["landmark"]["flatVerticesMm"])
            chosen = None
            for sgn in (1, -1):
                motion.rotationValue = math.radians(-sgn * geo["poses"]["stowDeg"])
                if landmark_error(bi["landmark"]["stowVerticesMm"]) < bi["landmark"]["toleranceMm"]:
                    chosen = sgn; break
            motion.rotationValue = 0.0
            if chosen is None or flat_err > bi["landmark"]["toleranceMm"]:
                raise ValueError("Pivot sign or floor landmark registration failed")
            phi = lambda theta: math.radians(-chosen * theta)
            lo, hi = sorted((phi(spec["thetaLimitsDeg"][0]), phi(spec["thetaLimitsDeg"][1])))
            lim = motion.rotationLimits
            lim.minimumValue = lo; lim.isMinimumValueEnabled = True
            lim.maximumValue = hi; lim.isMaximumValueEnabled = True
            lim.restValue = phi(spec["savedThetaDeg"]); lim.isRestValueEnabled = True
            r["joints"] = dict(rigid=dict(name=rigid.name, one=rigid.occurrenceOne.fullPathName, two=rigid.occurrenceTwo.fullPathName),
                               inlet=dict(name=inlet_joint.name, one=inlet_joint.occurrenceOne.fullPathName, two=inlet_joint.occurrenceTwo.fullPathName),
                               pivot=dict(name=joint.name, phiPerTheta=-chosen, minDeg=math.degrees(lo), maxDeg=math.degrees(hi),
                                          flatLandmarkErrorMm=flat_err))

        def chassis_body(name):
            occ = owner.get(name)
            return occ.bRepBodies.item(0) if occ else None

        def common_mm3(ba, bb):
            """Native shared volume of two bodies (interference analysis, coincident faces excluded). measureMinimumDistance
            reads 0 for both touching and overlapping bodies, so every zero-gap witness also needs this."""
            ents = core.ObjectCollection.create(); ents.add(ba); ents.add(bb)
            req = des.createInterferenceInput(ents)
            req.areCoincidentFacesIncluded = False
            found = des.analyzeInterference(req)
            return sum(found.item(i).interferenceBody.volume * 1000.0 for i in range(found.count))

        def gaps(label):
            rows = []
            for a, b in bi["nativeCheckPairs"]:
                ba = proxy[a].bRepBodies.item(0) if a in proxy else chassis_body(a)
                bb = proxy[b].bRepBodies.item(0) if b in proxy else chassis_body(b)
                if ba is None or bb is None:
                    rows.append(dict(pair=[a, b], pose=label, status="unresolved")); continue
                row = dict(pair=[a, b], pose=label, gapMm=app.measureManager.measureMinimumDistance(ba, bb).value * 10)
                if row["gapMm"] < bi["gapToleranceMm"]:
                    row["commonMm3"] = common_mm3(ba, bb)
                rows.append(row)
            return rows

        with read.operation("native gap and shared-volume witnesses at seven poses against the offline clearances", "pairs", 1, 1):
            r["checks"]["nativeGaps"] = []
            want = {(tuple(e["pair"]), e["pose"]): e for e in bi["expectedGaps"]}
            bad = []
            for row in bi["nativePoses"]:
                motion.rotationValue = phi(row["thetaDeg"]); adsk.doEvents()
                for g in gaps(row["label"]):
                    e = want.get((tuple(g["pair"]), g["pose"]))
                    g["expectedMm"] = e["gapMm"] if e else None
                    if "gapMm" not in g or e is None or abs(g["gapMm"] - e["gapMm"]) > bi["gapToleranceMm"]:
                        bad.append(g)
                    elif "commonMm3" in g:
                        # a zero gap must also agree on contact (no shared volume) versus overlap (shared volume)
                        g["expectedCommonMm3"] = e["commonMm3"]
                        if (g["commonMm3"] > COMMON_TOL_MM3) != (e["commonMm3"] > COMMON_TOL_MM3) or abs(g["commonMm3"] - e["commonMm3"]) > max(COMMON_TOL_MM3, 0.02 * e["commonMm3"]):
                            bad.append(g)
                    r["checks"]["nativeGaps"].append(g)
            r["checks"]["zeroGapWitnesses"] = dict(count=sum(1 for g in r["checks"]["nativeGaps"] if "commonMm3" in g),
                                                   overlapping=sum(1 for g in r["checks"]["nativeGaps"] if g.get("commonMm3", 0.0) > COMMON_TOL_MM3))
            if bad:
                raise ValueError(f"{len(bad)} native gap witnesses disagree with geometry.json: " + json.dumps(bad[:5])[:600])
            motion.rotationValue = phi(geo["poses"]["stowDeg"]); adsk.doEvents()
            if landmark_error(bi["landmark"]["stowVerticesMm"]) > bi["landmark"]["toleranceMm"]:
                raise ValueError("Saved stow pose not attained")

        with read.operation("save pan build", "design", 1, 1):
            if des.snapshots.hasPendingSnapshot:
                des.snapshots.add()           # capture the stow joint position so it persists in the archive
            if not des.computeAll():
                raise ValueError("Regeneration failed")
            new_errors = [e for e in errors(des) if e not in inherited]
            r["checks"]["newTimelineErrors"] = new_errors
            if new_errors:
                raise ValueError("New timeline errors after build")
            if not des.exportManager.execute(des.exportManager.createFusionArchiveExportOptions(str(f3d))):
                raise ValueError("Archive save failed")
            read.checkpoint(f3d)

        with read.operation("cold reopen, inventory and STEP export", "design", 1, 1):
            doc.close(False); doc = None
            NATIVE_JOB["verifyDependencies"]()
            doc = opened(f3d); des = des_of(doc); top = des.rootComponent
            expected = bi["parent"]["bodyCount"] + len(parts)
            if bodies(des) != expected:
                raise ValueError(f"Cold body count {bodies(des)} != {expected}")
            cold = {}
            for gname in GROUPS:
                g = top.occurrences.itemByName(gname + ":1")
                names = sorted(o.component.name for o in g.childOccurrences)
                if names != sorted(p["id"] for p in parts if p["group"] == gname):
                    raise ValueError("Cold roster differs in " + gname)
                cold[gname] = dict(articles=len(names), massKg=g.component.getPhysicalProperties(ACCURACY).mass,
                                   groundToParent=all(o.isGroundToParent for o in g.childOccurrences),
                                   unmaterialled=[o.component.name for o in g.childOccurrences if not o.bRepBodies.item(0).material])
            cold_fail = []
            moving = sum(p["volumeMm3"] * density[p["material"]] for p in parts if p["moving"])
            if abs(cold["Guarded pan"]["massKg"] - moving) > 0.002 * moving:
                cold_fail.append(f"cold moving mass {cold['Guarded pan']['massKg']:.4f} kg vs expected {moving:.4f} kg")
            for gname in GROUPS:
                if cold[gname]["unmaterialled"] or not cold[gname]["groundToParent"]:
                    cold_fail.append(f"{gname}: unmaterialled {cold[gname]['unmaterialled']}, groundToParent {cold[gname]['groundToParent']}")
            pj = next((j for j in top.asBuiltJoints if j.name == spec["name"]), None)
            if pj is None:
                raise ValueError("Pivot joint %r missing after cold reopen" % spec["name"])
            m = fusion.RevoluteJointMotion.cast(pj.jointMotion); lim = m.rotationLimits
            cold["pivot"] = dict(valueDeg=math.degrees(m.rotationValue), minDeg=math.degrees(lim.minimumValue), maxDeg=math.degrees(lim.maximumValue),
                                 minEnabled=lim.isMinimumValueEnabled, maxEnabled=lim.isMaximumValueEnabled,
                                 one=pj.occurrenceOne.fullPathName, two=pj.occurrenceTwo.fullPathName)
            if abs(cold["pivot"]["valueDeg"] + chosen * geo["poses"]["stowDeg"]) > 1e-6:
                cold_fail.append("saved stow pose not retained: %.6f deg" % cold["pivot"]["valueDeg"])
            want_lim = sorted((-chosen * spec["thetaLimitsDeg"][0], -chosen * spec["thetaLimitsDeg"][1]))
            if not (lim.isMinimumValueEnabled and lim.isMaximumValueEnabled) or max(abs(cold["pivot"]["minDeg"] - want_lim[0]), abs(cold["pivot"]["maxDeg"] - want_lim[1])) > 1e-6:
                cold_fail.append("pivot limits not retained")
            cold["rigid"] = sorted(j.name for j in top.asBuiltJoints if j.name in (geo["joints"][1]["name"], geo["joints"][2]["name"]))
            if len(cold["rigid"]) != 2:
                cold_fail.append("rigid support/inlet joints missing: %s" % cold["rigid"])
            cold["parameters"] = sum(1 for p in params if des.userParameters.itemByName(p))
            if cold["parameters"] != len(set(params)):
                cold_fail.append(f"user parameters {cold['parameters']} of {len(set(params))}")
            cold["seats"] = sorted(s.name for o in top.allOccurrences for s in o.component.sketches if s.attributes.itemByName(ATTR, "seat"))
            if cold["seats"] != sorted(row["name"] for row in geo["deferredSeats"]):
                cold_fail.append("seat sketches differ")
            cold["newTimelineErrors"] = [e for e in errors(des) if e not in inherited]
            if cold["newTimelineErrors"]:
                cold_fail.append("new timeline errors " + str(cold["newTimelineErrors"]))
            r["checks"]["cold"] = cold
            if cold_fail:
                r["checks"]["coldFailures"] = cold_fail
                raise ValueError("Cold reopen failed: " + "; ".join(cold_fail))
            if not des.exportManager.execute(des.exportManager.createSTEPExportOptions(str(step), top)):
                raise ValueError("Cold STEP export failed")
            # re-read the exported STEP and count its solids against the cold archive
            step_doc = app.importManager.importToNewDocument(app.importManager.createSTEPImportOptions(str(step)))
            try:
                step_bodies = bodies(des_of(step_doc))
            finally:
                step_doc.close(False)
            r["checks"]["stepReread"] = dict(solids=step_bodies, expected=expected)
            if step_bodies != expected:
                raise ValueError(f"Exported STEP holds {step_bodies} solids, cold archive {expected}")
            r.update(status="authored; purchased articles deferred to checked placement" + ("; with findings" if r["findings"] else ""),
                     nativeSha256=authority.digest(f3d), stepSha256=authority.digest(step), bodyCount=expected,
                     deferredPurchased=[s["name"] for s in geo["deferredSeats"]])
    except Exception:
        r.update(status="held", exception=traceback.format_exc())
    finally:
        if doc:
            doc.close(False)
        r["after"] = [d.creationId for d in app.documents]
        r["restored"] = r["after"] == start and app.activeDocument is None
        (out / "native-receipt.json").write_text(json.dumps(r, indent=1) + "\n")
    print(json.dumps({k: r.get(k) for k in ("status", "restored")}))
