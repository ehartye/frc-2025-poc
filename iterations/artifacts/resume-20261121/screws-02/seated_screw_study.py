"""Offline OCP evidence for natively installing four genuine 91251A542 screws in the modeled
1/4-20 nutbar threads of four-tapped-holes.step. Read-only on all inputs; writes one new JSON.

usage: python -I -B seated_screw_study.py PLUGIN_ROOT DESIGN_ROOT TAPS_DIR OUT_JSON
"""
import hashlib, json, math, subprocess, sys, time
from pathlib import Path
import numpy as np

SRC = "Screw, .25-20 X 1 Socket, 91251A542_91251A542 [1.1.1.1.1.20.5.1]"
NAZ = 360                      # 1 degree azimuth grid -> 1 degree phase resolution
BAND = (2.5591, 3.175)         # female minor radius .. shared major radius: the only radii where both solids hold material
NR = 24                        # midpoints never coincide with a modeled cylinder (coincident radii give tangent-hit artifacts)
GUARD = [2.40, 2.50, 3.20, 3.30, 3.60]   # outside-band radii that must show zero common
CTRL_BAND, CTRL_NR = (1.90, 3.90), 21
PITCH = 1.27
ZONE = (173.9, 184.4)  # axial window (mm) bracketing the 174.15..184.15 nutbar threads; odd-parity rays are classified only here
import os as _os
if _os.environ.get("SCREW_SMOKE"): NAZ, NR, CTRL_NR = 24, 3, 5  # smoke test only


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def main(pkg, design, taps, out):
    out = Path(out)
    if out.exists():
        raise SystemExit("Refusing to overwrite " + str(out))
    commit = subprocess.run(["git", "-C", str(pkg), "rev-parse", "HEAD"], capture_output=True, text=True).stdout.strip()
    sys.path.insert(0, str(Path(pkg) / "scripts"))
    import OCP
    from showcase_contacts import load_solids, box_gap
    from mechanical_motion import bounds, common, distance, bearing_area
    from component_shape import volume
    from OCP.gp import gp_Trsf, gp_Ax1, gp_Pnt, gp_Dir, gp_Lin, gp_Vec
    from OCP.TopLoc import TopLoc_Location
    from OCP.IntCurvesFace import IntCurvesFace_ShapeIntersector
    from OCP.BRepClass3d import BRepClass3d_SolidClassifier
    from OCP.TopAbs import TopAbs_IN, TopAbs_FACE, TopAbs_REVERSED, TopAbs_SOLID
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopoDS import TopoDS
    from OCP.BRepAdaptor import BRepAdaptor_Surface
    from OCP.GeomAbs import GeomAbs_Cylinder, GeomAbs_Plane, GeomAbs_BSplineSurface
    from OCP.BRepGProp import BRepGProp
    from OCP.GProp import GProp_GProps
    from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
    from OCP.BRepCheck import BRepCheck_Analyzer
    try:
        from OCP.TopTools import TopTools_ListOfShape
    except ImportError:
        from OCP.collections import List_TopoDS_Shape as TopTools_ListOfShape
    Face = getattr(TopoDS, "Face_s", None) or TopoDS.Face
    t_start = time.time()
    design = Path(design); taps = Path(taps)
    step = taps / "four-tapped-holes.step"
    sel = json.loads((design / "chassis/inset-clamp-hardware-selection.json").read_bytes())
    inputs = {"taps-02/four-tapped-holes.step": sha(step), "taps-02/four-tapped-holes.f3d": sha(taps / "four-tapped-holes.f3d"),
              "taps-02/native-receipt.json": sha(taps / "native-receipt.json")}
    for n in ["inset-clamp-hardware-selection.json", "inset-clamp-length-comparison.json", "inset-brace-clamp-window.json"]:
        inputs["iterations/002/design/chassis/" + n] = sha(design / "chassis" / n)
    if inputs["taps-02/four-tapped-holes.step"] != "166c5567fb38a6c9c446589c10b96ec7046bac9beb19673f2696c2b22eb2efdd": raise ValueError("STEP digest")
    if inputs["taps-02/four-tapped-holes.f3d"] != "e426e5768a25ab6038f69f5b43b6ab5ed80ad9aca997610c3217117731fa5471": raise ValueError("F3D digest")
    for n in ("inset-clamp-length-comparison.json", "inset-brace-clamp-window.json"):
        if sha(design / "chassis" / n) != sel["inputSHA256"][n]: raise ValueError("Selection input changed: " + n)
    receipt = json.loads((taps / "native-receipt.json").read_bytes())
    if receipt["stepSha256"] != inputs["taps-02/four-tapped-holes.step"]: raise ValueError("Receipt/STEP mismatch")
    ids = {}
    solids, sheets = load_solids(step, ids)
    if sheets: raise ValueError("Open shells in STEP")
    bb = {k: bounds(v) for k, v in solids.items()}
    src = solids[SRC]

    def faces(sh):
        it = TopExp_Explorer(sh, TopAbs_FACE)
        while it.More():
            f = Face(it.Current()); yield f, BRepAdaptor_Surface(f); it.Next()

    def area(f):
        g = GProp_GProps(); BRepGProp.SurfaceProperties_s(f, g); return g.Mass()

    def outward(f, s):
        n = np.array(s.Plane().Axis().Direction().Coord())
        return -n if f.Orientation() == TopAbs_REVERSED else n

    # ---------- 1. source screw identity and modeling ----------
    cyl = []; bspl = []; planes = []
    for f, s in faces(src):
        b = bounds(f)
        if s.GetType() == GeomAbs_Cylinder:
            c = s.Cylinder(); cyl.append(dict(r=c.Radius(), d=np.array(c.Axis().Direction().Coord()), o=np.array(c.Location().Coord()), z=(b[2], b[5]), a=area(f)))
        elif s.GetType() == GeomAbs_BSplineSurface: bspl.append(dict(boundsMm=b, areaMm2=area(f)))
        elif s.GetType() == GeomAbs_Plane: planes.append(dict(f=f, n=outward(f, s), b=b, a=area(f), z=s.Plane().Location().Z()))
    seat_given = np.array(sel["selectedRows"][0]["sourceSeatMm"])
    major = sorted({round(c["r"], 4) for c in cyl if c["a"] < 5 and c["r"] > 3})
    minor = sorted({round(c["r"], 4) for c in cyl if c["a"] < 5 and c["r"] < 2.5})
    head = max(cyl, key=lambda c: c["r"])
    under = [p for p in planes if abs(p["b"][2] - p["b"][5]) < 1e-6 and p["n"][2] > .999999 and abs(p["b"][2] - seat_given[2]) < .01]
    if len(under) != 1: raise ValueError("Under-head face not unique")
    under = under[0]
    axis_src = head["d"] / np.linalg.norm(head["d"])
    if axis_src[2] < .999999: raise ValueError("Source axis not +Z")
    cen = head["o"]; seat_meas = np.array([cen[0], cen[1], under["z"]])  # exact plane location, not tolerance-padded bounds
    tipz = max(p["z"] for p in planes if p["n"][2] > .999999); full_major_end = max(c["z"][1] for c in cyl if abs(c["r"] - 3.175) < 1e-3)
    thread_cyl_axes = [float(np.linalg.norm(c["o"][:2] - cen[:2])) for c in cyl]
    source = dict(solid=SRC, identity=ids[SRC], volumeMm3=volume(src), boundsMm=bb[SRC],
        sourceSeatMeasuredMm=seat_meas.tolist(), sourceSeatGivenMm=seat_given.tolist(),
        seatDeviationMm=float(np.linalg.norm(seat_meas - seat_given)),
        intoThreadAxis=axis_src.tolist(), maxCylinderAxisOffsetMm=max(thread_cyl_axes),
        headRadiusMm=head["r"], underHeadPlanarFaceAreaMm2=under["a"],
        threadModel="helical: two BSpline flank faces spanning the shank plus segmented coaxial crest (r3.175) and root (r2.350) cylinders, one per 1.27 mm pitch",
        helicalFlankFaces=bspl, majorDiameterMm=[2 * r for r in major], minorDiameterMm=[2 * r for r in minor],
        underHeadLengthMm=tipz - seat_meas[2], fullMajorThreadFromSeatMm=[0.0, full_major_end - seat_meas[2]],
        tipChamferAxialMm=tipz - full_major_end,
        note="Modeled thread is a basic profile (male major 6.350 = female major 6.350); not 2A/2B tolerance-class geometry.")
    print("source", source["seatDeviationMm"], major, minor, source["underHeadLengthMm"], flush=True)

    def host_thread(name, xy):
        rs = []
        for f, s in faces(solids[name]):
            if s.GetType() == GeomAbs_Cylinder:
                c = s.Cylinder(); o = np.array(c.Location().Coord())
                if abs(o[0] - xy[0]) < 1e-3 and abs(o[1] - xy[1]) < 1e-3:
                    b = bounds(f); rs.append((round(c.Radius(), 4), b[2], b[5]))
        hel = 0
        for f, s in faces(solids[name]):
            if s.GetType() == GeomAbs_BSplineSurface:
                b = bounds(f)
                if abs((b[0] + b[3]) / 2 - xy[0]) < .01 and abs((b[1] + b[4]) / 2 - xy[1]) < .01: hel += 1
        radii = sorted({r for r, _, _ in rs})
        return dict(cylinderRadiiMm=radii, helicalFlankFaces=hel, threadSpanZmm=[min(a for _, a, _ in rs), max(b for _, _, b in rs)],
                    minorDiameterMm=2 * radii[0], majorDiameterMm=2 * radii[-1])

    # ---------- 2. transforms ----------
    Rx = np.diag([1., -1., -1.])

    def trsf(M):
        t = gp_Trsf(); t.SetValues(*M[:3, :].ravel()); return t

    def leaf_location():
        from OCP.STEPCAFControl import STEPCAFControl_Reader
        from OCP.TCollection import TCollection_ExtendedString
        from OCP.TDocStd import TDocStd_Document
        from OCP.TDF import TDF_Label
        from OCP.XCAFDoc import XCAFDoc_DocumentTool
        try:
            from OCP.TDF import TDF_LabelSequence
        except ImportError:
            from OCP.collections import Sequence_TDF_Label as TDF_LabelSequence
        doc = TDocStd_Document(TCollection_ExtendedString("BinXCAF")); tab = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
        r = STEPCAFControl_Reader(); r.ReadFile(str(step)); r.Transfer(doc)
        roots = TDF_LabelSequence(); tab.GetFreeShapes(roots)
        path = [int(x) for x in ids[SRC]["instance"].split(".")][:-1]
        lab = roots.Value(path[0]); loc = TopLoc_Location()
        for i in path[1:]:
            ch = TDF_LabelSequence(); tab.GetComponents_s(lab, ch); c = ch.Value(i); d = TDF_Label()
            tab.GetReferredShape_s(c, d); loc = loc.Multiplied(tab.GetLocation_s(c)); lab = d
        loc = loc.Multiplied(tab.GetShape_s(lab).Location())
        t = loc.Transformation(); M = np.eye(4)
        for i in range(3):
            for j in range(4): M[i, j] = t.Value(i + 1, j + 1)
        return M
    P_src = leaf_location()

    rows = sel["selectedRows"]
    if len(rows) != 4 or any(r["source"] != SRC or r["sourceToTargetRotation"] != "180 about world X" or r["axis"] != [0, 0, -1] for r in rows):
        raise ValueError("Selection rows changed")
    if any(float(np.linalg.norm(np.array(r["sourceSeatMm"]) - seat_given)) > 1e-9 for r in rows): raise ValueError("Seat rows differ")
    cm = lambda M: [float(v) for v in np.concatenate([np.column_stack([M[:3, :3], M[:3, 3] / 10]).ravel(), [0, 0, 0, 1]])]
    screws = []
    for r in rows:
        U = np.array(r["underHead"], float)
        T = np.eye(4); T[:3, :3] = Rx; T[:3, 3] = U - Rx @ seat_meas
        placed = src.Moved(TopLoc_Location(trsf(T)))
        seat_check = (T @ np.append(seat_meas, 1))[:3]; ax_check = Rx @ axis_src
        uh = [f for f, s in faces(placed) if s.GetType() == GeomAbs_Plane and outward(f, s)[2] < -.999999
              and abs(bounds(f)[2] - U[2]) < 1e-6 and abs(area(f) - under["a"]) < 1e-6]
        hc = [s.Cylinder() for f, s in faces(placed) if s.GetType() == GeomAbs_Cylinder and abs(s.Cylinder().Radius() - head["r"]) < 1e-6][0]
        hco = np.array(hc.Location().Coord())
        Wt = T @ P_src
        label = "%d,%d" % (U[0], U[1])
        screws.append(dict(label=label, host=r["tappingHost"], underHead=U.tolist(), placed=placed, uhface=uh[0],
            transform=dict(
                composition="T = translate(underHead) * Rx(180 deg) * translate(-sourceSeat); target STEP occurrence = T * source STEP occurrence. Row-major 4x4; *Cm forms are Fusion Matrix3D.asArray() order with translation in cm.",
                deltaWorldMm=[float(v) for v in T.ravel()], deltaWorldCm=cm(T),
                sourceStepOccurrenceWorldMm=[float(v) for v in P_src.ravel()],
                targetStepOccurrenceWorldMm=[float(v) for v in Wt.ravel()], targetStepOccurrenceWorldCm=cm(Wt),
                seatImageMm=seat_check.tolist(), seatErrorMm=float(np.linalg.norm(seat_check - U)), axisImage=ax_check.tolist(),
                remeasuredFromPlacedGeometry=dict(underHeadFaces=len(uh), underHeadFaceZmm=bounds(uh[0])[2],
                    headAxisXYmm=hco[:2].tolist(), headAxisXYErrorMm=float(np.linalg.norm(hco[:2] - U[:2])),
                    headCylinderAxisDir=list(hc.Axis().Direction().Coord())),
                properRotation=bool(abs(np.linalg.det(T[:3, :3]) - 1) < 1e-12))))
        print("placed", label, screws[-1]["transform"]["seatErrorMm"], flush=True)

    # ---------- ray sampling: exact ray/face hits on the real solids; odd-parity rays resolved by the solid classifier ----------
    from OCP.BRepExtrema import BRepExtrema_DistShapeShape
    from interface_consistency import inscribed_balls

    def intervals(x, y, z0, z1, eng, cls, zone=None):
        ZONE_ = zone or ZONE
        eng.Perform(gp_Lin(gp_Pnt(x, y, z0), gp_Dir(0, 0, 1)), -1e-6, z1 - z0 + 1e-6)
        hs = sorted(set(round(eng.WParameter(i), 9) for i in range(1, eng.NbPnt() + 1)))
        if len(hs) % 2 == 0:
            return [(z0 + hs[i], z0 + hs[i + 1]) for i in range(0, len(hs), 2) if hs[i + 1] - hs[i] > 1e-9], False
        lim = [0.0] + [h for h in hs if 1e-9 < h < z1 - z0 - 1e-9] + [z1 - z0]; iv = []
        for a, b in zip(lim, lim[1:]):
            if b - a < 1e-9: continue
            if z0 + b < ZONE_[0] or z0 + a > ZONE_[1]: continue  # only the nutbar zone can hold a male/female common
            cls.Perform(gp_Pnt(x, y, z0 + (a + b) / 2), 1e-7)
            if cls.State() == TopAbs_IN:
                if iv and abs(iv[-1][1] - (z0 + a)) < 1e-9: iv[-1] = (iv[-1][0], z0 + b)
                else: iv.append((z0 + a, z0 + b))
        return iv, True

    def sample(shape, center, radii, z0, z1, verify=4):
        eng = IntCurvesFace_ShapeIntersector(); eng.Load(shape, 1e-7)
        cls = BRepClass3d_SolidClassifier(shape); grid = []; resolved = 0; bad = 0; checks = 0; rng = np.random.default_rng(7)
        for r in radii:
            row = []
            for k in range(NAZ):
                th = 2 * math.pi * k / NAZ
                # azimuth right-handed about the into-host axis d=(0,0,-1): u=+X, v=d x u=-Y
                x, y = center[0] + r * math.cos(th), center[1] - r * math.sin(th)
                iv, odd = intervals(x, y, z0, z1, eng, cls); resolved += odd
                if verify and not odd and rng.random() < verify / NAZ:
                    for a, b in iv[:4]:
                        cls.Perform(gp_Pnt(x, y, (a + b) / 2), 1e-7); checks += 1
                        if cls.State() != TopAbs_IN: bad += 1
                row.append(iv)
            grid.append(row)
        return grid, dict(rays=len(radii) * NAZ, oddParityResolvedByClassifier=resolved, classifierSpotChecks=checks, spotCheckFailures=bad)

    def ovl(A, B):
        out = []; i = j = 0
        while i < len(A) and j < len(B):
            lo = max(A[i][0], B[j][0]); hi = min(A[i][1], B[j][1])
            if hi > lo: out.append((lo, hi))
            if A[i][1] < B[j][1]: i += 1
            else: j += 1
        return out

    def vol(M, F, radii, dr, offset=0, dz=0.0):
        """Common volume with the male grid rotated by `offset` azimuth steps and shifted axially by dz."""
        v = 0.; dth = 2 * math.pi / NAZ
        for j, r in enumerate(radii):
            for k in range(NAZ):
                A = M[j][(k - offset) % NAZ]
                if dz: A = [(a + dz, b + dz) for a, b in A]
                v += sum(b - a for a, b in ovl(A, F[j][k])) * r * dth * dr
        return v

    def radii_of(band, n):
        dr = (band[1] - band[0]) / n; rs = [band[0] + dr * (i + .5) for i in range(n)]
        if any(abs(r - c) < 1e-3 for r in rs for c in (2.3501, 2.5591, 3.175)): raise ValueError("Sample radius on a modeled cylinder")
        return rs, dr

    def boolean(a, b):
        res = {}
        for fuzzy in (0.0, 1e-5):
            op = BRepAlgoAPI_Common(); la = TopTools_ListOfShape(); la.Append(a); lb = TopTools_ListOfShape(); lb.Append(b)
            op.SetArguments(la); op.SetTools(lb)
            if fuzzy: op.SetFuzzyValue(fuzzy)
            op.Build()
            key = "fuzzy=%g" % fuzzy
            if not op.IsDone() or op.Shape().IsNull(): res[key] = dict(done=False); continue
            sh = op.Shape(); n = 0; it = TopExp_Explorer(sh, TopAbs_SOLID)
            while it.More(): n += 1; it.Next()
            res[key] = dict(done=True, volumeMm3=volume(sh), solids=n, valid=BRepCheck_Analyzer(sh).IsValid())
        return res

    def inbox(p, b, tol=.01):
        return all(b[i] - tol <= p[i] <= b[i + 3] + tol for i in range(3))

    def rdist(a, b):
        """BRepExtrema distance whose witness points must lie on both shapes' boxes; otherwise face-by-face."""
        def one(x, y):
            q = BRepExtrema_DistShapeShape(x, y)
            if not q.IsDone(): return None
            bx, by = bounds(x), bounds(y); v = q.Value(); good = 0
            for i in range(1, q.NbSolution() + 1):
                p1 = q.PointOnShape1(i).Coord(); p2 = q.PointOnShape2(i).Coord()
                good += inbox(p1, bx) and inbox(p2, by) and abs(math.dist(p1, p2) - v) < 1e-6
            return v if good else None  # a value is accepted only with at least one genuine witness pair
        v = one(a, b)
        if v is not None: return v, "shape"
        vals = []
        for f, _ in faces(a):
            if box_gap(bounds(f), bounds(b)) > 6: continue
            w = one(f, b)
            if w is not None: vals.append(w)
        if not vals: raise ValueError("Unresolved face distance")
        return min(vals), "facewise-after-invalid-shape-witness"

    def _safe_common(x, y):
        try: return common(x, y)
        except ValueError as e: return "unresolved: " + str(e)

    def rotate(shape, U, deg):
        t = gp_Trsf(); t.SetRotation(gp_Ax1(gp_Pnt(*U), gp_Dir(0, 0, -1)), math.radians(deg)); return shape.Moved(TopLoc_Location(t))

    def floor_top(U, tip):
        crossm = solids["Pan crossmember 250"]; eng = IntCurvesFace_ShapeIntersector(); eng.Load(crossm, 1e-7)
        cls = BRepClass3d_SolidClassifier(crossm); tops = []
        for r in (0, 1, 2, 3, 3.175):
            for k in range(12):
                th = 2 * math.pi * k / 12
                iv = intervals(U[0] + r * math.cos(th), U[1] + r * math.sin(th), 140, tip, eng, cls, (140, tip))[0]
                tops.append(max((b for a, b in iv), default=None))
        return max(t for t in tops if t is not None), sum(t is None for t in tops)

    def neighbors(shape, host):
        b0 = bounds(shape); out_rows = []
        for k, b in bb.items():
            if k == host or box_gap(b0, b) > 6: continue
            g, how = rdist(shape, solids[k])
            def safe(x, y):
                try: return common(x, y)
                except ValueError: return None  # unresolved OCP Boolean: recorded as open, never as zero
            v = [safe(shape, solids[k]), safe(solids[k], shape)] if g < 1e-3 else [0., 0.]
            out_rows.append(dict(part=k, gapMm=g, distanceMethod=how, commonMm3=v))
        return sorted(out_rows, key=lambda r: r["gapMm"])

    def underface(sh, z):
        return [f for f, sf in faces(sh) if sf.GetType() == GeomAbs_Plane and abs(sf.Plane().Location().Z() - z) < 1e-6
                and abs(bounds(f)[5] - bounds(f)[2]) < 1e-5 and abs(area(f) - under["a"]) < 1e-6][0]

    def seat(shape, uhface, landing):
        g, how = rdist(uhface, solids[landing])
        top = [sf.Plane().Location().Z() for f, sf in faces(solids[landing]) if sf.GetType() == GeomAbs_Plane
               and abs(sf.Plane().Axis().Direction().Z()) > .999999 and abs(bounds(f)[2] - bb[landing][5]) < 1e-3][0]
        return dict(bearingBody=landing, underHeadFaceToBearingGapMm=g, distanceMethod=how,
                    landingTopPlaneZmm=top, planarContactAreaMm2=bearing_area(shape, solids[landing], [0, 0, -1]),
                    headToLandingCommonMm3=common(shape, solids[landing]))

    def witnesses(shape, host, M, F, radii, U, n=16):
        probes = []
        for j, r in enumerate(radii):
            for k in range(0, NAZ, 3):
                th = 2 * math.pi * k / NAZ
                for lo, hi in ovl(M[j][k], F[j][k]):
                    probes.append((hi - lo, [float(U[0] + r * math.cos(th)), float(U[1] - r * math.sin(th)), float((lo + hi) / 2)]))
        probes = [p for _, p in sorted(probes, reverse=True)][:n]
        balls, v = inscribed_balls(shape, host, probes, 1e-6) if probes else ([], 0.)
        return dict(probes=len(probes), balls=len(balls), interiorWitnessMm3=v,
                    largestRadiusMm=max((b["radiusMm"] for b in balls), default=0.), largest=sorted(balls, key=lambda b: -b["radiusMm"])[:2])

    radii, dr = radii_of(BAND, NR)
    Z0, Z1 = 165.0, 215.0
    results = []
    other_new = {s["label"]: s["placed"] for s in screws}
    best_by = {}
    # Bounded parallel execution: SCREW_ONLY=i runs one screw; SCREW_CONTROLS_PHASE=deg runs only the controls on screw 0.
    import os
    only = os.environ.get("SCREW_ONLY"); cphase = os.environ.get("SCREW_CONTROLS_PHASE")
    todo = [] if cphase else [(i, x) for i, x in enumerate(screws) if only is None or i == int(only)]
    if cphase: best_by[screws[0]["label"]] = float(cphase)
    for si, s in todo:
        U = np.array(s["underHead"]); host = s["host"]; landing = host.replace("Captive truss landing nutbar", "Truss landing")
        placed = s["placed"]; t0 = time.time()
        M, mq = sample(placed, U, radii, Z0, Z1)
        F, fq = sample(solids[host], U, radii, 170.0, 190.0)
        curve = [vol(M, F, radii, dr, o) for o in range(NAZ)]
        lo = int(np.argmin(curve)); hi = int(np.argmax(curve))
        # sub-degree refinement: in the nutbar zone a rotation by d deg equals an axial shift of sign*PITCH*d/360 (helix);
        # the sign is calibrated against the exact index-shift curve and the refined optimum is confirmed by direct resampling.
        step = 360 / NAZ
        cal = {sg: max(abs(vol(M, F, radii, dr, lo, sg * PITCH * d * step / 360) - curve[(lo + d) % NAZ]) for d in (-2, -1, 1, 2)) for sg in (1, -1)}
        sg = min(cal, key=cal.get)
        deltas = [round(-1.5 + .05 * i, 2) for i in range(61)]
        fine = [vol(M, F, radii, dr, lo, sg * PITCH * d / 360) for d in deltas]
        i = int(np.argmin(fine)); phi = round((lo * step + deltas[i]) % 360, 2); best_by[s["label"]] = phi
        inst = rotate(placed, U, phi)
        Md, mdq = sample(inst, U, radii, Z0, Z1)
        v_direct = vol(Md, F, radii, dr)
        Gm, _ = sample(inst, U, GUARD, Z0, Z1, verify=0); Gf, _ = sample(solids[host], U, GUARD, 170.0, 190.0, verify=0)
        guard = [sum(b - a for k in range(NAZ) for a, b in ovl(Gm[j][k], Gf[j][k])) for j in range(len(GUARD))]
        mx = rotate(placed, U, hi * step); Mx, _ = sample(mx, U, radii, Z0, Z1, verify=0)
        wit = dict(atInstallPhase=witnesses(inst, solids[host], Md, F, radii, U), atMaxPhase=witnesses(mx, solids[host], Mx, F, radii, U))
        conv = None
        if si == 0 and not os.environ.get("SCREW_SKIP_CONV"):
            r2, dr2 = radii_of(BAND, 2 * NR)
            F2, _ = sample(solids[host], U, r2, 170.0, 190.0, verify=0); M2, _ = sample(inst, U, r2, Z0, Z1, verify=0); X2, _ = sample(mx, U, r2, Z0, Z1, verify=0)
            conv = dict(radii=2 * NR, atInstallPhaseMm3=vol(M2, F2, r2, dr2), atMaxPhaseMm3=vol(X2, F2, r2, dr2))
        cross = {}
        for label, deg, sh in (("nominal0", 0, rotate(placed, U, 0)), ("install", phi, inst), ("max", hi * step, mx)):
            cross[label] = dict(phaseDeg=deg, boolean=boolean(sh, solids[host]), helperCommonMm3=_safe_common(sh, solids[host]))
        print(" phase", s["label"], "nom", curve[0], "lo", lo, curve[lo], "phi", phi, fine[i], v_direct, "max", hi, curve[hi], "guard", guard, "wit", wit["atInstallPhase"]["interiorWitnessMm3"], wit["atMaxPhase"]["interiorWitnessMm3"], "conv", conv, flush=True)
        ht = host_thread(host, U[:2])
        tipz = bounds(inst)[2]; ftop, miss = floor_top(U, tipz)
        tipz = U[2] - source["underHeadLengthMm"]
        eng = dict(maleFullMajorZmm=[U[2] - source["fullMajorThreadFromSeatMm"][1], U[2]], femaleThreadZmm=ht["threadSpanZmm"])
        eng["engagedFullThreadMm"] = max(0., min(eng["maleFullMajorZmm"][1], eng["femaleThreadZmm"][1]) - max(eng["maleFullMajorZmm"][0], eng["femaleThreadZmm"][0]))
        eng["engagedPitches"] = eng["engagedFullThreadMm"] / PITCH
        eng["tipBelowNutbarMm"] = ht["threadSpanZmm"][0] - tipz
        nb = neighbors(inst, host)
        nb_new = [dict(part="new screw " + k, gapMm=rdist(inst, other_new[k])[0]) for k in other_new if k != s["label"]]
        res = dict(label=s["label"], host=host, transform=s["transform"], hostThread=ht, rayQuality=dict(male=mq, female=fq, maleAtInstall=mdq),
            phase=dict(convention="degrees right-handed about the into-host axis (0,0,-1) through underHead; 0 = the selection transform as given",
                       method="exact ray/face intervals on both real solids, %d radii in (%.4f, %.4f) mm x %d azimuths; z-overlap integrated r dr dtheta; rotation = azimuth index shift" % (NR, BAND[0], BAND[1], NAZ),
                       commonMm3ByDeg=curve, nominalMm3=curve[0], coarseMinMm3=curve[lo], coarseMinAtDeg=lo * step, maxMm3=curve[hi], maxAtDeg=hi * step,
                       refinement=dict(helixSign=sg, calibrationMaxErrorMm3=cal, offsetsFromCoarseMinDeg=deltas, commonMm3=fine),
                       installPhaseDeg=phi, installPhaseHelixMm3=fine[i], installPhaseDirectMm3=v_direct,
                       guardRadiiMm=GUARD, guardOverlapLengthSumMm=guard, radialConvergence=conv,
                       interiorWitnesses=wit, booleanCrossChecks=cross),
            atInstallPhase=dict(seat=seat(inst, underface(inst, U[2]), landing),
                tip=dict(tipZmm=tipz, beamFloorTopZmm=ftop, floorRaysMissing=miss, tipToFloorMm=tipz - ftop, floorBody="Pan crossmember 250"),
                engagement=eng, neighbors=nb, newScrewNeighbors=nb_new))
        results.append(res)
        print("screw", s["label"], res["atInstallPhase"]["seat"], [(n["part"], round(n["gapMm"], 4), n["distanceMethod"], n["commonMm3"]) for n in nb][:5], round(time.time() - t0), flush=True)

    # ---------- controls on the first screw, displaced from its install phase ----------
    controls = []
    if cphase or only is None:
        s = screws[0]; U = np.array(s["underHead"]); host = s["host"]; landing = host.replace("Captive truss landing nutbar", "Truss landing")
        base = rotate(s["placed"], U, best_by[s["label"]])
        cradii, cdr = radii_of(CTRL_BAND, CTRL_NR)
        Fc, _ = sample(solids[host], U, cradii, 170.0, 190.0, verify=0)
        for name, d in [("as installed (baseline, wide band)", [0, 0, 0]), ("raised 0.5 mm", [0, 0, .5]), ("radial 0.5 mm (+X)", [.5, 0, 0]),
                        ("raised 0.5 mm and radial 0.5 mm (+X)", [.5, 0, .5])]:
            t = gp_Trsf(); t.SetTranslation(gp_Vec(*d)); sh = base.Moved(TopLoc_Location(t))
            Mc, _ = sample(sh, U, cradii, Z0, Z1, verify=0)
            cc = [vol(Mc, Fc, cradii, cdr, o) for o in range(0, NAZ, 5)]
            row = dict(screw=s["label"], case=name, offsetMm=d, radialBandMm=list(CTRL_BAND), radii=CTRL_NR,
                       commonAtInstallPhaseMm3=cc[0], minOverPhasesMm3=min(cc), minAtExtraRotationDeg=5 * int(np.argmin(cc)), maxOverPhasesMm3=max(cc),
                       seat=seat(sh, underface(sh, U[2] + d[2]), landing),
                       booleanAtInstallPhase=boolean(sh, solids[host]),
                       overlappingNeighbors=[n for n in neighbors(sh, host) if any(c is None or c > 1e-6 for c in n["commonMm3"])])
            controls.append(row)
            print("control", name, cc[0], min(cc), row["seat"], [(n["part"], n["commonMm3"]) for n in row["overlappingNeighbors"]], flush=True)

    doc = dict(schema="red2/seated-screw-study@1", created=time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        scope="Offline OCP geometry for natively installing four genuine McMaster 91251A542 1/4-20 x 1 in SHCS in the modeled 1/4-20 UNC-2B nutbar threads of the taps-02 derivative. No Fusion session was used.",
        plugin=dict(repo="agent-frc", commit=commit, helpers=["showcase_contacts.load_solids", "showcase_contacts.box_gap", "mechanical_motion.bounds/common/bearing_area",
                                                              "component_shape.volume", "interface_consistency.inscribed_balls"]),
        runtime=dict(python=sys.version.split()[0], ocp=getattr(OCP, "__version__", "unknown"), numpy=np.__version__),
        inputsSha256=inputs, stepSolidCount=len(solids), source=source, screws=results, controls=controls)
    doc["runtime"]["seconds"] = time.time() - t_start
    out.write_text(json.dumps(doc, indent=1, default=float) + "\n")
    print("done", doc["runtime"]["seconds"], flush=True)


if __name__ == "__main__":
    main(*sys.argv[1:5])
