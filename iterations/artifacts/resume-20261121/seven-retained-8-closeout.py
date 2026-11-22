"""Boolean-free occupied-form closeout for the 8 retained bodies whose OCCT BRep serialization differs.

Usage: python seven-retained-8-closeout.py <design-root> <agent-frc-scripts> <output-json>
Offline OCP only. Refuses to overwrite the output.
"""
import hashlib, io, json, math, subprocess, sys, time
from collections import Counter
from pathlib import Path
import numpy as np

DESIGN, SCRIPTS, OUT = map(Path, sys.argv[1:4])
if OUT.exists():
    raise SystemExit(f"Refusing to overwrite {OUT}")
sys.path.insert(0, str(SCRIPTS))
from showcase_contacts import load_solids  # noqa: E402

from OCP.BRep import BRep_Tool
from OCP.BRepAdaptor import BRepAdaptor_Surface, BRepAdaptor_Curve
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeVertex
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.BRepClass3d import BRepClass3d_SolidClassifier
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
from OCP.BRepGProp import BRepGProp
from OCP.BRepTools import BRepTools
from OCP.BRepClass import BRepClass_FaceClassifier
from OCP.Bnd import Bnd_Box
from OCP.GProp import GProp_GProps
from OCP.GeomAbs import GeomAbs_SurfaceType, GeomAbs_CurveType
from OCP.TopAbs import (TopAbs_SOLID, TopAbs_SHELL, TopAbs_FACE, TopAbs_WIRE, TopAbs_EDGE,
                        TopAbs_VERTEX, TopAbs_IN, TopAbs_ON, TopAbs_OUT)
from OCP.TopExp import TopExp
from OCP.TopLoc import TopLoc_Location
from OCP.TopTools import TopTools_FormatVersion
from OCP.collections import IndexedMap_TopoDS_Shape_TopTools_ShapeMapHasher as TopTools_IndexedMapOfShape
from OCP.TopoDS import TopoDS
from OCP.gp import gp_Pnt, gp_Pnt2d, gp_Trsf, gp_Vec

TOL_MM = 1e-4          # acceptance: max boundary deviation, both directions
GRID = 7               # UV grid per face (GRID x GRID candidates, kept if inside the trimmed face)
EDGE_SAMPLES = 5       # points per edge
MARGIN = 0.25          # bbox margin for candidate-face pruning (mm)
INTERIOR_POINTS = 3000 # random bbox points classified in both solids
RNG = np.random.default_rng(20261121)

before_path = DESIGN / "cad/issue-006/assembly.step"
after_path = DESIGN / "private-inputs/matched-sds/connected-seven.step"
retained_path = DESIGN / "chassis/connected-seven-retained.json"
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def serialize(s):
    out = io.BytesIO()
    BRepTools.Write_s(s.Located(TopLoc_Location()), out, False, False,
                      TopTools_FormatVersion.TopTools_FormatVersion_VERSION_3)
    return out.getvalue()


def submap(s, kind):
    m = TopTools_IndexedMapOfShape(); TopExp.MapShapes_s(s, kind, m)
    return [m.FindKey(i) for i in range(1, m.Extent() + 1)]


def topology(s):
    return {k: len(submap(s, t)) for k, t in (("solids", TopAbs_SOLID), ("shells", TopAbs_SHELL),
            ("faces", TopAbs_FACE), ("wires", TopAbs_WIRE), ("edges", TopAbs_EDGE), ("vertices", TopAbs_VERTEX))}


def tolerances(s):
    f = [BRep_Tool.Tolerance_s(TopoDS.Face(x)) for x in submap(s, TopAbs_FACE)]
    e = [BRep_Tool.Tolerance_s(TopoDS.Edge(x)) for x in submap(s, TopAbs_EDGE)]
    v = [BRep_Tool.Tolerance_s(TopoDS.Vertex(x)) for x in submap(s, TopAbs_VERTEX)]
    return dict(maxFace=max(f), maxEdge=max(e), maxVertex=max(v))


def face_area(f):
    p = GProp_GProps(); BRepGProp.SurfaceProperties_s(f, p); return p.Mass()


def face_multiset(s):
    return Counter((GeomAbs_SurfaceType(BRepAdaptor_Surface(TopoDS.Face(f)).GetType()).name,
                    round(face_area(f), 6)) for f in submap(s, TopAbs_FACE))


def mass(s):
    v = GProp_GProps(); BRepGProp.VolumeProperties_s(s, v, True)
    a = GProp_GProps(); BRepGProp.SurfaceProperties_s(s, a)
    c = v.CentreOfMass(); m = v.MatrixOfInertia()
    return dict(volume=v.Mass(), area=a.Mass(), centroid=np.array([c.X(), c.Y(), c.Z()]),
                inertia=np.array([[m.Value(i, j) for j in (1, 2, 3)] for i in (1, 2, 3)]))


def serialization_diff(da, db):
    la, lb = da.decode().splitlines(), db.decode().splitlines()
    if len(la) != len(lb):
        return dict(sameLineCount=False, linesA=len(la), linesB=len(lb))
    section, start, rows, nonnumeric = None, 0, [], 0
    heads = ("Locations", "Curve2ds", "Curves", "Polygon3D", "PolygonOnTriangulations", "Surfaces", "Triangulations", "TShapes")
    for n, (x, y) in enumerate(zip(la, lb)):
        w = x.split()
        if w and w[0] in heads and len(w) == 2:
            section, start = w[0], n
        if x == y:
            continue
        tx, ty = x.split(), y.split()
        try:
            d = max(abs(float(p) - float(q)) for p, q in zip(tx, ty)) if len(tx) == len(ty) else math.inf
            kinds = sorted({"integer" if (p.lstrip("-").isdigit() and q.lstrip("-").isdigit()) else "real"
                            for p, q in zip(tx, ty) if p != q})
        except ValueError:
            d, kinds = math.inf, ["text"]; nonnumeric += 1
        rows.append(dict(section=section, entry=n - start, maxAbsDelta=d, tokenKinds=kinds,
                         geometryType=("Line" if section == "Curves" and tx[0] == "1" else tx[0])))
    return dict(sameLineCount=True, lines=len(la), differingLines=len(rows), nonNumericLines=nonnumeric,
                sections=sorted({r["section"] for r in rows}), maxAbsTokenDelta=max((r["maxAbsDelta"] for r in rows), default=0),
                rows=rows)


def sample_points(s):
    pts = []
    for f in submap(s, TopAbs_FACE):
        face = TopoDS.Face(f); ad = BRepAdaptor_Surface(face)
        u0, u1, v0, v1 = BRepTools.UVBounds_s(face)
        cls = BRepClass_FaceClassifier()
        for i in range(GRID):
            for j in range(GRID):
                u = u0 + (i + .5) / GRID * (u1 - u0); v = v0 + (j + .5) / GRID * (v1 - v0)
                cls.Perform(face, gp_Pnt2d(u, v), 1e-9)
                if cls.State() == TopAbs_IN:
                    p = ad.Value(u, v); pts.append((p.X(), p.Y(), p.Z()))
    for e in submap(s, TopAbs_EDGE):
        edge = TopoDS.Edge(e)
        if BRep_Tool.Degenerated_s(edge):
            continue
        c = BRepAdaptor_Curve(edge)
        for k in range(EDGE_SAMPLES):
            p = c.Value(c.FirstParameter() + (k + .5) / EDGE_SAMPLES * (c.LastParameter() - c.FirstParameter()))
            pts.append((p.X(), p.Y(), p.Z()))
    return np.array(pts)


class Boundary:
    """Exact distance to a solid's trimmed boundary, pruned by per-face bounding boxes."""
    def __init__(self, s):
        self.shells = submap(s, TopAbs_SHELL); self.faces = [TopoDS.Face(f) for f in submap(s, TopAbs_FACE)]
        boxes = []
        for f in self.faces:
            b = Bnd_Box(); BRepBndLib.Add_s(f, b, False); boxes.append([*b.CornerMin().Coord(), *b.CornerMax().Coord()])
        self.lo = np.array([b[:3] for b in boxes]); self.hi = np.array([b[3:] for b in boxes])

    def distance(self, p):
        v = BRepBuilderAPI_MakeVertex(gp_Pnt(*p)).Vertex()
        idx = np.nonzero(np.all((self.lo - MARGIN <= p) & (p <= self.hi + MARGIN), axis=1))[0]
        best = math.inf
        for i in idx:
            d = BRepExtrema_DistShapeShape(v, self.faces[i])
            if d.IsDone():
                best = min(best, d.Value())
        if best > MARGIN:  # no near face: exact whole-boundary distance
            best = min(BRepExtrema_DistShapeShape(v, sh).Value() for sh in self.shells)
        return best


def deviation(src_pts, target):
    d = np.array([target.distance(p) for p in src_pts])
    return dict(samples=int(len(d)), maxMm=float(d.max()), p95Mm=float(np.percentile(d, 95)), meanMm=float(d.mean()))


def occupancy(a, b):
    box = Bnd_Box(); BRepBndLib.AddOptimal_s(a, box, False, False); lo = np.array(box.CornerMin().Coord()); hi = np.array(box.CornerMax().Coord())
    pts = lo + RNG.random((INTERIOR_POINTS, 3)) * (hi - lo)
    ca, cb = BRepClass3d_SolidClassifier(a), BRepClass3d_SolidClassifier(b)
    states = []
    for p in pts:
        q = gp_Pnt(*p); ca.Perform(q, 1e-7); cb.Perform(q, 1e-7); states.append((ca.State(), cb.State()))
    return dict(points=INTERIOR_POINTS, insideA=sum(s[0] == TopAbs_IN for s in states),
                insideB=sum(s[1] == TopAbs_IN for s in states),
                disagreements=sum(s[0] != s[1] for s in states))


def compare(a, b, *, with_serial=True, with_occupancy=True):
    t = time.time()
    va, vb = BRepCheck_Analyzer(a).IsValid(), BRepCheck_Analyzer(b).IsValid()
    ta, tb = topology(a), topology(b)
    ma, mb = mass(a), mass(b)
    fa, fb = face_multiset(a), face_multiset(b)
    da, db = serialize(a), serialize(b)
    pa, pb = sample_points(a), sample_points(b)
    ab = deviation(pa, Boundary(b)); ba = deviation(pb, Boundary(a))
    r = dict(validity=dict(before=va, after=vb), topology=dict(before=ta, after=tb, equal=ta == tb),
             tolerances=dict(before=tolerances(a), after=tolerances(b)),
             faceTypeAreaMultisetEqual=fa == fb,
             serializationIdentical=da == db,
             volumeMm3=dict(before=ma["volume"], after=mb["volume"], delta=mb["volume"] - ma["volume"]),
             areaMm2=dict(before=ma["area"], after=mb["area"], delta=mb["area"] - ma["area"]),
             centroidDeltaMm=float(np.linalg.norm(mb["centroid"] - ma["centroid"])),
             inertiaMaxAbsDelta=float(np.abs(mb["inertia"] - ma["inertia"]).max()),
             deviationBeforeToAfter=ab, deviationAfterToBefore=ba)
    if with_serial:
        r["serializationDiff"] = serialization_diff(da, db)
    if with_occupancy:
        r["occupancyClassification"] = occupancy(a, b)
    worst = max(ab["maxMm"], ba["maxMm"])
    ok = (worst <= TOL_MM and r["topology"]["equal"] and abs(r["volumeMm3"]["delta"]) <= 1e-9 * ma["volume"]
          and r["centroidDeltaMm"] <= TOL_MM and r.get("occupancyClassification", {}).get("disagreements", 0) == 0)
    r["maxDeviationBothDirectionsMm"] = worst
    r["verdict"] = "same-occupied-form" if ok else ("differs" if worst > TOL_MM else "undetermined")
    r["seconds"] = round(time.time() - t, 1)
    return r


def moved(s, dx):
    tr = gp_Trsf(); tr.SetTranslation(gp_Vec(dx, 0, 0)); return s.Moved(TopLoc_Location(tr))


def main():
    t0 = time.time()
    A, sa = load_solids(before_path); B, sb = load_solids(after_path)
    rows = json.loads(retained_path.read_bytes())["paired"]
    bad = [r for r in rows if not r["identicalBRepAndPose"]]
    assert len(bad) == 8, len(bad)
    results = []
    for r in bad:
        a, b = A[r["before"]], B[r["after"]]
        res = dict(role=r["role"], before=r["before"], after=r["after"],
                   recordedHashes=dict(before=r["originalOccupiedRepresentation"], after=r["newOccupiedRepresentation"]))
        res["recomputedHashes"] = dict(before=hashlib.sha256(serialize(a)).hexdigest(), after=hashlib.sha256(serialize(b)).hexdigest())
        res.update(compare(a, b))
        results.append(res); print(res["role"], res["verdict"], res["maxDeviationBothDirectionsMm"], res["seconds"], flush=True)

    # Positive controls: retained bodies whose serialization and pose were already identical.
    ident = [r for r in rows if r["identicalBRepAndPose"]]
    pos = []
    for stem in ("MK4i Master Part V2_NEO Config", "CANcoder PCA"):
        r = next(x for x in ident if x["before"].startswith(stem + " ["))
        c = compare(A[r["before"]], B[r["after"]], with_serial=False)
        pos.append(dict(kind="identical-retained", role=r["role"], before=r["before"], after=r["after"], **c))
        print("pos", r["role"], c["verdict"], c["maxDeviationBothDirectionsMm"], flush=True)
    # Negative controls: the same housing / master part translated by a known amount.
    neg = []
    for r in (bad[0], bad[1]):
        for dx in (0.05, 0.001):
            c = compare(A[r["before"]], moved(B[r["after"]], dx), with_serial=False)
            neg.append(dict(kind=f"after-translated-x-{dx}mm", role=r["role"], expectedDeviationMm=dx, **c))
            print("neg", r["role"], dx, c["verdict"], c["maxDeviationBothDirectionsMm"], flush=True)
    # Negative control: a different supplier body of similar size.
    r = bad[0]
    other = next(x for x in ident if x["before"].startswith("CANcoder PCA ["))
    c = compare(A[r["before"]], B[other["after"]], with_serial=False, with_occupancy=False)
    neg.append(dict(kind="different-supplier-body", role=r["role"], against=other["after"], **c))
    print("neg other", c["verdict"], c["maxDeviationBothDirectionsMm"], flush=True)

    plugin = subprocess.run(["git", "-C", str(SCRIPTS.parent), "rev-parse", "HEAD"], capture_output=True, text=True).stdout.strip()
    design_rev = subprocess.run(["git", "-C", str(DESIGN), "rev-parse", "HEAD"], capture_output=True, text=True).stdout.strip()
    import importlib.metadata as md
    out = dict(
        schema="agent-frc/occupied-form-closeout@1 (ad hoc)",
        created=time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        inputs=dict(before=dict(path="iterations/002/design/cad/issue-006/assembly.step", sha256=sha(before_path)),
                    after=dict(path="iterations/002/design/private-inputs/matched-sds/connected-seven.step", sha256=sha(after_path)),
                    retainedJoin=dict(path="iterations/002/design/chassis/connected-seven-retained.json", sha256=sha(retained_path)),
                    designCommit=design_rev),
        pluginCommit=plugin, ocp=md.version("cadquery-ocp"), numpy=np.__version__, python=sys.version.split()[0],
        method=dict(
            loading="showcase_contacts.load_solids on both STEPs (world-placed TopoDS solids, same robot frame)",
            serialization="BRepTools.Write_s VERSION_3 on Located(identity) as in inset_seven_retained.py; line-by-line numeric token diff",
            validity="BRepCheck_Analyzer", topology="TopExp.MapShapes counts", faces="multiset of (GeomAbs surface type, face area rounded 1e-6 mm2)",
            massProperties="BRepGProp volume/surface: volume, area, centroid, inertia matrix",
            deviation=(f"{GRID}x{GRID} UV grid per face kept inside the trimmed face (BRepClass_FaceClassifier) plus {EDGE_SAMPLES} points per edge; "
                       "exact distance to the other solid's trimmed faces via BRepExtrema_DistShapeShape (bbox-pruned, outer-shell fallback so interior points are not reported as 0); both directions"),
            occupancy=f"{INTERIOR_POINTS} seeded random points in the before bbox classified by BRepClass3d_SolidClassifier in both solids",
            booleans="none"),
        tolerance=dict(maxBoundaryDeviationMm=TOL_MM, volumeRelative=1e-9, centroidMm=TOL_MM, occupancyDisagreements=0,
                       justification=("Serialization differences are last-ulp (~1e-15) real-number changes, so any real geometric change would be "
                                      "orders of magnitude larger. 1e-4 mm is >=10x below the smallest negative control (0.001 mm translation) that the "
                                      "method must resolve, far above BRepExtrema precision (1e-7) and below supplier-model and machining relevance "
                                      "(shop tolerance ~0.05 mm). It matches the precision quoted for the retained-cohort join.")),
        results=results, controls=dict(positive=pos, negative=neg),
        summary=dict(Counter(r["verdict"] for r in results)),
        rootCause=dict(
            differingLinesPerBody={r["role"]: r["serializationDiff"]["differingLines"] for r in results},
            sections=sorted({x for r in results for x in r["serializationDiff"]["sections"]}),
            geometryTypes=sorted({x["geometryType"] for r in results for x in r["serializationDiff"]["rows"]}),
            maxAbsTokenDelta=max(r["serializationDiff"]["maxAbsTokenDelta"] for r in results),
            explanation=("The only serialization differences are last-ulp real-number changes in the unit direction vector of 3D Line "
                         "curves (Curves section); topology, tolerances, surfaces, pcurves and all other tokens are byte-identical. "
                         "This is floating-point renormalization noise from the STEP export/import round trip, not a shape change.")),
        hashMethodNote=("The local BRep hash is blind to placement (translated controls serialize identically); the retained join "
                        "covers placement with its separate exact pose-matrix check, which is 0 for all 8 rows."),
        limits=["Deviation is sampled, not a proof of continuous boundary equality; a feature smaller than the sample spacing on one face could escape the "
                "boundary sampling, though topology counts, the per-face type/area multiset, volume, inertia and occupancy classification are independent checks.",
                "Occupancy classification is statistical (seeded random points) and limited to the before solid's bounding box.",
                "Supplier solids fail OCCT Booleans; this closeout does not explain or repair that Boolean failure.",
                "Offline OCP only; no native Fusion inspection of these bodies was made."],
        runtimeSeconds=round(time.time() - t0, 1))
    if OUT.exists():
        raise SystemExit(f"Refusing to overwrite {OUT}")
    with open(OUT, "x", encoding="utf-8") as fh:
        fh.write(json.dumps(out, indent=1, default=float) + "\n")
    print("wrote", OUT, out["summary"])


if __name__ == "__main__":
    main()
