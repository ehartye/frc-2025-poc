"""Check the fixed tower from the independently dimensioned shop plate, shaft and four screw axes."""
import json
import sys
from pathlib import Path

from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common, BRepAlgoAPI_Cut
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
from OCP.GeomAbs import GeomAbs_Cylinder
from OCP.TopAbs import TopAbs_FACE
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS
from OCP.gp import gp_Pnt

from output_mount import ROOT, digest, read
from pan_purchase_conservation import volume
from pan_thrust_layout import cylinder, subtract, translate


def run(plugin):
    sys.path.insert(0, str(plugin / "scripts"))
    from mechanical_motion import bounds, distance
    from showcase_contacts import load_solids

    d = ROOT / "artifacts/red2-integrated/pan-thrust"
    issue = read(d / "r3-cold-independent-stock.json")
    step = d / "native/integrated-340mm-pan-r3-machined.step"
    refs = issue["stocks"]["Starboard bearing tower"]
    assert digest(ROOT / issue["authority"]["file"]) == issue["authority"]["sha256"]
    assert digest(step) == issue["stepSha256"] and digest(ROOT / refs["source"]) == refs["sourceSha256"]
    source, = load_solids(ROOT / refs["source"])[0].values()
    parts = load_solids(step)[0]
    installed = parts["Starboard bearing tower"]
    support = parts["Piloted bilateral bearing support"]

    # The fixed plate, shaft and four registered fasteners all refer to chassis datums.
    yface = -198.15
    commissioned = [(205., 580., 14.2875)] + [(x, z, 1.5) for x in (184., 226.) for z in (570., 590.)]
    drawing = BRepPrimAPI_MakeBox(gp_Pnt(171, yface, 487), gp_Pnt(239, -191.85, 605)).Shape()
    for x, z, radius in commissioned:
        drawing = subtract(drawing, cylinder((x, yface-1, z), (0, 1, 0), radius, 8.3))
    def detail(s):
        ex = TopExp_Explorer(s, TopAbs_FACE)
        faces = []
        count = 0
        while ex.More():
            face = TopoDS.Face(ex.Current())
            surf = BRepAdaptor_Surface(face)
            if surf.GetType() == GeomAbs_Cylinder:
                c = surf.Cylinder()
                faces.append(dict(radius=c.Radius(), boundsMm=bounds(face), axis=list(c.Axis().Direction().Coord())))
            count += 1
            ex.Next()
        return dict(boundsMm=bounds(s), volumeMm3=volume(s), valid=BRepCheck_Analyzer(s).IsValid(),
                    faceCount=count, cylinders=faces)
    def comparison(left, right):
        def measure(op, a, b):
            result = op(a,b)
            if not result.IsDone() or result.Shape().IsNull():
                raise ValueError("Unresolved tower Boolean")
            return volume(result.Shape())
        common = measure(BRepAlgoAPI_Common,left,right)
        absent = measure(BRepAlgoAPI_Cut,left,right)
        extra = measure(BRepAlgoAPI_Cut,right,left)
        return dict(commonMm3=common,absentMm3=absent,extraMm3=extra,gapMm=distance(left,right),
                    leftConservationResidualMm3=volume(left)-common-absent,
                    rightConservationResidualMm3=volume(right)-common-extra)
    output = dict(authority=issue["authority"], archiveSha256=issue["nativeSha256"], stepSha256=digest(step),
                  inputNeutralSha256=refs["sourceSha256"], installedOccurrence=refs["installedPath"],
                  commissionedShaftAndScrewAxesXZradiusMm=commissioned, outwardYFaceMm=yface,
                  cold=detail(installed), source=detail(source), drawing=detail(drawing), support=detail(support))
    output["comparisons"] = dict(sourceToCold=comparison(source,installed),drawingToCold=comparison(drawing,installed),
                                 drawingToSource=comparison(drawing,source),selfCold=comparison(installed,installed),
                                 selfSource=comparison(source,source),
                                 knownDisplaced1mmX=comparison(drawing,translate(drawing,[1.,0.,0.])))
    target = d / "r3-starboard-tower-registration.json"
    target.write_text(json.dumps(output,indent=2)+"\n")
    for k in ["cold","source","drawing","support"]:
        print(k, output[k]["boundsMm"], output[k]["valid"], output[k]["volumeMm3"],flush=True)
    print(json.dumps(output["comparisons"],indent=2),flush=True)


if __name__ == "__main__":
    run(Path(sys.argv[1]))
