"""Build the project-owned structural coupons and one continuous backed rear bumper."""

import argparse
import hashlib
import json
import math
import sys
from pathlib import Path

from shape import facet, prism, volume
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
from OCP.BRepCheck import BRepCheck_Analyzer

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT / "cad/assembly-engineering"))
from attachments import base, common, write_step

sys.path.insert(0, str(ROOT / "experiments/six-wheel/design/bumper-kit"))
import bumper


def outline(edge, intercept, seam, low, high):
    tip = intercept - edge
    return prism(
        [
            (seam, -edge),
            (tip, -edge),
            (edge, -tip),
            (edge, tip),
            (tip, edge),
            (seam, edge),
        ],
        1,
        low,
        high,
    )


def u_section(cut, inner, outer, seam, low, high):
    def side(edge, inset, bottom, top):
        intercept = 700 - cut + (edge - bumper.FRAME) * math.sqrt(2)
        return outline(edge, intercept, inset, bottom, top)

    return BRepAlgoAPI_Cut(
        side(outer, seam, low, high), side(inner, seam - 2, low - 1, high + 1)
    ).Shape()


def produce(output):
    cut = 19.05
    parts = {
        f"rear-{name}-100x20x2-miter": facet(cut, side)
        for side, name in [(1, "left"), (-1, "right")]
    }
    old = bumper.specifications("red")
    for part, inner, outer in [("backing", 351, 355), ("foam", 355, 424.5)]:
        key = "bumper-" + part + "-rear"
        unbored = bumper.cad.brep(bumper.u_shape(-1, inner, outer, 43, 181))
        original = bumper.cad.brep(old[key][1])
        source_bores = BRepAlgoAPI_Cut(unbored, original).Shape()
        revised = u_section(cut, inner, outer, 1, 43, 181)
        parts[key] = BRepAlgoAPI_Cut(revised, source_bores).Shape()
    outside = u_section(cut, 355, 425, 0.5, 42.5, 181.5)
    inside = u_section(cut, 354.9, 424.5, 1, 43, 181)
    parts["bumper-cover-rear"] = BRepAlgoAPI_Cut(outside, inside).Shape()
    output.mkdir(exist_ok=True, parents=True)
    path = output / "rear-facet-construction.step"
    write_step(parts, path)
    cold = base.named(path)
    if any(not BRepCheck_Analyzer(s).IsValid() for s in cold.values()):
        raise ValueError("Invalid cold BRep")
    if any(len(list(base.subsolids(s))) != 1 for s in cold.values()):
        raise ValueError("A continuous part separated into pieces")
    interferences = {
        a + " / " + b: common(cold[a], cold[b])
        for i, a in enumerate(cold)
        for b in list(cold)[i + 1 :]
    }
    assert max(interferences.values()) < 0.001
    face = 700 - cut + 75 * math.sqrt(2)
    result = dict(
        parentArchive="a5d4c627f7266414270b7f9389c9396b25ce50fd1ca429f64646b1469e77a065",
        stepSha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        frameSetbackMm=cut,
        frameFacetLengthMm=cut * math.sqrt(2),
        backerHeightMm=138,
        backerNormalMm=4,
        foamNormalMm=69.5,
        coverNormalMm=0.5,
        frameToBackerGapMm=1,
        tubeOutsideSectionMm=[100, 20, 2],
        outsideDiagonalInterceptMm=face,
        atOuterSideFacetStartsXmm=-(face - 425),
        retainedRightSideNumberClearanceMm=(face - 425) - 335,
        rearAnchorBoreCount=4,
        unchangedFrontAnchorCount=4,
        existingRearRailEndAbsYmm=325,
        newRearOuterMiterAbsYmm=350 - cut,
        minimumExtraRawCrossRailReachEachEndMm=350 - cut - 325,
        coldPartVolumesMm3={k: volume(v) for k, v in cold.items()},
        pairInterferenceMm3=interferences,
        scope="Reference blanks for a new chassis and one rear U. Rail ends, miter welds, peripheral contact strips, backing fabrication and operating evidence require a new complete assembly.",
    )
    (output / "construction.json").write_bytes(
        (json.dumps(result, indent=2) + "\n").encode()
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    produce(parser.parse_args().output)
