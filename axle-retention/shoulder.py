"""Check positive axial stops against the nominal center-bearing envelope."""
import argparse
import json
from pathlib import Path

from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut, BRepAlgoAPI_Fuse
from measure import along_y, cad, distance, read_shop, SHOP_STEP


def size(artifacts):
    shop,_ = read_shop(artifacts)
    result = {}
    for side,sign in (("L",1),("R",-1)):
        bearing = shop[f"bearing-center-outer-{side}"]
        shaft = shop[f"axle-center-{side}"]
        old_spacer = shop[f"spacer-center-3-{side}"]
        shoulder = along_y(0,sign*316,sign,8.35,1.26)
        new_shaft = BRepAlgoAPI_Fuse(shaft,shoulder).Shape()
        # Cut the original spacer back to 315.9 to preserve running clearance.
        trim = along_y(0,sign*315.9,sign,10,3)
        new_spacer = BRepAlgoAPI_Cut(old_spacer,trim).Shape()
        # Shim rides around the shaft and touches the underside of its seated head.
        outer = along_y(0,sign*327,-sign,8.35,1.55)
        bore = along_y(0,sign*328,-sign,7,3)
        ring = BRepAlgoAPI_Cut(outer,bore).Shape()
        def overlap(shape,delta):
            return cad.intersection(cad.translated(shape,[0,sign*delta,0]),bearing)
        row = {"shaftShoulderToShimmedSpacerMm":distance(new_shaft,new_spacer),
               "shoulderToBearingEnvelopeMm":distance(shoulder,bearing),
               "outerShimToBearingEnvelopeMm":distance(ring,bearing),
               "outwardOverlapAt025Mm3":overlap(shoulder,.25),
               "inwardOverlapAt020Mm3":overlap(ring,-.20),
               "outwardOverlapAt010Mm3":overlap(shoulder,.10),
               "inwardOverlapAt005Mm3":overlap(ring,-.05)}
        if min(row["outwardOverlapAt025Mm3"],row["inwardOverlapAt020Mm3"])<6:
            raise ValueError("Proposed stop did not engage")
        result[side]=row
    return {"shopSha256":SHOP_STEP,"centerStops":result,
            "limit":"The native bearing ring has one material envelope. Measure the real inner race and shield before seating these faces."}


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("artifacts",type=Path)
    parser.add_argument("output",type=Path)
    args=parser.parse_args()
    args.output.write_text(json.dumps(size(args.artifacts),indent=2)+"\n",encoding="utf-8",newline="\n")
