"""Isolated underside cut on the moving plate; compare both single-line controls."""
import argparse
import json
import math
from pathlib import Path
from compare import cad, contact, digest, shared


def relieved_plate(shape, side, height):
    # 24 mm along Y. Four millimeters of depth retains 8 mm of the 12 mm plate.
    y0 = 210 if side == 1 else -234
    cutter = cad.brep(["box",[-79.5,y0,height-161,6.5,24,5]])
    cut = cad.BRepAlgoAPI_Cut(shape,cutter)
    if not cut.IsDone() or cad.solid_count(cut.Shape()) != 1:
        raise ValueError("Pocket produced an invalid carriage plate")
    return cut.Shape()


def review(folder):
    path = folder/"place.step"
    sha = digest(path)
    if sha != json.loads((folder/"review.json").read_bytes())["place"]["export_sha256"]:
        raise ValueError("Unexpected native placement bytes")
    frames = json.loads((folder/"part-frames.json").read_bytes())
    bodies = cad.read_named(path,[{"id":k} for k in frames])
    fixed = {k:v for k,v in bodies.items() if frames[k]=="base"}
    bounds = {k:cad.bounds(v) for k,v in fixed.items()}
    results = {}
    for tag,side in (("L",1),("R",-1)):
        original = bodies[f"backup-{tag}-carriage-plate"]
        revised = relieved_plate(original, side, 560)
        axis = cad.gp_Ax1(cad.gp_Pnt(0,-side*252,400),cad.gp_Dir(1,0,0))
        cases = []
        for drop in (0,5,5.679,6.568,8,12):
            tf = cad.gp_Trsf()
            tf.SetRotation(axis, -side*math.asin(drop/504))
            for source, plate in (("original", original),("relieved",revised)):
                rotated = cad.BRepBuilderAPI_Transform(plate,tf,False).Shape()
                for xshift in ((0,) if source=="original" else (-.5,0,.5)):
                    moved = cad.translated(rotated,(xshift,0,0))
                    hits = contact(moved, fixed, bounds)
                    cases.append({"plate":source,"dropMm":drop,"xShiftMm":xshift,"fixedContactMm3":hits})
        results[tag]={"removedMm3":cad.volume(original)[0]-cad.volume(revised)[0],"cases":cases}
    return {"source":sha,"plateThicknessMm":12,"reliefDepthMm":4,"remainingRoofMm":8,
            "m6HoleEdgeToPocketMm":87-79.5-3.3,"casesBySide":results,
            "releasedForMachining":False}


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("folder",type=Path)
    p.add_argument("receipt",type=Path)
    args=p.parse_args()
    args.receipt.write_text(json.dumps(review(args.folder),indent=2)+"\n",encoding="utf-8",newline="\n")
