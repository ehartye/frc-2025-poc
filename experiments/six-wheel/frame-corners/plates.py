"""Evaluate a pair of external cap plates at each primary chassis corner."""
import argparse
import csv
import json
import math
from pathlib import Path

from geometry import cad, cut, distance, join, nearest, prism, read_shop
from geometry import digest, finalize_step, write_step, SHOP_STEP

THICKNESS = 3.175
BORE = 4.925
HEAD_RADIUS = 0.385 * 25.4 / 2
HEAD_HEIGHT = 0.090 * 25.4
OUTLINE = ((0.75, 3), (3, 0.75), (121, 0.75), (121, 24),
           (52, 24), (19, 57), (19, 143), (0.75, 143))
SIDE_X = (240, 264, 288, 312, 336)
END_Y = (218, 242, 266, 290, 314)


def vertical(x, y, a, b, r):
    return cad.brep(["cyl", [x, y, a, 0, 0, 1 if b > a else -1, r, abs(b-a)]])


def evaluate(artifacts, output):
    original, _ = read_shop(artifacts)
    installed = dict(original)
    joints = {}
    rows = []
    receipt = {}
    deck = "deck-main"
    rails = {"cross-front", "cross-rear", "drive-rail-outer-L", "drive-rail-outer-R"}
    for sx, end in ((1, "front"), (-1, "rear")):
        for sy, side in ((1, "L"), (-1, "R")):
            tag = f"{end}-{side}"
            # Open relief in the deck. A 6 mm dogbone clears the inside tool radius.
            relief = ((-5,-5), (21,-5), (21,145), (-5,145))
            def blank(outline, low, high):
                return prism(sx, sy, [(u-20,v-25) for u,v in outline], low, high)
            notch = join(blank(relief, 149, 155), vertical(sx*329,sy*205,149,155,3))
            installed[deck] = cut(installed[deck], notch)
            members = {}
            access = []
            for level, surface, sign in (("top",150,1), ("bottom",50,-1)):
                outside = surface + sign*THICKNESS
                plate = blank(OUTLINE,min(surface,outside),max(surface,outside))
                drills = ([(sx*x, sy*337.5, "drive-rail-outer-"+side) for x in SIDE_X]
                          + [(sx*340, sy*y, "cross-"+end) for y in END_Y])
                for index,(x,y,rail) in enumerate(drills,1):
                    bore = vertical(x,y,outside+sign,surface-3*sign,BORE/2)
                    before = cad.volume(installed[rail])[0]
                    installed[rail] = cut(installed[rail],bore)
                    removed = before-cad.volume(installed[rail])[0]
                    if not math.isclose(removed, math.pi*(BORE/2)**2*2, abs_tol=.002):
                        raise ValueError(f"Drill {rail} at {x}, {y} removed {removed} cubic mm")
                    plate = cut(plate,bore)
                    key = f"corner-rivet-{tag}-{level}-{index:02d}"
                    tip = outside+sign*HEAD_HEIGHT
                    head = vertical(x,y,outside,tip,HEAD_RADIUS)
                    stem = vertical(x,y,outside,surface-2*sign,.192*25.4/2)
                    bulb = vertical(x,y,surface-2*sign,surface-8.5*sign,5.2)
                    members[key] = join(join(head,stem),bulb)
                    access.append((key,vertical(x,y,tip,tip+44.45*sign,9.525),
                                   vertical(x,y,tip+44.45*sign,tip+210*sign,27),
                                   vertical(x,y,surface-2.01*sign,surface-(2+.415*25.4)*sign,5.2)))
                    rows.append((tag,level,index,rail,x,y,surface,BORE))
                members[f"corner-plate-{tag}-{level}"] = plate
            joints.update(members)
            # The working wheel translates continuously between measured samples.
            # Translation by at most 0.25 mm bounds error everywhere in the interval.
            wheel = original["wheel-"+tag]
            near_wheel = math.inf
            for step in range(-8,9):
                tire = cad.translated(wheel,[sx*step/2,0,0])
                near_wheel=min(near_wheel,nearest(tire,members,{k:cad.bounds(v) for k,v in members.items()})["mm"])
            others = {k:v for k,v in installed.items() if k not in rails}
            boxes = {k:cad.bounds(v) for k,v in others.items()}
            gaps = [(nearest(v,others,boxes),k) for k,v in members.items()]
            nearest_part, member = min(gaps,key=lambda entry:entry[0]["mm"])
            tool_parts = {k:installed[k] for k in rails}
            tool_parts.update(members)
            tb = {k:cad.bounds(v) for k,v in tool_parts.items()}
            top = members[f"corner-plate-{tag}-top"]
            receipt[tag] = {
                "wheelContinuousLowerMm": near_wheel-.251,
                "nonrailClearanceMm": nearest_part["mm"], "nonrailWitness":[member,nearest_part["bodies"]],
                "outerAdjustmentWheelOverlapMm3": sum(cad.intersection(cad.translated(wheel,[sx*4,0,0]),v)
                                                      for v in members.values()),
                "originalDeckInterferenceMm3": cad.intersection(top,original[deck]),
                "relievedDeckClearanceMm": distance(top,installed[deck]),
                "bareFrameToolGapMm": min(nearest(tool,tool_parts,tb,{key})["mm"]
                                          for key,nose,body,_ in access for tool in (nose,body)),
                "blindReserveGapMm": min(nearest(blind,tool_parts,tb,{key})["mm"]
                                         for key,_,_,blind in access)}
    output.mkdir(exist_ok=True,parents=True)
    changed = rails | {deck}
    package = {k:installed[k] for k in sorted(changed)}
    package.update(joints)
    exchange = output/"frame-plates.step"
    write_step(exchange,package)
    finalize_step(exchange)
    reopened = cad.read_named(exchange,[{"id":k} for k in package])
    errors = [abs(cad.volume(v)[0]-cad.volume(reopened[k])[0]) for k,v in package.items()]
    if max(errors)>.001:
        raise ValueError("Reopened volumes disagree with the design")
    with (output/"drill-table.csv").open("w",newline="",encoding="utf-8") as stream:
        writer = csv.writer(stream,lineterminator="\n")
        writer.writerow(("corner","level","index","rail","x_mm","y_mm","surface_z_mm","diameter_mm"))
        writer.writerows(rows)
    result = {"sourceSha256":SHOP_STEP,"stepSha256":digest(exchange),"corners":receipt,
              "reopened":len(reopened),"maxReopenErrorMm3":max(errors),
              "plateVolumeMm3":sum(cad.volume(v)[0] for k,v in joints.items() if "plate" in k),
              "removedVolumeMm3":{k:cad.volume(original[k])[0]-cad.volume(installed[k])[0] for k in sorted(changed)},
              "installedRivets":len(rows)}
    (output/"plates.json").write_text(json.dumps(result,indent=2)+"\n",encoding="utf-8")
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("artifacts",type=Path)
    parser.add_argument("output",type=Path)
    args = parser.parse_args()
    print(json.dumps(evaluate(args.artifacts,args.output),indent=2))
