"""Build and reopen the four-tube frame joint in the registered shop assembly."""
import argparse
import json
from pathlib import Path
import sys

from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut, BRepAlgoAPI_Fuse
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace, BRepBuilderAPI_MakePolygon
from OCP.BRepPrimAPI import BRepPrimAPI_MakePrism
from OCP.gp import gp_Pnt, gp_Vec

HERE = Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent/"moving-harness"))
from continuous_screen import SHOP_STEP, read_shop
from converter_panel import cad, digest, distance, lower_bound
from service import finalize_step, write_step
sys.path.insert(0,str(HERE.parent/"design/bumper-kit"))
from check_bumpers import removal_sweep

LEG = 2.5
BEVEL = 1.2
CROWN = .6
RAILS = ("cross-front","cross-rear","drive-rail-outer-L","drive-rail-outer-R")


def boolean(operation, first, second):
    query = operation(first,second)
    if not query.IsDone():
        raise ValueError("Native joint Boolean failed")
    return query.Shape()


def extrude(points, delta):
    loop = BRepBuilderAPI_MakePolygon()
    for point in points:
        loop.Add(gp_Pnt(*point))
    loop.Close()
    face = BRepBuilderAPI_MakeFace(loop.Wire()).Face()
    return BRepPrimAPI_MakePrism(face,gp_Vec(*delta)).Shape()


def corner(sx, sy):
    # Right, up and out coordinates from the inner corner on the front-left rail.
    def span(points, vector):
        return extrude([(sx*x,sy*y,z) for x,y,z in points],
                       (sx*vector[0],sy*vector[1],vector[2]))
    def vertical(profile):
        return span([(x,y,50) for x,y in profile],(0,0,100))
    prepared = [vertical([(350-BEVEL,325),(350,325-BEVEL),(350,325)])]
    outside = [vertical([(330,325),(330-LEG,325),(330,325-LEG)])]
    for z, direction in ((50,1),(150,-1)):
        # Single bevel on the accessible end of the crossrail. Restore with weld metal.
        prepared.append(span([(330,325,z+direction*BEVEL),(330,325-BEVEL,z),(330,325,z)],(20,0,0)))
        outside.append(span([(330,325-BEVEL,z),(330,325,z-direction*CROWN),(330,325+BEVEL,z)],(20,0,0)))
    groove = prepared[0]
    for shape in prepared[1:]:
        groove = boolean(BRepAlgoAPI_Fuse,groove,shape)
    # Fillet and transverse seam domains meet only along edges in this idealization.
    return groove, outside


def check(artifacts, output):
    shop, swept = read_shop(artifacts)
    shop_bounds = {key:cad.bounds(shape) for key,shape in shop.items()}
    moving = {key:box for key,box in swept.items()
              if key not in shop or any(abs(a-b)>.01 for a,b in zip(box,shop_bounds[key]))}
    removal = {f"{end}-{strip}":solid for end,sign in (("front",1),("rear",-1))
               for strip,solid in removal_sweep(sign).items()}
    rails = {k:shop[k] for k in RAILS}
    package = {}
    witnesses = {}
    for sx,end in ((1,"front"),(-1,"rear")):
        for sy,side in ((1,"L"),(-1,"R")):
            tag = f"{end}-{side}"
            end_id = "cross-"+end
            side_id = "drive-rail-outer-"+side
            groove, caps = corner(sx,sy)
            original = rails[end_id]
            rails[end_id] = boolean(BRepAlgoAPI_Cut,original,groove)
            cut_volume = cad.volume(original)[0]-cad.volume(rails[end_id])[0]
            groove_volume = cad.volume(groove)[0]
            if abs(cut_volume-groove_volume)>.001:
                raise ValueError("Groove does not lie in the 2 mm crossrail wall")
            solids = dict(zip(("groove","fillet","bottom-cap","top-cap"),(groove,*caps)))
            tire = shop["wheel-"+tag]
            minimum = min(distance(shape,cad.translated(tire,[sx*step/2,0,0]))
                          for step in range(-8,9) for shape in solids.values())
            # Continuous conservative box check for moving bodies and auxiliary panel.
            motion_gap = min((lower_bound(cad.bounds(s),box),k)
                             for s in solids.values() for k,box in moving.items())
            removal_volume = max(cad.intersection(shape,gauge)
                                 for shape in solids.values() for gauge in removal.values())
            close = []
            max_overlap = 0
            for name, shape in solids.items():
                bounds = cad.bounds(shape)
                for k,other in shop.items():
                    if k in (end_id,side_id) or lower_bound(bounds,shop_bounds[k])>2:
                        continue
                    gap = distance(shape,other)
                    volume = cad.intersection(shape,other)
                    close.append({"seam":name,"body":k,"clearanceMm":round(gap,4),
                                  "overlapMm3":round(volume,5)})
                    max_overlap = max(max_overlap,volume)
                rail_overlap = sum(cad.intersection(shape,rails[k]) for k in (end_id,side_id))
                if rail_overlap>.001:
                    raise ValueError(f"Double counted deposited volume in {tag}/{name}")
                package[f"corner-{tag}-{name}"] = shape
            # Unfinished production proud of the outer face collides with a bearing strip.
            proud = extrude([(sx*350,sy*323,60),(sx*350,sy*327,60),(sx*350,sy*327,140),
                             (sx*350,sy*323,140)],(sx*.6,0,0))
            control = {k:round(cad.intersection(proud,shop[k]),4) for k in shop
                       if k.startswith("bumper-contact-") and lower_bound(cad.bounds(proud),cad.bounds(shop[k]))<.001}
            witnesses[tag] = {"removedMm3":cut_volume,"depositedMm3":sum(cad.volume(v)[0] for v in solids.values()),
                              "minimumWheelClearanceLowerMm":minimum-.25,
                              "bothBumperRemovalSweepsMaxOverlapMm3":removal_volume,
                              "movingBoxLowerMmAndBody":motion_gap,"nearby":close,
                              "maxOtherInterferenceMm3":max_overlap,"proudFaceControlMm3":control}
            if max(max_overlap,removal_volume)>.001 or minimum-.25<2 or motion_gap[0]<=0 or not any(control.values()):
                raise ValueError(f"Corner layout or its interference control failed: {tag} {witnesses[tag]}")
    package.update(rails)
    output.mkdir(parents=True,exist_ok=True)
    step = output/"frame-weldment.step"
    write_step(step,package)
    finalize_step(step)
    reopened = cad.read_named(step,[{"id":k} for k in package])
    error = max(abs(cad.volume(v)[0]-cad.volume(reopened[k])[0]) for k,v in package.items())
    if error>.001:
        raise ValueError("Weldment STEP changed shape volumes")
    gross = sum(row["depositedMm3"] for row in witnesses.values())
    removed = sum(row["removedMm3"] for row in witnesses.values())
    result = {"shopStepSha256":SHOP_STEP,"stepSha256":digest(step),"reopenedBodies":len(reopened),
              "maxReopenVolumeErrorMm3":error,"corners":witnesses,"grossFillMm3":gross,
              "removedMm3":removed,"nominalNetKg":(gross-removed)*2.70e-6}
    (output/"weld-fit.json").write_text(json.dumps(result,indent=2)+"\n",encoding="utf-8",newline="\n")
    return result


if __name__ == "__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("artifacts",type=Path)
    parser.add_argument("output",type=Path)
    args=parser.parse_args()
    print(json.dumps(check(args.artifacts,args.output),indent=2))
