"""Project-authored metric damper and mounting reserves. Dimensions in robot mm."""
import argparse
import json
import sys
from pathlib import Path
from compare import cad, contact, cylinder, digest, shared

CENTER_X, CENTER_Y = -60, 298
BODY_BOTTOM, BODY_END, FREE_END = 313.5, 392.7, 410.2
LOWEST_FACE = 397.5


def shapes_at(height):
    parts = {}
    for side, tag in ((1,"L"),(-1,"R")):
        y = side*CENTER_Y
        def box(x, y0, z, length, width, thick):
            return ["box",[x,y0 if side == 1 else -y0-width,z,length,width,thick]]
        def cyl(x, yy, z, radius, length):
            return ["cyl",[x,yy,z,0,0,1,radius,length]]
        # A fixed right-angle lug with a tapped pilot specified by thread major diameter.
        # The M6 through-bolts need sleeves in the hollow mast and sized nuts/access.
        holes = [["cyl",[-46, side*199,z, 0,side,0,3.3,45]] for z in (344,373)]
        flange = box(-65,235,336,37,8,46)
        arm = box(-67,239,347,32,64,16)
        head = cyl(CENTER_X,y,347,17,16)
        boss = ["cut",[["union",[flange,arm,head]],cyl(CENTER_X,y,346,10.1,18),*holes]]
        parts[tag+"-bracket-reserve"] = cad.brep(boss)
        # For the contact screen the machined bracket uses an oversize clearance bore.
        # A production bored bracket must retain a second nut, or be tapped to supplier fit.
        parts[tag+"-body-reserve"] = cylinder(CENTER_X,y,BODY_BOTTOM,10,BODY_END-BODY_BOTTOM)
        parts[tag+"-nut-reserve"] = cad.brep(["cut",[cyl(CENTER_X,y,363,13.3,8),cyl(CENTER_X,y,362,10.05,10)]])
        rod_end = max(LOWEST_FACE,min(FREE_END,height-160))
        parts[tag+"-rod"] = cylinder(CENTER_X,y,BODY_END,3.15,rod_end-BODY_END)
    return parts


def review(folder, pose):
    height = {"place":560,"transport":670,"receive":940,"stowed":940}[pose]
    sha = digest(folder/(pose+".step"))
    if sha != json.loads((folder/"review.json").read_bytes())[pose]["export_sha256"]:
        raise ValueError("Unrecognized source bytes")
    frames = json.loads((folder/"part-frames.json").read_bytes())
    shapes = cad.read_named(folder/(pose+".step"), [{"id":k} for k in frames])
    bounds = {key:cad.bounds(v) for key,v in shapes.items()}
    added = shapes_at(height)
    fit = {k:contact(v,shapes,bounds) for k,v in added.items()}
    pins = [cad.brep(["cyl",[-171,side*282,height-165,1,0,0,4,104]]) for side in (-1,1)]
    return {"pose":pose,"source":sha,"addedGeometry":len(added),"sourceBodies":len(shapes),
            "intersectionsMm3":{k:v for k,v in fit.items() if v},
            "pinSweepMm3":sum(shared(a,b) for a in added.values() for b in pins),
            "shopDrillingRequired":True,"hardwareRetentionQualified":False}


def export(path, height=560):
    sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"receiver-lift"))
    from check_routing import write_step
    from duplex import finalize_step
    shapes = shapes_at(height)
    write_step(path, shapes)
    finalize_step(path)
    reopened = cad.read_named(path,[{"id":k} for k in shapes])
    if any(cad.solid_count(v) != 1 for v in reopened.values()):
        raise ValueError("Invalid named end-stop body")
    return {"stepSha256":digest(path),"count":len(reopened),
            "perBracketAluminumKg":cad.volume(reopened["L-bracket-reserve"])[0]*2.70e-6,
            "massWithoutFastenersKg":2*(.130+cad.volume(reopened["L-bracket-reserve"])[0]*2.70e-6)}


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("folder",type=Path)
    p.add_argument("pose", choices=["stowed","receive","transport","place"])
    p.add_argument("receipt",type=Path)
    p.add_argument("--step",type=Path)
    args=p.parse_args()
    result=review(args.folder,args.pose)
    if args.step:
        result["export"]=export(args.step,{"place":560,"transport":670,"receive":940,"stowed":940}[args.pose])
    args.receipt.write_text(json.dumps(result,indent=2)+"\n",encoding="utf-8",newline="\n")
