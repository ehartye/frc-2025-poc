"""Form and positively locate four removable drive-cover halves in the issued bays."""
import argparse
from math import hypot
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT / "chain-covers"))
from study import axle_opening, shop
from OCP.BRepFilletAPI import BRepFilletAPI_MakeFillet
from OCP.TopAbs import TopAbs_EDGE
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS

SPLIT = 76.2
INNER = 235.4
OUTER = 272.9376
GAUGE = 0.5
SEAT = OUTER - GAUGE
POSITIONS = ((-295, SPLIT), (-120, 64), (-120, 88), (120, 64), (120, 88), (295, SPLIT))


def rounded_box(sy, inset):
    # Keep the ends short at the welded ears, then give the end locator and
    # its reinforcing lip room beyond the chain. Offset each plane normally.
    slope = (305-292.8)/18
    low,high=INNER+inset,OUTER-inset
    end=305-inset
    start=292.8+slope*inset-inset*hypot(1,slope)
    knee=INNER+(end-292.8+inset*hypot(1,slope))/slope
    plan=[(-start,low),(start,low),(end,knee),(end,high),(-end,high),(-end,knee)]
    box=shop.extrude([(x,sy*y,34+inset) for x,y in plan],(0,0,86-2*inset))
    fillet = BRepFilletAPI_MakeFillet(box)
    edges = TopExp_Explorer(box, TopAbs_EDGE)
    while edges.More():
        fillet.Add(4.5-inset, TopoDS.Edge(edges.Current()))
        edges.Next()
    fillet.Build()
    if not fillet.IsDone():
        raise ValueError("Could not form the continuous rounded shell corner")
    return fillet.Shape()


def pilot(x, sy, z):
    # A one-flat keyed shoulder is seated in a matching pocket in the inner rail.
    pin = shop.rod(x, sy, 233, 235, z, 3)
    return shop.cut(pin, shop.box(x+2.3, x+4, sy*232, sy*236, z-4, z+4))


def make_trays(sy):
    shell = shop.cut(rounded_box(sy, 0), rounded_box(sy, GAUGE))
    shell = shop.cut(shell, shop.box(-310,310,sy*230,sy*(INNER+1),47,114))
    for x in (-247.65, 0, 247.65):
        shell = shop.cut(shell, axle_opening(x, sy, 4 if x else 0, 12))
    for x, z in POSITIONS:
        shell = shop.cut(shell, shop.rod(x,sy,270,276,z,3.4 if z==SPLIT else 2.3))
        if z==SPLIT:
            shell = shop.cut(shell, shop.rod(x,sy,232,254,z,4.5))
    return {"upper":shop.cut(shell, shop.box(-310,310,sy*232,sy*276,30,SPLIT)),
            "lower":shop.cut(shell, shop.box(-310,310,sy*232,sy*276,SPLIT,125))}


def mounting_station(x, sy, z):
    locator = z == SPLIT
    post = shop.fuse(pilot(x,sy,z),shop.rod(x,sy,235,243.5,z,3.6))
    post = shop.fuse(post,shop.rod(x,sy,243.5,SEAT-8.5,z,2.5))
    seat_end = SEAT-(.15 if locator else .30)
    post = shop.fuse(post,shop.rod(x,sy,SEAT-8.5,seat_end,z,4))
    # The bores are major-diameter thread reservations. Do not intersect smooth screw stock.
    post = shop.cut(post,shop.rod(x,sy,232,243,z,1.5))
    base = shop.fuse(shop.rod(x,sy,226,229,z,2.75),shop.rod(x,sy,229,241,z,1.5))
    if locator:
        post = shop.fuse(post,shop.rod(x,sy,seat_end,OUTER+.2,z,3))
        post = shop.fuse(post,shop.rod(x,sy,OUTER+.2,OUTER+1.5,z,5))
        return {"post":post,"base-screw":base}
    stop = OUTER+.10
    post = shop.cut(post,shop.rod(x,sy,stop-8.5,stop+1,z,1.5))
    blank = shop.fuse(shop.rod(x,sy,seat_end,stop,z,2),shop.rod(x,sy,stop,stop+.8,z,4.5))
    washer = shop.cut(blank,shop.rod(x,sy,seat_end-1,stop+1,z,1.6))
    face = shop.fuse(shop.rod(x,sy,stop+.8-8,stop+.8,z,1.5),
                     shop.rod(x,sy,stop+.8,stop+3.8,z,2.75))
    return {"post":post,"base-screw":base,"retainer":washer,"face-screw":face}


def baseline(artifacts):
    scene, pins = shop.load_source(artifacts, ROOT/"axle-retention")
    from fit import add_exchange
    joints = ROOT/"drive-side-attachment"
    fit = shop.read(joints/"fit.json")
    pinned = ROOT/"chain-covers"
    prior = shop.read(pinned/"study.json")
    for step, ids, digest in ((joints/"inner-rail-joints.step", fit["volumeMm3"], fit["stepSha256"]),
                              (pinned/"front-loop-repack.step",prior["repack"]["volumeChangesMm3"],
                               prior["repack"]["stepSha256"])):
        add_exchange(scene,step,ids,digest)
        pins[step.relative_to(ROOT).as_posix()] = digest
    return scene, pins


def run(artifacts, out):
    scene, pins = baseline(artifacts)
    kit = {}
    removed = {}
    for side, sy in (("L",1),("R",-1)):
        rail = "drive-rail-inner-"+side
        shape = scene[rail]
        for half, tray in make_trays(sy).items():
            kit[f"tray-{half}-{side}"] = tray
        for x,z in POSITIONS:
            tag = f"{x:g}_{z:g}-{side}"
            # Manufacturing pocket with radial play. The small flat remains a positive key.
            pocket = shop.cut(shop.rod(x,sy,232.9,235.1,z,3.1),
                              shop.box(x+2.4,x+4,sy*232,sy*236,z-4,z+4))
            through = shop.rod(x,sy,228,236,z,1.7)
            before = shop.cad.volume(shape)[0]
            shape = shop.cut(shop.cut(shape,pocket),through)
            removed[tag] = before-shop.cad.volume(shape)[0]
            if removed[tag] < 80:
                raise ValueError("The proposed keyed socket falls off rail stock: "+tag)
            for name,part in mounting_station(x,sy,z).items():
                kit[name+"-"+tag] = part
        kit[rail] = shape
    out.mkdir(parents=True,exist_ok=True)
    step=out/"supported-covers.step"
    shop.write_step(step,kit)
    shop.finalize_step(step)
    reopened=shop.cad.read_named(step,[{"id":k} for k in kit])
    volumes={k:round(shop.cad.volume(v)[0],5) for k,v in reopened.items()}
    if max(abs(shop.cad.volume(v)[0]-volumes[k]) for k,v in kit.items())>.002:
        raise ValueError("Exchange volume changed")
    installed={**scene,**reopened}
    fits={}
    for name,part in reopened.items():
        if name.startswith("drive-rail-inner"):
            continue
        fits[name]=shop.clearance(part,{k:v for k,v in installed.items() if k!=name})
        if fits[name]["overlapMm3"]:
            raise ValueError(f"{name} penetrates {fits[name]['overlapMm3']}")
    radius={}
    for name,part in reopened.items():
        if name.startswith("drive-rail-inner"):
            continue
        side=name[-1]
        movers={k:v for k,v in scene.items() if k.endswith("-"+side) and
                k.startswith(("wheel-","axle-","spacer-","sprocket-","chain-"))}
        gaps=[]
        for shift in (-4,-2,0,2,4):
            shifted={k:v if "-center-" in k or k.startswith(("sprocket-0-b-","sprocket-1-a-"))
                     else shop.cad.translated(v,[shift,0,0]) for k,v in movers.items()}
            check=shop.prove(shop.clearance(part,shifted))
            gaps.append((check["nearestMm"],check["nearestBody"],shift))
        sampled=min(gaps)
        radius[name]={"sampleMin":sampled,"continuousLowerMm":round(sampled[0]-1.001,5)}
    result={"sourceSha256":pins,"stepSha256":shop.digest(step),"volumeMm3":volumes,
            "railRemovedMm3":removed,"fits":fits,"drive":radius}
    shop.write(out/"fit.json",result)


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("artifacts",type=Path)
    parser.add_argument("--out",type=Path,default=HERE)
    args=parser.parse_args()
    run(args.artifacts.resolve(),args.out.resolve())
