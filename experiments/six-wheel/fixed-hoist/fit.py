"""Native static shaft, drum stops and service checks for the permanently locked receiver."""
import argparse

import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT / "drive-side-attachment"))
import joint as j
sys.path.insert(0, str(ROOT / "design/receiver-lift"))
from check_routing import routing

SHAFT = "fixed-haul-shaft"
X, Z = -92., 330.
ENDS = (("L", 1), ("R", -1))
SECONDARY = ("drum-core", "upper-sheave", "drum-flange-0", "sheave-flange-0",
             "drum-flange-1", "sheave-flange-1", "upper-pin", "top-hanger",
             "outboard-bearing", "bearing-support", "running-line", "clevis",
             "clevis-pin", "thimble", "spliced-eye-envelope", "return-line")


def retired():
    names = list(SECONDARY)
    names.extend(f"mast-{part}-{height}" for part in ("bolt", "spacer") for height in (275, 298, 994, 1008))
    return {f"backup-{side}-{name}" for side, _ in ENDS for name in names}


def cylinder(x, y, z, axis, radius, length):
    return j.cad.brep(["cyl", [x, y, z, *axis, radius, length]])


def annulus(sign, start, end, outer, inner):
    return j.cut(j.rod(X, sign, start, end, Z, outer), j.rod(X, sign, start-1, end+1, Z, inner))


def cross(y, x, end, radius):
    return cylinder(x, y, Z, (1, 0, 0), radius, end-x)


def make(original):
    shaft = j.cut(original, j.box(-114,-70,-322,-280,310,350))
    shaft = j.cut(shaft, j.box(-114,-70,280,322,310,350))
    cut_ends = j.cad.volume(original)[0] - j.cad.volume(shaft)[0]
    parts = {}
    for side, sign in ENDS:
        y = sign*274.
        shaft = j.cut(shaft, cross(y, X-14, X+14, 2.10))
        # The shoulder lands on the fixed arm outside the bearing OD. Recess the race.
        sleeve = annulus(sign,224.15,236.40,10.5,7.55)
        flange = annulus(sign,224.15,226.15,17.0,14.65)
        sleeve = j.cut(sleeve, j.rod(X,sign,223.,225.15,Z,12.))
        web = annulus(sign,225.15,226.15,17.,7.55)
        parts["haul-sleeve-"+side] = j.fuse(j.fuse(sleeve,web),flange)
        block = j.box(X-10,X+10,sign*267.65,sign*279.8,Z-10,Z+10)
        bore = j.cad.brep(["hex",[X,min(sign*267,sign*281),Z,12.93,14.]])
        parts["haul-stop-"+side] = j.cut(j.cut(block,bore),cross(y,X-11,X+11,2.10))
        # M4 x 30 cross screw. Envelopes omit thread form and overbound the head.
        parts["haul-screw-"+side] = j.fuse(cross(y,X-14,X-10,3.5),cross(y,X-10,X+20,2.))
        # Hexagonal locknut with thread cylinder at nominal screw OD for fit.
        rad = 3.5/math.cos(math.pi/6)
        points = [(X+10,y+rad*math.cos(i*math.pi/3),Z+rad*math.sin(i*math.pi/3)) for i in range(6)]
        parts["haul-locknut-"+side] = j.cut(j.extrude(points,(5.,0,0)),cross(y,X+9,X+16,2.))
    parts[SHAFT] = shaft
    return parts, cut_ends


def box_from_bounds(b):
    return j.box(b[0],b[3],b[1],b[4],b[2],b[5])


def overlap(a,b):
    return j.cad.intersection(a,b)


def run(registry, out):
    out.mkdir(parents=True,exist_ok=True)
    data = j.read(registry)
    if j.digest(registry) != "291d68c43e21f20c33885064e6bbe8c0e47b684d13a8a06e8af61f9fd9db5886":
        raise ValueError("Unregistered robot scene")
    for name, digest in data["inputs"].items():
        if j.digest(ROOT/name) != digest:
            raise ValueError(name)
    folder = ROOT/"design/receiver-lift"
    ids = j.read(folder/"duplex-inventory.json")["poses"]["place"]
    native = j.cad.read_named(folder/"duplex-place.step",[{"id":k} for k in ids])
    original = native["long-common-shaft"]
    parts, cut_ends = make(original)
    exchange = out/"stationary-shaft.step"
    j.write_step(exchange,parts)
    j.finalize_step(exchange)
    reopened = j.cad.read_named(exchange,[{"id":k} for k in parts])
    for name, shape in reopened.items():
        if j.cad.solid_count(shape)!=1 or abs(j.cad.volume(shape)[0]-j.cad.volume(parts[name])[0])>.001:
            raise ValueError("Exchange solid or volume mismatch: "+name)
    exact = {**routing(560), **native}
    bounds = data["bounds"]
    if not (retired()|{"long-common-shaft"}) <= bounds.keys():
        raise ValueError("Missing original hardware")
    obstacles = {}
    # Replace registered bounding boxes only when a native part has the exact same extents.
    for name, bound in bounds.items():
        if name in retired() or name=="long-common-shaft":
            continue
        shape = exact.get(name)
        if shape is not None and max(abs(a-b) for a,b in zip(j.cad.bounds(shape),bound))<.001:
            obstacles[name] = shape
        else:
            obstacles[name] = box_from_bounds(bound)
    # The two single boxes each cover the entire bent feed. Bound each segment
    # of the issued centerlines with its actual jacket and sample-spacing allowance.
    sys.path.insert(0,str(ROOT/"selected-match"))
    from clearance import power_feed, data_feed, PITCH
    import numpy as np
    for key, path, radius in (("power",power_feed(-329),4.35),("data",data_feed(-317),3.5)):
        del obstacles["wire/"+key+"-feed"]
        for i, (_, line) in enumerate(path):
            for start in range(0,len(line)-1,16):
                section = np.asarray(line[start:start+17])
                pad = radius+PITCH/2
                obstacles[f"legacy-feed/{key}/{i}/{start}"] = box_from_bounds(
                    np.r_[section.min(axis=0)-pad,section.max(axis=0)+pad])
    sys.path.insert(0,str(ROOT/"fixed-camera-route"))
    import hardware
    from paths import routes
    host,_ = hardware.solids()
    obstacles.update({"optical/"+k:v for k,v in host.items()})
    for key,path in routes()[0].items():
        for i,(_,line,_) in enumerate(path):
            pad = hardware.RADIUS[key]+1.
            obstacles[f"jacket/{key}/{i}"] = box_from_bounds(np.r_[line.min(axis=0)-pad,line.max(axis=0)+pad])
    for directory, file, receipt in (("fixed-camera-route","harness-guides.step","guides.json"),
                                      ("cover-retention","supported-covers.step","fit.json")):
        source = ROOT/directory
        record = j.read(source/receipt)
        if j.digest(source/file)!=record["stepSha256"]:
            raise ValueError(file)
        obstacles.update(j.cad.read_named(source/file,[{"id":k} for k in record["volumeMm3"]]))
    screens = {}
    for name, shape in reopened.items():
        screens[name] = j.prove(j.clearance(shape,{**obstacles,**{k:v for k,v in reopened.items() if k!=name}}))
    stops = {}
    for side, sign in ENDS:
        sleeve = reopened["haul-sleeve-"+side]
        block = reopened["haul-stop-"+side]
        flanges = ["drum-flange-"+str(n)+"-"+side for n in ((0,1) if sign==1 else (1,0))]
        def contact(moving, target, outward):
            return overlap(j.cad.translated(moving,(0,.6*sign*outward,0)),target)
        support = obstacles["bearing-arm-"+side]
        race = obstacles["bearing-envelope-"+side]
        screw = reopened["haul-screw-"+side]
        stops[side] = {
            "spacerToArmMm3":contact(sleeve,support,-1),
            "spacerToBearingMm3":contact(sleeve,race,-1),
            "drumInToSleeveMm3":contact(obstacles[flanges[0]],sleeve,-1),
            "drumOutToStopMm3":contact(obstacles[flanges[1]],block,1),
            "stopToPinMm3":contact(block,screw,1),
            "pinToShaftMm3":contact(screw,reopened[SHAFT],1),
            "withUndrilledShaftMm3":overlap(original,screw),
        }
        if any(v<.5 for k,v in stops[side].items() if k!="spacerToBearingMm3") or stops[side]["spacerToBearingMm3"]>.001:
            raise ValueError(stops[side])
    # Check straight radial screw withdrawal and driver/socket access with stationary neighboring stock.
    service = {}
    for side, sign in ENDS:
        y = sign*274.
        ambient = {**obstacles,**reopened}
        for k in ("haul-screw-"+side,"haul-locknut-"+side):
            ambient.pop(k)
        tools = {"driver":cross(y,X-65,X-14,4.5), "nutSocket":cross(y,X+10,X+36,5.5)}
        for k, tool in tools.items():
            service[side+k] = j.prove(j.clearance(tool,ambient))
        # A screw translated along its own axis stays in the same pair of bores.
        service[side+"screwWithdrawal"] = j.prove(j.clearance(
            j.fuse(cross(y,X-48,X-10,3.5),cross(y,X-10,X+20,2.)),ambient))
    # With both pins out and loose drums separately restrained, this swept prism
    # bounds every position of the drilled shaft until its inner tip clears the robot.
    pull = {}
    unpinned = {**obstacles,**{k:v for k,v in reopened.items() if k.startswith(("haul-stop-","haul-sleeve-"))}}
    for side, sign in ENDS:
        travel = 750.
        sweep = j.cad.brep(["hex",[X,-280+min(0,sign*travel),Z,12.7,560+travel]])
        pull[side] = j.clearance(sweep,unpinned)
        pull[side]["installedPinInterferenceMm3"] = overlap(sweep,reopened["haul-screw-"+side])
        if pull[side]["installedPinInterferenceMm3"]<1:
            raise ValueError("Missing pin removal control")
    result = {"shaftPull750mm":pull,"registeredObstacles":len(obstacles),"sourceSha256":{
        "registeredScene":j.digest(registry),"duplexPlace":j.digest(folder/"duplex-place.step"),
        "routing":j.digest(folder/"check_routing.py")},"stepSha256":j.digest(exchange),
        "volumeMm3":{k:j.cad.volume(v)[0] for k,v in reopened.items()},
        "trimmedEndsVolumeMm3":cut_ends,"installedChecks":screens,"serviceChecks":service,
        "axialContactsAt0p6mm":stops,"measuredMassKg":None,"physicalQualification":None}
    j.write(out/"fit.json",result)


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("registry",type=Path)
    parser.add_argument("--out",type=Path,default=HERE)
    args=parser.parse_args()
    run(args.registry,args.out)
