"""Reopen four table-mounted service stands under the registered, stationary low carriage."""
import argparse
import math
import sys
from pathlib import Path

import numpy as np
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.gp import gp_Trsf, gp_Ax1, gp_Pnt, gp_Dir

import importlib.util

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
spec = importlib.util.spec_from_file_location("stationary_hoist", ROOT/"fixed-hoist/fit.py")
hoist = importlib.util.module_from_spec(spec)
spec.loader.exec_module(hoist)
ENDS, box_from_bounds, retired, j = hoist.ENDS, hoist.box_from_bounds, hoist.retired, hoist.j

LOCATIONS = {"rear": (-77., 294., 400.), "front": (136., 146., 433.)}
COLUMN = 490.


def cylinder(x, y, z, radius, height):
    return j.cad.brep(["cyl", [x, y, z, 0, 0, 1, radius, height]])


def hexagon(x, y, z, across, height):
    radius = across / math.sqrt(3)
    points = [(x+radius*math.cos(i*math.pi/3), y+radius*math.sin(i*math.pi/3), z) for i in range(6)]
    return j.extrude(points, (0,0,height))


def union(shapes):
    result = shapes[0]
    for shape in shapes[1:]:
        result = j.fuse(result, shape)
    return result


def orient(name, sign, shape):
    if name == "rear":
        return shape
    x,y,z = LOCATIONS[name]
    transform=gp_Trsf()
    transform.SetRotation(gp_Ax1(gp_Pnt(x,sign*y,z),gp_Dir(0,0,1)),-sign*math.pi/2)
    return BRepBuilderAPI_Transform(shape,transform,True).Shape()


def stand(name, sign):
    x, y, top = LOCATIONS[name]
    def box(x0, x1, y0, y1, z0, z1):
        return j.box(x+x0,x+x1,sign*y0,sign*y1,z0,z1)
    base = box(-55,55,440,550,0,8)
    for dx in (-38,38):
        for station in (452,538):
            base = j.cut(base,cylinder(x+dx,sign*station,-1,4.5,10))
    post = j.cut(box(-20,20,COLUMN-20,COLUMN+20,8,top-5),
                 box(-17,17,COLUMN-17,COLUMN+17,7,top-4))
    boom = j.cut(box(-15,15,y-12,COLUMN-20,top-40,top-5),
                 box(-12,12,y-13,COLUMN-19,top-37,top-8))
    boss = cylinder(x,sign*y,top-40,10,35)
    frame = union([base,post,boom,boss])
    # Nominal mating male/female thread cylinders; only engaged length and envelope are represented.
    frame = j.cut(frame,cylinder(x,sign*y,top-41,6,37))
    screw = union([hexagon(x,sign*y,top-59,9,8),
                   cylinder(x,sign*y,top-51,6,48),cylinder(x,sign*y,top-3,9,3)])
    nut = j.cut(hexagon(x,sign*y,top-50,18,10),cylinder(x,sign*y,top-51,6,12))
    kit={"frame":frame,"screw":screw,"locknut":nut}
    if name=="rear":
        def pin_x(start, length, radius):
            return j.cad.brep(["cyl",[start,sign*y,top+16,1,0,0,radius,length]])
        palm=box(-26,29,y-11,y+13,top-8,top-5)
        palm=j.cut(palm,cylinder(x,sign*y,top-10,10.2,10))
        # Rear carriage plate spans X -98..-53 and ends at |Y|303.
        fork=union([palm,box(-26,-21.7,y-6,y+6,top-5,top+22),
                    box(24.7,29,y-6,y+6,top-5,top+22),
                    box(-18,18,303.7,307,top-5,top+6)])
        kit["frame"]=j.fuse(frame,j.cut(fork,pin_x(-105,62,3.2)))
        kit["keeper"]=j.fuse(pin_x(-48,5,5),pin_x(-113,65,3))
        nut_blank=hexagon(0,0,0,10,6)
        transform=gp_Trsf()
        transform.SetRotation(gp_Ax1(gp_Pnt(0,0,0),gp_Dir(0,1,0)),math.pi/2)
        nut_shape=BRepBuilderAPI_Transform(nut_blank,transform,True).Shape()
        kit["keeper-nut"]=j.cut(j.cad.translated(nut_shape,(-109,sign*y,top+16)),pin_x(-110,8,3))
    return {key:orient(name,sign,value) for key,value in kit.items()}


def read_kit(folder, file, receipt, key, volume_key="volumeMm3"):
    meta = j.read(folder/receipt)
    if j.digest(folder/file)!=meta[key]:
        raise ValueError("Unexpected native exchange "+file)
    return j.cad.read_named(folder/file,[{"id":k} for k in meta[volume_key]])


def scene(registry):
    data = j.read(registry)
    if j.digest(registry)!=j.read(ROOT/"fixed-hoist/fit.json")["sourceSha256"]["registeredScene"]:
        raise ValueError("Unregistered fixture datum")
    # Service requires the slide stowed and restrained. Use the exact registered stowed boxes.
    obs = {k:box_from_bounds(b) for k,b in data["stowed"].items()
           if k not in retired() | {"long-common-shaft","wire/data-feed","wire/power-feed"}}
    lift = ROOT/"design/receiver-lift"
    for name,shape in j.cad.read_named(lift/"duplex-place.step",
            [{"id":k} for k in j.read(lift/"duplex-inventory.json")["poses"]["place"]]).items():
        if name in obs and max(abs(a-b) for a,b in zip(j.cad.bounds(shape),data["stowed"][name]))<.001:
            obs[name]=shape
    obs.update(read_kit(ROOT/"fixed-hoist","stationary-shaft.step","fit.json","stepSha256"))
    obs.update(read_kit(ROOT/"shelf-structure","stow-receiver.step","installed.json","manufacturingSha256"))
    obs.update(read_kit(ROOT/"mast-attachment","mast-joints.step","fit.json","stepSha256"))
    obs.update(read_kit(ROOT/"cover-retention","supported-covers.step","fit.json","stepSha256"))
    obs.update(read_kit(ROOT/"fixed-camera-route","harness-guides.step","guides.json","stepSha256"))
    sys.path.insert(0,str(ROOT/"selected-match"))
    from clearance import data_feed, power_feed, PITCH
    lines = []
    for name,path,r in (("original-power",power_feed(-329),4.35),("original-data",data_feed(-317),3.5)):
        lines.extend((name,points,r+PITCH/2) for _,points in path)
    sys.path.insert(0,str(ROOT/"fixed-camera-route"))
    import hardware
    from paths import routes
    host,_=hardware.solids()
    obs.update({"optical/"+k:v for k,v in host.items()})
    for name,path in routes()[0].items():
        lines.extend((name,line,hardware.RADIUS[name]+1.) for _,line,_ in path)
    for i,(name,line,radius) in enumerate(lines):
        for start in range(0,len(line)-1,12):
            pts=np.asarray(line[start:start+13])
            obs[f"feed/{name}/{i}/{start}"]=box_from_bounds(np.r_[pts.min(axis=0)-radius,pts.max(axis=0)+radius])
    # Keep the plunger and nut assemblies in a conservative common cylindrical reserve.
    for y in (-66.,42.):
        obs[f"damper/{y:g}"]=cylinder(118,y,423.5,9,61.4)
    return obs


def check(shape, obstacles):
    return j.prove(j.clearance(shape,obstacles))


def run(registry, output, carriage=None):
    output.mkdir(parents=True,exist_ok=True)
    obstacles=scene(registry)
    if carriage is not None:
        obstacles.update(read_kit(carriage,"carriage-support.step","fit.json","stepSha256","volumesMm3"))
    parts={}
    for loc in LOCATIONS:
        for side,sign in ENDS:
            parts.update({f"{loc}-{side}-{k}":v for k,v in stand(loc,sign).items()})
    step=output/"service-stands.step"
    j.write_step(step,parts)
    j.finalize_step(step)
    actual=j.cad.read_named(step,[{"id":k} for k in parts])
    for key,shape in actual.items():
        if (j.cad.solid_count(shape)!=1 or not BRepCheck_Analyzer(shape).IsValid()
                or abs(j.cad.volume(shape)[0]-j.cad.volume(parts[key])[0])>.002):
            raise ValueError("Invalid reopened part "+key)
    record={}
    for loc,(x,y,z) in LOCATIONS.items():
        for side,sign in ENDS:
            tag=f"{loc}-{side}"
            target=f"backup-{side}-carriage-plate" if loc=="rear" else "slide-base-bridge"
            fixture={k:actual[tag+"-"+k] for k in ("frame","screw","locknut")}
            if loc=="rear":
                fixture.update({k:actual[tag+"-"+k] for k in ("keeper","keeper-nut")})
            elif carriage:
                target="support-machined-bridge"
            tests={}
            for key,shape in fixture.items():
                adjacent={**obstacles,**{k:v for k,v in actual.items() if k!=tag+"-"+key}}
                tests[key]=check(shape,adjacent)
            pad=fixture["screw"]
            # A quarter mm of forced settlement must engage the whole Ø18 mm table pad.
            area=j.cad.intersection(j.cad.translated(pad,(0,0,.25)),obstacles[target])/.25
            if not math.isclose(area,math.pi*9**2,abs_tol=.02):
                raise ValueError((tag,"partial support land",area))
            # Continuous inward entry envelope with top pad retracted by two mm.
            # Split the base/post and the narrow overhanging arm to avoid unoccupied volume.
            # The far end is beyond the entire robot-side envelope.
            def sidebox(a,b,c,d,h,H):
                return j.box(a,b,sign*c,sign*d,h,H)
            approach={
                "baseAndPost":sidebox(x-55,x+55,440,830,0,z-5),
                "arm":sidebox(x-15,x+15,y-12,800,z-40,z-5),
                "adjuster":sidebox(x-11,x+11,y-11,800,z-61,z-2),
            }
            if loc=="rear":
                approach.update({
                    "palm":sidebox(x-26,x+29,y-11,800,z-8,z-5),
                    "rearEar":sidebox(x-26,x-21.7,y-6,800,z-5,z+22),
                    "frontEar":sidebox(x+24.7,x+29,y-6,800,z-5,z+22),
                    "lateralStop":sidebox(x-18,x+18,303.7,800,z-5,z+6)})
            other={k:v for k,v in actual.items() if not k.startswith(tag+"-")}
            for key,shape in approach.items():
                tests["entry-"+key]=check(orient(loc,sign,shape),{**obstacles,**other})
            # Vertical socket and a side-entry wrench under the boss, with the frame in place.
            tests["driverSocket"]=check(cylinder(x,sign*y,z-95,8,36),{**obstacles,**other,tag+"-frame":fixture["frame"]})
            tests["locknutWrench"]=check(orient(loc,sign,sidebox(x-16,x+16,y-16,y+111,z-48,z-42)),{**obstacles,**other})
            # Wrong side of the wedge: advancing the screw against a rigid supported carriage.
            tests["twoMmOvertravelMm3"]=j.cad.intersection(j.cad.translated(pad,(0,0,2)),obstacles[target])
            if loc=="rear":
                plate=obstacles[target]
                trials={"forward":(2,0,0),"rearward":(-2,0,0),"outboard":(0,sign*2,0)}
                tests["captureOverlapMm3"]={key:j.cad.intersection(j.cad.translated(plate,d),fixture["frame"])
                                          for key,d in trials.items()}
                tests["captureOverlapMm3"]["uplift"]=j.cad.intersection(j.cad.translated(plate,(0,0,2)),fixture["keeper"])
                if min(tests["captureOverlapMm3"].values())<1.:
                    raise ValueError("Missing captive stop for "+tag)
                wrong_way=j.cad.brep(["cyl",[-180,sign*y,z+16,1,0,0,5,77]])
                tests["aftHeadRailCollisionMm3"]=j.cad.intersection(wrong_way,obstacles[f"backup-{side}-service-pin-rail"])
                shaft=j.cad.brep(["cyl",[-113,sign*y,z+16,1,0,0,3,143]])
                head=j.cad.brep(["cyl",[-48,sign*y,z+16,1,0,0,5,78]])
                tests["keeperExtraction"]=check(j.fuse(shaft,head),{**obstacles,**other,"fork":fixture["frame"]})
                tests["keeperNutSideAccess"]=check(sidebox(-109,-104,y-9,y+100,z+8,z+24),
                       {**obstacles,**other,"fork":fixture["frame"]})
            record[tag]={"target":target,"areaMm2":area,"checks":tests}
    result={"registrySha256":j.digest(registry),"carriageSupportSha256":j.digest(carriage/"carriage-support.step") if carriage else None,"stepSha256":j.digest(step),"parts":sorted(actual),
            "registeredObstacles":len(obstacles),"contacts":record,"locationsMm":LOCATIONS,
            "qualification":"stowed fixture geometry","measuredPadReactionsN":None}
    j.write(output/"fit.json",result)


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("registry",type=Path)
    parser.add_argument("--out",type=Path,default=HERE)
    parser.add_argument("--carriage",type=Path)
    args=parser.parse_args()
    run(args.registry,args.out,args.carriage)
