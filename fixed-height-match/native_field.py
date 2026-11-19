"""Interrogate low bumper material and the occupied second scoring lane at the registered poses."""
import json
import math
from pathlib import Path
import sys
import importlib.util

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
sys.path.insert(0,str(ROOT/"design/fixed-height"))
from native import hardware,cad
sys.path.insert(0,str(ROOT/"design/competition-package/shop-duplex"))
from assembly import pose_shapes
spec=importlib.util.spec_from_file_location("park_material",ROOT/"leave-park/rehearse.py")
park=importlib.util.module_from_spec(spec)
spec.loader.exec_module(park)
from match import build_park
from installed_stock import across,check,stationary


def run():
    f=park.corrected()
    deliveries=json.loads((HERE/"placements.json").read_bytes())["placements"]
    stock=json.loads(park.STOCK.read_bytes())["stock"]
    sets=[]
    for stroke in (0,185,370):
        raw=pose_shapes(560,stroke,"red")
        changed,add=hardware(raw,height=560)
        sets.append({**raw,**changed,**add})
    held,mid,extended=sets
    boxes=across(*[{k:cad.bounds(shape) for k,shape in shapes.items()} for shapes in sets])
    # Universally enclosing the three rotating pinions avoids credit at favorable tooth angles.
    for name in ("slide-fixed-rack-pinion","slide-inner-rack-pinion","slide-power-pinion"):
        box=list(boxes[name])
        for axis in (0,2):
            box[axis]-=22
            box[axis+3]+=22
        boxes[name]=box
    for name in boxes:
        if "draw-wire" in name:
            # Enclose the entire pull cord between its spring housing and the slide tip.
            boxes[name]=across({"x":boxes[name]},{"x":(-175,-170,425,535,170,650)})["x"]
    routes=[f.routes(d["sector"]) for d in deliveries]
    auto=routes[0][0]
    endpoint=build_park(f,routes[1][1].end).end
    tape=(park.TAPE[0],0,park.TAPE[1],f.PROFILE["field"]["width"])
    start=park.overlap(f,held,auto.start,tape)
    final=park.overlap(f,held,endpoint,park.ZONE)
    source=pose_shapes(940,0,"red")
    from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
    difference={}
    for key in final:
        common=cad.volume(BRepAlgoAPI_Common(source[key],held[key]).Shape())[0]
        difference[key]=max(0,cad.volume(source[key])[0]+cad.volume(held[key])[0]-2*common)
    if max(difference.values())>1e-4:
        raise ValueError("Bumper material differs between registered configurations")
    dock_stock=[check(f,stock,boxes,stationary(route[1].end)) for route in routes]
    x,y,h=routes[1][1].end
    if abs(h)%(2*math.pi)>1e-7 and abs(h)%(2*math.pi)<2*math.pi-1e-7:
        raise ValueError("Second docking chassis must point along field X")
    first=deliveries[0]
    dx=(f.LIP[0]+first["finalXZ"][0]-x)*1000
    dy=(f.LIP[1]+first["sector"]-y)*1000
    dz=first["finalXZ"][1]*1000
    # Include 2 mm on all faces around the nominal transverse finite settled tube.
    tube=(dx-59.15,dy-152.8125,dz-59.15,dx+59.15,dy+152.8125,dz+59.15)
    separation={k:math.hypot(*(max(0,b[i]-tube[i+3],tube[i]-b[i+3]) for i in range(3))) for k,b in boxes.items()}
    minimum=min(separation,key=separation.get)
    return {"bodyCount":len(boxes),"stowedToFixedBumperSymmetricDifferenceMm3":difference,
            "startBumperTapeMm3":start,"parkClothInZoneMm3":final,
            "autoEndWholeHullToStartTapeLowerM":auto.end[0]-.425*math.sqrt(2)-park.TAPE[1]*f.IN,
            "parkPose":endpoint,"targetStrokeSweepInstalledStock":dock_stock,
            "firstTubeToSecondDockMm":{"part":minimum,"gap":separation[minimum],"inflatedTubeBoxMm":tube}}


if __name__=="__main__":
    (HERE/"native.json").write_text(json.dumps(run(),indent=2)+"\n",encoding="utf-8",newline="\n")
