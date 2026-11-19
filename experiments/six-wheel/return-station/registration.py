"""Register the issued low robot and its fixed cameras against the exact physical corner station."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
sys.path.insert(0,str(ROOT/"station-supply"))
import dock_tolerance as dock
from native_station import parts
sys.path.insert(0,str(ROOT/"bridge-tolerance"))
import replay_candidate as bridge
from design import mount
from optics import camera
sys.path.insert(0,str(ROOT/"design/fixed-no-air"))
import whole


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def model(lamp,stroke=0):
    r=json.loads((ROOT/"design/fixed-no-air/whole-robot.json").read_bytes())
    if sha(lamp)!=r["supplied_lamp_sha256"]:raise ValueError("Unregistered lamp")
    roof=mount.cad.read_named(ROOT/"bridge-tolerance/moving-bridge.step",[{"id":k} for k in [*bridge.parts()[0],"receiver-R6-drilled-for-camera"]])
    deck=mount.cad.read_named(whole.LIGHT/"deck-mount.step",[{"id":k} for k in ("front-electronics-platform","light-spacer")])
    _,robot=whole.model(stroke,roof,deck,whole.mounted(whole.product(lamp),whole.X,whole.Y,whole.PANEL))
    if len(robot)!=593:raise ValueError("Incorrect body inventory")
    return robot


def poses():
    return {r["mode"]:r["returnStationRobotDxDyYawMmDeg"] for r in json.loads((ROOT/"spatial-contact/match.json").read_bytes())["cases"] if r["pipe"]=="coral-A"}


def run(lamp,field,cache,relief=False):
    from receiver_relief import low_relief
    robot=low_relief(lamp) if relief else model(lamp)
    background=cache/("station-views-relief.npz" if relief else "station-views.npz")
    station=parts(field)
    boxes={k:mount.cad.bounds(s) for k,s in station.items()}
    cache.mkdir(parents=True,exist_ok=True)
    _,groups,colors,_=mount.pose_layers(560,0,"red")
    mesh=[]
    for k,s in sorted(robot.items()):
        v,t=mount.original.triangulate(s)
        mesh.append([k,groups.get(k,"Mount"),colors.get(k,"#737d85"),v,t])
    views=camera.images_for_pose({"parts":mesh})
    np.savez_compressed(background,left=views[0][0],left_z=views[0][1],right=views[1][0],right_z=views[1][1])
    with (cache/"station-parts.tsv").open("w") as file:
        for k,b in boxes.items():
            file.write(k+"\t"+"\t".join(str(round(n,3)) for n in b)+"\n")
    result={}
    corrected={"bias-"+k:r["stationRobotDxDyYawMmDeg"] for k,r in json.loads((HERE/"return.json").read_bytes())["cases"].items()}
    for mode,(dx,dy,a) in {"center":[0,0,0],**poses(),**corrected,"toward-wall":[8,0,1]}.items():
        # Station X points out of the wall; station Y is opposite robot left.
        result[mode]=dock.native_dock(robot,station,boxes,10-dx,a,-dy)
    (HERE/("relief-dock.json" if relief else "dock.json")).write_text(json.dumps({"fieldSHA256":sha(field),"bridgeSHA256":sha(ROOT/"bridge-tolerance/moving-bridge.step"),"sourceRobotBodies":len(robot),
        "stationOccurrences":len(station),"stowedCameraBackgroundSHA256":sha(background),"poses":result},indent=2)+"\n",encoding="utf8",newline="\n")


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    for name in ("lamp","field","cache"):p.add_argument(name,type=Path)
    p.add_argument("--relief",action="store_true");a=p.parse_args();run(a.lamp,a.field,a.cache,a.relief)
