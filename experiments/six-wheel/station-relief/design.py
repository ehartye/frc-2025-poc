"""Subtractive relief on the inner edges of the four stock 5 mm cheek plates."""
import argparse
import hashlib
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT / "station-supply"))
from entry_window import ASSEMBLY, BASE, cad, clearance, moved, registered_robot, receiver_drives
from duplex import finalize_step
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut, BRepAlgoAPI_Common

RELIEF = .45
INWARD = .6
OUTWARD = 1.4
SIDES = ("L", "R")
CASES = ((5,.50,.25), (5,.75,.25),(5,.80,.25), (5,.90,.20), (5,1.0,.15), (5,1.15,.10), (5,1.25,.15), (5,2,.50))


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def revised_parts(robot, amount, preserve_insert_lip=True):
    changed = {}
    for side in SIDES:
        for deck in ("upper", "lower"):
            key = f"grip-{side}-{deck}-cheek"
            shape = robot[key]
            x,y,z,xx,yy,zz = cad.bounds(shape)
            edge = y if side=="L" else yy
            tool = receiver_drives.block(x-1, edge-1 if side=="L" else edge-amount,
                                         z-1, xx-x+2, 1+amount, zz-z+2)
            if preserve_insert_lip:
                insert = robot[f"grip-{side}-{deck}-replaceable-offset-insert"]
                i,j,k,ii,jj,kk = cad.bounds(insert)
                # Keep the entire original narrow web around the offset insert. A cutter runout
                # is required at the two ends of the straight relief; cutting short is safe for stock.
                keep = receiver_drives.block(i-1.,y-2.,z-2.,ii-i+2.,yy-y+4.,zz-z+4.)
                tool = BRepAlgoAPI_Cut(tool,keep).Shape()
            changed[key] = BRepAlgoAPI_Cut(shape,tool).Shape()
    return changed


def inspect(path):
    robot=registered_robot()
    stops=ROOT/"station-supply/receiver-stops.step"
    robot.update(cad.read_named(stops,[{"id":f"grip-{s}-inboard-hardstop"} for s in SIDES]))
    original = dict(robot)
    speed=json.loads((ROOT/"station-supply/transfer.json").read_bytes())["feed"]["lip_mps"]
    results={}
    for label, amount, keep in (("retained-insert-web",RELIEF,True),("revised-pocket",RELIEF,False)):
        changes = revised_parts(original,amount,keep)
        if keep:
            stock={}
        else:
            # Move the mating pocket boundary by the same amount as the relieved face.
            # A replacement plate from 5 mm stock keeps the original 1 mm pocket wall.
            for side in SIDES:
                for deck in ("upper","lower"):
                    plate=f"grip-{side}-{deck}-cheek"
                    key=f"grip-{side}-{deck}-replaceable-offset-insert"
                    insert=original[key]
                    x,y,z,xx,yy,zz=cad.bounds(insert)
                    cutter=receiver_drives.block(x-1,y-1 if side=="L" else yy-amount,
                                                  z-1,xx-x+2,1+amount,zz-z+2)
                    strip=BRepAlgoAPI_Common(insert,cutter).Shape()
                    changes[plate]=cad.BRepAlgoAPI_Fuse(changes[plate],strip).Shape()
                    changes[key]=BRepAlgoAPI_Cut(insert,cutter).Shape()
            tags=[{"id":k,"group":"machined"} for k in changes]
            cad.write_named(path,list(changes.values()),tags,{"machined":"#acb5c5"})
            finalize_step(path)
            reopened=cad.read_named(path,tags)
            for key,shape in reopened.items():
                before=cad.volume(original[key])[0]
                candidate=cad.volume(shape)[0]
                mate=key.replace("cheek","replaceable-offset-insert") if key.endswith("cheek") else key.replace("replaceable-offset-insert","cheek")
                parent=cad.BRepAlgoAPI_Fuse(original[key],original[mate]).Shape()
                if cad.solid_count(shape)!=1 or abs(cad.intersection(shape,parent)-candidate)>.01:
                    raise ValueError("Reopened part outside stock pair or splits "+key)
                if cad.intersection(reopened[mate],shape)>.01:
                    raise ValueError("Insert overlaps cheek "+key)
                stock[key]={"removed_mm3":before-candidate,"remaining_mm3":candidate}
            changes=reopened
        placed=moved(original|changes, BASE+40, 0, 0)
        results[label]=[clearance(placed,speed,e,INWARD) for e in CASES]
    return {"schema":"station-cheek-relief/1","sourceSha256":{
        p.relative_to(ROOT).as_posix():sha(p) for p in (stops,ROOT/"station-supply/entry-window.json",
        ASSEMBLY/"red/source-register.json",ROOT/"station-supply/transfer.json",ROOT/"design/receiver-drives/receiver_drives.py")},
        "sourceBodies":len(robot),"reopenedStepSha256":sha(path),"stock":stock,"reliefMm":RELIEF,
        "inwardWorstMm":INWARD,"outwardAvailableMm":OUTWARD,"cases":results}


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--step",type=Path,required=True)
    parser.add_argument("--output",type=Path,required=True)
    args=parser.parse_args()
    args.output.write_text(json.dumps(inspect(args.step),indent=2)+"\n",encoding="utf-8",newline="\n")
