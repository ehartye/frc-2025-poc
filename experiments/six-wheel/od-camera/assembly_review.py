"""Reconcile the exact no-air roof robot, two receiver-mounted probes and OD gate in robot coordinates."""
import argparse
from functools import cache
import hashlib
import json
import math
from pathlib import Path
import sys
HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
for path in ("od-gate","design/fixed-no-air","moving-observer"):
    sys.path.insert(0,str(ROOT/path))
import gate_cad as gate
from gate_cad import cad,interior,sha,box,rod,ann,rotate,subtract,beam,AY,AZ
import whole
import motion
from duplex import finalize_step

def read(p):return json.loads(p.read_bytes())

def source(lamp_file):
    folder=ROOT/"moving-observer"
    ids=[*read(folder/"mount.json")["parts"],"receiver-R6-drilled-for-camera"]
    roof=cad.read_named(folder/"moving-bridge.step",[{"id":k} for k in ids])
    deck=cad.read_named(whole.LIGHT/"deck-mount.step",[{"id":k} for k in ("front-electronics-platform","light-spacer")])
    lamp=whole.mounted(whole.product(lamp_file),whole.X,whole.Y,whole.PANEL)
    return roof,deck,lamp

def baseline(stroke,kit):
    result=whole.model(stroke,*kit)[1]
    if len(result)!=593:raise ValueError("Owner 593-body inventory mismatch")
    return result

def retired(ids):
    return {k for k in ids if gate.retired(k) or k=="receiver-R6-drilled-for-camera"}

def probe(kit):
    source_robot=baseline(370,kit)
    roof={k:s for k,s in source_robot.items() if k.startswith("roof-")}
    gone=retired(source_robot)
    def cross(shapes,others):
        pairs={}
        bs={k:cad.bounds(s) for k,s in others.items()}
        for k,s in shapes.items():
            b=cad.bounds(s)
            for n,t in others.items():
                if interior(b,bs[n]):
                    v=cad.intersection(s,t)
                    if v>.05:pairs[k+" / "+n]=v
        return pairs
    return {"sourceCount":len(source_robot),"retiredReceiverIds":sorted(gone),"roofNames":sorted(roof),
            "installedRackToCameraFeet":cross(roof,{k:s for k,s in source_robot.items() if k not in roof and k not in gone}),
            "geometryVsRoof":{str(i):cross(gate.shapes(370,i),roof) for i in range(0,113,7)}}
if __name__=="__main__":
    p=argparse.ArgumentParser();p.add_argument("--lamp",type=Path,required=True);args=p.parse_args()
    (HERE/"unmodified.json").write_text(json.dumps(probe(source(args.lamp)),indent=2)+"\n",encoding="utf8",newline="\n")
