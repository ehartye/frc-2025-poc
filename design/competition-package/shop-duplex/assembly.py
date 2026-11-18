"""Combine the pinned bumper chassis, positively fastened shop joints and duplex lift."""
from functools import cache
import hashlib
import json
from pathlib import Path
import sys

HERE=Path(__file__).resolve().parent
BASE=HERE.parent
ROOT=BASE.parents[1]
sys.path.insert(0,str(BASE))
import build as original
import duplex
from layer_inventory import metadata

cad=original.native
SHOP=ROOT/"design/shop-package"
LIFT=ROOT/"design/receiver-lift"
OLD_SHOP={f"anchor-block-{lr}-{x}" for lr in "LR" for x in [-80,140]}|{"slide-L-inner","slide-R-inner"}
SHOP_SHA="fc8b3941e51b547c1812ea575967bbaf22a73e03c2b30314a6995ece4657a456"
DUPLEX_SHA="f67034bf376d9d76d045576ca39dfc24e56014e35f0fc23eb29ecf47534946fd"


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def shop_frames():
    result={}
    for lr in "LR":
        for station in ["rear","center","front"]:
            prefix=station+"-"+lr+"-"
            result[prefix+"nut-saddle"]="base"
            for x in [-19,19]:
                for dz in [-17,17]:
                    for key in ["washer","screw"]:
                        result[prefix+f"{key}-{x}_{dz}"]="base"
        result["drilled-slide-"+lr+"-inner"]="slide"
        for x in [-80,140]:
            result[f"tapped-anchor-block-{lr}-{x}"]="slide"
            for dx in [-5,5]:result[f"pin-{lr}-{x}-{dx}"]="slide"
    return result


@cache
def registered_shop():
    if sha(SHOP/"proposed-joints.step")!=SHOP_SHA:raise ValueError("Different shop STEP")
    return cad.read_named(SHOP/"proposed-joints.step",[{"id":k} for k in shop_frames()])


@cache
def retention(height):
    # The layer reconstructions are checked in all poses against their independent register.
    if sha(LIFT/"duplex-place.step")!=DUPLEX_SHA:raise ValueError("Different duplex reference STEP")
    shapes=duplex.native(height)
    if height==560:
        exact=cad.read_named(LIFT/"duplex-place.step",[{"id":k} for k in shapes])
        for k,body in shapes.items():
            overlap=cad.intersection(body,exact[k])
            if max(abs(cad.volume(a)[0]-overlap) for a in [body,exact[k]])>.01:
                raise ValueError("Reconstructed lift differs from reviewed STEP: "+k)
    return shapes


def pose_layers(height_mm,stroke_mm,alliance="blue"):
    layers,groups,colors,frames=original.pose_layers(height_mm,stroke_mm,alliance)
    replaced=OLD_SHOP|duplex.REPLACES
    existing=original.flatten(layers)
    if not replaced <= existing.keys():raise ValueError("Missing expected replaced IDs")
    # Keep the original source-level family and frame contracts for all remaining parts.
    layers={name:{k:v for k,v in bodies.items() if k not in replaced} for name,bodies in layers.items()}
    for k in replaced:
        for register in [groups,colors,frames]:del register[k]
    shop=registered_shop()
    for k,frame in shop_frames().items():
        sid="Shop clamps" if frame=="base" else "Shop slide joints"
        layers.setdefault(sid,{})[k]=shop[k] if frame=="base" else cad.translated(shop[k],(stroke_mm,0,height_mm-940))
        groups[k],colors[k],frames[k]="Shop joints","#ba884f" if frame=="base" else "#a77048",frame
    for k,shape in retention(height_mm).items():
        frame,material,_=metadata(k)
        sid="Duplex "+frame
        layers.setdefault(sid,{})[k]=shape
        groups[k],colors[k],frames[k]="Lift retention",{"steel":"#718698","aluminum":"#ce914e","braid-envelope":"#3b8c87"}[material],frame
    all_ids=set(original.flatten(layers))
    if len(all_ids)!=545 or any(set(d)!=all_ids for d in [groups,colors,frames]):
        raise ValueError("Inconsistent successor inventory")
    return layers,groups,colors,frames


def pose_shapes(height_mm,stroke_mm,alliance="blue"):
    return original.flatten(pose_layers(height_mm,stroke_mm,alliance)[0])


def source_register():
    tracked=[BASE/"part-frames.json",BASE/"review.json",BASE/"source-register.json",
             BASE/"viewer.template.html",BASE/"build.py",BASE/"travel.py",
             SHOP/"native_check.py",SHOP/"proposed-joints.step",SHOP/"native-review.json",
             LIFT/"duplex.py",LIFT/"layer_inventory.py",LIFT/"duplex-place.step",
             LIFT/"duplex-fit.json",LIFT/"duplex-inventory.json",LIFT/"delivery-retention.json",
             HERE/"assembly.py",HERE/"build_successor.py",HERE/"motion.py",HERE/"mass.py",
             ROOT/"full-match/compare_kits.py"]
    inherited=json.loads((BASE/"source-register.json").read_text())
    for name,expected in inherited.items():
        if sha(ROOT/name)!=expected:raise ValueError("Changed 401-source dependency: "+name)
    return {p.relative_to(ROOT).as_posix():sha(p) for p in tracked}
