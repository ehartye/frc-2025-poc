"""Reopen the issued stowed shop assembly and register its 3-D bounds for the moving route."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
sys.path.insert(0,str(ROOT/"integrated-slide"))
from render import select,scenes


def sha(p):
    with p.open("rb") as handle:
        return hashlib.file_digest(handle,"sha256").hexdigest()


def run(assets,lamp,machined=False):
    model=scenes.load_assembly(ROOT/"selected-match/clearance.py")
    sys.path.insert(0,str(ROOT/"relieved-match"))
    from cameras import mount
    from mechanics import source,pose,mapped,vendor
    baseline=json.loads((ROOT/"station-guide/background.json").read_bytes())
    if sha(lamp)!=baseline["sourceSha256"]["supplied lamp STEP"]:
        raise ValueError("Unregistered signaling lamp")
    if machined:
        sys.path.insert(0,str(ROOT/"shelf-structure"))
        from background import installed_poses
        body,_=installed_poses(model,assets,lamp)
    else:
        body,_=select(model,assets,lamp)
    fixed,moving=model.wire_boxes()
    for name in ("roof-camera-envelope","observer-camera-proxy","observer-slotted-cradle-weldment"):
        del body[name]
    for name,bounds in {**fixed,**moving}.items():
        if name!="registered-camera-case":
            body["wire/"+name]=model.native_box(bounds)
    manifest=json.loads((ROOT/"selected-optics/mount.json").read_bytes())
    pose_src=ROOT/"selected-optics/fixed-datum.step"
    if sha(pose_src)!=manifest["stepSha256"]:
        raise ValueError("Unregistered bolted eye support")
    body.update(model.cad.read_named(pose_src,[{"id":key} for key in manifest["reopenedParts"]]))
    supplier,eyes,original,_=source(assets/"camera-disconnects")
    for key in ("fixed-datum-up","roof"):
        transform=pose(key,original,eyes.mean(axis=0))
        for i,part in enumerate(supplier):
            body[f"{key}-vendor-{i}"]=vendor.transform(part,*transform)
    if len(body)!=baseline["hardware"]["selectedPartsIncludingWire"]+(7 if machined else 0):
        raise ValueError("Unreconciled stowed native inventory")
    guide=ROOT/"station-guide/narrow-face.step"
    body.update(model.cad.read_named(guide,[{"id":"guide-positive-y"},{"id":"guide-negative-y"}]))
    out={key:list(model.cad.bounds(shape)) for key,shape in body.items()}
    # Test the fixed-cable option as additional, independently sourced material.
    sys.path.insert(0,str(ROOT/"fixed-camera-route"))
    from visibility import register
    harness=assets/"fixed-camera-route/scene.json"
    if sha(harness)!="291d68c43e21f20c33885064e6bbe8c0e47b684d13a8a06e8af61f9fd9db5886":
        raise ValueError("Unregistered fixed routing option")
    boxes=register(json.loads(harness.read_bytes())["stowed"])
    for i,box in enumerate(boxes):
        out[f"optional-fixed-harness-{i}"]=list(box)
    packet={"nativeBodyCount":len(body),"optionalCableEnvelopeCount":len(boxes),"guideSha256":sha(guide),
        "cableSceneSha256":sha(harness),"backgroundSources":baseline["sourceSha256"],"boxesMm":out}
    if machined:
        packet["machinedReceiverSha256"]=sha(ROOT/"shelf-structure/stow-receiver.step")
    (HERE/("machined-bounds.json" if machined else "bounds.json")).write_text(json.dumps(packet,separators=(",",":"))+"\n",encoding="utf8",newline="\n")


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("assets",type=Path);p.add_argument("lamp",type=Path)
    p.add_argument("--machined",action="store_true");a=p.parse_args();run(a.assets,a.lamp,a.machined)
