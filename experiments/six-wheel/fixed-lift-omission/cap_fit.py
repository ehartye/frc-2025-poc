"""Check an original cap on the manufacturer long motor block and in the selected robot."""
import argparse
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT / "drive-side-attachment"))
import joint as j
sys.path.insert(0, str(ROOT / "integrated-slide"))
import render
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.STEPControl import STEPControl_Reader
from OCP.gp import gp_Trsf

BLOCK_SHA = "35981c739f1226c601641b61e2cb951e3e83ca6be8ccbfd00df3198bde408391"
X, FACE, Z = -45., -36.75, 330.
OFFSET = 25.4 / math.sqrt(2)
CENTERS = [(X+d, Z+d) for d in (-OFFSET, OFFSET)]


def cylinder(x, z, y, length, radius):
    return j.cad.brep(["cyl",[x,y,z,0,1,0,radius,length]])


def kit():
    # Two raised tapped lands preserve screw engagement behind the thin central web.
    part = cylinder(X,Z,FACE-3,3,31.25)
    for x,z in CENTERS:
        part = j.fuse(part,cylinder(x,z,FACE-9,6,4.8))
        # Major diameter instead of helical threads for conservative net aluminum.
        part = j.cut(part,cylinder(x,z,FACE-10,11,2.413))
    return {"winch-input-cap":part}


def load_block(path):
    if render.digest(path) != BLOCK_SHA:
        raise ValueError("Expected the 2017 long am-3770 block")
    reader = STEPControl_Reader()
    reader.ReadFile(str(path))
    reader.TransferRoots()
    to_robot = gp_Trsf()
    to_robot.SetValues(1,0,0,X,0,0,1,FACE+26.797,0,-1,0,Z)
    return BRepBuilderAPI_Transform(reader.OneShape(),to_robot,True).Shape()


def compare(parts, cap):
    """Bound translating/variable shaped objects by both ends of their registered sweeps."""
    bounds=j.cad.bounds(cap)
    near=[]
    for name in parts[0]:
        if name in ("motor-body","sport-ratchet-and-mount-reserve"):
            continue
        ends=[j.cad.bounds(p[name]) for p in parts]
        swept=[(min if i<3 else max)(v[i] for v in ends) for i in range(6)]
        pad=22. if "pinion" in name else 0.
        for i in range(3):
            swept[i]-=pad
            swept[i+3]+=pad
        if name.startswith("grip-"):
            swept[1]-=1.5
            swept[4]+=1.5
        if name=="carrier":
            swept[0]-=15
            swept[3]+=15
        distance=j.lower_bound(bounds,swept)
        if distance < 100:
            near.append((round(distance,5),name))
    return sorted(near)[:8]


def run(artifacts,lamp,block_path):
    model=render.scenes.load_assembly(ROOT/"selected-match/clearance.py")
    poses=render.select(model,artifacts,lamp)
    motor=poses[0]["motor-body"]
    datum=j.read(ROOT/"selected-optics/mount.json")
    fixed,travel=model.wire_boxes()
    additional={}
    for key,box in {**fixed,**travel}.items():
        if key=="registered-camera-case":
            continue
        a=list(box)
        b=[n+370 if key in travel and i in (0,3) else n for i,n in enumerate(a)]
        additional["wire/"+key]=(a,b)
    for key,row in datum["reopenedParts"].items():
        additional["camera/"+key]=(row["boundsMm"],row["boundsMm"])
    sys.path.insert(0,str(ROOT/"relieved-match"))
    import cameras  # Registers the source camera modules used by the issued renderer.
    from mechanics import source,pose,vendor
    raw,eyes,case_box,_=source(artifacts/"camera-disconnects")
    for key in ("fixed-datum-up","roof"):
        position=pose(key,case_box,eyes.mean(axis=0))
        for index,part in enumerate(raw):
            a=j.cad.bounds(vendor.transform(part,*position))
            b=[v+370 if key=="roof" and i in (0,3) else v for i,v in enumerate(a)]
            additional[f"{key}/case-{index}"]=(a,b)
    block=load_block(block_path)
    parts=kit()
    output=HERE/"input-cap.step"
    j.write_step(output,parts)
    j.finalize_step(output)
    cap=j.cad.read_named(output,[{"id":"winch-input-cap"}])["winch-input-cap"]
    screws=[]
    for x,z in CENTERS:
        # Motor-block seat counterbore starts at supplier local Z -8.509.
        seat=FACE+18.288
        shoulder=seat-25.4
        stem=cylinder(x,z,shoulder,25.4,2.40)
        head=cylinder(x,z,seat,4.7,3.9)
        screws.append({"shaftBlockCommonMm3":j.cad.intersection(stem,block),
                       "headBlockCommonMm3":j.cad.intersection(head,block),
                       "capEngagementMm":FACE-shoulder,"backMarginMm":shoulder-(FACE-9),
                       "longer31_75ProtrusionMm":(31.75-18.288)-9})
    cap_to_block=j.distance(cap,block)
    uplift={}
    for key in ("deck-main","battery-tray","backup-L-drilled-mast","backup-R-drilled-mast",
                "harness-fixed-anchor","platform-post-0","platform-post-1","platform-post-2","platform-post-3"):
        uplift[key]=j.cad.bounds(poses[0][key])
    lock_contacts={}
    end=poses[1]
    for side in ("L","R"):
        pin=end[f"height-{side}-pin"]
        sleeve=end[f"height-{side}-mast-sleeve"]
        chain=(("crossbarToPin",end["lift-crossbar-drilled"],pin),("pinToSleeve",pin,sleeve),
               ("sleeveToMast",sleeve,end[f"backup-{side}-drilled-mast"]))
        lock_contacts[side]={k:j.cad.intersection(j.cad.translated(moving,[0,0,-1]),stationary)
                             for k,moving,stationary in chain}
    geometry=j.cad.bounds(cap)
    extra=sorted((j.lower_bound(geometry,[(min if i<3 else max)(a[i],b[i]) for i in range(6)]),k)
                 for k,(a,b) in additional.items())
    receipt={"sources":{f:render.digest(ROOT/f) for f in ("integrated-slide/render.py","integrated-slide/result.json",
        "design/fixed-height/review.json","design/electrical-inspection/inspection.json","design/selected-shop/supply.csv")},
        "blockSha256":BLOCK_SHA,"capSha256":render.digest(output),"stockVolumeMm3":j.cad.volume(cap)[0],
        "partBoundsMm":j.cad.bounds(cap),"priorMotorBoundsMm":j.cad.bounds(motor),"registeredBodiesBefore":len(poses[0]),
        "blockBoundsMm":j.cad.bounds(block),"nominalCapToBlockMm":cap_to_block,
        "capBlockCommonMm3":j.cad.intersection(cap,block),
        "contactAt0_10mmMm3":j.cad.intersection(j.cad.translated(cap,[0,.1,0]),block),
        "hardware":screws,"closestOtherSweepBoundsMm":compare(poses,cap),"deckLoadPaths":uplift,
        "addedCameraAndWireEnvelopes":len(extra),"nearestAddedSweepMm":extra[0],
        "twoPermanentLockChainsAt1mmMm3":lock_contacts,"supplierFitMeasured":False}
    if cap_to_block>1e-4 or receipt["capBlockCommonMm3"]>.001 or receipt["contactAt0_10mmMm3"]<60:
        raise ValueError("Supplier face registration failed")
    if min(v[0] for v in receipt["closestOtherSweepBoundsMm"])<=0 or max(row[k] for row in screws for k in ("shaftBlockCommonMm3","headBlockCommonMm3"))>.001:
        raise ValueError(receipt)
    if extra[0][0]<=0 or any(v<=0 for row in lock_contacts.values() for v in row.values()):
        raise ValueError("Additional optical, wiring, or passive lock interface failed")
    j.write(HERE/"fit.json",receipt)


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ("artifacts","lamp","block"):
        parser.add_argument(name,type=Path)
    args=parser.parse_args()
    run(args.artifacts,args.lamp,args.block)
