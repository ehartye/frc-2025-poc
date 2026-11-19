"""Reserve removable tools and the whole undetailed supplier package independently."""
import numpy as np
from sp1_hardware import FACE, GUIDE, MOUNT, PIN, Y, Z, box, cad, yrod, zrod
from clearance import dock, native_box, read, wire_boxes
from fit import unite


def check_space(probe, scene, omit):
    bounds=cad.bounds(probe)
    hits={}
    for key,shape in scene.items():
        if key in omit or dock.separation(bounds,cad.bounds(shape))>0.001:
            continue
        volume=cad.intersection(probe,shape)
        if volume>.001:
            hits[key]=volume
    return hits


def run(start,end):
    own={"sp1-supplier-reservation","sp1-foot-M3-0","sp1-foot-M3-1","sp1-live-cable"}
    # Whole outer box of the page-3 package with 1 mm free-side tolerance.
    # Its upper datum is the clamped mounting face. The two foot screws and the
    # changing pull line necessarily enter this box: check those locally in scan().
    gross=box(-40.6,Y-18.7,374.4,GUIDE+1,Y+30.6,FACE)
    envelope={}
    fixed,mobile=wire_boxes()
    rotates=read("design/competition-package/shop-duplex/red/rotation-bounds.json")
    frames=read("design/competition-package/shop-duplex/red/part-frames.json")
    for key in start:
        if key in own:
            continue
        a,b=np.array(cad.bounds(start[key])),np.array(cad.bounds(end[key]))
        if key in rotates:
            a=np.array(rotates[key]["bounds_mm"])+[0,0,560,0,0,560]
            b=a+[185,0,0,185,0,0] if frames[key]=="middle" else a
        pad=np.array([0,1.5,0]) if key.startswith("grip-") else np.zeros(3)
        bound=native_box(np.r_[np.minimum(a[:3],b[:3])-pad,np.maximum(a[3:],b[3:])+pad])
        if dock.separation(cad.bounds(gross),cad.bounds(bound))>1e-6:
            continue
        hit=cad.intersection(bound,gross)
        if hit>.001:
            envelope[key]={"sweptBoxIntersectionMm3":hit}
    for name,flat in {**fixed,**mobile}.items():
        b=np.array(flat)
        if name in mobile:
            b=b+[0,0,0,370,0,0]
        vol=cad.intersection(native_box(b),gross) if dock.separation(b,cad.bounds(gross))<1e-6 else 0
        if vol>.001:
            envelope[name]={"sweptBoxIntersectionMm3":vol}
    service={}
    for stroke,scene in ((0,start),(370,end)):
        x=PIN+stroke
        probes={
            "counterhold-blade":(yrod(x,Y-45,Z,2.4,37.75),{"sp1-eye-shoulder-pin"}),
            "5.5AF-socket":(yrod(x,Y+6.05,Z,4.3,38.95),{"sp1-eye-shoulder-pin","sp1-eye-locknut"}),
            # Swept pin: head moves away, its smaller shaft leaves through the reamed
            # ear and the eye. Only those service mating passages may be traversed.
            "removed-pin":(unite(unite(yrod(x,Y-35,Z,3.75,29.75),yrod(x,Y-33,Z,2,38.55)),
                           yrod(x,Y+5.55,Z,1.5,5.7)),{"sp1-eye-shoulder-pin","sp1-eye-locknut"}),
        }
        for i,(u,v) in enumerate(MOUNT):
            probes[f"foot-driver-{i}"]=(zrod(u,v,FACE-38,2.8,33),{f"sp1-foot-M3-{i}"})
        service[str(stroke)]={label:check_space(probe,scene,omit) for label,(probe,omit) in probes.items()}
    return {"supplierFreeSidePadMm":1,"grossThirdBodyCandidates":envelope,"serviceToolHits":service,
            "faceToCenterlineMm":12.2,"screwCenterSpanMm":35.6,
            "eyeRetractedGuideToPinMm":63.4-39.5,"freeStrokeMm":[49-(63.4-39.5),419-(63.4-39.5)]}
