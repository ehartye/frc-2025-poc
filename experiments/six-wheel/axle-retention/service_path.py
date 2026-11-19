"""Check full shaft extraction and the center ring's radial service corridor."""
import argparse
import json
from pathlib import Path

from fit import CLIP_THICK, GROOVE_START, GROOVE_WIDTH, STATIONS, add_exchange, cad, chassis_scene
from fit import digest, distance, lower_bound, unite, witnesses


def material_hits(shape,parts,bounds,ignore):
    bb=cad.bounds(shape)
    hits={}
    for name,other in parts.items():
        if name not in ignore and lower_bound(bb,bounds[name])<.001:
            volume=cad.intersection(shape,other)
            if volume>.001:
                hits[name]=round(volume,4)
    return hits


def fork(sy,short_blade=False):
    y=sy*(GROOVE_START+GROOVE_WIDTH/2)
    def prism(x0,x1,thick,z0,z1):
        return cad.brep(["box",[x0,y-thick/2,z0,x1-x0,thick,z1-z0]])
    # Swept volume for vertical introduction from below the supported chassis.
    # A 57 mm blade keeps the wider handle below the stationary bearing keeper.
    # The short-blade control intentionally puts a thick handle alongside the cap.
    top = 62 if short_blade else 20.2
    tool=unite(prism(-11,11,5,-40,top),prism(-10.5,10.5,.75,-40,68.95))
    return unite(unite(tool,prism(-10.5,-7.05,.75,-40,77.2)),prism(7.05,10.5,.75,-40,77.2))


def check(artifacts,directory):
    stock,scene=chassis_scene(artifacts)
    for step,result in (("axle-kit.step","fit.json"),("bearing-keepers.step","keepers.json")):
        r=json.loads((directory/result).read_bytes())
        add_exchange(scene,directory/step,r["volumeMm3"],r["stepSha256"])
    bounds={k:cad.bounds(v) for k,v in scene.items()}
    wheels={}
    rings={}
    for pos in STATIONS:
        for side,sy in (("L",1),("R",-1)):
            axle=f"axle-{pos}-{side}"
            ends=("front","rear") if pos=="center" else (pos,)
            ignore={k for k in scene if k.startswith("bumper-") and "sleeve" not in k
                    and any("-"+end in k for end in ends)}
            ignore.update({axle,f"clip-sweep-{side}"} if pos=="center" else {axle})
            for f in ("outer","inner"):
                ignore.update(f"{part}-{pos}-{side}-{f}" for part in ("retaining-screw","select-shim"))
            # A constant, ungrooved bar sweeps a superset of the tapped/grooved part.
            # Overlapping sections span the whole continuous pull, including the slot.
            original=stock[axle]
            sweep=unite(unite(original,cad.translated(original,[0,sy*80,0])),cad.translated(original,[0,sy*160,0]))
            hit=material_hits(sweep,scene,bounds,ignore)
            terminal=cad.bounds(cad.translated(original,[0,sy*160,0]))
            outside=min(abs(terminal[1]),abs(terminal[4]))-350
            wheels[pos+"-"+side]={"outwardTravelMm":160,"finalBareRailClearanceMm":outside,
                                  "sweptMaterialOverlapMm3":hit,"removedBumperHalves":ends}
            if hit or outside<20:
                raise ValueError("Full axle extraction failed: "+str((axle,hit,outside)))
            if pos=="center":
                tool=fork(sy)
                clip="clip-sweep-"+side
                # Give the washer and wheel-side spacer their measured service positions.
                spacer="spacer-center-3-"+side
                washer="clip-thrust-"+side
                parked=dict(scene)
                parked[spacer]=cad.translated(parked[spacer],[0,-sy*.05,0])
                parked[washer]=cad.translated(parked[washer],[0,sy*.05,0])
                pb={k:cad.bounds(v) for k,v in parked.items()}
                clearance=witnesses(tool,parked,pb,{clip})
                if clearance["overlapMm3"] or clearance["clearanceMm"]<.10:
                    raise ValueError("Ring tool corridor: "+str(clearance))
                short=material_hits(fork(sy,True),parked,pb,{clip})
                if not any(k.startswith("bearing-keeper") for k in short):
                    raise ValueError("Short-blade service control lost its keeper witness")
                # Installed stop travel is measured at the inboard groove flank after
                # all free ring travel is taken up. Keep the housing motion separate.
                stop=317.41-(GROOVE_START+CLIP_THICK+2.18)
                rings[side]={"toolCorridor":clearance,"shortBladeControlCommonMm3":short,"nominalHardStopOutwardMm":stop,
                             "nominalSocketEngagementAtStopMm":10-stop,
                             "bareGrooveToInstalledBladeMm":distance(tool,parked[axle])}
    result={"kitSha256":digest(directory/"axle-kit.step"),"keeperSha256":digest(directory/"bearing-keepers.step"),
            "wheels":wheels,"centerRings":rings}
    (directory/"service.json").write_text(json.dumps(result,indent=2)+"\n",encoding="utf-8",newline="\n")


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ("artifacts","directory"):
        parser.add_argument(name,type=Path)
    a=parser.parse_args()
    check(a.artifacts,a.directory)
