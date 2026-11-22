"""Register both real housing washers seated and sweep the admitted axial end float of the races."""
import itertools
import json
import sys
from pathlib import Path

import numpy as np

from output_mount import ROOT, digest, read, place
from pan_thrust_layout import translate


def run(plugin):
    sys.path.insert(0,str(plugin/"scripts"))
    from mechanical_motion import common,distance
    from showcase_contacts import load_solids
    t=ROOT/"artifacts/red2-integrated/pan-thrust"
    plan=read(t/"r3-native-plan.json")
    cold=read(t/"native/pan-r3-stock-machining-01.json")
    file=t/"native/integrated-340mm-pan-r3-machined.step"
    if digest(ROOT/plan["authority"]["file"])!=plan["authority"]["sha256"] or digest(file)!=cold["stepSha256"]:
        raise ValueError("Need the real issued machining checkpoint")
    support=load_solids(file)[0]["Piloted bilateral bearing support"]
    all_parts={};out={}
    for side in ["inboard","outboard"]:
        fixed_delta_y=-.05 if side=="inboard" else .05
        sources={}
        for name,rec in plan["bearingPlacements"].items():
            if not name.startswith(side+" "):continue
            ref=rec["originalTransfer"];part=ROOT/ref["file"]
            if digest(part)!=ref["sha256"]:raise ValueError(name)
            s,=load_solids(part)[0].values()
            sources[name]=(rec,place(s,np.array(rec["matrixMm"])))
        fixed_name,=[k for k,(r,s) in sources.items() if r["group"]=="fixed"]
        mobile_name,=[k for k,(r,s) in sources.items() if r["group"]=="moving"]
        ball_names=[k for k in sources if "ball bearing" in k.lower() or "ball " in k.lower()]
        print(side,list(sources),flush=True)
        if len(ball_names)!=8:
            ball_names=[k for k in sources if "ball" in k and "housing" not in k]
        if len(ball_names)!=8:raise ValueError("Missing eight actual balls "+str(ball_names))
        cage_name,=[k for k,(r,s) in sources.items() if r["group"]=="cage" and k not in ball_names]
        measured={}
        fixed_raw=sources[fixed_name][1]
        housing=translate(fixed_raw,[0,fixed_delta_y,0])
        row=dict(fixedName=fixed_name,shaftName=mobile_name,sourceFaceGapMm=distance(fixed_raw,support),
                 finalSeatGapMm=distance(housing,support),finalSeatCommonMm3=common(housing,support),
                 fixedShiftYmm=fixed_delta_y,source=sources[fixed_name][0],states=measured)
        if abs(row["sourceFaceGapMm"]-.05)>.0005 or row["finalSeatGapMm"]>.0005 or row["finalSeatCommonMm3"]>.001:
            raise ValueError("Proposed fixed seat failed "+str(row))
        for shaft_y in [-.05,0.,.05]:
            shaft=translate(sources[mobile_name][1],[0,shaft_y,0])
            # At a loaded end the rolling set is seated in both race tracks. With
            # clearance, test all cage axial boundaries plus the centered value.
            for alpha in ([.5] if shaft_y==fixed_delta_y else [0.,.5,1.]):
                cage_y=shaft_y*(1-alpha)+fixed_delta_y*alpha
                row_states={}
                for k in [*ball_names,cage_name]:
                    s=translate(sources[k][1],[0,cage_y,0])
                    row_states[k]=dict(toFixedGapMm=distance(s,housing),toFixedCommonMm3=common(s,housing),
                                       toShaftGapMm=distance(s,shaft),toShaftCommonMm3=common(s,shaft),toPlateCommonMm3=common(s,support))
                measured[f"shaftY={shaft_y};cageFraction={alpha}"]=dict(shaftYmm=shaft_y,cageYmm=cage_y,
                         shaftSideWasherToPlateCommonMm3=common(shaft,support),contacts=row_states)
        out[side]=row
        (t/"r3-housing-races-seated.json").write_text(json.dumps(dict(authority=plan["authority"],sourcePlanSha256=digest(t/"r3-native-plan.json"),
                   machinedNativeSha256=cold["nativeSha256"],machinedStepSha256=digest(file),sourceBearing=plan["sourceBearing"],
                   installed=out,scope="Measured real neutral races/balls and actual fixed plate, across clearance states. Loaded roller compliance, frictional torque, lifetime and independent cage angular evolution remain explicit duties."),indent=2)+"\n")
        print("max common",max(c[v] for s in measured.values() for c in s["contacts"].values() for v in ("toFixedCommonMm3","toShaftCommonMm3","toPlateCommonMm3")),flush=True)


if __name__=="__main__":run(Path(sys.argv[1]))
