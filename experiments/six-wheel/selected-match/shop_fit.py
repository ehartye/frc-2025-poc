"""Reopen the manufactured changes and screen added material over the full slide travel."""
import argparse
import json
from pathlib import Path

import numpy as np

from clearance import ROOT, add_exchange, cad, digest, dock, native_box, poses, read
from fit import distance, finalize_step, subtract, write_step
from rail_profile import CHANNEL_Z, WIRE_Y, profile, revised


def swept_box(shape, delta):
    b = np.array(cad.bounds(shape))
    return b + [min(0,delta),0,0,max(0,delta),0,0]


def check_translation(part, obstacle, delta):
    if abs(delta)<1e-8:
        return cad.intersection(part,obstacle), False
    # A box is allowed to touch another surface. Confirm material with a Boolean
    # before refining the interval. Narrow intervals also use the native distance.
    intervals=[(min(0,delta),max(0,delta))]
    while intervals:
        a,b=intervals.pop()
        midpoint=(a+b)/2
        at_mid=cad.translated(part,[midpoint,0,0])
        if distance(at_mid,obstacle)>(b-a)/2+1e-6:
            continue
        bb=swept_box(part,b-a)+[a,0,0,a,0,0]
        if cad.intersection(native_box(bb),obstacle)<.0001:
            continue
        common=cad.intersection(at_mid,obstacle)
        if common>.001:
            return common,False
        if b-a<.025:
            return 0.,True
        intervals.extend(((a,midpoint),(midpoint,b)))
    return 0.,False


def beam_screen():
    # One inner U cantilevering beyond the front of its middle guide at full stroke.
    # Apply the entire published one-block uplift case to one front rail location.
    outline=[(-160.,CHANNEL_Z),*profile()]
    trials=[]
    for x in np.linspace(-25,140,661):
        bottom=float(np.interp(x,[p[0] for p in outline],[p[1] for p in outline]))
        depth=479-bottom
        area=20*3 + 2*3*(depth-3)
        neutral=(60*1.5 + 6*(depth-3)*(3+(depth-3)/2))/area
        inertia=20*3**3/12+60*(neutral-1.5)**2+2*(3*(depth-3)**3/12+3*(depth-3)*(3+(depth-3)/2-neutral)**2)
        moment=1000*(140-x)
        trials.append((moment*max(neutral,depth-neutral)/inertia,float(x)))
    worst=max(trials)
    return {"frontBlockN":1000,"supportXmm":-25,"loadXmm":140,"highestGrossBendingMPa":worst[0],
            "governingXmm":worst[1],"sameStation500N_MPa":worst[0]/2,
            "frontHoleBottomLigamentMm":475.5-1.55-(465+3),
            "retainedTopHoleLigamentMm":479-(475.5+1.55)}


def linear_travel(a,b):
    diff=np.array(b)-np.array(a)
    if np.max(abs(diff[[1,2,4,5]]))<1e-5 and abs(diff[3]-diff[0])<1e-5:
        for shift in (0,185,370):
            if abs(diff[0]-shift)<1e-4:
                return shift
    return None


def run(artifacts,lamp,output):
    source,placed=poses(artifacts,lamp)
    joints=read("drive-side-attachment/fit.json")
    step=ROOT/"drive-side-attachment/inner-rail-joints.step"
    for group in (source,placed):
        add_exchange(group,step,joints["volumeMm3"],joints["stepSha256"])
    kit=revised(source,0)
    finish=revised(placed,370)
    current={**source,**kit}
    final={**placed,**finish}
    before={k:cad.bounds(s) for k,s in current.items()}
    after={k:cad.bounds(s) for k,s in final.items()}
    moves={k:linear_travel(before[k],after[k]) for k in current}
    rotation=read("design/competition-package/shop-duplex/red/rotation-bounds.json")
    frames=read("design/competition-package/shop-duplex/red/part-frames.json")
    for k,row in rotation.items():
        if k not in current:
            continue
        b=np.array(row["bounds_mm"])+[0,0,560,0,0,560]
        current[k]=native_box(b)
        before[k]=b
        moves[k]=185 if frames[k]=="middle" else 0
        after[k]=b+[moves[k],0,0,moves[k],0,0]
    new={k:(subtract(v,source[k]) if k in source else v) for k,v in kit.items()}
    patches={k:p for k,p in new.items() if cad.solid_count(p) and cad.volume(p)[0]>.001}
    # The wire changes length with the slide. Its entire relative swept volume is
    # a single coaxial rod for any base, middle or slide-mounted counterpart.
    wire="slide-position-line"
    ranges={}
    for k in kit:
        if k==wire:
            continue
        if moves[k] is None: raise ValueError("Unclassified changed shape: "+k)
    for n,shape in patches.items():
        if n==wire:continue
        delta=moves[n]
        for k,other in current.items():
            if k==n: continue
            if k==wire:
                box=before[k]; end=after[k]
                lo=min(box[0],end[0]-delta); hi=max(box[3],end[3]-delta)
                other=cad.brep(["cyl",[lo,WIRE_Y,415,1,0,0,.24,hi-lo]])
                travel=0.
            elif moves[k] is None:
                # The end hull includes the entire changing straight-line length.
                b0,b1=np.array(before[k]),np.array(after[k])-[delta,0,0,delta,0,0]
                otherbox=np.r_[np.minimum(b0[:3],b1[:3]),np.maximum(b0[3:],b1[3:])]
                if dock.separation(cad.bounds(shape),otherbox)>.001:continue
                other=native_box(otherbox)
                travel=0.
            else:
                travel=delta-moves[k]
            if dock.separation(swept_box(shape,travel),cad.bounds(other))>1e-6:
                continue
            hit,unresolved=check_translation(shape,other,travel)
            if hit>.001 or unresolved:
                ranges[n+"/"+k]={"intersectionMm3":hit,"unresolved":unresolved}
    # Sweep the entire moved wire against every third body, including fixed hardware.
    for k,other in current.items():
        if k==wire:continue
        if moves[k] is None:
            b0,b1=np.array(before[k]),np.array(after[k])
            other=native_box(np.r_[np.minimum(b0[:3],b1[:3]),np.maximum(b0[3:],b1[3:])])
        motion=moves[k] or 0.
        lo=26-motion;hi=445-motion
        hi=max(75,hi)
        lead=cad.brep(["cyl",[lo,WIRE_Y,415,1,0,0,.24,hi-lo]])
        if dock.separation(cad.bounds(lead),cad.bounds(other))>1e-6:continue
        common=cad.intersection(lead,other)
        if common>.001:ranges[wire+"/"+k]={"intersectionMm3":common,"unresolved":False}
    # Write separately from the full assembly. Only original fabricated parts and
    # hardware envelopes can be distributed here. Supplier reservations remain local.
    fabricated={k:v for k,v in kit.items() if k not in ("slide-position-SP1-25",wire)}
    out=output/"receiver-clearance-kit.step"
    write_step(out,fabricated);finalize_step(out)
    reopened=cad.read_named(out,[{"id":k} for k in fabricated])
    volumes={k:cad.volume(v)[0] for k,v in reopened.items()}
    original={k:cad.volume(source[k])[0] for k in fabricated if k in source}
    adverse_flag = cad.translated(kit["slide-position-target"],[16,-142-WIRE_Y,0])
    flag_hit = cad.intersection(adverse_flag,source["slide-extended-stop"])
    adverse_sensor = cad.translated(source["slide-position-SP1-25"],[-42,0,0])
    sensor_hit = cad.intersection(adverse_sensor,source["height-R-pin"])
    if min(flag_hit,sensor_hit) < 1.0:
        raise ValueError("Installed adverse material controls did not reproduce")
    return {"kitSha256":digest(out),"lampSha256":digest(lamp),"reopenedCount":len(reopened),
            "volumeMm3":volumes,"originalVolumeMm3":original,
            "maxVolumeRoundTripMm3":max(abs(volumes[k]-cad.volume(v)[0]) for k,v in fabricated.items()),
            "materialOrUnresolved":ranges,"rejectedLocators":{"targetAtYminus142Stroke16Mm3":flag_hit,
            "sensorHousingAtOriginalYBack42HitsHeightPinMm3":sensor_hit},"registeredBodiesAndEnvelopes":len(current),
            "wireStowedMm":before[wire][3]-before[wire][0],"wireExtendedMm":after[wire][3]-after[wire][0],
            "beamScreen":beam_screen()}


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    for arg in ("artifacts","lamp","output"):
        parser.add_argument(arg,type=Path)
    args=parser.parse_args()
    result=run(args.artifacts,args.lamp,args.output)
    (args.output/"kit.json").write_text(json.dumps(result,indent=2,allow_nan=False)+"\n",encoding="utf-8",newline="\n")
    print(result["materialOrUnresolved"],flush=True)
