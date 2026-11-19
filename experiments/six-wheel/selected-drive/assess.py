"""Evaluate the selected chassis, bounded competition loads and short pre-match alternative."""
import argparse
import csv
import json
from pathlib import Path

import numpy as np
from route import HERE,ROOT,read,samples,sensitivity,sha
from electrical import screen
import landing

def bounds(t,arcs,tolerance,track_rel):
    # Lipschitz bound for integrated SE(2) odometry. Wheel arcs can take independent
    # scale errors of either sign each interval. Heading is allowed to drift throughout.
    center=abs(arcs.mean(axis=1));wheel_sum=np.sum(abs(arcs),axis=1)
    turn=abs(arcs[:,1]-arcs[:,0])
    arc_error=.5*tolerance*wheel_sum
    direction=np.cumsum((tolerance*wheel_sum+track_rel*turn)/(.580*(1-track_rel)))
    pos=np.cumsum(arc_error+center*direction)
    return float(np.max(1000*(pos+.425*np.sqrt(2)*direction)))

def run(images,rack,jdk):
    native=read(HERE/"wheels.json")
    if native["nominalTrackMm"]!=580 or any(abs(r["outerRadiusMm"]-76.2)>1e-5 for r in native["wheels"].values()):
        raise ValueError("Unreviewed wheel geometry")
    masses=read(ROOT/"axle-retention/accounting.json")["representedMassKg"]
    limits=(masses[0]+6.17+5.9,masses[1]+.001+6.17+7.5)
    joints=read(ROOT/"drive-side-attachment/accounting.json")["representedRangeKg"]
    attached_high=joints[1]+.001+6.17+7.5
    close=read(ROOT/"relieved-match/match.json")["cases"]["nominal"]["stowedStockLowerMm"]
    cases={};raw_contact={}
    for mode in ("nominal","bridge26","single","near","nearSlow"):
        t,p,arcs,labels,events,f,legs=samples(images,mode)
        arguments=(t,arcs,labels,events)
        t_end=legs[0][2].duration
        if mode in ("nominal","bridge26"):
            target=rack/mode;target.mkdir(exist_ok=True)
            landing.run(rack,jdk,rack/("path-"+mode+"-coral-A.csv"),rack/"events.csv",scenario=mode,registration=rack,output=target)
            with (target/"landing-observed.csv").open() as fh:
                raw_contact[mode]=[{k:r[k] for k in ("case","outcome","min_front_m","min_riser_m","max_compression_m")} for r in csv.DictReader(fh)]
        params={
            "attachedPlus50g":(attached_high+.05,(11.8,.027),.38,.44,.75),
            "attachedScreened":(attached_high,(11.8,.027),.38,.44,.75),
            "attachedWeak":(attached_high,(11.8,.032),.38,.44,.75),
            "fresh":(limits[1],(12.4,.025),.38,.44,.75),
            "continuousMaxGrip":(limits[1],(11.8,.027),.38,.44,.75),
            "lower":(limits[0],(11.8,.027),.38,.44,.75),
            "calibrated":(limits[1],(11.8,.027),.38,.44,.75),
            "screened":(limits[1],(11.8,.027),.38,.44,.75),
            "weak":(limits[1],(11.8,.032),.38,.44,.75),
            "depleted":(limits[1],(11.3,.042),.38,.44,.75),
            "poorBalance":(limits[1],(11.8,.027),.55,.32,.95)}
        motors={}
        for name,(kg,pack,scrub,side,outer) in params.items():
            motors[name]=screen(*arguments,kg,mode,pack,rack,scrub=scrub,side=side,outer=outer,trace=rack/(mode+"-"+name+"-bus.csv") if name in ("screened","weak") and mode=="nominal" else None,cap_grip_speed=name=="continuousMaxGrip",calibrated=name=="calibrated",rack_case="extra50g" if name=="attachedPlus50g" else None)
        cases[mode]={"routeEndWallS":float(max(x[0]+(x[2][-1,0] if isinstance(x[2],np.ndarray) else x[2].duration) for x in legs)),
                     "openingEndWallS":t_end,"cases":motors,"kinematicSensitivity":sensitivity(t,p,arcs,labels,legs,f,mode),
                     "motionStockSourceMarginMm":close if mode in ("nominal","bridge26") else None,
                     "worstFullPathHullDriftAt1ppmMm":bounds(t,arcs,1e-6,1e-6),
                     "pointsAwarded":None}
    record={"scope":"source command feasibility; selected field occupancy, hardware and mechanism qualification remain separate",
            "R103Kg":masses,"upperRevisionReserveKg":.001,"emptyFieldKg":limits,"jointedR103Kg":joints,"jointedHighFieldKg":attached_high,"hardwareMeasured":False,
            "cases":cases,"reference597SurfaceWithShopLoadedRack":raw_contact,
            "sources":{n:sha(ROOT/n) for n in ("design/selected-shop/articles.json","axle-retention/accounting.json","drive-side-attachment/accounting.json","design/competition-package/shop-duplex/mass.py",
                    "design/drivetrain/route-receipt.json","relieved-match/navigation.json","relieved-match/match.json","packet-recovery/events.csv",
                    "design/electrical-inspection/circuits.json","camera-disconnects/mass-conditions.json","camera-disconnects/power/corners.json",
                    "selected-drive/wheels.json","selected-drive/dock.json","selected-drive/rack.json")}}
    (HERE/"results.json").write_text(json.dumps(record,indent=2)+"\n",encoding="utf8",newline="\n")

if __name__=="__main__":
    ap=argparse.ArgumentParser(description=__doc__)
    for k in ("images","rack","jdk"):ap.add_argument(k,type=Path)
    a=ap.parse_args();run(a.images,a.rack,a.jdk)
